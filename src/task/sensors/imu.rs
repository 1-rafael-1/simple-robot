//! IMU (Inertial Measurement Unit) reading functionality using the ICM20948 sensor.
//!
//! Note: High-frequency diagnostics (IMU loop dt, magnetometer vector logs) should be gated
//! behind the `telemetry_logs` feature so production-like runs without a log consumer don't
//! spend time formatting/queuing logs.
//!
//! This module provides an asynchronous task that handles reading IMU data
//! from an ICM20948 9-axis sensor over I2C for motion control.
//!
//! # Architecture
//!
//! The module operates in two modes:
//! 1. Standby - not reading
//! 2. Active - continuous IMU readings at 50Hz with AHRS fusion
//!
//! # Orientation Reference Frame
//!
//! **Important:** This implementation provides **absolute orientation** values, not relative to startup:
//! - **Yaw**: Absolute compass heading (0-360° or -180 to +180°) based on magnetometer
//! - **Pitch**: Absolute forward/backward tilt relative to gravity
//! - **Roll**: Absolute left/right tilt relative to gravity
//!
//! The magnetometer provides an absolute heading reference, preventing yaw drift over time.
//! This is different from relative orientation systems that zero at startup and drift over time.
//!
//! For navigation tasks that need relative orientation (e.g., "turn 90° from current heading"),
//! compute deltas between current and target yaw values in your control logic.
//!
//! # AHRS Filter Configuration
//!
//! The Madgwick filter is configured for an indoor tracked robot with:
//! - Beta = 0.15: Balanced filter for motor vibration rejection and responsiveness
//! - 100 Hz update rate: Fast enough for motion tracking, low enough to avoid noise amplification
//! - 9-axis fusion: Uses magnetometer for absolute heading (no yaw drift)
//!
//! For a small tracked robot (~30cm) with brushed motors:
//! - Vibration filtering needs tuning (DLPF settings)
//! - Moderate beta value balances convergence speed and noise rejection
//! - Magnetometer provides heading reference but may need calibration in magnetic environments
//!
//! # Usage
//!
//! ```rust
//! // Start IMU readings
//! sensors::imu::start_imu_readings();
//!
//! // Stop readings when done
//! sensors::imu::stop_imu_readings();
//! ```

use core::sync::atomic::{AtomicBool, Ordering};

use ahrs::{Ahrs, Madgwick};
use defmt::{info, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Delay, Duration, Instant, Timer};
use icm20948::{
    AccelConfig, AccelDlpf, AccelFullScale, GyroConfig, GyroDlpf, GyroFullScale, I2cInterface, Icm20948Driver,
    MagConfig, MagMode,
};
use micromath::F32Ext;
use nalgebra::{UnitQuaternion, Vector3};

use crate::{
    I2cBusShared,
    system::{
        event::{Events, raise_event},
        state::motion,
    },
    task::{drive, io::flash_storage},
};

/// Type alias for the ICM20948 IMU sensor driver with the specific I2C interface
type ImuSensor = Icm20948Driver<
    I2cInterface<
        I2cDevice<
            'static,
            CriticalSectionRawMutex,
            embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>,
        >,
    >,
>;

// Sampling configuration
// We run the IMU + AHRS fusion at 50Hz to match a commonly supported magnetometer continuous mode,
// while still keeping CPU + I2C load reasonable.

/// The sampling interval for reading IMU data and updating AHRS fusion. Set to 20ms for 50Hz operation.
const SAMPLE_INTERVAL: Duration = Duration::from_millis(20);

/// Initial delay after power-up before starting IMU initialization.
const IMU_BOOT_DELAY_MS: u64 = 200;
/// Delay between IMU initialization attempts if the sensor is not detected or fails to initialize.
const IMU_INIT_RETRY_DELAY_MS: u64 = 50;
/// Maximum number of attempts to initialize the IMU before giving up and terminating the task.
const IMU_INIT_MAX_ATTEMPTS: u32 = 60; // ~3s total

/// Sample rate and Madgwick filter configuration
const SAMPLE_RATE_HZ: f32 = 50.0;
/// Beta parameter for Madgwick filter (0.0-1.0). Higher values give faster convergence but more noise sensitivity.
const BETA: f32 = 0.15; // Balanced for indoor tracked robot with motor vibrations

// Stationary gyro bias trim (runtime-only; no flash writes).
// Helps keep 6-axis yaw stable at rest by trimming residual gyro bias when "still" is detected.

/// Accel magnitude (g) window for stillness detection (tighter than the general accel gating window).
const STILL_ACCEL_MIN_G: f32 = 0.95;
/// Upper bound for stillness accel magnitude (g).
const STILL_ACCEL_MAX_G: f32 = 1.05;

/// Gyro norm threshold (deg/s) for stillness detection.
const STILL_GYRO_NORM_MAX_DPS: f32 = 0.8;

/// Time the IMU must be still before trimming starts.
const STILL_REQUIRED_MS: u32 = 1000;

/// Runtime gyro bias trim update gain when still (0..1).
const RUNTIME_GYRO_TRIM_ALPHA: f32 = 0.005;

/// Clamp runtime trim to a safe maximum (deg/s).
const RUNTIME_GYRO_TRIM_MAX_ABS_DPS: f32 = 3.0;

/// Stillness/bias-trim diagnostics logging interval in milliseconds.
///
/// Only used when `telemetry_logs` is enabled.
#[cfg(feature = "telemetry_logs")]
const STILLNESS_DIAG_LOG_INTERVAL_MS: u32 = 1000;

/// Maximum consecutive read failures before terminating the IMU task. This prevents infinite loops if the sensor becomes unresponsive.
const MAX_CONSECUTIVE_FAILURES: u32 = 10;

/// Magnetometer lower validation threshold in microteslas (μT).
const MAG_MIN_UT: f32 = 20.0;
/// Magnetometer upper validation threshold in microteslas (μT).
const MAG_MAX_UT: f32 = 200.0;

/// Magnetometer diagnostics logging interval in milliseconds.
/// Log at low rate to avoid spamming RTT/defmt while still giving enough signal to debug yaw issues.
#[cfg(feature = "telemetry_logs")]
const MAG_DIAG_LOG_INTERVAL_MS: u32 = 500;

/// IMU loop diagnostics logging interval in milliseconds.
/// Used to debug timing/cadence differences with/without debugger attached.
#[cfg(feature = "telemetry_logs")]
const IMU_LOOP_DIAG_LOG_INTERVAL_MS: u32 = 500;

/// Complete IMU measurement data
#[derive(Debug, Clone, Copy)]
pub struct ImuMeasurement {
    /// Orientation in 3D space
    pub orientation: Orientation,
    /// Timestamp of measurement in milliseconds
    pub timestamp_ms: u64,
}

/// 3D orientation using Euler angles (derived from quaternion via AHRS)
///
/// **Important:** These are **absolute** orientation values:
/// - Yaw is absolute compass heading from magnetometer (not relative to startup)
/// - Pitch and roll are absolute tilts relative to gravity
///
/// For relative motion control (e.g., "turn 90° left"), compute the difference
/// between current and target yaw values in your control logic.
#[derive(Debug, Clone, Copy)]
pub struct Orientation {
    /// Absolute rotation around vertical axis (compass heading) in degrees
    /// Range: -180 to +180, or normalized to 0-360
    /// **Note**: This is absolute heading from magnetometer when using 9-axis fusion. In 6-axis
    /// mode it becomes a *relative* yaw that will drift over time (but is great for short turns).
    pub yaw: f32,
    /// Absolute forward/backward tilt relative to gravity in degrees
    /// Positive = nose up, Negative = nose down
    pub pitch: f32,
    /// Absolute left/right tilt relative to gravity in degrees
    /// Positive = right side down, Negative = left side down
    pub roll: f32,
}

/// Commands for IMU reading control
#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
pub enum AhrsFusionMode {
    /// 6-axis fusion (gyro + accel). Yaw is relative and will drift, but works
    /// in magnetically noisy environments and is suitable for short precise turns.
    Axis6,
    /// 9-axis fusion (gyro + accel + mag). Yaw is stabilized to magnetic north (after calibration)
    /// and does not drift, but can be disturbed by motor/steel interference.
    Axis9,
}

/// System default for AHRS fusion mode.
/// Change this to switch the system between 6-axis and 9-axis fusion.
pub const DEFAULT_FUSION_MODE: AhrsFusionMode = AhrsFusionMode::Axis6;

/// Control commands sent to the IMU task.
enum ImuCommand {
    /// Start IMU readings
    Start,
    /// Stop IMU readings
    Stop,
    /// Load and apply calibration data
    LoadCalibration(flash_storage::ImuCalibration),
    /// Select which AHRS fusion mode to use when producing Euler angles.
    SetFusionMode(AhrsFusionMode),
}

/// Control signal for IMU reading state
static IMU_CONTROL: Signal<CriticalSectionRawMutex, ImuCommand> = Signal::new();

/// Tracks whether the magnetometer is available.
static MAG_AVAILABLE: AtomicBool = AtomicBool::new(false);

/// Latest IMU orientation snapshot for UI display.
static LATEST_ORIENTATION: Mutex<CriticalSectionRawMutex, Option<Orientation>> = Mutex::new(None);

/// Latest calibrated accelerometer reading (g).
static LATEST_CALIBRATED_ACCEL: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest calibrated gyroscope reading (deg/s).
static LATEST_CALIBRATED_GYRO: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest calibrated magnetometer reading (μT, before normalization).
static LATEST_CALIBRATED_MAG: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest raw accelerometer reading (g, before bias correction).
static LATEST_RAW_ACCEL: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest raw gyroscope reading (deg/s, before bias correction).
static LATEST_RAW_GYRO: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest raw magnetometer reading (μT, before interference correction).
static LATEST_RAW_MAG: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Start continuous IMU readings
pub fn start_imu_readings() {
    IMU_CONTROL.signal(ImuCommand::Start);
}

/// Select AHRS fusion mode (6-axis vs 9-axis)
///
/// - `Axis6`: uses gyro+accel (yaw drifts, but works for turn control)
/// - `Axis9`: uses gyro+accel+mag when magnetometer is considered valid
pub fn set_ahrs_fusion_mode(mode: AhrsFusionMode) {
    IMU_CONTROL.signal(ImuCommand::SetFusionMode(mode));
}

/// Stop IMU readings
pub fn stop_imu_readings() {
    IMU_CONTROL.signal(ImuCommand::Stop);
}

/// Get the latest IMU orientation for display purposes.
pub async fn get_latest_orientation() -> Option<Orientation> {
    let latest = LATEST_ORIENTATION.lock().await;
    *latest
}

/// Get the latest calibrated accelerometer reading (g).
pub async fn get_latest_calibrated_accel() -> Option<Vector3<f32>> {
    let latest = LATEST_CALIBRATED_ACCEL.lock().await;
    *latest
}

/// Get the latest calibrated gyroscope reading (deg/s).
pub async fn get_latest_calibrated_gyro() -> Option<Vector3<f32>> {
    let latest = LATEST_CALIBRATED_GYRO.lock().await;
    *latest
}

/// Get the latest calibrated magnetometer reading (μT, before normalization).
pub async fn get_latest_calibrated_mag() -> Option<Vector3<f32>> {
    let latest = LATEST_CALIBRATED_MAG.lock().await;
    *latest
}

/// Get the latest raw accelerometer reading (g, before bias correction).
pub async fn get_latest_raw_accel() -> Option<Vector3<f32>> {
    let latest = LATEST_RAW_ACCEL.lock().await;
    *latest
}

/// Get the latest raw gyroscope reading (deg/s, before bias correction).
pub async fn get_latest_raw_gyro() -> Option<Vector3<f32>> {
    let latest = LATEST_RAW_GYRO.lock().await;
    *latest
}

/// Get the latest raw magnetometer reading (μT, before interference correction).
pub async fn get_latest_raw_mag() -> Option<Vector3<f32>> {
    let latest = LATEST_RAW_MAG.lock().await;
    *latest
}

/// Load and apply IMU calibration data
pub fn load_imu_calibration(calibration: flash_storage::ImuCalibration) {
    IMU_CONTROL.signal(ImuCommand::LoadCalibration(calibration));
}

/// Convert nalgebra `UnitQuaternion` to Euler angles (in degrees)
///
/// Uses the aerospace sequence (ZYX - yaw, pitch, roll) which is standard for vehicle orientation:
/// - Yaw (Z-axis): heading/compass direction
/// - Pitch (Y-axis): nose up/down
/// - Roll (X-axis): banking left/right
fn quaternion_to_euler(q: &UnitQuaternion<f32>) -> Orientation {
    // Use nalgebra's built-in euler_angles() method
    // Returns (roll, pitch, yaw) in radians
    let (roll, pitch, yaw) = q.euler_angles();

    // Convert to degrees and round to 0.1 degree precision
    Orientation {
        roll: (roll.to_degrees() * 10.0).round() / 10.0,
        pitch: (pitch.to_degrees() * 10.0).round() / 10.0,
        yaw: (yaw.to_degrees() * 10.0).round() / 10.0,
    }
}

/// Interpolate motor interference for a single track (left or right)
fn interpolate_single_track(calibration: &flash_storage::ImuCalibration, speed: f32, is_left: bool) -> (f32, f32, f32) {
    let track_idx = if is_left { 1 } else { 2 };

    if speed <= 50.0 {
        // Scale down from 50% measurement
        let factor = speed / 50.0;
        (
            calibration.mag_x_interference_50[track_idx] * factor,
            calibration.mag_y_interference_50[track_idx] * factor,
            calibration.mag_z_interference_50[track_idx] * factor,
        )
    } else {
        // Interpolate between 50% and 100%
        let factor = (speed - 50.0) / 50.0;
        (
            calibration.mag_x_interference_50[track_idx]
                + (calibration.mag_x_interference_100[track_idx] - calibration.mag_x_interference_50[track_idx])
                    * factor,
            calibration.mag_y_interference_50[track_idx]
                + (calibration.mag_y_interference_100[track_idx] - calibration.mag_y_interference_50[track_idx])
                    * factor,
            calibration.mag_z_interference_50[track_idx]
                + (calibration.mag_z_interference_100[track_idx] - calibration.mag_z_interference_50[track_idx])
                    * factor,
        )
    }
}

/// Interpolate motor interference for equal motor speeds (both tracks same speed)
fn interpolate_equal_motors(calibration: &flash_storage::ImuCalibration, speed: f32) -> (f32, f32, f32) {
    let all_idx = 0;

    if speed <= 50.0 {
        let factor = speed / 50.0;
        (
            calibration.mag_x_interference_50[all_idx] * factor,
            calibration.mag_y_interference_50[all_idx] * factor,
            calibration.mag_z_interference_50[all_idx] * factor,
        )
    } else {
        let factor = (speed - 50.0) / 50.0;
        (
            calibration.mag_x_interference_50[all_idx]
                + (calibration.mag_x_interference_100[all_idx] - calibration.mag_x_interference_50[all_idx]) * factor,
            calibration.mag_y_interference_50[all_idx]
                + (calibration.mag_y_interference_100[all_idx] - calibration.mag_y_interference_50[all_idx]) * factor,
            calibration.mag_z_interference_50[all_idx]
                + (calibration.mag_z_interference_100[all_idx] - calibration.mag_z_interference_50[all_idx]) * factor,
        )
    }
}

/// Calculate motor interference correction based on current motor speeds
/// Uses interpolation for speeds between calibration points (50% and 100%)
/// and superposition for mixed left/right speeds
fn interpolate_interference(
    calibration: &flash_storage::ImuCalibration,
    left_speed: i8,
    right_speed: i8,
) -> (f32, f32, f32) {
    let left_abs = f32::from(left_speed.abs());
    let right_abs = f32::from(right_speed.abs());

    // Determine dominant pattern
    if left_abs == 0.0 && right_abs == 0.0 {
        // Motors off - no correction needed
        (0.0, 0.0, 0.0)
    } else if (left_abs - right_abs).abs() < 10.0 {
        // Both tracks approximately equal - use all_motors pattern
        let avg_speed = f32::midpoint(left_abs, right_abs);
        interpolate_equal_motors(calibration, avg_speed)
    } else {
        // Mixed speeds - superposition
        let left_contrib = interpolate_single_track(calibration, left_abs, true);
        let right_contrib = interpolate_single_track(calibration, right_abs, false);

        // Weight by relative speed
        let total_speed = left_abs + right_abs;
        let left_weight = left_abs / total_speed;
        let right_weight = right_abs / total_speed;

        (
            left_contrib.0 * left_weight + right_contrib.0 * right_weight,
            left_contrib.1 * left_weight + right_contrib.1 * right_weight,
            left_contrib.2 * left_weight + right_contrib.2 * right_weight,
        )
    }
}

/// Apply motor interference correction to magnetometer data
fn apply_motor_interference_correction(
    mag_data: &mut Vector3<f32>,
    calibration: &flash_storage::ImuCalibration,
    left_speed: i8,
    right_speed: i8,
) {
    // Calculate interference based on current motor speeds
    let interference = interpolate_interference(calibration, left_speed, right_speed);

    // Apply correction (subtract interference from reading)
    mag_data.x -= interference.0;
    mag_data.y -= interference.1;
    mag_data.z -= interference.2;
}

/// Apply magnetometer bias and soft-iron scaling
fn apply_mag_calibration(mag_data: &mut Vector3<f32>, calibration: &flash_storage::ImuCalibration) {
    mag_data.x = (mag_data.x - calibration.mag_x_bias) * calibration.mag_x_scale;
    mag_data.y = (mag_data.y - calibration.mag_y_bias) * calibration.mag_y_scale;
    mag_data.z = (mag_data.z - calibration.mag_z_bias) * calibration.mag_z_scale;
}

/// Initialize the IMU sensor with retries
async fn init_imu_sensor(i2c_bus: &'static I2cBusShared) -> ImuSensor {
    // On battery power-up, the IMU can still be in POR / internal regulator startup when
    // this task begins. Under a debugger, the extra attach/flash/reset time often masks
    // this. Add a deterministic startup delay and retry init to reduce init flakiness.
    info!("Waiting {}ms for IMU power-up...", IMU_BOOT_DELAY_MS);
    Timer::after(Duration::from_millis(IMU_BOOT_DELAY_MS)).await;

    let mut delay = Delay;

    // Initialize ICM20948 with retries
    let mut attempt: u32 = 0;
    let sensor = loop {
        attempt += 1;

        info!(
            "Initializing ICM20948 (attempt {}/{})...",
            attempt, IMU_INIT_MAX_ATTEMPTS
        );

        match Icm20948Driver::new(I2cInterface::default(I2cDevice::new(i2c_bus))).await {
            Ok(mut imu) => {
                // Detected at I2C level; now do full init.
                match imu.init(&mut delay).await {
                    Ok(()) => {
                        info!("ICM20948 detected + initialized!");
                        break imu;
                    }
                    Err(e) => {
                        warn!(
                            "ICM20948 init failed (attempt {}/{}): {:?}",
                            attempt, IMU_INIT_MAX_ATTEMPTS, e
                        );
                    }
                }
            }
            Err(e) => {
                warn!(
                    "ICM20948 detect failed (attempt {}/{}): {:?}",
                    attempt, IMU_INIT_MAX_ATTEMPTS, e
                );
            }
        }

        if attempt >= IMU_INIT_MAX_ATTEMPTS {
            info!(
                "Failed to detect/initialize ICM20948 after {} attempts",
                IMU_INIT_MAX_ATTEMPTS
            );
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }

        Timer::after(Duration::from_millis(IMU_INIT_RETRY_DELAY_MS)).await;
    };

    Timer::after(Duration::from_millis(100)).await;
    info!("ICM20948 initialized successfully!");

    sensor
}

/// Configure the IMU sensor
async fn configure_imu_sensor(sensor: &mut ImuSensor) -> bool {
    // Configure accelerometer for tracked robot:
    // ±4g range: Handles bumps and shocks from tracked drive
    // 111 Hz DLPF: Good vibration filtering for brushed motors
    // ~50 Hz internal sample rate: Matches the 50 Hz readout loop (reduced CPU + I2C load)
    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G4,
        dlpf: AccelDlpf::Hz111,
        dlpf_enable: true,
        sample_rate_div: 22, // (1125 Hz / (1 + 22)) = ~48.9 Hz
    };

    if let Err(e) = sensor.configure_accelerometer(accel_config).await {
        info!("Failed to configure accelerometer: {:?}", e);
        info!("Sensor configuration failed - IMU task terminating");
        return false;
    }
    info!("Accelerometer: ±4g, 111Hz DLPF, ~49Hz sample rate");

    // Configure gyroscope for tracked robot:
    // ±1000°/s range: Lower sensitivity, tolerates fast rotations without saturation
    // 24 Hz DLPF: Stronger filtering for low-noise idle readings
    // ~50 Hz internal sample rate: Matches the 50 Hz readout loop (reduced CPU + I2C load)
    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps1000,
        dlpf: GyroDlpf::Hz24,
        dlpf_enable: true,
        sample_rate_div: 22, // (1125 Hz / (1 + 22)) = ~48.9 Hz
    };

    if let Err(e) = sensor.configure_gyroscope(gyro_config).await {
        info!("Failed to configure gyroscope: {:?}", e);
        info!("Sensor configuration failed - IMU task terminating");
        return false;
    }
    info!("Gyroscope: ±1000°/s, 24Hz DLPF, ~49Hz sample rate");

    // Configure magnetometer for absolute heading:
    // Continuous 50Hz mode for smooth compass data
    // Provides absolute yaw reference (no drift)
    let mag_config = MagConfig {
        mode: MagMode::Continuous50Hz,
    };

    let mut delay = Delay;
    if let Err(e) = sensor.init_magnetometer(mag_config, &mut delay).await {
        info!("Failed to initialize magnetometer: {:?}", e);
        info!("Magnetometer unavailable - continuing with 6-axis fusion");
        MAG_AVAILABLE.store(false, Ordering::Relaxed);
    } else {
        info!("Magnetometer: Continuous 50Hz mode");
        MAG_AVAILABLE.store(true, Ordering::Relaxed);
    }

    Timer::after(Duration::from_millis(100)).await;
    info!("All sensors configured!");

    true
}

/// Initialize the Madgwick AHRS filter
fn init_madgwick() -> Madgwick<f32> {
    let madgwick = Madgwick::new(1.0 / SAMPLE_RATE_HZ, BETA);

    info!("Madgwick AHRS filter initialized:");
    info!("  Sample rate: {} Hz", SAMPLE_RATE_HZ);
    info!("  Beta: {} (balanced for tracked robot)", BETA);
    info!("  Default fusion mode: {:?}", DEFAULT_FUSION_MODE);

    madgwick
}

/// Check if magnetometer reading is within reasonable bounds to be considered valid for fusion
fn should_use_magnetometer(mag_norm: f32) -> bool {
    mag_norm > MAG_MIN_UT && mag_norm < MAG_MAX_UT
}

/// Logs magnetometer diagnostics for debugging yaw drift issues.
///
/// This function is only compiled when the `telemetry_logs` feature is enabled.
#[cfg(feature = "telemetry_logs")]
fn log_mag_diagnostics(mag_data: &Vector3<f32>, use_magnetometer: bool) {
    let mag_magnitude = mag_data.norm();
    #[allow(clippy::cast_possible_truncation)]
    let timestamp_ms = Instant::now().as_millis() as u32;
    if timestamp_ms % MAG_DIAG_LOG_INTERVAL_MS < 25 {
        info!(
            "MAG diag: mag=({=f32},{=f32},{=f32}) uT | |mag|={=f32} uT | use_mag={}",
            mag_data.x, mag_data.y, mag_data.z, mag_magnitude, use_magnetometer
        );
        if !use_magnetometer {
            warn!("MAG diag: rejecting magnetometer due to magnitude out of bounds");
        }
    }
}

/// Stub for magnetometer diagnostics when `telemetry_logs` feature is disabled
#[cfg(not(feature = "telemetry_logs"))]
const fn log_mag_diagnostics(_mag_data: &Vector3<f32>, _use_magnetometer: bool) {}

/// Mutable IMU sampling state held across ticks.
#[derive(Default)]
struct SampleState {
    /// Last valid accel direction used for fallback.
    last_good_accel_dir: Option<Vector3<f32>>,
    /// Runtime gyro bias trim (deg/s), applied on top of persisted calibration.
    runtime_gyro_bias_trim_dps: Vector3<f32>,
    /// Accumulated stillness time (ms).
    stillness_ms_accum: u32,
    /// Consecutive accelerometer read failures.
    accel_consecutive_failures: u32,
    /// Consecutive gyroscope read failures.
    gyro_consecutive_failures: u32,
    /// Consecutive magnetometer read failures.
    mag_consecutive_failures: u32,
}

/// Update stillness detection and runtime gyro bias trim.
fn update_stillness_and_trim(
    accel_norm: f32,
    raw_gyro: Vector3<f32>,
    gyro_data: Vector3<f32>,
    gyro_bias: (f32, f32, f32),
    dt_seconds: f32,
    state: &mut SampleState,
) {
    let accel_still = (STILL_ACCEL_MIN_G..=STILL_ACCEL_MAX_G).contains(&accel_norm);
    let raw_gyro_norm_dps = raw_gyro.norm();
    let gyro_still = raw_gyro_norm_dps <= STILL_GYRO_NORM_MAX_DPS;

    if accel_still && gyro_still {
        let dt_ms = (dt_seconds * 1000.0).clamp(0.0, 1000.0);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        {
            state.stillness_ms_accum = state.stillness_ms_accum.saturating_add(dt_ms as u32);
        }

        if state.stillness_ms_accum >= STILL_REQUIRED_MS {
            // Residual after persisted bias subtraction (deg/s).
            let residual = Vector3::new(
                gyro_data.x - gyro_bias.0,
                gyro_data.y - gyro_bias.1,
                gyro_data.z - gyro_bias.2,
            );

            let alpha = RUNTIME_GYRO_TRIM_ALPHA.clamp(0.0, 1.0);
            state.runtime_gyro_bias_trim_dps.x = ((1.0 - alpha) * state.runtime_gyro_bias_trim_dps.x
                + (alpha * residual.x))
                .clamp(-RUNTIME_GYRO_TRIM_MAX_ABS_DPS, RUNTIME_GYRO_TRIM_MAX_ABS_DPS);
            state.runtime_gyro_bias_trim_dps.y = ((1.0 - alpha) * state.runtime_gyro_bias_trim_dps.y
                + (alpha * residual.y))
                .clamp(-RUNTIME_GYRO_TRIM_MAX_ABS_DPS, RUNTIME_GYRO_TRIM_MAX_ABS_DPS);
            state.runtime_gyro_bias_trim_dps.z = ((1.0 - alpha) * state.runtime_gyro_bias_trim_dps.z
                + (alpha * residual.z))
                .clamp(-RUNTIME_GYRO_TRIM_MAX_ABS_DPS, RUNTIME_GYRO_TRIM_MAX_ABS_DPS);
        }
    } else {
        state.stillness_ms_accum = 0;
    }

    #[cfg(feature = "telemetry_logs")]
    {
        use core::sync::atomic::{AtomicU32, Ordering};
        static LAST_STILL_DIAG_MS: AtomicU32 = AtomicU32::new(0);

        let now_ms_u64 = Instant::now().as_millis();
        let now_ms = u32::try_from(now_ms_u64).unwrap_or(u32::MAX);

        let last = LAST_STILL_DIAG_MS.load(Ordering::Relaxed);
        if now_ms.wrapping_sub(last) >= STILLNESS_DIAG_LOG_INTERVAL_MS {
            LAST_STILL_DIAG_MS.store(now_ms, Ordering::Relaxed);
            defmt::info!(
                "IMU still: a={=bool} g={=bool} still_ms={} trim_z={=f32} raw_gz={=f32}",
                accel_still,
                gyro_still,
                state.stillness_ms_accum,
                state.runtime_gyro_bias_trim_dps.z,
                raw_gyro.z
            );
        }
    }
}

/// Outcome of a single IMU sampling attempt, used to control the main loop flow
enum SampleOutcome {
    /// Successfully obtained a new IMU measurement to process
    Measurement(ImuMeasurement),
    /// Failed to read sensor data, but should keep trying (e.g., transient I2C error)
    Continue,
    /// Too many consecutive failures, should terminate the IMU task
    Terminate,
}

/// Remap IMU axes from sensor frame to robot frame
fn remap_imu_axes(data: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(-data.y, data.x, data.z)
}

/// Read accelerometer data with error handling and failure tracking
async fn read_accelerometer(
    sensor: &mut ImuSensor,
    accel_consecutive_failures: &mut u32,
) -> Result<Vector3<f32>, SampleOutcome> {
    match sensor.read_accelerometer().await {
        Ok(data) => {
            *accel_consecutive_failures = 0;
            Ok(remap_imu_axes(Vector3::new(data.x, data.y, data.z)))
        }
        Err(e) => {
            *accel_consecutive_failures += 1;
            info!(
                "Failed to read accelerometer: {:?} (failure {} of {})",
                e, *accel_consecutive_failures, MAX_CONSECUTIVE_FAILURES
            );
            if *accel_consecutive_failures >= MAX_CONSECUTIVE_FAILURES {
                info!("Max consecutive accelerometer failures reached - IMU task terminating");
                Err(SampleOutcome::Terminate)
            } else {
                Err(SampleOutcome::Continue)
            }
        }
    }
}

/// Read gyroscope data with error handling and failure tracking
async fn read_gyroscope(
    sensor: &mut ImuSensor,
    gyro_consecutive_failures: &mut u32,
) -> Result<Vector3<f32>, SampleOutcome> {
    match sensor.read_gyroscope().await {
        Ok(data) => {
            *gyro_consecutive_failures = 0;
            Ok(remap_imu_axes(Vector3::new(data.x, data.y, data.z)))
        }
        Err(e) => {
            *gyro_consecutive_failures += 1;
            info!(
                "Failed to read gyroscope: {:?} (failure {} of {})",
                e, *gyro_consecutive_failures, MAX_CONSECUTIVE_FAILURES
            );
            if *gyro_consecutive_failures >= MAX_CONSECUTIVE_FAILURES {
                info!("Max consecutive gyroscope failures reached - IMU task terminating");
                Err(SampleOutcome::Terminate)
            } else {
                Err(SampleOutcome::Continue)
            }
        }
    }
}

/// Read magnetometer data with error handling and failure tracking
async fn read_magnetometer(
    sensor: &mut ImuSensor,
    mag_consecutive_failures: &mut u32,
) -> Result<Vector3<f32>, SampleOutcome> {
    match sensor.read_magnetometer().await {
        Ok(data) => {
            *mag_consecutive_failures = 0;
            Ok(remap_imu_axes(Vector3::new(data.x, data.y, data.z)))
        }
        Err(e) => {
            *mag_consecutive_failures += 1;
            info!(
                "Failed to read magnetometer: {:?} (failure {} of {})",
                e, *mag_consecutive_failures, MAX_CONSECUTIVE_FAILURES
            );
            if *mag_consecutive_failures >= MAX_CONSECUTIVE_FAILURES {
                info!("Max consecutive magnetometer failures reached - IMU task terminating");
                Err(SampleOutcome::Terminate)
            } else {
                Err(SampleOutcome::Continue)
            }
        }
    }
}

/// Prepare magnetometer data for AHRS fusion by reading the sensor, applying motor interference correction,
async fn prepare_magnetometer(
    sensor: &mut ImuSensor,
    current_calibration: Option<&flash_storage::ImuCalibration>,
    mag_consecutive_failures: &mut u32,
) -> Result<(Vector3<f32>, bool), SampleOutcome> {
    let mut mag_data = read_magnetometer(sensor, mag_consecutive_failures).await?;

    {
        let mut latest = LATEST_RAW_MAG.lock().await;
        *latest = Some(mag_data);
    }

    // Send raw magnetometer reading to drive task for calibration purposes
    // (before applying interference correction)
    drive::send_mag_measurement(mag_data).await;

    // Apply motor interference correction if calibrated
    if let Some(cal) = current_calibration {
        let (left_speed, right_speed) = motion::get_track_speeds_atomic();
        apply_motor_interference_correction(&mut mag_data, cal, left_speed, right_speed);
        apply_mag_calibration(&mut mag_data, cal);
    }

    {
        let mut latest = LATEST_CALIBRATED_MAG.lock().await;
        *latest = Some(mag_data);
    }

    // Check if magnetometer reading is reasonable
    // Typical Earth's magnetic field: 25-65 μT
    // Lower bound: 20.0 μT rejects clearly invalid readings (below Earth's minimum)
    // Upper bound: 200.0 μT allows for magnetic interference while rejecting sensor errors
    let mag_norm = mag_data.norm();
    let use_magnetometer = should_use_magnetometer(mag_norm);

    // Low-rate magnetometer diagnostics to debug yaw "sticking" / magnetic issues.
    // This is especially useful when validating hard/soft-iron calibration and interference correction.
    log_mag_diagnostics(&mag_data, use_magnetometer);

    if use_magnetometer {
        let denom = mag_norm.max(1e-3);
        mag_data /= denom;
    }

    Ok((mag_data, use_magnetometer))
}

/// Perform a single IMU sampling step: read sensors, apply corrections, update AHRS filter, and produce measurement
async fn sample_once(
    sensor: &mut ImuSensor,
    madgwick: &mut Madgwick<f32>,
    fusion_mode: AhrsFusionMode,
    dt_seconds: f32,
    current_calibration: Option<&flash_storage::ImuCalibration>,
    state: &mut SampleState,
) -> SampleOutcome {
    // Read accelerometer (in g-force)
    let accel_data = match read_accelerometer(sensor, &mut state.accel_consecutive_failures).await {
        Ok(data) => data,
        Err(outcome) => return outcome,
    };

    // Read gyroscope (in degrees/second) and apply bias correction
    let gyro_data = match read_gyroscope(sensor, &mut state.gyro_consecutive_failures).await {
        Ok(data) => data,
        Err(outcome) => return outcome,
    };

    // Send raw gyroscope reading to drive task for calibration purposes
    // (before bias correction is applied)
    let raw_gyro = Vector3::new(gyro_data.x, gyro_data.y, gyro_data.z);
    drive::send_gyro_measurement(raw_gyro).await;

    {
        let mut latest = LATEST_RAW_GYRO.lock().await;
        *latest = Some(raw_gyro);
    }

    // Send raw accelerometer reading to drive task for calibration purposes
    drive::send_accel_measurement(accel_data).await;

    {
        let mut latest = LATEST_RAW_ACCEL.lock().await;
        *latest = Some(accel_data);
    }

    let (gyro_bias_x, gyro_bias_y, gyro_bias_z) = current_calibration.map_or((0.0, 0.0, 0.0), |cal| {
        (cal.gyro_x_bias, cal.gyro_y_bias, cal.gyro_z_bias)
    });

    // Apply persisted bias + runtime trim (deg/s).
    let gyro_x = gyro_data.x - gyro_bias_x - state.runtime_gyro_bias_trim_dps.x;
    let gyro_y = gyro_data.y - gyro_bias_y - state.runtime_gyro_bias_trim_dps.y;
    let gyro_z = gyro_data.z - gyro_bias_z - state.runtime_gyro_bias_trim_dps.z;

    {
        let mut latest = LATEST_CALIBRATED_GYRO.lock().await;
        *latest = Some(Vector3::new(gyro_x, gyro_y, gyro_z));
    }

    let mut accel_corrected = accel_data;
    if let Some(cal) = current_calibration {
        accel_corrected.x -= cal.accel_x_bias;
        accel_corrected.y -= cal.accel_y_bias;
        accel_corrected.z -= cal.accel_z_bias;
    }

    {
        let mut latest = LATEST_CALIBRATED_ACCEL.lock().await;
        *latest = Some(accel_corrected);
    }

    // Normalize accel and gate it when linear acceleration is likely.
    let accel_norm = accel_corrected.norm();
    let accel_dir = if (0.85..=1.15).contains(&accel_norm) && accel_norm > 0.0 {
        let unit = accel_corrected / accel_norm;
        state.last_good_accel_dir = Some(unit);
        unit
    } else if let Some(last) = state.last_good_accel_dir.as_ref() {
        *last
    } else {
        accel_corrected / accel_norm.max(1e-3)
    };

    update_stillness_and_trim(
        accel_norm,
        raw_gyro,
        gyro_data,
        (gyro_bias_x, gyro_bias_y, gyro_bias_z),
        dt_seconds,
        state,
    );

    // Convert gyroscope from degrees/s to radians/s for AHRS.
    // Scale by measured dt to correct for loop timing drift.
    let dt_ratio = (dt_seconds / (1.0 / SAMPLE_RATE_HZ)).clamp(0.5, 2.0);
    let gyro_rad = Vector3::new(gyro_x.to_radians(), gyro_y.to_radians(), gyro_z.to_radians()) * dt_ratio;

    // NOTE: In 6-axis mode we must NOT read the magnetometer at all.
    // This avoids I2C/mag failures from stalling IMU output during turns.
    let result = match fusion_mode {
        AhrsFusionMode::Axis6 => {
            // 6-axis fusion: gyro + accel (yaw will drift over time)
            madgwick.update_imu(&gyro_rad, &accel_dir)
        }
        AhrsFusionMode::Axis9 => {
            let (mag_data, use_magnetometer) =
                match prepare_magnetometer(sensor, current_calibration, &mut state.mag_consecutive_failures).await {
                    Ok(data) => data,
                    Err(outcome) => return outcome,
                };

            if use_magnetometer {
                // 9-axis fusion: gyro + accel + mag (absolute heading, no yaw drift)
                madgwick.update(&gyro_rad, &accel_dir, &mag_data)
            } else {
                // 6-axis fallback when mag is not trustworthy
                madgwick.update_imu(&gyro_rad, &accel_dir)
            }
        }
    };

    if let Ok(quat) = result {
        // Convert quaternion to Euler angles
        let orientation = quaternion_to_euler(quat);
        let timestamp_ms = Instant::now().as_millis();

        {
            let mut latest = LATEST_ORIENTATION.lock().await;
            *latest = Some(orientation);
        }

        let imu_measurement = ImuMeasurement {
            orientation,
            timestamp_ms,
        };

        return SampleOutcome::Measurement(imu_measurement);
    }

    SampleOutcome::Continue
}

/// Main command loop for handling IMU reading state and processing commands
#[allow(clippy::too_many_lines)]
async fn run_imu_command_loop(sensor: &mut ImuSensor, madgwick: &mut Madgwick<f32>) {
    // Store current calibration data
    // Calibration will be loaded by orchestrator during initialization
    // and sent via LoadCalibration command
    let mut current_calibration: Option<flash_storage::ImuCalibration> = None;

    // Selectable AHRS fusion mode (default to DEFAULT_FUSION_MODE).
    // Testing can still switch modes explicitly while validating control flow.
    let mut fusion_mode = if DEFAULT_FUSION_MODE == AhrsFusionMode::Axis9 && !MAG_AVAILABLE.load(Ordering::Relaxed) {
        AhrsFusionMode::Axis6
    } else {
        DEFAULT_FUSION_MODE
    };

    'command: loop {
        // Wait for a command, consuming it
        match IMU_CONTROL.wait().await {
            ImuCommand::Start => {
                // Enter reading loop
                info!("Starting IMU reading");

                // Per-run sampling state (resets when IMU restarts)
                let mut sample_state = SampleState::default();

                // IMU loop timing diagnostics (rate-limited)
                #[cfg(feature = "telemetry_logs")]
                let mut last_loop_ts_ms: Option<u32> = None;
                #[cfg(feature = "telemetry_logs")]
                let mut last_loop_diag_ts_ms: u32 = 0;
                let mut last_sample_ms: Option<u64> = None;

                loop {
                    match select(IMU_CONTROL.wait(), Timer::after(SAMPLE_INTERVAL)).await {
                        Either::First(ImuCommand::Stop) => {
                            info!("IMU stopped. Waiting for next command.");
                            continue 'command;
                        }
                        Either::First(ImuCommand::SetFusionMode(mode)) => {
                            if mode == AhrsFusionMode::Axis9 && !MAG_AVAILABLE.load(Ordering::Relaxed) {
                                fusion_mode = AhrsFusionMode::Axis6;
                                info!("IMU AHRS fusion mode set to Axis6 (mag unavailable)");
                            } else {
                                fusion_mode = mode;
                                info!("IMU AHRS fusion mode set to {:?}", fusion_mode);
                            }
                        }
                        Either::First(_) => {}
                        Either::Second(()) => {
                            let now_ms = Instant::now().as_millis();
                            let dt_seconds = last_sample_ms.map_or(1.0 / SAMPLE_RATE_HZ, |last| {
                                let dt_ms = now_ms.saturating_sub(last);
                                let dt_ms = dt_ms.min(u64::from(u32::MAX));
                                #[allow(clippy::cast_precision_loss)]
                                {
                                    (dt_ms as f32 / 1000.0).clamp(0.005, 0.1)
                                }
                            });
                            last_sample_ms = Some(now_ms);

                            match sample_once(
                                sensor,
                                madgwick,
                                fusion_mode,
                                dt_seconds,
                                current_calibration.as_ref(),
                                &mut sample_state,
                            )
                            .await
                            {
                                SampleOutcome::Measurement(imu_measurement) => {
                                    // IMU loop timing diagnostics (rate-limited):
                                    // - Logs dt between produced IMU measurements
                                    // - Logs current fusion mode, so we can confirm Axis6 is active in testing
                                    #[cfg(feature = "telemetry_logs")]
                                    {
                                        #[allow(clippy::cast_possible_truncation)]
                                        let timestamp_ms = imu_measurement.timestamp_ms as u32;
                                        if timestamp_ms.wrapping_sub(last_loop_diag_ts_ms)
                                            >= IMU_LOOP_DIAG_LOG_INTERVAL_MS
                                        {
                                            let dt_ms = last_loop_ts_ms.map(|last| timestamp_ms.wrapping_sub(last));
                                            let dt_ratio = (dt_seconds / (1.0 / SAMPLE_RATE_HZ)).clamp(0.5, 2.0);
                                            defmt::info!(
                                                "IMU diag: mode={:?} dt_ms={:?} dt_s={=f32} dt_ratio={=f32}",
                                                fusion_mode,
                                                dt_ms,
                                                dt_seconds,
                                                dt_ratio
                                            );
                                            last_loop_diag_ts_ms = timestamp_ms;
                                        }
                                        last_loop_ts_ms = Some(timestamp_ms);
                                    }

                                    raise_event(Events::ImuMeasurementTaken(imu_measurement)).await;
                                }
                                SampleOutcome::Continue => {}
                                SampleOutcome::Terminate => {
                                    return;
                                }
                            }
                        }
                    }
                }
            }
            ImuCommand::Stop => {
                info!("IMU stopped. Waiting for next command.");
            }
            ImuCommand::LoadCalibration(cal) => {
                current_calibration = Some(cal);
                info!("IMU calibration loaded and applied");
            }
            ImuCommand::SetFusionMode(mode) => {
                if mode == AhrsFusionMode::Axis9 && !MAG_AVAILABLE.load(Ordering::Relaxed) {
                    fusion_mode = AhrsFusionMode::Axis6;
                    info!("IMU AHRS fusion mode set to Axis6 (mag unavailable)");
                } else {
                    fusion_mode = mode;
                    info!("IMU AHRS fusion mode set to {:?}", fusion_mode);
                }
            }
        }
    }
}

/// Embassy task that handles IMU measurements with 9-axis AHRS fusion
#[embassy_executor::task]
pub async fn inertial_measurement_read(i2c_bus: &'static I2cBusShared) {
    let mut sensor = init_imu_sensor(i2c_bus).await;

    if !configure_imu_sensor(&mut sensor).await {
        return;
    }

    let mut madgwick = init_madgwick();

    info!("Starting AHRS processing at {} Hz", SAMPLE_RATE_HZ);

    run_imu_command_loop(&mut sensor, &mut madgwick).await;
}
