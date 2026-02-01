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
//! - Vibration filtering is critical (DLPF settings)
//! - Moderate beta value balances convergence speed and noise rejection
//! - Magnetometer provides heading reference but may need calibration in magnetic environments
//!
//! # Usage
//!
//! ```rust
//! // Start IMU readings
//! imu_read::start_imu_readings();
//!
//! // Stop readings when done
//! imu_read::stop_imu_readings();
//! ```

use core::f32::consts::PI;

use ahrs::{Ahrs, Madgwick};
use defmt::{info, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
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
        state::SYSTEM_STATE,
    },
    task::{drive, flash_storage},
};

// Sampling configuration
// We run the IMU + AHRS fusion at 50Hz to match a commonly supported magnetometer continuous mode,
// while still keeping CPU + I2C load reasonable.
const SAMPLE_INTERVAL: Duration = Duration::from_millis(20); // 50Hz sampling

// Magnetometer diagnostics logging
// Log at low rate to avoid spamming RTT/defmt while still giving enough signal to debug yaw issues.
#[cfg(feature = "telemetry_logs")]
const MAG_DIAG_LOG_INTERVAL_MS: u32 = 500;

// IMU loop diagnostics logging
// Used to debug timing/cadence differences with/without debugger attached.
#[cfg(feature = "telemetry_logs")]
const IMU_LOOP_DIAG_LOG_INTERVAL_MS: u32 = 500;

/// Complete IMU measurement data
#[derive(Debug, Clone, Copy)]
pub struct ImuMeasurement {
    /// Orientation in 3D space
    pub orientation: Orientation,
    /// Timestamp of measurement in milliseconds
    pub timestamp_ms: u32,
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
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum AhrsFusionMode {
    /// 6-axis fusion (gyro + accel). Yaw is relative and will drift, but is robust
    /// in magnetically noisy environments and ideal for short precise turns.
    Axis6,
    /// 9-axis fusion (gyro + accel + mag). Yaw is stabilized to magnetic north (after calibration)
    /// and does not drift, but can be disturbed by motor/steel interference.
    Axis9,
}

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

/// Start continuous IMU readings
pub fn start_imu_readings() {
    IMU_CONTROL.signal(ImuCommand::Start);
}

/// Select AHRS fusion mode (6-axis vs 9-axis)
///
/// - `Axis6`: uses gyro+accel (yaw drifts, but robust for turn control)
/// - `Axis9`: uses gyro+accel+mag when magnetometer is considered valid
pub fn set_ahrs_fusion_mode(mode: AhrsFusionMode) {
    IMU_CONTROL.signal(ImuCommand::SetFusionMode(mode));
}

/// Stop IMU readings
pub fn stop_imu_readings() {
    IMU_CONTROL.signal(ImuCommand::Stop);
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
        roll: (roll * 180.0 / PI * 10.0).round() / 10.0,
        pitch: (pitch * 180.0 / PI * 10.0).round() / 10.0,
        yaw: (yaw * 180.0 / PI * 10.0).round() / 10.0,
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
    let left_abs = left_speed.abs() as f32;
    let right_abs = right_speed.abs() as f32;

    // Determine dominant pattern
    if left_abs == 0.0 && right_abs == 0.0 {
        // Motors off - no correction needed
        (0.0, 0.0, 0.0)
    } else if (left_abs - right_abs).abs() < 10.0 {
        // Both tracks approximately equal - use all_motors pattern
        let avg_speed = (left_abs + right_abs) / 2.0;
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

/// Embassy task that handles IMU measurements with 9-axis AHRS fusion
#[embassy_executor::task]
pub async fn inertial_measurement_read(i2c_bus: &'static I2cBusShared) {
    // On battery power-up, the IMU can still be in POR / internal regulator startup when
    // this task begins. Under a debugger, the extra attach/flash/reset time often masks
    // this. Add a deterministic startup delay and retry init for robustness.
    const IMU_BOOT_DELAY_MS: u64 = 200;
    const IMU_INIT_RETRY_DELAY_MS: u64 = 50;
    const IMU_INIT_MAX_ATTEMPTS: u32 = 60; // ~3s total

    info!("Waiting {}ms for IMU power-up...", IMU_BOOT_DELAY_MS);
    Timer::after(Duration::from_millis(IMU_BOOT_DELAY_MS)).await;

    let mut delay = Delay;

    // Initialize ICM20948 with retries
    let mut attempt: u32 = 0;
    let mut sensor = loop {
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
        return;
    }
    info!("Accelerometer: ±4g, 111Hz DLPF, ~49Hz sample rate");

    // Configure gyroscope for tracked robot:
    // ±500°/s range: Captures fast rotations (tracks can spin quickly)
    // 51 Hz DLPF: Good vibration filtering for brushed motors
    // ~50 Hz internal sample rate: Matches the 50 Hz readout loop (reduced CPU + I2C load)
    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps500,
        dlpf: GyroDlpf::Hz51,
        dlpf_enable: true,
        sample_rate_div: 22, // (1125 Hz / (1 + 22)) = ~48.9 Hz
    };

    if let Err(e) = sensor.configure_gyroscope(gyro_config).await {
        info!("Failed to configure gyroscope: {:?}", e);
        info!("Sensor configuration failed - IMU task terminating");
        return;
    }
    info!("Gyroscope: ±500°/s, 51Hz DLPF, ~49Hz sample rate");

    // Configure magnetometer for absolute heading:
    // Continuous 50Hz mode for smooth compass data
    // Provides absolute yaw reference (no drift)
    let mag_config = MagConfig {
        mode: MagMode::Continuous50Hz,
    };

    if let Err(e) = sensor.init_magnetometer(mag_config, &mut delay).await {
        info!("Failed to initialize magnetometer: {:?}", e);
        info!("Sensor configuration failed - IMU task terminating");
        return;
    }
    info!("Magnetometer: Continuous 50Hz mode");

    Timer::after(Duration::from_millis(100)).await;
    info!("All sensors configured!");

    // Initialize Madgwick filter for AHRS sensor fusion
    //
    // We align the filter timestep to the sampling interval. 50Hz is a good middle ground:
    // responsive enough for precise turns, but lower noise + load than 100Hz.
    //
    // The Madgwick filter fuses:
    // - Gyroscope: Fast response, accurate short-term, but drifts
    // - Accelerometer: Stable long-term gravity reference, but noisy
    // - Magnetometer: Absolute heading reference, prevents yaw drift
    const SAMPLE_RATE_HZ: f32 = 50.0;
    const BETA: f32 = 0.15; // Balanced for indoor tracked robot with motor vibrations
    let mut madgwick = Madgwick::new(1.0 / SAMPLE_RATE_HZ, BETA);

    info!("Madgwick AHRS filter initialized:");
    info!("  Sample rate: {} Hz", SAMPLE_RATE_HZ);
    info!("  Beta: {} (balanced for tracked robot)", BETA);
    info!("  Mode: 9-axis fusion (gyro + accel + mag)");

    // TODO: Load gyroscope bias calibration values from flash memory
    // For now, assume zero bias (will be implemented as dedicated calibration feature)
    let gyro_bias_x = 0.0f32;
    let gyro_bias_y = 0.0f32;
    let gyro_bias_z = 0.0f32;

    info!("Starting AHRS processing at {} Hz", SAMPLE_RATE_HZ);

    // Store current calibration data
    // Calibration will be loaded by orchestrator during initialization
    // and sent via LoadCalibration command
    let mut current_calibration: Option<flash_storage::ImuCalibration> = None;

    // Selectable AHRS fusion mode (default to 9-axis for normal operation).
    // Testing can switch this to Axis6 to avoid magnetometer issues while validating control flow.
    let mut fusion_mode = AhrsFusionMode::Axis9;

    'command: loop {
        // Wait for a command, consuming it
        match IMU_CONTROL.wait().await {
            ImuCommand::Start => {
                // Enter reading loop
                info!("Starting IMU reading");

                // Track consecutive sensor read failures per sensor
                let mut accel_consecutive_failures = 0u32;
                let mut gyro_consecutive_failures = 0u32;
                let mut mag_consecutive_failures = 0u32;
                const MAX_CONSECUTIVE_FAILURES: u32 = 10;

                // IMU loop timing diagnostics (rate-limited)
                #[cfg(feature = "telemetry_logs")]
                let mut last_loop_ts_ms: Option<u32> = None;
                #[cfg(feature = "telemetry_logs")]
                let mut last_loop_diag_ts_ms: u32 = 0;

                loop {
                    match select(IMU_CONTROL.wait(), Timer::after(SAMPLE_INTERVAL)).await {
                        Either::First(ImuCommand::Stop) => {
                            info!("IMU stopped. Waiting for next command.");
                            continue 'command;
                        }
                        Either::First(ImuCommand::SetFusionMode(mode)) => {
                            fusion_mode = mode;
                            info!("IMU AHRS fusion mode set to {:?}", fusion_mode);
                            continue;
                        }
                        Either::First(_) => continue,
                        Either::Second(_) => {
                            // Read accelerometer (in g-force)
                            let accel_data = match sensor.read_accelerometer().await {
                                Ok(data) => {
                                    accel_consecutive_failures = 0;
                                    Vector3::new(data.x, data.y, data.z)
                                }
                                Err(e) => {
                                    accel_consecutive_failures += 1;
                                    info!(
                                        "Failed to read accelerometer: {:?} (failure {} of {})",
                                        e, accel_consecutive_failures, MAX_CONSECUTIVE_FAILURES
                                    );
                                    if accel_consecutive_failures >= MAX_CONSECUTIVE_FAILURES {
                                        info!("Max consecutive accelerometer failures reached - IMU task terminating");
                                        return;
                                    }
                                    continue;
                                }
                            };

                            // Read gyroscope (in degrees/second) and apply bias correction
                            let gyro_data = match sensor.read_gyroscope().await {
                                Ok(data) => {
                                    gyro_consecutive_failures = 0;
                                    data
                                }
                                Err(e) => {
                                    gyro_consecutive_failures += 1;
                                    info!(
                                        "Failed to read gyroscope: {:?} (failure {} of {})",
                                        e, gyro_consecutive_failures, MAX_CONSECUTIVE_FAILURES
                                    );
                                    if gyro_consecutive_failures >= MAX_CONSECUTIVE_FAILURES {
                                        info!("Max consecutive gyroscope failures reached - IMU task terminating");
                                        return;
                                    }
                                    continue;
                                }
                            };

                            let gyro_x = gyro_data.x - gyro_bias_x;
                            let gyro_y = gyro_data.y - gyro_bias_y;
                            let gyro_z = gyro_data.z - gyro_bias_z;

                            // Send raw gyroscope reading to drive task for calibration purposes
                            // (before bias correction is applied)
                            drive::send_gyro_measurement(Vector3::new(gyro_data.x, gyro_data.y, gyro_data.z)).await;

                            // Send raw accelerometer reading to drive task for calibration purposes
                            drive::send_accel_measurement(accel_data).await;

                            // Convert gyroscope from degrees/s to radians/s for AHRS
                            let gyro_rad = Vector3::new(gyro_x * PI / 180.0, gyro_y * PI / 180.0, gyro_z * PI / 180.0);

                            // NOTE: In 6-axis mode we must NOT read the magnetometer at all.
                            // This avoids I2C/mag failures from stalling IMU output during turns.
                            let result = match fusion_mode {
                                AhrsFusionMode::Axis6 => {
                                    // 6-axis fusion: gyro + accel (yaw will drift over time)
                                    madgwick.update_imu(&gyro_rad, &accel_data)
                                }
                                AhrsFusionMode::Axis9 => {
                                    // Read magnetometer (in μT)
                                    let mut mag_data = match sensor.read_magnetometer().await {
                                        Ok(data) => {
                                            mag_consecutive_failures = 0;
                                            Vector3::new(data.x, data.y, data.z)
                                        }
                                        Err(e) => {
                                            mag_consecutive_failures += 1;
                                            info!(
                                                "Failed to read magnetometer: {:?} (failure {} of {})",
                                                e, mag_consecutive_failures, MAX_CONSECUTIVE_FAILURES
                                            );
                                            if mag_consecutive_failures >= MAX_CONSECUTIVE_FAILURES {
                                                info!(
                                                    "Max consecutive magnetometer failures reached - IMU task terminating"
                                                );
                                                return;
                                            }
                                            continue;
                                        }
                                    };

                                    // Send raw magnetometer reading to drive task for calibration purposes
                                    // (before applying interference correction)
                                    drive::send_mag_measurement(mag_data).await;

                                    // Apply motor interference correction if calibrated
                                    if let Some(cal) = &current_calibration {
                                        let state = SYSTEM_STATE.lock().await;
                                        apply_motor_interference_correction(
                                            &mut mag_data,
                                            cal,
                                            state.left_track_speed,
                                            state.right_track_speed,
                                        );
                                    }

                                    // Check if magnetometer reading is reasonable
                                    // Typical Earth's magnetic field: 25-65 μT
                                    // Lower bound: 20.0 μT rejects clearly invalid readings (below Earth's minimum)
                                    // Upper bound: 200.0 μT allows for magnetic interference while rejecting sensor errors
                                    let mag_magnitude = mag_data.norm();
                                    let use_magnetometer = mag_magnitude > 20.0 && mag_magnitude < 200.0;

                                    // Low-rate magnetometer diagnostics to debug yaw "sticking" / magnetic issues.
                                    // This is especially useful before hard/soft-iron calibration is implemented.
                                    #[cfg(feature = "telemetry_logs")]
                                    {
                                        let timestamp_ms = Instant::now().as_millis() as u32;
                                        if timestamp_ms % MAG_DIAG_LOG_INTERVAL_MS < 25 {
                                            info!(
                                                "MAG diag: mag=({=f32},{=f32},{=f32}) uT | |mag|={=f32} uT | use_mag={}",
                                                mag_data.x, mag_data.y, mag_data.z, mag_magnitude, use_magnetometer
                                            );
                                            if !use_magnetometer {
                                                warn!(
                                                    "MAG diag: rejecting magnetometer due to magnitude out of bounds"
                                                );
                                            }
                                        }
                                    }

                                    if use_magnetometer {
                                        // 9-axis fusion: gyro + accel + mag (absolute heading, no yaw drift)
                                        madgwick.update(&gyro_rad, &accel_data, &mag_data)
                                    } else {
                                        // 6-axis fallback when mag is not trustworthy
                                        madgwick.update_imu(&gyro_rad, &accel_data)
                                    }
                                }
                            };

                            if let Ok(quat) = result {
                                // Convert quaternion to Euler angles
                                let orientation = quaternion_to_euler(quat);
                                let timestamp_ms = Instant::now().as_millis() as u32;

                                // IMU loop timing diagnostics (rate-limited):
                                // - Logs dt between produced IMU measurements
                                // - Logs current fusion mode, so we can confirm Axis6 is active in testing
                                #[cfg(feature = "telemetry_logs")]
                                {
                                    if timestamp_ms.wrapping_sub(last_loop_diag_ts_ms) >= IMU_LOOP_DIAG_LOG_INTERVAL_MS
                                    {
                                        let dt_ms = last_loop_ts_ms.map(|last| timestamp_ms.wrapping_sub(last));
                                        defmt::info!("IMU diag: mode={:?} dt_ms={:?}", fusion_mode, dt_ms);
                                        last_loop_diag_ts_ms = timestamp_ms;
                                    }
                                    last_loop_ts_ms = Some(timestamp_ms);
                                }

                                let imu_measurement = ImuMeasurement {
                                    orientation,
                                    timestamp_ms,
                                };

                                raise_event(Events::ImuMeasurementTaken(imu_measurement)).await;
                            }
                        }
                    }
                }
            }
            ImuCommand::Stop => {
                info!("IMU stopped. Waiting for next command.");
                // Return to waiting for next command
                continue 'command;
            }
            ImuCommand::LoadCalibration(cal) => {
                current_calibration = Some(cal);
                info!("IMU calibration loaded and applied");
                // Stay in current state (continue waiting for commands)
                continue 'command;
            }
            ImuCommand::SetFusionMode(mode) => {
                fusion_mode = mode;
                info!("IMU AHRS fusion mode set to {:?}", fusion_mode);
                continue 'command;
            }
        }
    }
}
