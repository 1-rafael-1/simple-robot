//! IMU (Inertial Measurement Unit) reading functionality using the ICM-20948 DMP.
//!
//! This module drives the ICM-20948's on-chip Digital Motion Processor (DMP) to
//! produce quaternion-derived orientation at a configurable output rate.
//! DMP packets are polled from the FIFO on a fixed timer — no INT pin is required.
//!
//! # Architecture
//!
//! The module operates in two states:
//! 1. Standby — not reading
//! 2. Active — DMP FIFO is polled every `POLL_INTERVAL`; each packet that contains
//!    a quaternion raises an `ImuMeasurementTaken` event and updates shared state.
//!
//! # Fusion modes
//!
//! - `Axis6`: 6-axis DMP fusion (gyro + accel). Yaw is relative and drifts, but is
//!   magnetically robust and well-suited to short precise turns. This is the default.
//! - `Axis9`: 9-axis DMP fusion (gyro + accel + mag). Yaw is stabilized to magnetic
//!   north. Falls back to `Axis6` automatically if the magnetometer could not be
//!   initialized at start-up.
//!
//! # Mode switching
//!
//! Runtime mode switches signal the task via `set_ahrs_fusion_mode`. The running DMP
//! is stopped (`dmp_enable(false)`), reconfigured, and restarted without reloading
//! firmware. The magnetometer is always initialized at start-up so that switching to
//! `Axis9` later is possible without a full re-init.
//!
//! # Calibration
//!
//! Magnetometer hard/soft-iron and motor-interference correction are applied in
//! software to the raw mag readings exposed via `LATEST_CALIBRATED_MAG`. The DMP's
//! own internal calibration engines handle gyroscope and accelerometer bias correction
//! automatically — no host-injected bias values are needed for those axes.
//!
//! # Orientation reference frame
//!
//! Euler angles are derived directly from the DMP quaternion. In `Axis9` mode yaw is
//! an absolute compass heading; in `Axis6` mode yaw is relative to start-up heading.
//!
//! # Note on telemetry
//!
//! High-frequency diagnostics are gated behind the `telemetry_logs` feature to keep
//! production builds free of formatting overhead when no log consumer is attached.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{info, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Delay, Duration, Instant, Timer};
use icm20948::{I2cInterface, Icm20948Driver, dmp::DmpConfig};
use nalgebra::Vector3;

use crate::{
    I2cBusShared,
    system::{
        event::{Events, raise_event},
        state::motion,
    },
    task::{drive, io::flash_storage},
};

// ── Type alias ────────────────────────────────────────────────────────────────

/// ICM-20948 driver instance using the project's shared async I2C bus.
type ImuSensor = Icm20948Driver<
    I2cInterface<
        I2cDevice<
            'static,
            CriticalSectionRawMutex,
            embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>,
        >,
    >,
>;

// ── Timing constants ──────────────────────────────────────────────────────────

/// DMP output rate in Hz. The DMP firmware emits packets at this rate.
const DMP_SAMPLE_RATE_HZ: u16 = 100;

/// FIFO poll interval. Polling faster than the DMP rate ensures packets are
/// drained promptly; `10 ms` (100 Hz) matches `DMP_SAMPLE_RATE_HZ`.
const POLL_INTERVAL: Duration = Duration::from_millis(10);

/// Initial delay after power-up before starting IMU initialisation.
const IMU_BOOT_DELAY_MS: u64 = 200;

/// Delay between consecutive initialisation attempts.
const IMU_INIT_RETRY_DELAY_MS: u64 = 50;

/// Maximum number of initialisation attempts before the task halts permanently.
const IMU_INIT_MAX_ATTEMPTS: u32 = 60; // ≈ 3 s total

/// Maximum consecutive FIFO-read failures before the IMU task terminates.
const MAX_FIFO_FAILURES: u32 = 10;

// ── Sensor conversion constants ───────────────────────────────────────────────

/// Accelerometer LSB → g for the DMP's ±4 g internal full-scale (8192 LSB/g).
const ACCEL_SCALE_G: f32 = 1.0 / 8192.0;

/// Gyroscope LSB → deg/s for ±2000 dps full-scale (16.384 LSB/dps).
/// The icm20948-rs DMP initialisation explicitly programs `GYRO_FS_SEL = 0b11`
/// (±2000 dps) in its DMP-enable sequence (see `device.rs`, `dmp_enable`).
/// Exact conversion: 32768 LSB / 2000 dps = 16.384 LSB/dps.
const GYRO_SCALE_DPS: f32 = 1.0 / 16.384;

/// AK09916 magnetometer sensitivity: 0.15 µT per LSB.
const MAG_SCALE_UT: f32 = 0.15;

// ── Telemetry ─────────────────────────────────────────────────────────────────

/// Rate-limiting interval for IMU loop diagnostics when `telemetry_logs` is on.
#[cfg(feature = "telemetry_logs")]
const IMU_LOOP_DIAG_LOG_INTERVAL_MS: u32 = 500;

// ── Public types ──────────────────────────────────────────────────────────────

/// Complete IMU measurement data raised as an event.
#[derive(Debug, Clone, Copy)]
pub struct ImuMeasurement {
    /// Sensor orientation derived from the DMP quaternion.
    pub orientation: Orientation,
    /// Timestamp of this measurement in milliseconds since boot.
    pub timestamp_ms: u64,
}

/// 3D orientation expressed as Euler angles derived from the DMP quaternion.
///
/// In `Axis9` mode `yaw` is an absolute compass heading.
/// In `Axis6` mode `yaw` is relative to the heading at start-up and drifts.
#[derive(Debug, Clone, Copy)]
pub struct Orientation {
    /// Heading / compass direction in degrees.
    pub yaw: f32,
    /// Forward / backward tilt relative to gravity in degrees.
    pub pitch: f32,
    /// Left / right tilt relative to gravity in degrees.
    pub roll: f32,
}

/// DMP fusion-mode selection.
#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
pub enum AhrsFusionMode {
    /// 6-axis fusion (gyro + accel). Yaw is relative; no magnetometer dependency.
    Axis6,
    /// 9-axis fusion (gyro + accel + mag). Yaw is absolute; degrades gracefully to
    /// `Axis6` if the magnetometer was unavailable at start-up.
    Axis9,
}

/// System-wide default fusion mode used at start-up.
pub const DEFAULT_FUSION_MODE: AhrsFusionMode = AhrsFusionMode::Axis6;

// ── Internal command type ─────────────────────────────────────────────────────

/// Commands delivered to the IMU task via [`IMU_CONTROL`].
enum ImuCommand {
    /// Begin reading and publishing measurements.
    Start,
    /// Stop reading; enter standby.
    Stop,
    /// Replace the active calibration data (mag hard/soft-iron + interference).
    LoadCalibration(flash_storage::ImuCalibration),
    /// Switch DMP fusion mode at runtime.
    SetFusionMode(AhrsFusionMode),
}

// ── Shared statics ────────────────────────────────────────────────────────────

/// Control signal carrying [`ImuCommand`]s into the IMU task.
static IMU_CONTROL: Signal<CriticalSectionRawMutex, ImuCommand> = Signal::new();

/// `true` when `dmp_init_magnetometer` succeeded at start-up; gates `Axis9` usage.
static MAG_AVAILABLE: AtomicBool = AtomicBool::new(false);

/// Latest DMP-derived orientation snapshot (updated on every valid packet).
static LATEST_ORIENTATION: Mutex<CriticalSectionRawMutex, Option<Orientation>> = Mutex::new(None);

/// Latest DMP host-calibrated accelerometer reading (g).
static LATEST_CALIBRATED_ACCEL: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest DMP-calibrated gyroscope reading (deg/s, DMP internal bias subtracted).
static LATEST_CALIBRATED_GYRO: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest magnetometer reading with hard/soft-iron + motor-interference correction (µT).
static LATEST_CALIBRATED_MAG: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest raw accelerometer reading before DMP correction (g).
static LATEST_RAW_ACCEL: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest raw gyroscope reading before DMP correction (deg/s).
static LATEST_RAW_GYRO: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest raw magnetometer reading before any host correction (µT).
static LATEST_RAW_MAG: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

// ── Public API ────────────────────────────────────────────────────────────────

/// Signal the IMU task to start continuous DMP measurements.
pub fn start_imu_readings() {
    IMU_CONTROL.signal(ImuCommand::Start);
}

/// Signal the IMU task to stop measurements and enter standby.
pub fn stop_imu_readings() {
    IMU_CONTROL.signal(ImuCommand::Stop);
}

/// Select DMP fusion mode.
///
/// `Axis9` is silently downgraded to `Axis6` if the magnetometer was not
/// available at start-up, preserving the same behaviour as the previous
/// software-fusion implementation.
pub fn set_ahrs_fusion_mode(mode: AhrsFusionMode) {
    IMU_CONTROL.signal(ImuCommand::SetFusionMode(mode));
}

/// Load magnetometer calibration data (hard/soft-iron + motor-interference).
///
/// Applied to `LATEST_CALIBRATED_MAG` on every subsequent DMP packet.
pub fn load_imu_calibration(calibration: flash_storage::ImuCalibration) {
    IMU_CONTROL.signal(ImuCommand::LoadCalibration(calibration));
}

/// Return the latest DMP-derived orientation.
pub async fn get_latest_orientation() -> Option<Orientation> {
    *LATEST_ORIENTATION.lock().await
}

/// Return the latest DMP host-calibrated accelerometer reading (g).
pub async fn get_latest_calibrated_accel() -> Option<Vector3<f32>> {
    *LATEST_CALIBRATED_ACCEL.lock().await
}

/// Return the latest DMP-calibrated gyroscope reading (deg/s).
pub async fn get_latest_calibrated_gyro() -> Option<Vector3<f32>> {
    *LATEST_CALIBRATED_GYRO.lock().await
}

/// Return the latest host-corrected magnetometer reading (µT).
pub async fn get_latest_calibrated_mag() -> Option<Vector3<f32>> {
    *LATEST_CALIBRATED_MAG.lock().await
}

/// Return the latest raw accelerometer reading (g, before DMP correction).
pub async fn get_latest_raw_accel() -> Option<Vector3<f32>> {
    *LATEST_RAW_ACCEL.lock().await
}

/// Return the latest raw gyroscope reading (deg/s, before DMP correction).
pub async fn get_latest_raw_gyro() -> Option<Vector3<f32>> {
    *LATEST_RAW_GYRO.lock().await
}

/// Return the latest raw magnetometer reading (µT, before any correction).
pub async fn get_latest_raw_mag() -> Option<Vector3<f32>> {
    *LATEST_RAW_MAG.lock().await
}

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Convert a DMP quaternion to [`Orientation`] (Euler angles in degrees).
///
/// The ICM-20948 is mounted with its sensor X-axis aligned with the robot's
/// lateral (side-to-side) axis and its sensor Y-axis aligned with the robot's
/// forward axis.  The DMP therefore emits:
///   - `roll`  (rotation around sensor X) → robot **pitch** (nose-up/down)
///   - `pitch` (rotation around sensor Y) → robot **roll**  (lean left/right)
///   - `yaw`   (rotation around sensor Z) → robot **yaw**   (heading) — unchanged
fn dmp_quat_to_orientation(quat: &icm20948::dmp::Quaternion) -> Orientation {
    let angles = quat.to_euler_angles();
    let (roll_deg, pitch_deg, yaw_deg) = angles.to_degrees();
    Orientation {
        // Swap sensor roll↔pitch to match the robot body frame.
        roll: pitch_deg,
        pitch: roll_deg,
        yaw: yaw_deg,
    }
}

/// Interpolate motor-interference correction for a single track.
fn interpolate_single_track(calibration: &flash_storage::ImuCalibration, speed: f32, is_left: bool) -> (f32, f32, f32) {
    let idx = if is_left { 1 } else { 2 };

    if speed <= 50.0 {
        let factor = speed / 50.0;
        (
            calibration.mag_x_interference_50[idx] * factor,
            calibration.mag_y_interference_50[idx] * factor,
            calibration.mag_z_interference_50[idx] * factor,
        )
    } else {
        let factor = (speed - 50.0) / 50.0;
        (
            calibration.mag_x_interference_50[idx]
                + (calibration.mag_x_interference_100[idx] - calibration.mag_x_interference_50[idx]) * factor,
            calibration.mag_y_interference_50[idx]
                + (calibration.mag_y_interference_100[idx] - calibration.mag_y_interference_50[idx]) * factor,
            calibration.mag_z_interference_50[idx]
                + (calibration.mag_z_interference_100[idx] - calibration.mag_z_interference_50[idx]) * factor,
        )
    }
}

/// Interpolate motor-interference correction when both tracks run at equal speed.
fn interpolate_equal_motors(calibration: &flash_storage::ImuCalibration, speed: f32) -> (f32, f32, f32) {
    const ALL: usize = 0;

    if speed <= 50.0 {
        let factor = speed / 50.0;
        (
            calibration.mag_x_interference_50[ALL] * factor,
            calibration.mag_y_interference_50[ALL] * factor,
            calibration.mag_z_interference_50[ALL] * factor,
        )
    } else {
        let factor = (speed - 50.0) / 50.0;
        (
            calibration.mag_x_interference_50[ALL]
                + (calibration.mag_x_interference_100[ALL] - calibration.mag_x_interference_50[ALL]) * factor,
            calibration.mag_y_interference_50[ALL]
                + (calibration.mag_y_interference_100[ALL] - calibration.mag_y_interference_50[ALL]) * factor,
            calibration.mag_z_interference_50[ALL]
                + (calibration.mag_z_interference_100[ALL] - calibration.mag_z_interference_50[ALL]) * factor,
        )
    }
}

/// Compute the total motor-interference correction vector given current track speeds.
fn interpolate_interference(
    calibration: &flash_storage::ImuCalibration,
    left_speed: i8,
    right_speed: i8,
) -> (f32, f32, f32) {
    let left_abs = f32::from(left_speed.abs());
    let right_abs = f32::from(right_speed.abs());

    if left_abs == 0.0 && right_abs == 0.0 {
        (0.0, 0.0, 0.0)
    } else if (left_abs - right_abs).abs() < 10.0 {
        interpolate_equal_motors(calibration, f32::midpoint(left_abs, right_abs))
    } else {
        let (lx, ly, lz) = interpolate_single_track(calibration, left_abs, true);
        let (rx, ry, rz) = interpolate_single_track(calibration, right_abs, false);
        let total = left_abs + right_abs;
        let lw = left_abs / total;
        let rw = right_abs / total;
        (lx * lw + rx * rw, ly * lw + ry * rw, lz * lw + rz * rw)
    }
}

/// Update all `LATEST_*` statics and drive-task sensor channels from a DMP packet.
async fn update_statics_from_dmp(packet: &icm20948::dmp::DmpData, calibration: Option<&flash_storage::ImuCalibration>) {
    // Raw accelerometer -------------------------------------------------------
    if let Some((ax, ay, az)) = packet.raw_accel {
        let v = Vector3::new(
            f32::from(ax) * ACCEL_SCALE_G,
            f32::from(ay) * ACCEL_SCALE_G,
            f32::from(az) * ACCEL_SCALE_G,
        );
        *LATEST_RAW_ACCEL.lock().await = Some(v);
    }

    // Raw gyroscope -----------------------------------------------------------
    if let Some((gx, gy, gz)) = packet.raw_gyro {
        let v = Vector3::new(
            f32::from(gx) * GYRO_SCALE_DPS,
            f32::from(gy) * GYRO_SCALE_DPS,
            f32::from(gz) * GYRO_SCALE_DPS,
        );
        *LATEST_RAW_GYRO.lock().await = Some(v);
    }

    // Host-calibrated accelerometer (DMP applies host offsets when configured) -
    if let Some((ax, ay, az)) = packet.host_calibrated_accel {
        let v = Vector3::new(
            f32::from(ax) * ACCEL_SCALE_G,
            f32::from(ay) * ACCEL_SCALE_G,
            f32::from(az) * ACCEL_SCALE_G,
        );
        *LATEST_CALIBRATED_ACCEL.lock().await = Some(v);
    }

    // DMP-calibrated gyroscope (bias subtracted by DMP calibration engine) ----
    if let Some((gx, gy, gz)) = packet.calibrated_gyro {
        let v = Vector3::new(
            f32::from(gx) * GYRO_SCALE_DPS,
            f32::from(gy) * GYRO_SCALE_DPS,
            f32::from(gz) * GYRO_SCALE_DPS,
        );
        *LATEST_CALIBRATED_GYRO.lock().await = Some(v);
    }

    // Raw magnetometer (Axis9 mode only) --------------------------------------
    if let Some((mx, my, mz)) = packet.raw_mag {
        let v = Vector3::new(
            f32::from(mx) * MAG_SCALE_UT,
            f32::from(my) * MAG_SCALE_UT,
            f32::from(mz) * MAG_SCALE_UT,
        );
        *LATEST_RAW_MAG.lock().await = Some(v);

        // Forward raw mag to the drive task for the magnetometer calibration procedure.
        drive::send_mag_measurement(v).await;

        // Apply host-side hard/soft-iron + motor-interference correction if calibrated.
        let mut mag_cal = v;
        if let Some(cal) = calibration {
            let (left_speed, right_speed) = motion::get_track_speeds_atomic();
            let (ix, iy, iz) = interpolate_interference(cal, left_speed, right_speed);
            mag_cal.x = (v.x - ix - cal.mag_x_bias) * cal.mag_x_scale;
            mag_cal.y = (v.y - iy - cal.mag_y_bias) * cal.mag_y_scale;
            mag_cal.z = (v.z - iz - cal.mag_z_bias) * cal.mag_z_scale;
        }
        *LATEST_CALIBRATED_MAG.lock().await = Some(mag_cal);
    }
}

/// Power-up, detect, and fully initialise the ICM-20948, retrying on failure.
async fn init_imu_sensor(i2c_bus: &'static I2cBusShared) -> ImuSensor {
    info!("Waiting {}ms for IMU power-up...", IMU_BOOT_DELAY_MS);
    Timer::after(Duration::from_millis(IMU_BOOT_DELAY_MS)).await;

    let mut delay = Delay;
    let mut attempt: u32 = 0;

    loop {
        attempt += 1;
        info!("Initializing ICM-20948 (attempt {}/{})", attempt, IMU_INIT_MAX_ATTEMPTS);

        match Icm20948Driver::try_new(I2cInterface::default(I2cDevice::new(i2c_bus))).await {
            Ok(mut imu) => match imu.init(&mut delay).await {
                Ok(()) => {
                    info!("ICM-20948 initialized successfully");
                    Timer::after(Duration::from_millis(100)).await;
                    return imu;
                }
                Err(e) => warn!("ICM-20948 init failed (attempt {}): {:?}", attempt, e),
            },
            Err(e) => warn!("ICM-20948 detect failed (attempt {}): {:?}", attempt, e),
        }

        if attempt >= IMU_INIT_MAX_ATTEMPTS {
            warn!(
                "ICM-20948 init gave up after {} attempts — task halted",
                IMU_INIT_MAX_ATTEMPTS
            );
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }

        Timer::after(Duration::from_millis(IMU_INIT_RETRY_DELAY_MS)).await;
    }
}

/// Load DMP firmware and initialise the magnetometer.
///
/// Sets [`MAG_AVAILABLE`] based on whether `dmp_init_magnetometer` succeeds.
/// Returns `false` if firmware loading fails (caller should terminate the task).
async fn init_dmp(sensor: &mut ImuSensor) -> bool {
    let mut delay = Delay;

    if let Err(e) = sensor.dmp_init(&mut delay).await {
        warn!("DMP firmware load failed: {:?}", e);
        return false;
    }
    info!("DMP firmware loaded");

    // Always attempt magnetometer init so Axis9 is available for later mode switches.
    match sensor.dmp_init_magnetometer(&mut delay).await {
        Ok(()) => {
            info!("DMP magnetometer initialized — Axis9 fusion available");
            MAG_AVAILABLE.store(true, Ordering::Relaxed);
        }
        Err(e) => {
            warn!("DMP magnetometer init failed: {:?} — Axis9 unavailable, Axis6 only", e);
            MAG_AVAILABLE.store(false, Ordering::Relaxed);
        }
    }

    true
}

/// Build a [`DmpConfig`] for the given fusion mode.
const fn build_dmp_config(mode: AhrsFusionMode) -> DmpConfig {
    match mode {
        AhrsFusionMode::Axis6 => DmpConfig::new()
            .with_quaternion_6axis(true)
            .with_host_calibrated_accel(true)
            .with_raw_accel(true)
            .with_raw_gyro(true)
            .with_calibrated_gyro(true)
            .with_sample_rate(DMP_SAMPLE_RATE_HZ),
        AhrsFusionMode::Axis9 => DmpConfig::new()
            .with_quaternion_9axis(true)
            .with_host_calibrated_accel(true)
            .with_raw_accel(true)
            .with_raw_gyro(true)
            .with_calibrated_gyro(true)
            .with_raw_mag(true)
            .with_sample_rate(DMP_SAMPLE_RATE_HZ),
    }
}

/// Apply a DMP configuration, enable the DMP, and flush the FIFO.
///
/// Returns `false` if any step fails.
async fn apply_dmp_config(sensor: &mut ImuSensor, config: &DmpConfig) -> bool {
    if let Err(e) = sensor.dmp_configure(config).await {
        warn!("DMP configure failed: {:?}", e);
        return false;
    }
    if let Err(e) = sensor.dmp_enable(true).await {
        warn!("DMP enable failed: {:?}", e);
        return false;
    }
    if let Err(e) = sensor.reset_fifo().await {
        warn!("FIFO reset failed: {:?}", e);
        return false;
    }
    true
}

/// Resolve the effective mode: downgrade `Axis9` to `Axis6` if mag is unavailable.
fn effective_mode(requested: AhrsFusionMode) -> AhrsFusionMode {
    if requested == AhrsFusionMode::Axis9 && !MAG_AVAILABLE.load(Ordering::Relaxed) {
        info!("Axis9 requested but magnetometer unavailable — using Axis6");
        AhrsFusionMode::Axis6
    } else {
        requested
    }
}

/// Core command/sampling loop.
///
/// Waits for [`ImuCommand::Start`], then polls the DMP FIFO on a fixed timer
/// while also handling in-band commands (stop, mode switch, calibration load).
#[allow(clippy::too_many_lines)]
async fn run_imu_command_loop(sensor: &mut ImuSensor) {
    let mut current_calibration: Option<flash_storage::ImuCalibration> = None;
    let mut fusion_mode = effective_mode(DEFAULT_FUSION_MODE);

    #[cfg(feature = "telemetry_logs")]
    let mut last_loop_diag_ts_ms: u32 = 0;

    'command: loop {
        match IMU_CONTROL.wait().await {
            // ── Start command ─────────────────────────────────────────────────
            ImuCommand::Start => {
                info!("Starting IMU readings (DMP mode: {:?})", fusion_mode);

                let config = build_dmp_config(fusion_mode);
                if !apply_dmp_config(sensor, &config).await {
                    warn!("DMP start failed — waiting for next command");
                    continue 'command;
                }

                let mut fifo_failures: u32 = 0;

                loop {
                    match select(IMU_CONTROL.wait(), Timer::after(POLL_INTERVAL)).await {
                        // ── In-band: Stop ─────────────────────────────────────
                        Either::First(ImuCommand::Stop) => {
                            info!("IMU stopped");
                            let _ = sensor.dmp_enable(false).await;
                            continue 'command;
                        }

                        // ── In-band: mode switch ──────────────────────────────
                        Either::First(ImuCommand::SetFusionMode(mode)) => {
                            let new_mode = effective_mode(mode);
                            if new_mode == fusion_mode {
                                info!("IMU fusion mode already {:?} — no change", fusion_mode);
                            } else {
                                fusion_mode = new_mode;
                                info!("Switching DMP fusion mode to {:?}", fusion_mode);
                                let _ = sensor.dmp_enable(false).await;
                                let new_config = build_dmp_config(fusion_mode);
                                if !apply_dmp_config(sensor, &new_config).await {
                                    warn!("DMP mode switch failed — continuing on previous config");
                                }
                                fifo_failures = 0;
                            }
                        }

                        // ── In-band: calibration load ─────────────────────────
                        Either::First(ImuCommand::LoadCalibration(cal)) => {
                            current_calibration = Some(cal);
                            info!("IMU calibration data loaded");
                        }

                        // ── In-band: Start (already running — ignore) ─────────
                        Either::First(ImuCommand::Start) => {}

                        // ── Timer: poll FIFO ──────────────────────────────────
                        Either::Second(()) => {
                            match sensor.dmp_read_fifo().await {
                                Ok(Some(packet)) => {
                                    fifo_failures = 0;

                                    // Prefer 9-axis quaternion; fall back to 6-axis.
                                    let quat_opt =
                                        packet.quaternion_9axis.as_ref().or(packet.quaternion_6axis.as_ref());

                                    if let Some(quat) = quat_opt {
                                        let orientation = dmp_quat_to_orientation(quat);
                                        let timestamp_ms = Instant::now().as_millis();

                                        *LATEST_ORIENTATION.lock().await = Some(orientation);

                                        update_statics_from_dmp(&packet, current_calibration.as_ref()).await;

                                        let measurement = ImuMeasurement {
                                            orientation,
                                            timestamp_ms,
                                        };

                                        #[cfg(feature = "telemetry_logs")]
                                        {
                                            #[allow(clippy::cast_possible_truncation)]
                                            let ts_ms = timestamp_ms as u32;
                                            if ts_ms.wrapping_sub(last_loop_diag_ts_ms) >= IMU_LOOP_DIAG_LOG_INTERVAL_MS
                                            {
                                                defmt::info!(
                                                    "IMU diag: mode={:?} yaw={=f32} pitch={=f32} roll={=f32}",
                                                    fusion_mode,
                                                    orientation.yaw,
                                                    orientation.pitch,
                                                    orientation.roll
                                                );
                                                last_loop_diag_ts_ms = ts_ms;
                                            }
                                        }

                                        raise_event(Events::ImuMeasurementTaken(measurement)).await;
                                    } else {
                                        // Packet present but no quaternion yet (DMP warming up).
                                        update_statics_from_dmp(&packet, current_calibration.as_ref()).await;
                                    }
                                }

                                Ok(None) => {
                                    // FIFO not ready — normal during DMP warm-up.
                                }

                                Err(e) => {
                                    fifo_failures += 1;
                                    warn!("DMP FIFO read error ({}/{}): {:?}", fifo_failures, MAX_FIFO_FAILURES, e);

                                    // Attempt a FIFO reset to clear any overflow.
                                    let _ = sensor.reset_fifo().await;

                                    if fifo_failures >= MAX_FIFO_FAILURES {
                                        warn!("Max FIFO failures reached — IMU task terminating");
                                        return;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // ── Standby: commands received before Start ────────────────────────
            ImuCommand::Stop => {
                info!("IMU stop received (already in standby)");
            }
            ImuCommand::LoadCalibration(cal) => {
                current_calibration = Some(cal);
                info!("IMU calibration loaded (standby)");
            }
            ImuCommand::SetFusionMode(mode) => {
                fusion_mode = effective_mode(mode);
                info!("IMU fusion mode set to {:?} (standby)", fusion_mode);
            }
        }
    }
}

// ── Embassy task ──────────────────────────────────────────────────────────────

/// Embassy task that drives DMP-based IMU measurements on the ICM-20948.
///
/// Initialises the sensor and DMP firmware, then enters the command/sampling
/// loop. The task terminates only on unrecoverable sensor failures.
#[embassy_executor::task]
pub async fn inertial_measurement_read(i2c_bus: &'static I2cBusShared) {
    let mut sensor = init_imu_sensor(i2c_bus).await;

    if !init_dmp(&mut sensor).await {
        warn!("DMP firmware load failed — IMU task terminating");
        return;
    }

    info!(
        "IMU DMP ready. MAG_AVAILABLE={}. Default mode={:?}",
        MAG_AVAILABLE.load(Ordering::Relaxed),
        DEFAULT_FUSION_MODE
    );

    run_imu_command_loop(&mut sensor).await;
}
