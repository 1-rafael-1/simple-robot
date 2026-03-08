//! Sensor feedback channels and measurement forwarding for the drive task.
//!
//! This module owns all static sensor data channels and the functions that
//! write into or read from them. It is the *data plane* of the sensor
//! infrastructure: measurements flow in from sensor tasks (via the orchestrator)
//! and are stored here so that drive control loops can read them on demand.
//!
//! # Channel design
//!
//! - **Encoder measurements** are stored in a mutex-guarded `Option` so the
//!   drive task always reads the latest value rather than draining a queue.
//! - **IMU measurements** are queued (capacity 16) to buffer bursts during
//!   calibration sequences at 50 Hz sampling. These measurements carry
//!   calibrated orientation when the IMU task has loaded calibration data.
//! - **Raw IMU axes** (magnetometer, gyroscope, accelerometer) are stored as
//!   mutex-guarded `Option`s, written by the IMU task before correction and
//!   read by calibration routines on demand.
//!
//! # Visibility
//!
//! All items in this module are `pub` — access is naturally limited because
//! this module itself is private within the `drive` module tree. Items called
//! by the orchestrator or IMU task are re-exported from `drive/mod.rs` where
//! needed; items used only within the drive task remain reachable only through
//! the private module path.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex};
use embassy_time::{Duration, Timer};
use nalgebra::Vector3;

use crate::task::sensors::{encoders::EncoderMeasurement, imu::ImuMeasurement};

// ── Static channels ───────────────────────────────────────────────────────────

/// Latest encoder measurement.
///
/// The orchestrator forwards encoder measurements here. Drive control loops
/// read the latest value on demand rather than draining a queue, ensuring
/// they always see fresh data even if they miss intermediate samples.
pub static LATEST_ENCODER_MEASUREMENT: Mutex<CriticalSectionRawMutex, Option<EncoderMeasurement>> = Mutex::new(None);

/// Channel for receiving IMU measurements from the orchestrator.
///
/// Capacity of 16 buffers bursts during calibration sequences at 50 Hz
/// sampling. The rotation control loop drains this channel each tick.
const IMU_FEEDBACK_QUEUE_SIZE: usize = 16;

/// IMU feedback channel shared between the orchestrator and drive control loops.
/// Measurements carry orientation already corrected by any loaded IMU calibration.
pub static IMU_FEEDBACK_CHANNEL: Channel<CriticalSectionRawMutex, ImuMeasurement, IMU_FEEDBACK_QUEUE_SIZE> =
    Channel::new();

/// Latest raw magnetometer measurement.
///
/// Written by the IMU task with uncorrected readings. Read by the IMU
/// calibration routine to measure motor-interference effects.
pub static LATEST_MAG_MEASUREMENT: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest raw gyroscope measurement.
///
/// Written by the IMU task before bias correction is applied. Read by the
/// IMU calibration routine to measure gyroscope bias.
pub static LATEST_GYRO_MEASUREMENT: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest raw accelerometer measurement.
///
/// Written by the IMU task before any correction is applied. Read by the
/// IMU calibration routine to measure accelerometer bias.
pub static LATEST_ACCEL_MEASUREMENT: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

// ── Public write functions (called by orchestrator / IMU task) ────────────────

/// Try to update the latest encoder measurement without blocking.
///
/// Returns `true` if the update succeeded, `false` if the mutex is currently
/// held by the drive task. Used by the orchestrator to avoid blocking when
/// the drive task holds the mutex during calibration.
pub fn try_send_encoder_measurement(measurement: EncoderMeasurement) -> bool {
    LATEST_ENCODER_MEASUREMENT.try_lock().is_ok_and(|mut latest| {
        *latest = Some(measurement);
        true
    })
}

/// Try to send an IMU measurement to the drive task without blocking.
///
/// Returns `true` if sent, `false` if the channel is full. Used by the
/// orchestrator to avoid blocking; IMU measurements arrive at 50 Hz so
/// dropping occasional readings during heavy load is acceptable. The
/// orientation data is calibrated when the IMU task has loaded calibration.
pub fn try_send_imu_measurement(measurement: ImuMeasurement) -> bool {
    IMU_FEEDBACK_CHANNEL.sender().try_send(measurement).is_ok()
}

/// Update the latest raw magnetometer measurement.
///
/// Called by the IMU task to provide uncorrected magnetometer readings for
/// calibration. Only used during the IMU calibration procedure.
pub async fn send_mag_measurement(mag: Vector3<f32>) {
    let mut latest = LATEST_MAG_MEASUREMENT.lock().await;
    *latest = Some(mag);
}

/// Update the latest raw gyroscope measurement.
///
/// Called by the IMU task to provide uncorrected gyroscope readings for
/// calibration. Only used during the IMU calibration procedure.
pub async fn send_gyro_measurement(gyro: Vector3<f32>) {
    let mut latest = LATEST_GYRO_MEASUREMENT.lock().await;
    *latest = Some(gyro);
}

/// Update the latest raw accelerometer measurement.
///
/// Called by the IMU task to provide uncorrected accelerometer readings for
/// calibration. Only used during the IMU calibration procedure.
pub async fn send_accel_measurement(accel: Vector3<f32>) {
    let mut latest = LATEST_ACCEL_MEASUREMENT.lock().await;
    *latest = Some(accel);
}

// ── Internal read / clear functions (used by drive submodules) ────────────────

/// Get the latest encoder measurement.
///
/// Returns the most recent encoder measurement, or `None` if no measurement
/// is available yet (e.g., before the encoder task has started sampling).
pub async fn get_latest_encoder_measurement() -> Option<EncoderMeasurement> {
    let latest = LATEST_ENCODER_MEASUREMENT.lock().await;
    *latest
}

/// Clear the latest encoder measurement.
///
/// Called before starting a new distance drive or calibration step to ensure
/// the next read waits for a fresh sample after the encoder counters are reset.
pub async fn clear_encoder_measurement() {
    let mut latest = LATEST_ENCODER_MEASUREMENT.lock().await;
    *latest = None;
}

/// Get the latest raw magnetometer measurement.
///
/// Returns the most recent raw reading, or `None` if none has arrived yet.
pub async fn get_latest_mag_measurement() -> Option<Vector3<f32>> {
    let latest = LATEST_MAG_MEASUREMENT.lock().await;
    *latest
}

/// Get the latest raw gyroscope measurement.
///
/// Returns the most recent raw reading, or `None` if none has arrived yet.
pub async fn get_latest_gyro_measurement() -> Option<Vector3<f32>> {
    let latest = LATEST_GYRO_MEASUREMENT.lock().await;
    *latest
}

/// Get the latest raw accelerometer measurement.
///
/// Returns the most recent raw reading, or `None` if none has arrived yet.
pub async fn get_latest_accel_measurement() -> Option<Vector3<f32>> {
    let latest = LATEST_ACCEL_MEASUREMENT.lock().await;
    *latest
}

/// Clear the latest magnetometer measurement.
///
/// Called before each calibration measurement step to ensure the next read
/// returns a fresh sample.
pub async fn clear_mag_measurement() {
    let mut latest = LATEST_MAG_MEASUREMENT.lock().await;
    *latest = None;
}

/// Clear the latest gyroscope measurement.
///
/// Called before each calibration measurement step to ensure the next read
/// returns a fresh sample.
pub async fn clear_gyro_measurement() {
    let mut latest = LATEST_GYRO_MEASUREMENT.lock().await;
    *latest = None;
}

/// Clear the latest accelerometer measurement.
///
/// Called before each calibration measurement step to ensure the next read
/// returns a fresh sample.
pub async fn clear_accel_measurement() {
    let mut latest = LATEST_ACCEL_MEASUREMENT.lock().await;
    *latest = None;
}

// ── Calibration helpers ───────────────────────────────────────────────────────

/// Measure the average magnetometer reading over `samples` samples.
///
/// Used during IMU calibration to obtain a stable baseline and to measure
/// motor-interference effects. Waits for fresh data at ~50 Hz.
pub async fn measure_mag_average(samples: u16) -> Vector3<f32> {
    let mut sum = Vector3::<f64>::new(0.0, 0.0, 0.0);
    let mut count = 0u16;

    for _ in 0..samples {
        clear_mag_measurement().await;
        if let Some(mag) = wait_for_mag_event_timeout(50).await {
            sum += mag.cast::<f64>();
            count += 1;
        }
    }

    if count > 0 {
        (sum / f64::from(count)).cast::<f32>()
    } else {
        Vector3::new(0.0, 0.0, 0.0)
    }
}

/// Wait for a fresh magnetometer measurement with a timeout.
///
/// Polls `get_latest_mag_measurement` at ~50 Hz until a value is available
/// or `timeout_ms` elapses. Used during magnetometer calibration.
pub async fn wait_for_mag_event_timeout(timeout_ms: u64) -> Option<Vector3<f32>> {
    let start = embassy_time::Instant::now();
    while embassy_time::Instant::now().duration_since(start).as_millis() < timeout_ms {
        if let Some(mag) = get_latest_mag_measurement().await {
            return Some(mag);
        }
        Timer::after(Duration::from_millis(20)).await;
    }
    None
}

/// Wait for a fresh encoder measurement with a timeout.
///
/// Polls `get_latest_encoder_measurement` at 10 ms intervals until a value
/// is available or `timeout_ms` elapses. Used during motor calibration to
/// wait for the first sample after a counter reset.
pub async fn wait_for_encoder_event_timeout(timeout_ms: u64) -> Option<EncoderMeasurement> {
    use embassy_time::{Duration, Instant, Timer};

    let start = Instant::now();
    let timeout_duration = Duration::from_millis(timeout_ms);

    loop {
        if let Some(measurement) = get_latest_encoder_measurement().await {
            return Some(measurement);
        }

        if start.elapsed() >= timeout_duration {
            return None;
        }

        Timer::after(Duration::from_millis(10)).await;
    }
}

/// Subtract two magnetometer vectors component-wise.
///
/// Used during IMU calibration to compute motor interference by subtracting
/// a baseline (motors off) from a motors-on measurement.
pub fn subtract_mag(a: Vector3<f32>, b: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(a.x - b.x, a.y - b.y, a.z - b.z)
}
