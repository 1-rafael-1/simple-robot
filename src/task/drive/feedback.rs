//! Sensor feedback handling for drive control
//!
//! This module manages all sensor data channels and functions for forwarding
//! measurements from the orchestrator to the drive task.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex};
use embassy_time::{Duration, Timer};
use nalgebra::Vector3;

use crate::task::sensors::{encoders::EncoderMeasurement, imu::ImuMeasurement};

/// Latest encoder measurement
///
/// The orchestrator forwards encoder measurements here. During calibration,
/// the drive task reads the latest measurement on demand rather than draining a queue.
/// This ensures we always get fresh data and don't miss measurements due to channel overflow.
pub(super) static LATEST_ENCODER_MEASUREMENT: Mutex<CriticalSectionRawMutex, Option<EncoderMeasurement>> =
    Mutex::new(None);

/// Channel for receiving IMU measurements from orchestrator
/// Capacity of 16 allows buffering during calibration sequences (100Hz sampling)
const IMU_FEEDBACK_QUEUE_SIZE: usize = 16;
/// The orchestrator sends IMU measurements here during calibration to provide real-time feedback to the drive task.
pub(super) static IMU_FEEDBACK_CHANNEL: Channel<CriticalSectionRawMutex, ImuMeasurement, IMU_FEEDBACK_QUEUE_SIZE> =
    Channel::new();

/// Latest raw magnetometer measurement
///
/// Used during IMU calibration to measure motor interference effects.
/// The IMU task sends raw magnetometer readings here before applying any corrections.
pub(super) static LATEST_MAG_MEASUREMENT: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest raw gyroscope measurement
///
/// Used during IMU calibration to measure gyroscope bias.
/// The IMU task sends raw gyroscope readings here before applying bias correction.
pub(super) static LATEST_GYRO_MEASUREMENT: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Latest raw accelerometer measurement
///
/// Used during IMU calibration to measure accelerometer bias.
/// The IMU task sends raw accelerometer readings here before applying any corrections.
pub(super) static LATEST_ACCEL_MEASUREMENT: Mutex<CriticalSectionRawMutex, Option<Vector3<f32>>> = Mutex::new(None);

/// Update the latest encoder measurement
///
/// Called by orchestrator to forward encoder events from the `encoder_read` task.
/// Stores the measurement in a mutex so the drive task can read it on demand.
pub async fn send_encoder_measurement(measurement: EncoderMeasurement) {
    let mut latest = LATEST_ENCODER_MEASUREMENT.lock().await;
    *latest = Some(measurement);
}

/// Try to update the latest encoder measurement without blocking
///
/// Returns true if updated successfully.
/// Used by orchestrator to avoid blocking when drive task is busy during calibration.
pub async fn try_send_encoder_measurement(measurement: EncoderMeasurement) -> bool {
    let mut latest = LATEST_ENCODER_MEASUREMENT.lock().await;
    *latest = Some(measurement);
    true
}

/// Clear the latest encoder measurement (internal use by drive task)
///
/// Used before each calibration step to ensure we wait for fresh data after a reset.
pub(super) async fn clear_encoder_measurement() {
    let mut latest = LATEST_ENCODER_MEASUREMENT.lock().await;
    *latest = None;
}

/// Try to send IMU measurement to drive task without blocking
///
/// Returns true if sent, false if channel is full.
/// Used by orchestrator to avoid blocking when drive task is busy.
/// IMU measurements arrive at 100Hz, so dropping occasional readings is acceptable.
pub fn try_send_imu_measurement(measurement: ImuMeasurement) -> bool {
    IMU_FEEDBACK_CHANNEL.sender().try_send(measurement).is_ok()
}

/// Get the latest encoder measurement (internal use by drive task)
///
/// Returns the most recent encoder measurement, or None if no measurement available.
pub(super) async fn get_latest_encoder_measurement() -> Option<EncoderMeasurement> {
    let latest = LATEST_ENCODER_MEASUREMENT.lock().await;
    *latest
}

/// Receive IMU measurement from channel (internal use by drive task)
pub(super) async fn receive_imu_measurement() -> ImuMeasurement {
    IMU_FEEDBACK_CHANNEL.receive().await
}

/// Update the latest magnetometer measurement
///
/// Called by IMU task to provide raw magnetometer readings for calibration.
/// Only used during IMU calibration procedure.
pub async fn send_mag_measurement(mag: Vector3<f32>) {
    let mut latest = LATEST_MAG_MEASUREMENT.lock().await;
    *latest = Some(mag);
}

/// Update the latest gyroscope measurement
///
/// Called by IMU task to provide raw gyroscope readings for calibration.
/// Only used during IMU calibration procedure.
pub async fn send_gyro_measurement(gyro: Vector3<f32>) {
    let mut latest = LATEST_GYRO_MEASUREMENT.lock().await;
    *latest = Some(gyro);
}

/// Update the latest accelerometer measurement
///
/// Called by IMU task to provide raw accelerometer readings for calibration.
/// Only used during IMU calibration procedure.
pub async fn send_accel_measurement(accel: Vector3<f32>) {
    let mut latest = LATEST_ACCEL_MEASUREMENT.lock().await;
    *latest = Some(accel);
}

/// Get the latest magnetometer measurement (internal use by drive task)
///
/// Returns the most recent raw magnetometer reading, or None if no measurement available.
pub(super) async fn get_latest_mag_measurement() -> Option<Vector3<f32>> {
    let latest = LATEST_MAG_MEASUREMENT.lock().await;
    *latest
}

/// Get the latest gyroscope measurement (internal use by drive task)
///
/// Returns the most recent raw gyroscope reading, or None if no measurement available.
pub(super) async fn get_latest_gyro_measurement() -> Option<Vector3<f32>> {
    let latest = LATEST_GYRO_MEASUREMENT.lock().await;
    *latest
}

/// Get the latest accelerometer measurement (internal use by drive task)
///
/// Returns the most recent raw accelerometer reading, or None if no measurement available.
pub(super) async fn get_latest_accel_measurement() -> Option<Vector3<f32>> {
    let latest = LATEST_ACCEL_MEASUREMENT.lock().await;
    *latest
}

/// Clear the latest magnetometer measurement (internal use by drive task)
///
/// Used before each calibration measurement to ensure we wait for fresh data.
pub(super) async fn clear_mag_measurement() {
    let mut latest = LATEST_MAG_MEASUREMENT.lock().await;
    *latest = None;
}

/// Clear the latest gyroscope measurement (internal use by drive task)
///
/// Used before each calibration measurement to ensure we wait for fresh data.
pub(super) async fn clear_gyro_measurement() {
    let mut latest = LATEST_GYRO_MEASUREMENT.lock().await;
    *latest = None;
}

/// Clear the latest accelerometer measurement (internal use by drive task)
///
/// Used before each calibration measurement to ensure we wait for fresh data.
pub(super) async fn clear_accel_measurement() {
    let mut latest = LATEST_ACCEL_MEASUREMENT.lock().await;
    *latest = None;
}

/// Measure average magnetometer reading over N samples
///
/// Used during IMU calibration to get stable baseline and interference measurements.
/// Waits for fresh magnetometer data from IMU task at 50Hz sampling rate.
pub(super) async fn measure_mag_average(samples: u16) -> Vector3<f32> {
    let mut sum = Vector3::<f64>::new(0.0, 0.0, 0.0);
    let mut count = 0;

    for _ in 0..samples {
        if let Some(mag) = get_latest_mag_measurement().await {
            sum += mag.cast::<f64>();
            count += 1;
        }
        Timer::after(Duration::from_millis(20)).await; // 50Hz
    }

    if count > 0 {
        (sum / f64::from(count)).cast::<f32>()
    } else {
        Vector3::new(0.0, 0.0, 0.0)
    }
}

/// Wait for IMU measurement with timeout
///
/// Used during IMU calibration to collect gyro/accel samples.
pub(super) async fn wait_for_imu_event_timeout(timeout_ms: u64) -> Option<ImuMeasurement> {
    embassy_time::with_timeout(Duration::from_millis(timeout_ms), receive_imu_measurement())
        .await
        .ok()
}

/// Wait for magnetometer measurement with timeout
///
/// Used during magnetometer calibration to collect samples.
pub(super) async fn wait_for_mag_event_timeout(timeout_ms: u64) -> Option<Vector3<f32>> {
    let start = embassy_time::Instant::now();
    while embassy_time::Instant::now().duration_since(start).as_millis() < timeout_ms {
        if let Some(mag) = get_latest_mag_measurement().await {
            return Some(mag);
        }
        Timer::after(Duration::from_millis(20)).await;
    }
    None
}

/// Wait for a fresh encoder measurement with timeout
///
/// Polls for a new encoder measurement, waiting up to `timeout_ms`.
/// Returns None if timeout occurs before a measurement is available.
pub(super) async fn wait_for_encoder_event_timeout(timeout_ms: u64) -> Option<EncoderMeasurement> {
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

        // Small delay to avoid busy-waiting
        Timer::after(Duration::from_millis(10)).await;
    }
}

/// Subtract two magnetometer vectors
///
/// Used to calculate motor interference by subtracting baseline from motor-on measurements.
pub(super) fn subtract_mag(a: Vector3<f32>, b: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(a.x - b.x, a.y - b.y, a.z - b.z)
}
