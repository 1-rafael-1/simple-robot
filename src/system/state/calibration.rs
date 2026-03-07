//! Calibration state module.
//!
//! Holds calibration status flags and exposes helpers for initialization checks.
//!
//! Lock order (when multiple state mutexes are needed):
//! 1) `POWER_STATE`
//! 2) `SYSTEM_STATE`
//! 3) `CALIBRATION_STATE`
//! 4) `PERCEPTION_STATE`

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use crate::system::state::CalibrationStatus;

/// Global calibration state protected by a mutex.
pub static CALIBRATION_STATE: Mutex<CriticalSectionRawMutex, CalibrationState> = Mutex::new(CalibrationState {
    motor_calibration_status: CalibrationStatus::NotLoaded,
    imu_calibration_status: CalibrationStatus::NotLoaded,
    mag_calibration_status: CalibrationStatus::NotLoaded,
    accel_calibration_status: CalibrationStatus::NotLoaded,
    gyro_calibration_status: CalibrationStatus::NotLoaded,
});

/// Calibration state shared across the system.
#[allow(clippy::struct_field_names)]
pub struct CalibrationState {
    /// Motor calibration status.
    pub motor_calibration_status: CalibrationStatus,
    /// IMU calibration status (aggregate).
    pub imu_calibration_status: CalibrationStatus,
    /// Magnetometer calibration status.
    pub mag_calibration_status: CalibrationStatus,
    /// Accelerometer calibration status.
    pub accel_calibration_status: CalibrationStatus,
    /// Gyroscope calibration status.
    pub gyro_calibration_status: CalibrationStatus,
}

impl CalibrationState {
    /// Checks if system initialization is complete.
    /// Returns true when both motor and IMU calibration have been queried.
    pub fn is_initialized(&self) -> bool {
        self.motor_calibration_status != CalibrationStatus::NotLoaded
            && self.imu_calibration_status != CalibrationStatus::NotLoaded
    }
}

/// Checks if system initialization is complete (async helper).
pub async fn is_initialized() -> bool {
    let state = CALIBRATION_STATE.lock().await;
    state.is_initialized()
}
