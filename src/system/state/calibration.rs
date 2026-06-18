//! Calibration state module.
//!
//! Holds calibration status flags and exposes helpers for initialization checks.
//!
//! Lock order (when multiple state mutexes are needed):
//! 1) `POWER_STATE`
//! 2) `CALIBRATION_STATE`
//! 3) perception mutex (use accessor functions, see `perception` module)
//! 4) `MOTION_STATE`

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use crate::system::state::CalibrationStatus;

/// Global calibration state protected by a mutex.
pub static CALIBRATION_STATE: Mutex<CriticalSectionRawMutex, CalibrationState> = Mutex::new(CalibrationState {
    motor_calibration_status: CalibrationStatus::NotLoaded,
    imu_calibration_status: CalibrationStatus::NotLoaded,
    mag_calibration_status: CalibrationStatus::NotLoaded,
    distance_calibration_status: CalibrationStatus::NotLoaded,
    distance_factor: 1.0,
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
    /// Distance calibration status.
    pub distance_calibration_status: CalibrationStatus,
    /// Distance calibration factor (1.0 = no correction).
    pub distance_factor: f32,
}

impl CalibrationState {
    /// Checks if system initialization is complete.
    pub fn is_initialized(&self) -> bool {
        self.motor_calibration_status != CalibrationStatus::NotLoaded
            && self.imu_calibration_status != CalibrationStatus::NotLoaded
            && self.distance_calibration_status != CalibrationStatus::NotLoaded
    }
}

/// Checks if system initialization is complete (async helper).
pub async fn is_initialized() -> bool {
    let state = CALIBRATION_STATE.lock().await;
    state.is_initialized()
}

/// Return the distance calibration factor from the global state.
pub async fn get_distance_factor() -> f32 {
    let state = CALIBRATION_STATE.lock().await;
    state.distance_factor
}
