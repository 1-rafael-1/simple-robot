//! Power state module.
//!
//! Holds battery status and standby flag in a dedicated mutex.
//!
//! Lock order (when multiple state mutexes are needed):
//! 1) `POWER_STATE`
//! 2) `CALIBRATION_STATE`
//! 3) `PERCEPTION_STATE`
//! 4) `MOTION_STATE`

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

/// Global power state protected by a mutex.
pub static POWER_STATE: Mutex<CriticalSectionRawMutex, PowerState> = Mutex::new(PowerState {
    battery_level: None,
    battery_voltage: None,
    standby: false,
});

/// Power-related state shared across the system.
pub struct PowerState {
    /// Battery level percentage (0-100), if available.
    pub battery_level: Option<u8>,
    /// Battery voltage in volts, if available.
    pub battery_voltage: Option<f32>,
    /// Standby mode status.
    pub standby: bool,
}
