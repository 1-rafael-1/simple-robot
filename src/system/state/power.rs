//! Power state module.
//!
//! Holds battery status in a private mutex, accessed through public functions.
//! All fields start as `None` — populated on first battery measurement.
//!
//! Lock order (when multiple state mutexes are needed):
//! 1) `POWER_STATE`
//! 2) `CALIBRATION_STATE`
//! 3) perception mutex (use accessor functions, see `perception` module)
//! 4) `MOTION_STATE`
//!
//! Callers that need both Power and Calibration data should call Power
//! accessors before locking `CALIBRATION_STATE`.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

/// Global power state — private, use accessor functions.
static POWER_STATE: Mutex<CriticalSectionRawMutex, PowerState> = Mutex::new(PowerState {
    battery_level: None,
    battery_voltage: None,
});

/// Power-related state shared across the system.
struct PowerState {
    /// Battery level percentage (0-100), if available.
    battery_level: Option<u8>,
    /// Battery voltage in volts, if available.
    battery_voltage: Option<f32>,
}

/// Write battery level and voltage in one atomic update.
pub async fn set_battery(level: u8, voltage: f32) {
    let mut state = POWER_STATE.lock().await;
    state.battery_level = Some(level);
    state.battery_voltage = Some(voltage);
}

/// Read battery level (0-100). Returns `None` before first measurement.
pub async fn get_battery_level() -> Option<u8> {
    POWER_STATE.lock().await.battery_level
}

/// Read battery voltage in volts. Returns `None` before first measurement.
pub async fn get_battery_voltage() -> Option<f32> {
    POWER_STATE.lock().await.battery_voltage
}

/// Non-blocking voltage read for hot paths (e.g. motor control loop).
/// Returns `None` if no reading available yet or if the mutex is busy.
pub fn try_get_battery_voltage() -> Option<f32> {
    POWER_STATE.try_lock().ok()?.battery_voltage
}
