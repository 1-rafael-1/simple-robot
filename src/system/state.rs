//! System state
//!
//! Global robot state including operation mode, battery, and sensors.

use defmt::Format;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

/// Global system state
pub static SYSTEM_STATE: Mutex<CriticalSectionRawMutex, SystemState> = Mutex::new(SystemState {
    operation_mode: OperationMode::Manual,
    battery_level: 100,
    obstacle_detected: false,
    standby: false,
});

/// Robot system state
#[derive(Format)]
pub struct SystemState {
    /// Current operation mode
    pub operation_mode: OperationMode,
    /// Battery level (0-100)
    pub battery_level: u8,
    /// Obstacle detection status
    pub obstacle_detected: bool,
    /// Standby mode status
    pub standby: bool,
}

impl SystemState {
    /// Updates operation mode
    pub fn set_operation_mode(&mut self, new_mode: OperationMode) {
        self.operation_mode.set(new_mode);
    }
}

/// Robot operation modes
#[derive(Debug, Clone, PartialEq, Format, Copy)]
pub enum OperationMode {
    /// User-controlled operation
    Manual,
    /// Self-controlled operation
    Autonomous,
}

impl OperationMode {
    /// Updates operation mode
    fn set(&mut self, new_mode: OperationMode) {
        *self = new_mode;
    }
}
