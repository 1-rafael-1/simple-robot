//! System State Management
//!
//! Manages the robot's global state including:
//! - Operation mode (Manual/Autonomous)
//! - Battery status
//! - Sensor states
//! - Power management
//!
//! The state is protected by a mutex to ensure safe concurrent access
//! from multiple tasks. All state changes are atomic and immediately
//! visible to all tasks.
//!
//! # State Components
//! - Operation Mode: Determines if robot is under manual control or autonomous
//! - Battery Level: Current charge level as percentage (0-100)
//! - Obstacle Detection: Whether an obstacle is currently detected
//! - Standby Status: Power saving mode indicator
//!
//! # State Access Pattern
//! ```rust
//! let state = SYSTEM_STATE.lock().await;
//! // Read or modify state here
//! // Lock automatically released when state goes out of scope
//! ```

use defmt::Format;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

/// Global system state protected by a mutex
///
/// Initialized to:
/// - Manual operation mode
/// - 100% battery level
/// - No obstacles detected
/// - Standby mode disabled
pub static SYSTEM_STATE: Mutex<CriticalSectionRawMutex, SystemState> = Mutex::new(SystemState {
    operation_mode: OperationMode::Manual,
    battery_level: 100,
    obstacle_detected: false,
    standby: false,
});

/// Robot system state containing all runtime state information
///
/// This struct represents the complete state of the robot at any given time.
/// Changes to these values trigger corresponding system behaviors through
/// the event system.
#[derive(Format)]
pub struct SystemState {
    /// Current operation mode (Manual/Autonomous)
    pub operation_mode: OperationMode,
    /// Battery level percentage (0-100)
    /// - 0: Critical, needs immediate charging
    /// - 1-20: Low battery warning
    /// - 21-99: Normal operation
    /// - 100: Fully charged
    pub battery_level: u8,
    /// Obstacle detection status
    /// - true: Obstacle detected within threshold distance
    /// - false: Path is clear
    pub obstacle_detected: bool,
    /// Standby mode status
    /// - true: Power saving mode active
    /// - false: Normal power mode
    pub standby: bool,
}

impl SystemState {
    /// Updates operation mode and ensures state consistency
    pub fn set_operation_mode(&mut self, new_mode: OperationMode) {
        self.operation_mode.set(new_mode);
    }
}

/// Robot operation modes defining control behavior
#[derive(Debug, Clone, PartialEq, Format, Copy)]
pub enum OperationMode {
    /// Manual mode: Robot responds to RC commands
    /// - Direct control through button inputs
    /// - Immediate response to commands
    /// - No autonomous features active
    Manual,
    /// Autonomous mode: Robot operates independently
    /// - Self-driving enabled
    /// - Obstacle avoidance active
    /// - RC input only used for mode switching
    Autonomous,
}

impl OperationMode {
    /// Updates operation mode while maintaining state consistency
    fn set(&mut self, new_mode: OperationMode) {
        *self = new_mode;
    }
}
