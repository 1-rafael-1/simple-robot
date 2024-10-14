//! System State Module
//!
//! This module defines the global system state of the robot, including
//! its operation mode, battery level, and obstacle detection status.
//! It provides a thread-safe way to access and modify the system state.

use defmt::Format;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

/// Global system state protected by a mutex
///
/// This static variable holds the current state of the system,
/// allowing safe concurrent access from multiple tasks.
pub static SYSTEM_STATE: Mutex<CriticalSectionRawMutex, SystemState> = Mutex::new(SystemState {
    operation_mode: OperationMode::Manual,
    battery_level: 100,
    obstacle_detected: false,
});

/// Represents the current state of the robot system
///
/// This struct encapsulates all the key information about the robot's current state,
/// including its operation mode, battery level, and whether an obstacle is detected.
#[derive(Format)]
pub struct SystemState {
    /// Current operation mode of the robot
    pub operation_mode: OperationMode,

    /// Current battery level (0-100)
    pub battery_level: u8,

    /// Indicates whether an obstacle is currently detected
    pub obstacle_detected: bool,
}

/// Represents the possible operation modes of the robot
///
/// This enum defines the different modes in which the robot can operate,
/// allowing for distinction between manual control and autonomous operation.
#[derive(Debug, Clone, PartialEq, Format, Copy)]
pub enum OperationMode {
    /// Manual operation mode, where the robot is controlled directly by user input
    Manual,

    /// Autonomous operation mode, where the robot operates independently based on programmed behaviors
    Autonomous,
}
