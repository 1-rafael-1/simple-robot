//! System State Management
//!
//! Manages the robot's global state including:
//! - Shared enums and cross-cutting state modules
//!
//! State is compartmentalized into domain-specific modules, each with its own
//! synchronization. Shared enums live here; domain state lives in dedicated
//! submodules (e.g. `power`, `motion`, `perception`, `calibration`).
//!
//! # State Components
//! - Operation Mode: Defined here and used by the `motion` module
//! - Domain state lives in dedicated submodules

use defmt::Format;

pub mod calibration;
pub mod motion;
pub mod perception;
pub mod power;

/// Calibration data status
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
pub enum CalibrationStatus {
    /// Calibration data has not been queried yet
    NotLoaded,
    /// Calibration data was loaded from flash
    Loaded,
    /// No calibration data exists in flash (needs calibration run)
    NotAvailable,
}

/// Main menu selections
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
pub enum MenuSelection {
    /// Show system info screen
    SystemInfo,
    /// Enter calibration submenu
    Calibrate,
    /// Enter drive mode submenu
    DriveMode,
    /// Enter test submenu
    TestMode,
}

/// Autonomous drive mode selection
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
pub enum DriveMode {
    /// Coast until obstacle detected, then back up and turn randomly
    CoastAndAvoid,
}

/// Test submenu selections
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
pub enum TestSelection {
    /// Combined test sequence
    Combined,
    /// IMU live display test
    Imu,
    /// Basic motor test
    BasicMotor,
    /// IR + ultrasonic live display test
    IrUltrasonic,
    /// Ultrasonic sweep test
    UltrasonicSweep,
}

/// Calibration submenu selections
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
pub enum CalibrationSelection {
    /// Motor calibration
    Motor,
    /// Magnetometer calibration
    Mag,
    /// Accelerometer calibration
    Accel,
    /// Gyroscope calibration
    Gyro,
}

// Motion state helpers moved to the `motion` module.

/// Robot operation modes defining control behavior
#[derive(Debug, Clone, Eq, PartialEq, Format, Copy)]
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
    const fn set(&mut self, new_mode: Self) {
        *self = new_mode;
    }
}
