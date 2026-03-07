//! System State Management
//!
//! Manages the robot's global state including:
//! - Shared enums and cross-cutting state modules
//!
//! The state is protected by a mutex to ensure safe concurrent access
//! from multiple tasks. All state changes are atomic and immediately
//! visible to all tasks.
//!
//! # State Components
//! - Operation Mode: Defined here and used by the `motion` module
//! - Domain state lives in dedicated submodules
//!
//! # State Access Pattern
//! ```rust
//! let state = SYSTEM_STATE.lock().await;
//! // Read or modify state here
//! // Lock automatically released when state goes out of scope
//! ```

use defmt::Format;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

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

/// Top-level UI mode
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
pub enum UiMode {
    /// Main menu display
    MainMenu,
    /// System info screen (scrollable)
    SystemInfo {
        /// Current scroll offset into the system info line list
        scroll_offset: u8,
    },
    /// Calibration submenu
    CalibrateMenu,
    /// Drive mode submenu
    DriveModeMenu,
    /// Test mode submenu
    TestMenu,
    /// Autonomous drive mode is active
    RunningAutonomous {
        /// Which drive mode is running
        mode: DriveMode,
    },
    /// Combined test sequence running
    RunningTest,
    /// IMU test mode (live display)
    RunningImuTest,
    /// Basic motor test mode
    RunningBasicMotorTest,
    /// IR + ultrasonic live test mode
    RunningIrUltrasonicTest,
    /// Ultrasonic sweep test mode
    RunningUltrasonicSweepTest,
    /// Optional: calibration running state
    Calibrating {
        /// Selected calibration kind
        kind: CalibrationSelection,
    },
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

/// Global system state protected by a mutex
///
/// Initialized to an empty struct; domain state lives in submodules.
pub static SYSTEM_STATE: Mutex<CriticalSectionRawMutex, SystemState> = Mutex::new(SystemState {});

/// Robot system state containing all runtime state information
///
/// This struct represents the complete state of the robot at any given time.
/// Changes to these values trigger corresponding system behaviors through
/// the event system.
#[derive(Format)]
pub struct SystemState {}

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
