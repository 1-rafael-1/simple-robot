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
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use crate::system::event::UltrasonicReading;

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
/// Initialized to:
/// - Manual operation mode
/// - 100% battery level
/// - No battery voltage reading yet
/// - No obstacles detected
/// - Standby mode disabled
/// - No calibration data loaded
pub static SYSTEM_STATE: Mutex<CriticalSectionRawMutex, SystemState> = Mutex::new(SystemState {
    operation_mode: OperationMode::Manual,
    battery_level: None,
    battery_voltage: None,
    obstacle_detected: false,
    ultrasonic_reading: None,
    ultrasonic_angle_deg: None,
    standby: false,
    motor_calibration_status: CalibrationStatus::NotLoaded,
    imu_calibration_status: CalibrationStatus::NotLoaded,
    mag_calibration_status: CalibrationStatus::NotLoaded,
    accel_calibration_status: CalibrationStatus::NotLoaded,
    gyro_calibration_status: CalibrationStatus::NotLoaded,
    left_track_speed: 0,
    right_track_speed: 0,
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
    pub battery_level: Option<u8>,
    /// Battery voltage in volts (2S Li-Ion: 6.0V-8.4V)
    /// - None: No reading available yet
    /// - Some(voltage): Latest voltage measurement
    ///
    /// Used by motor driver for voltage compensation
    pub battery_voltage: Option<f32>,
    /// Obstacle detection status
    /// - true: Obstacle detected within threshold distance
    /// - false: Path is clear
    pub obstacle_detected: bool,
    /// Latest ultrasonic reading status, if available
    pub ultrasonic_reading: Option<UltrasonicReading>,
    /// Latest ultrasonic angle reading (degrees), if available
    pub ultrasonic_angle_deg: Option<f32>,
    /// Standby mode status
    /// - true: Power saving mode active
    /// - false: Normal power mode
    pub standby: bool,
    /// Motor calibration status
    /// - `NotLoaded`: Haven't queried flash yet
    /// - Loaded: Calibration data loaded from flash and applied
    /// - `NotAvailable`: No calibration data in flash (needs calibration run)
    pub motor_calibration_status: CalibrationStatus,
    /// IMU calibration status (aggregate)
    /// - `NotLoaded`: Haven't queried flash yet
    /// - Loaded: Calibration data loaded from flash and applied
    /// - `NotAvailable`: No calibration data in flash (needs calibration run)
    pub imu_calibration_status: CalibrationStatus,
    /// Magnetometer calibration status
    /// - `NotLoaded`: Haven't queried flash yet
    /// - Loaded: Calibration data loaded from flash and applied
    /// - `NotAvailable`: No calibration data in flash (needs calibration run)
    pub mag_calibration_status: CalibrationStatus,
    /// Accelerometer calibration status
    /// - `NotLoaded`: Haven't queried flash yet
    /// - Loaded: Calibration data loaded from flash and applied
    /// - `NotAvailable`: No calibration data in flash (needs calibration run)
    pub accel_calibration_status: CalibrationStatus,
    /// Gyroscope calibration status
    /// - `NotLoaded`: Haven't queried flash yet
    /// - Loaded: Calibration data loaded from flash and applied
    /// - `NotAvailable`: No calibration data in flash (needs calibration run)
    pub gyro_calibration_status: CalibrationStatus,
    /// Current left track speed (-100 to +100)
    /// - Negative values: reverse
    /// - 0: stopped
    /// - Positive values: forward
    pub left_track_speed: i8,
    /// Current right track speed (-100 to +100)
    /// - Negative values: reverse
    /// - 0: stopped
    /// - Positive values: forward
    pub right_track_speed: i8,
}

impl SystemState {
    /// Updates operation mode and ensures state consistency
    pub const fn set_operation_mode(&mut self, new_mode: OperationMode) {
        self.operation_mode.set(new_mode);
    }

    /// Checks if system initialization is complete
    /// - Returns true when both calibrations have been queried (loaded or not available)
    pub fn is_initialized(&self) -> bool {
        self.motor_calibration_status != CalibrationStatus::NotLoaded
            && self.imu_calibration_status != CalibrationStatus::NotLoaded
    }
}

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
