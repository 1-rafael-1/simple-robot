//! UI state owned by the UI controller.

use defmt::Format;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use crate::system::state::{CalibrationSelection, DriveMode};

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
    /// IMU test mode (6-axis live display)
    RunningImu6Test,
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

/// UI state owned by the UI controller.
#[derive(Clone, Copy)]
pub struct UiState {
    /// Current UI mode.
    pub mode: UiMode,
    /// Selected index in the main menu.
    pub main_index: usize,
    /// Selected index in the calibration menu.
    pub calibrate_index: usize,
    /// Selected index in the drive mode menu.
    pub drive_mode_index: usize,
    /// Selected index in the test menu.
    pub test_index: usize,
    /// Whether the current calibration run has completed.
    pub calibration_complete: bool,
}

impl UiState {
    /// Creates the default UI state (main menu with first items selected).
    pub const fn new() -> Self {
        Self {
            mode: UiMode::MainMenu,
            main_index: 0,
            calibrate_index: 0,
            drive_mode_index: 0,
            test_index: 0,
            calibration_complete: false,
        }
    }
}

/// Global UI state mutex owned by the UI controller.
pub static UI_STATE: Mutex<CriticalSectionRawMutex, UiState> = Mutex::new(UiState::new());
