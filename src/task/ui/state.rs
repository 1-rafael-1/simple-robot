//! UI state owned by the UI controller.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use crate::system::state::UiMode;

/// UI state owned by the UI controller.
#[derive(Clone, Copy)]
pub struct UiState {
    /// Current UI mode.
    pub mode: UiMode,
    /// Selected index in the main menu.
    pub main_index: usize,
    /// Selected index in the calibration menu.
    pub calibrate_index: usize,
}

impl UiState {
    /// Creates the default UI state (main menu with first items selected).
    pub const fn new() -> Self {
        Self {
            mode: UiMode::MainMenu,
            main_index: 0,
            calibrate_index: 0,
        }
    }
}

/// Global UI state mutex owned by the UI controller.
pub static UI_STATE: Mutex<CriticalSectionRawMutex, UiState> = Mutex::new(UiState::new());
