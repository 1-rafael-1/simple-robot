//! UI controller module.
//!
//! Owns UI state, user interactions, and view rendering logic.

use crate::{
    system::{
        event::RotaryDirection,
        state::{CalibrationSelection, SYSTEM_STATE, UiMode},
    },
    task::{drive, testmode},
};

pub mod menu;
pub mod render;
pub mod state;
pub mod ui_menu;

use menu::{calibration_selection_from_index, menu_selection_from_index, next_menu_index};
use render::render_current_ui;
use state::UI_STATE;

/// Returns true once calibration data has been queried.
pub async fn ui_initialized() -> bool {
    let state = SYSTEM_STATE.lock().await;
    state.is_initialized()
}

/// Handle rotary encoder turns.
pub async fn handle_rotary_turned(direction: RotaryDirection) {
    if !ui_initialized().await {
        return;
    }

    let mode = {
        let ui = UI_STATE.lock().await;
        ui.mode
    };

    match mode {
        UiMode::MainMenu => {
            let mut ui = UI_STATE.lock().await;
            ui.main_index = next_menu_index(ui.main_index, ui_menu::MAIN_MENU_ITEMS.len(), direction);
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
        }
        UiMode::CalibrateMenu => {
            let mut ui = UI_STATE.lock().await;
            ui.calibrate_index = next_menu_index(ui.calibrate_index, ui_menu::CALIBRATE_MENU_ITEMS.len(), direction);
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
        }
        UiMode::SystemInfo { scroll_offset } => {
            let info = render::build_system_info_data().await;
            let max_scroll = menu::max_system_info_scroll(&info);
            let new_offset = match direction {
                RotaryDirection::Clockwise => (scroll_offset as usize + 1).min(max_scroll),
                RotaryDirection::CounterClockwise => (scroll_offset as usize).saturating_sub(1),
            };
            let new_offset_u8 = u8::try_from(new_offset).unwrap_or(u8::MAX);

            let mut ui = UI_STATE.lock().await;
            ui.mode = UiMode::SystemInfo {
                scroll_offset: new_offset_u8,
            };
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
        }
        UiMode::RunningTest | UiMode::Calibrating { .. } => {}
    }
}

/// Handle rotary encoder button press.
pub async fn handle_rotary_button_pressed() {
    if !ui_initialized().await {
        return;
    }

    let ui_snapshot = {
        let ui = UI_STATE.lock().await;
        *ui
    };

    match ui_snapshot.mode {
        UiMode::MainMenu => match menu_selection_from_index(ui_snapshot.main_index) {
            crate::system::state::MenuSelection::SystemInfo => {
                let mut ui = UI_STATE.lock().await;
                ui.mode = UiMode::SystemInfo { scroll_offset: 0 };
                let snapshot = *ui;
                drop(ui);
                render_current_ui(&snapshot).await;
            }
            crate::system::state::MenuSelection::Calibrate => {
                let mut ui = UI_STATE.lock().await;
                ui.calibrate_index = 0;
                ui.mode = UiMode::CalibrateMenu;
                let snapshot = *ui;
                drop(ui);
                render_current_ui(&snapshot).await;
            }
            crate::system::state::MenuSelection::TestMode => {
                let mut ui = UI_STATE.lock().await;
                ui.mode = UiMode::RunningTest;
                let snapshot = *ui;
                drop(ui);
                render_current_ui(&snapshot).await;
                testmode::start_testing_sequence();
            }
        },
        UiMode::SystemInfo { .. } => {
            show_main_menu().await;
        }
        UiMode::CalibrateMenu => {
            let selection = calibration_selection_from_index(ui_snapshot.calibrate_index);

            let mut ui = UI_STATE.lock().await;
            ui.mode = UiMode::Calibrating { kind: selection };
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;

            match selection {
                CalibrationSelection::Motor => {
                    drive::send_drive_command(drive::DriveCommand::RunMotorCalibration);
                }
                CalibrationSelection::Mag => {
                    drive::send_drive_command(drive::DriveCommand::RunImuCalibration(drive::ImuCalibrationKind::Mag));
                }
                CalibrationSelection::Accel => {
                    drive::send_drive_command(drive::DriveCommand::RunImuCalibration(drive::ImuCalibrationKind::Accel));
                }
                CalibrationSelection::Gyro => {
                    drive::send_drive_command(drive::DriveCommand::RunImuCalibration(drive::ImuCalibrationKind::Gyro));
                }
            }
        }
        UiMode::RunningTest | UiMode::Calibrating { .. } => {}
    }
}

/// Handle rotary encoder button hold start.
pub async fn handle_rotary_button_hold_start() {
    if !ui_initialized().await {
        return;
    }

    handle_ui_back().await;
}

/// Handle rotary encoder button hold end.
pub const fn handle_rotary_button_hold_end() {
    // No-op for now.
}

/// Handle testing completion by returning to the main menu.
pub async fn handle_testing_completed() {
    show_main_menu().await;
}

/// Handle a UI back action based on the current mode.
pub async fn handle_ui_back() {
    let mode = {
        let ui = UI_STATE.lock().await;
        ui.mode
    };

    match mode {
        UiMode::SystemInfo { .. } | UiMode::CalibrateMenu => {
            show_main_menu().await;
        }
        _ => {}
    }
}

/// Set UI state to main menu and render it.
pub async fn show_main_menu() {
    let mut ui = UI_STATE.lock().await;
    ui.mode = UiMode::MainMenu;
    let snapshot = *ui;
    drop(ui);
    render_current_ui(&snapshot).await;
}

/// Refresh the current UI view by re-rendering the latest state.
pub async fn refresh() {
    let ui = UI_STATE.lock().await;
    let snapshot = *ui;
    drop(ui);
    render_current_ui(&snapshot).await;
}
