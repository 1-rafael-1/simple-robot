//! UI rendering helpers.
//!
//! Renders UI screens based on the current UI state and system data.

use core::fmt::Write;

use heapless::String;

use crate::{
    system::state::{CalibrationSelection, UiMode, SYSTEM_STATE},
    task::io::display::{self, DisplayAction},
};

use super::{state::UiState, ui_menu};

/// Render the current UI view based on the UI state.
pub async fn render_current_ui(state: &UiState) {
    match state.mode {
        UiMode::MainMenu => ui_menu::render_main_menu(state.main_index).await,
        UiMode::CalibrateMenu => ui_menu::render_calibrate_menu(state.calibrate_index).await,
        UiMode::SystemInfo { scroll_offset } => {
            let info = build_system_info_data().await;
            ui_menu::render_system_info(scroll_offset as usize, &info).await;
        }
        UiMode::RunningTest => {
            render_test_running().await;
        }
        UiMode::Calibrating { kind } => {
            render_calibrating(kind).await;
        }
    }
}

/// Render the "test running" status screen.
pub async fn render_test_running() {
    display::display_update(DisplayAction::Clear).await;
    show_line(0, "Test Mode").await;
    show_line(1, "Running...").await;
    show_line(2, "").await;
    show_line(3, "").await;
}

/// Render the calibration-in-progress screen for the selected kind.
pub async fn render_calibrating(kind: CalibrationSelection) {
    display::display_update(DisplayAction::Clear).await;
    show_line(0, "Calibrating").await;
    show_line(1, super::menu::calibration_label(kind)).await;
    show_line(2, "").await;
    show_line(3, "").await;
}

/// Write a single line of text to the display.
pub async fn show_line(line: u8, msg: &str) {
    let mut s: String<20> = String::new();
    for ch in msg.chars() {
        if s.push(ch).is_err() {
            break;
        }
    }
    display::display_update(DisplayAction::ShowText(s, line)).await;
}

/// Build a snapshot of system info for the UI renderer.
pub async fn build_system_info_data() -> ui_menu::SystemInfoData {
    let state = SYSTEM_STATE.lock().await;
    ui_menu::SystemInfoData {
        battery_level: state.battery_level,
        battery_voltage: state.battery_voltage,
        motor_calibration_status: state.motor_calibration_status,
        mag_calibration_status: state.mag_calibration_status,
        accel_calibration_status: state.accel_calibration_status,
        gyro_calibration_status: state.gyro_calibration_status,
    }
}
