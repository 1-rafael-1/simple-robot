//! UI rendering helpers.
//!
//! Renders UI screens based on the current UI state and system data.

use heapless::String;

use super::{screens, state::UiState};
use crate::{
    system::state::{CalibrationSelection, DriveMode, SYSTEM_STATE, UiMode},
    task::io::display::{self, DisplayAction},
};

/// Render the current UI view based on the UI state.
pub async fn render_current_ui(state: &UiState) {
    match state.mode {
        UiMode::MainMenu => screens::render_main_menu(state.main_index).await,
        UiMode::CalibrateMenu => screens::render_calibrate_menu(state.calibrate_index).await,
        UiMode::DriveModeMenu => screens::render_drive_mode_menu(state.drive_mode_index).await,
        UiMode::TestMenu => screens::render_test_menu(state.test_index).await,
        UiMode::SystemInfo { scroll_offset } => {
            let info = build_system_info_data().await;
            screens::render_system_info(scroll_offset as usize, &info).await;
        }
        UiMode::RunningTest => {
            render_test_running().await;
        }
        UiMode::RunningImuTest => {
            render_imu_test_running().await;
        }
        UiMode::RunningIrUltrasonicTest => {
            render_ir_ultrasonic_test_running().await;
        }
        UiMode::RunningAutonomous { mode } => {
            render_autonomous_running(mode).await;
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

/// Render the IMU test placeholder screen.
pub async fn render_imu_test_running() {
    display::display_update(DisplayAction::Clear).await;
    show_line(0, "IMU Test").await;
    show_line(1, "Euler angles").await;
    show_line(2, "Streaming...").await;
    show_line(3, "Press to exit").await;
}

/// Render the IR + ultrasonic test placeholder screen.
pub async fn render_ir_ultrasonic_test_running() {
    display::display_update(DisplayAction::Clear).await;
    show_line(0, "IR+US Test").await;
    show_line(1, "IR: ----").await;
    show_line(2, "US: ---- cm").await;
    show_line(3, "Press to exit").await;
}

/// Render the autonomous drive mode running screen.
pub async fn render_autonomous_running(mode: DriveMode) {
    display::display_update(DisplayAction::Clear).await;
    show_line(0, "Drive Mode").await;
    let mode_label = match mode {
        DriveMode::CoastAndAvoid => "Coast & Avoid",
    };
    show_line(1, mode_label).await;
    show_line(2, "Running...").await;
    show_line(3, "Hold to stop").await;
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
pub async fn build_system_info_data() -> screens::SystemInfoData {
    let state = SYSTEM_STATE.lock().await;
    screens::SystemInfoData {
        battery_level: state.battery_level,
        battery_voltage: state.battery_voltage,
        motor_calibration_status: state.motor_calibration_status,
        mag_calibration_status: state.mag_calibration_status,
        accel_calibration_status: state.accel_calibration_status,
        gyro_calibration_status: state.gyro_calibration_status,
    }
}
