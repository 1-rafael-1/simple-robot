//! UI rendering helpers.
//!
//! Renders UI screens based on the current UI state and system data.

use heapless::String;

use super::{
    screens,
    state::{UiMode, UiState},
};
use crate::{
    system::{
        event::UltrasonicReading,
        state::{CalibrationSelection, DriveMode, calibration, perception, power},
    },
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
        UiMode::RunningTurnsTest => {
            render_turns_test_running().await;
        }
        UiMode::RunningStraightDriveTest => {
            render_straight_drive_test_running().await;
        }
        UiMode::RunningArcDriveTest => {
            render_arc_drive_test_running().await;
        }
        UiMode::RunningImuTest => {
            render_imu_test_running().await;
        }
        UiMode::RunningImu6Test => {
            render_imu6_test_running().await;
        }
        UiMode::RunningBasicMotorTest => {
            render_basic_motor_test_running().await;
        }
        UiMode::RunningIrUltrasonicTest => {
            render_ir_ultrasonic_test_running().await;
        }
        UiMode::RunningUltrasonicSweepTest => {
            render_ultrasonic_sweep_test_running().await;
        }
        UiMode::RunningAutonomous { mode } => {
            render_autonomous_running(mode).await;
        }
        UiMode::Calibrating { kind } => {
            render_calibrating(kind).await;
        }
        UiMode::EnteringDistance { .. } => {
            // Rendering handled by the rotary-turn handler — this arm exists
            // for exhaustiveness but should not be reached via render_current_ui.
        }
    }
}

/// Render the turns test initial status screen.
pub async fn render_turns_test_running() {
    display::display_update(DisplayAction::Clear).await;
    show_line(0, "TURN TEST").await;
    show_line(1, "Starting...").await;
    show_line(2, "").await;
    show_line(3, "").await;
}

/// Render the straight drive test initial status screen.
pub async fn render_straight_drive_test_running() {
    display::display_update(DisplayAction::Clear).await;
    show_line(0, "DIST TEST").await;
    show_line(1, "Starting...").await;
    show_line(2, "").await;
    show_line(3, "").await;
}

/// Render the arc drive test initial status screen.
pub async fn render_arc_drive_test_running() {
    display::display_update(DisplayAction::Clear).await;
    show_line(0, "ARC TEST").await;
    show_line(1, "Starting...").await;
    show_line(2, "").await;
    show_line(3, "").await;
}

/// Render the IMU test placeholder screen.
pub async fn render_imu_test_running() {
    display::display_update(DisplayAction::Clear).await;
    show_line(0, "IMU Test 9-axis").await;
    show_line(1, "Euler angles").await;
    show_line(2, "Streaming...").await;
    show_line(3, "Press to exit").await;
}

/// Render the IMU 6-axis test placeholder screen.
pub async fn render_imu6_test_running() {
    display::display_update(DisplayAction::Clear).await;
    show_line(0, "IMU Test 6-axis").await;
    show_line(1, "Euler angles").await;
    show_line(2, "Streaming...").await;
    show_line(3, "Press to exit").await;
}

/// Render the basic motor test placeholder screen.
pub async fn render_basic_motor_test_running() {
    display::display_update(DisplayAction::Clear).await;
    show_line(0, "Basic Motor Test").await;
    show_line(1, "Motor: ----").await;
    show_line(2, "ENC: ------").await;
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

/// Render the ultrasonic sweep test placeholder screen.
pub async fn render_ultrasonic_sweep_test_running() {
    display::display_update(DisplayAction::Clear).await;
    show_line(0, "US Sweep: Press").await;
}

/// Render the autonomous drive mode running screen.
pub async fn render_autonomous_running(mode: DriveMode) {
    let ir_detected = perception::is_ir_obstacle_detected();
    let (ultrasonic_reading, ultrasonic_angle) = perception::ultrasonic_sweep_snapshot().await;
    let obstacle_detected = perception::is_obstacle_detected();

    let mode_label = match mode {
        DriveMode::CoastAndAvoid => "Coast & Avoid",
    };

    let mut rows: [String<20>; 4] = core::array::from_fn(|_| String::new());
    let _ = rows[0].push_str(mode_label);

    let ir_label = if ir_detected { "IR: detect" } else { "IR: clear" };
    let _ = rows[1].push_str(ir_label);

    match (ultrasonic_reading, ultrasonic_angle) {
        (Some(UltrasonicReading::Distance(cm)), Some(a)) => {
            let _ = core::fmt::write(&mut rows[2], format_args!("US:{cm:>5.1}cm @{a:>3.0}"));
        }
        (Some(UltrasonicReading::Timeout), Some(a)) => {
            let _ = core::fmt::write(&mut rows[2], format_args!("US:timeout @{a:>3.0}"));
        }
        (Some(UltrasonicReading::Error), _) => {
            let _ = rows[2].push_str("US: error");
        }
        _ => {
            let _ = rows[2].push_str("US: ----");
        }
    }

    let obs_label = if obstacle_detected { "OBS: YES" } else { "OBS: NO" };
    let _ = rows[3].push_str(obs_label);

    if !display::display_try_update(DisplayAction::ShowLines(rows.clone())) {
        display::display_update(DisplayAction::ShowLines(rows)).await;
    }
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
    // Read Power first (via accessors) to preserve lock order: Power → Calibration.
    let battery = power::get_battery_snapshot().await;
    let calibration_state = calibration::CALIBRATION_STATE.lock().await;
    screens::SystemInfoData {
        battery_level: battery.level,
        battery_voltage: battery.voltage,
        motor_calibration_status: calibration_state.motor_calibration_status,
        mag_calibration_status: calibration_state.mag_calibration_status,
        distance_calibration_status: calibration_state.distance_calibration_status,
    }
}
