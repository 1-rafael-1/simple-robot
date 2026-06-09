//! UI controller module.
//!
//! Owns UI state, user interactions, and view rendering logic.

use crate::{
    system::{
        event::RotaryDirection,
        state::{CalibrationSelection, DriveMode, TestSelection, calibration, perception},
    },
    task::{
        autonomous_mode, drive,
        io::display::{DisplayAction, display_update},
        io::flash_storage,
        testmode,
    },
};

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};

pub mod menu;
pub mod render;
pub mod screens;
pub mod state;

use menu::{calibration_selection_from_index, menu_selection_from_index, next_menu_index, test_selection_from_index};
use render::{render_current_ui, show_line};
use state::{UI_STATE, UiMode};

/// Channel for requesting distance calibration drive from the controller task.
static DIST_CAL_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

/// Initialise the UI calibration controller (spawned once at boot).
#[allow(clippy::unwrap_used)]
pub fn init_ui(spawner: Spawner) {
    spawner.spawn(calibration_controller(spawner).unwrap());
}

/// Controller task: waits for distance calibration requests and spawns the drive.
#[embassy_executor::task]
#[allow(clippy::unwrap_used)]
async fn calibration_controller(spawner: Spawner) {
    loop {
        DIST_CAL_CHANNEL.receive().await;
        spawner.spawn(calibration_drive_task().unwrap());
    }
}

/// Returns true once calibration data has been queried.
pub async fn ui_initialized() -> bool {
    calibration::is_initialized().await
}

/// Returns true if the UI is currently showing a calibration flow.
pub async fn ui_is_calibrating() -> bool {
    let ui = UI_STATE.lock().await;
    matches!(ui.mode, UiMode::Calibrating { .. })
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
            ui.main_index = next_menu_index(ui.main_index, screens::MAIN_MENU_ITEMS.len(), direction);
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
        }
        UiMode::CalibrateMenu => {
            let mut ui = UI_STATE.lock().await;
            ui.calibrate_index = next_menu_index(ui.calibrate_index, screens::CALIBRATE_MENU_ITEMS.len(), direction);
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
        }
        UiMode::DriveModeMenu => {
            let mut ui = UI_STATE.lock().await;
            ui.drive_mode_index = next_menu_index(ui.drive_mode_index, screens::DRIVE_MODE_MENU_ITEMS.len(), direction);
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
        }
        UiMode::TestMenu => {
            let mut ui = UI_STATE.lock().await;
            ui.test_index = next_menu_index(ui.test_index, screens::TEST_MENU_ITEMS.len(), direction);
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
        }
        UiMode::SystemInfo { scroll_offset } => {
            let info = render::build_system_info_data().await;
            let max_scroll = menu::max_system_info_scroll(&info);
            let new_offset = match direction {
                RotaryDirection::Clockwise => (scroll_offset as usize).saturating_sub(1),
                RotaryDirection::CounterClockwise => (scroll_offset as usize + 1).min(max_scroll),
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
        UiMode::RunningTurnsTest
        | UiMode::RunningStraightDriveTest
        | UiMode::RunningArcDriveTest
        | UiMode::RunningImuTest
        | UiMode::RunningImu6Test
        | UiMode::RunningBasicMotorTest
        | UiMode::RunningIrUltrasonicTest
        | UiMode::RunningUltrasonicSweepTest
        | UiMode::RunningAutonomous { .. }
        | UiMode::Calibrating { .. } => {}
        UiMode::EnteringDistance { value } => {
            let new_value = match direction {
                RotaryDirection::Clockwise => value.saturating_sub(1),
                RotaryDirection::CounterClockwise => (value + 1).min(200),
            };
            let mut ui = UI_STATE.lock().await;
            // Re-read to avoid TOCTOU — value may have changed.
            if let UiMode::EnteringDistance { value: current } = &mut ui.mode {
                *current = new_value;
            }
            drop(ui);
            render_entering_distance(new_value).await;
        }
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
        UiMode::MainMenu => handle_main_menu_press(ui_snapshot.main_index).await,
        UiMode::SystemInfo { .. } => show_main_menu().await,
        UiMode::CalibrateMenu => handle_calibrate_menu_press(ui_snapshot.calibrate_index).await,
        UiMode::DriveModeMenu => handle_drive_mode_menu_press(ui_snapshot.drive_mode_index).await,
        UiMode::TestMenu => handle_test_menu_press(ui_snapshot.test_index).await,
        UiMode::Calibrating { .. } => {
            if ui_snapshot.calibration_complete {
                show_main_menu().await;
            }
        }
        UiMode::RunningImuTest => handle_running_imu_test_press().await,
        UiMode::RunningImu6Test => handle_running_imu6_test_press().await,
        UiMode::RunningBasicMotorTest => handle_running_basic_motor_test_press().await,
        UiMode::RunningIrUltrasonicTest => handle_running_ir_ultrasonic_test_press().await,
        UiMode::RunningUltrasonicSweepTest => handle_running_ultrasonic_sweep_test_press().await,
        UiMode::RunningAutonomous { .. } => handle_ui_back().await,
        UiMode::RunningTurnsTest | UiMode::RunningStraightDriveTest | UiMode::RunningArcDriveTest => {}
        UiMode::EnteringDistance { value } => handle_distance_entry_press(value).await,
    }
}

/// Handle rotary encoder button hold start.
pub async fn handle_rotary_button_hold_start() {
    if !ui_initialized().await {
        return;
    }

    let mode = {
        let ui = UI_STATE.lock().await;
        ui.mode
    };

    if matches!(mode, UiMode::RunningAutonomous { .. }) {
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

/// Handle calibration completion by enabling exit via button press.
pub async fn handle_calibration_completed() {
    let mut ui = UI_STATE.lock().await;
    ui.calibration_complete = true;
}

/// Handle a UI back action based on the current mode.
pub async fn handle_ui_back() {
    let mode = {
        let ui = UI_STATE.lock().await;
        ui.mode
    };

    match mode {
        UiMode::SystemInfo { .. } | UiMode::CalibrateMenu | UiMode::DriveModeMenu | UiMode::TestMenu => {
            show_main_menu().await;
        }
        UiMode::RunningImuTest => {
            testmode::stop_imu_test_mode();
            show_test_menu().await;
        }
        UiMode::RunningImu6Test => {
            testmode::stop_imu6_test_mode();
            show_test_menu().await;
        }
        UiMode::RunningBasicMotorTest => {
            testmode::stop_basic_motor_test_mode();
            show_test_menu().await;
        }
        UiMode::RunningIrUltrasonicTest => {
            testmode::stop_ir_ultrasonic_test_mode();
            show_test_menu().await;
        }
        UiMode::RunningUltrasonicSweepTest => {
            testmode::stop_ultrasonic_sweep_test_mode();
            show_test_menu().await;
        }
        UiMode::RunningAutonomous { mode } => {
            match mode {
                DriveMode::CoastAndAvoid => {
                    autonomous_mode::coast_obstacle_avoid::stop();
                }
            }
            show_main_menu().await;
        }
        _ => {}
    }
}

/// Handle a button press while the main menu is active.
async fn handle_main_menu_press(index: usize) {
    match menu_selection_from_index(index) {
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
        crate::system::state::MenuSelection::DriveMode => {
            let mut ui = UI_STATE.lock().await;
            ui.drive_mode_index = 0;
            ui.mode = UiMode::DriveModeMenu;
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
        }
        crate::system::state::MenuSelection::TestMode => {
            let mut ui = UI_STATE.lock().await;
            ui.test_index = 0;
            ui.mode = UiMode::TestMenu;
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
        }
    }
}

/// Handle a button press while the calibration menu is active.
async fn handle_calibrate_menu_press(index: usize) {
    if let Some(selection) = calibration_selection_from_index(index) {
        let mut ui = UI_STATE.lock().await;
        ui.mode = UiMode::Calibrating { kind: selection };
        ui.calibration_complete = false;
        let snapshot = *ui;
        drop(ui);
        render_current_ui(&snapshot).await;

        match selection {
            CalibrationSelection::Motor => {
                drive::send_drive_command(drive::DriveCommand::RunMotorCalibration).await;
            }
            CalibrationSelection::Mag => {
                drive::send_drive_command(drive::DriveCommand::RunImuCalibration(drive::ImuCalibrationKind::Mag)).await;
            }
            CalibrationSelection::Distance => {
                run_distance_calibration().await;
            }
        }
    } else {
        show_main_menu().await;
    }
}

/// Handle a button press while the drive mode menu is active.
async fn handle_drive_mode_menu_press(index: usize) {
    if let Some(mode) = menu::drive_mode_from_index(index) {
        crate::task::behavior::obstacle::reset_obstacle_state().await;
        {
            let mut state = perception::PERCEPTION_STATE.lock().await;
            state.ultrasonic_reading = None;
            state.ultrasonic_angle_deg = None;
        }
        let mut ui = UI_STATE.lock().await;
        ui.mode = UiMode::RunningAutonomous { mode };
        let snapshot = *ui;
        drop(ui);
        render_current_ui(&snapshot).await;

        match mode {
            DriveMode::CoastAndAvoid => {
                autonomous_mode::coast_obstacle_avoid::start().await;
            }
        }
    } else {
        show_main_menu().await;
    }
}

/// Handle a button press while the test menu is active.
async fn handle_test_menu_press(index: usize) {
    match test_selection_from_index(index) {
        Some(TestSelection::Turns) => {
            let mut ui = UI_STATE.lock().await;
            ui.mode = UiMode::RunningTurnsTest;
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
            testmode::start_turns_test().await;
        }
        Some(TestSelection::StraightDrive) => {
            let mut ui = UI_STATE.lock().await;
            ui.mode = UiMode::RunningStraightDriveTest;
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
            testmode::start_straight_drive_test().await;
        }
        Some(TestSelection::ArcDrive) => {
            let mut ui = UI_STATE.lock().await;
            ui.mode = UiMode::RunningArcDriveTest;
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
            testmode::start_arc_drive_test().await;
        }
        Some(TestSelection::Imu) => {
            let mut ui = UI_STATE.lock().await;
            ui.mode = UiMode::RunningImuTest;
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
            testmode::start_imu_test_mode().await;
        }
        Some(TestSelection::Imu6) => {
            let mut ui = UI_STATE.lock().await;
            ui.mode = UiMode::RunningImu6Test;
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
            testmode::start_imu6_test_mode().await;
        }
        Some(TestSelection::BasicMotor) => {
            let mut ui = UI_STATE.lock().await;
            ui.mode = UiMode::RunningBasicMotorTest;
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
            testmode::start_basic_motor_test_mode().await;
        }
        Some(TestSelection::IrUltrasonic) => {
            crate::task::behavior::obstacle::reset_obstacle_state().await;
            {
                let mut state = perception::PERCEPTION_STATE.lock().await;
                state.ultrasonic_reading = None;
                state.ultrasonic_angle_deg = None;
            }
            let mut ui = UI_STATE.lock().await;
            ui.mode = UiMode::RunningIrUltrasonicTest;
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
            testmode::start_ir_ultrasonic_test_mode().await;
        }
        Some(TestSelection::UltrasonicSweep) => {
            let mut ui = UI_STATE.lock().await;
            ui.mode = UiMode::RunningUltrasonicSweepTest;
            let snapshot = *ui;
            drop(ui);
            render_current_ui(&snapshot).await;
            testmode::start_ultrasonic_sweep_test_mode().await;
        }
        None => {
            show_main_menu().await;
        }
    }
}

/// Handle a button press while the IMU test mode is active.
async fn handle_running_imu_test_press() {
    testmode::stop_imu_test_mode();
    show_test_menu().await;
}

/// Handle a button press while the IMU 6-axis test mode is active.
async fn handle_running_imu6_test_press() {
    testmode::stop_imu6_test_mode();
    show_test_menu().await;
}

/// Handle a button press while the basic motor test mode is active.
async fn handle_running_basic_motor_test_press() {
    testmode::stop_basic_motor_test_mode();
    show_test_menu().await;
}

/// Handle a button press while the IR + ultrasonic test mode is active.
async fn handle_running_ir_ultrasonic_test_press() {
    testmode::stop_ir_ultrasonic_test_mode();
    show_test_menu().await;
}

/// Handle a button press while the ultrasonic sweep test mode is active.
async fn handle_running_ultrasonic_sweep_test_press() {
    testmode::stop_ultrasonic_sweep_test_mode();
    show_test_menu().await;
}

/// Set UI state to test menu and render it.
pub async fn show_test_menu() {
    let mut ui = UI_STATE.lock().await;
    ui.mode = UiMode::TestMenu;
    let snapshot = *ui;
    drop(ui);
    render_current_ui(&snapshot).await;
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

// ── Distance calibration flow ────────────────────────────────────────────────────

/// Handle a button press while the distance entry screen is active.
async fn handle_distance_entry_press(value: u8) {
    if value == 0 {
        // Cancel — don't save, return to main menu.
        show_main_menu().await;
        return;
    }

    let factor = (150.0 / f32::from(value)).clamp(
        flash_storage::DistanceCalibration::MIN_FACTOR,
        flash_storage::DistanceCalibration::MAX_FACTOR,
    );

    flash_storage::set_distance_factor(factor).await;

    // Show confirmation.
    display_update(DisplayAction::Clear).await;
    show_line(0, "Distance Cal").await;
    {
        let mut s: heapless::String<20> = heapless::String::new();
        let _ = core::fmt::write(&mut s, format_args!("Factor: {factor:.2}"));
        display_update(DisplayAction::ShowText(s, 1)).await;
    }
    show_line(2, "Saved").await;
    show_line(3, "").await;

    Timer::after(Duration::from_secs(2)).await;
    show_main_menu().await;
}

/// Run the distance calibration procedure: countdown → auto-drive → entry screen.
///
/// The countdown runs in the orchestrator context (yields via `Timer::after`).
/// The drive is spawned as a separate task so the orchestrator stays free to
/// forward encoder and IMU events to the drive control loop.
async fn run_distance_calibration() {
    // Countdown.
    display_update(DisplayAction::Clear).await;
    show_line(0, "Distance Cal").await;
    show_line(1, "Driving 150cm").await;
    for sec in (1u8..=3).rev() {
        let mut s: heapless::String<20> = heapless::String::new();
        let _ = core::fmt::write(&mut s, format_args!("in {sec}..."));
        show_line(2, &s).await;
        Timer::after(Duration::from_secs(1)).await;
    }

    show_line(2, "Driving...").await;
    show_line(3, "").await;

    // Reset to uncalibrated factor 1.0 for the calibration baseline drive.
    let saved_factor = flash_storage::get_distance_factor().await;
    if (saved_factor - 1.0).abs() > f32::EPSILON {
        flash_storage::set_distance_factor(1.0).await;
    }

    // Request the controller to spawn the drive task (non-blocking send).
    DIST_CAL_CHANNEL.send(()).await;
}

/// Spawned task: submit the 150cm drive, wait for completion, then transition
/// to the distance entry screen.
#[embassy_executor::task]
async fn calibration_drive_task() {
    use crate::task::drive::{DriveAction, DriveCommand, DriveDirection, DriveDistanceKind, DriveQueueBuilder};

    let mut queue = DriveQueueBuilder::new();

    if queue
        .push_abort_on_fail(DriveCommand::Drive(DriveAction::DriveDistance {
            kind: DriveDistanceKind::Straight { distance_cm: 150.0 },
            direction: DriveDirection::Forward,
            speed: 70,
        }))
        .is_err()
    {
        show_line(2, "Queue full").await;
        Timer::after(Duration::from_secs(2)).await;
        show_main_menu().await;
        return;
    }

    if queue
        .push_abort_on_fail(DriveCommand::Drive(DriveAction::Brake))
        .is_err()
    {
        show_line(2, "Queue full").await;
        Timer::after(Duration::from_secs(2)).await;
        show_main_menu().await;
        return;
    }

    // Let the robot settle after braking, then release motors.
    let _ = queue.push(DriveCommand::Drive(DriveAction::Coast));

    let _completion = queue.submit().await;

    // Transition to entry screen.
    let mut ui = UI_STATE.lock().await;
    ui.mode = UiMode::EnteringDistance { value: 150 };
    drop(ui);
    render_entering_distance(150).await;
}

/// Render the distance entry screen with the current entered value.
async fn render_entering_distance(value: u8) {
    display_update(DisplayAction::Clear).await;
    show_line(0, "Enter distance:").await;
    {
        let mut s: heapless::String<20> = heapless::String::new();
        let _ = core::fmt::write(&mut s, format_args!("  {value} cm"));
        display_update(DisplayAction::ShowText(s, 1)).await;
    }
    show_line(2, "Turn to adj").await;
    show_line(3, "Press to save").await;
}
