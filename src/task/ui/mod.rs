//! UI controller module.
//!
//! Owns UI state, user interactions, and view rendering logic.
//!
//! # Architecture
//!
//! The [`ui_controller_task`] owns most UI rendering. It selects over two sources:
//! 1. **`UiEvent` channel** — rotary encoder input, lifecycle events (testing/calibration
//!    completed, show-main-menu requests). Sent by the orchestrator and initialization.
//! 2. **15 Hz timer** — drives autonomous-mode refresh by reading perception state
//!    and re-rendering only when the displayed values change.
//!
//! Test modes and calibration flows spawn their own display tasks and render
//! directly, bypassing the controller.

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Ticker, Timer};

use crate::{
    system::{
        event::{RotaryDirection, UltrasonicReading},
        state::{CalibrationSelection, DriveMode, TestSelection, calibration, perception},
    },
    task::{
        autonomous_mode, drive,
        io::{
            display::{DisplayAction, display_update},
            flash_storage,
        },
        testmode,
    },
};

pub mod menu;
pub mod render;
pub mod screens;
pub mod state;

use menu::{calibration_selection_from_index, menu_selection_from_index, next_menu_index, test_selection_from_index};
use render::{render_autonomous_running_from_values, render_current_ui, show_line};
use state::{UI_STATE, UiMode, UiState};

// ── UI event channel ────────────────────────────────────────────────────────────

/// Events delivered to the UI controller task from the orchestrator and initialisation.
#[derive(Debug, Clone, Copy)]
pub enum UiEvent {
    /// Rotary encoder turned (clockwise / counter-clockwise).
    RotaryTurned(RotaryDirection),
    /// Rotary encoder button short press.
    RotaryButtonPressed,
    /// Rotary encoder button hold started.
    RotaryButtonHoldStart,
    /// Rotary encoder button hold ended.
    RotaryButtonHoldEnd,
    /// Testing sequence finished — show main menu.
    TestingCompleted,
    /// Calibration procedure finished — enable exit.
    CalibrationCompleted,
    /// Request to show the main menu (from initialisation).
    ShowMainMenu,
}

/// Channel carrying [`UiEvent`]s into the UI controller task.
/// Capacity 64 ensures the orchestrator never blocks on UI delivery.
static UI_EVENT_CHANNEL: Channel<CriticalSectionRawMutex, UiEvent, 64> = Channel::new();

/// Send an event to the UI controller task.
pub async fn send_ui_event(event: UiEvent) {
    UI_EVENT_CHANNEL.sender().send(event).await;
}

/// 15 Hz refresh interval for autonomous-mode re-rendering (ms).
const AUTONOMOUS_REFRESH_INTERVAL_MS: u64 = 67;

// ── Distance calibration channel ─────────────────────────────────────────────────

/// Channel for requesting distance calibration drive from the controller task.
static DIST_CAL_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

/// Stores the distance factor from before calibration started, so it can be
/// restored if the calibration is cancelled or fails before saving.
static PREVIOUS_DISTANCE_FACTOR: embassy_sync::mutex::Mutex<CriticalSectionRawMutex, Option<f32>> =
    embassy_sync::mutex::Mutex::new(None);

/// Initialise the UI (spawns the controller task and calibration controller).
#[allow(clippy::unwrap_used)]
pub fn init_ui(spawner: Spawner) {
    spawner.spawn(calibration_controller(spawner).unwrap());
    spawner.spawn(ui_controller_task().unwrap());
}

// ── UI controller task ───────────────────────────────────────────────────────────

/// Main UI task — dispatches [`UiEvent`]s and runs the 15 Hz autonomous refresh loop.
#[embassy_executor::task]
async fn ui_controller_task() {
    let mut ticker = Ticker::every(Duration::from_millis(AUTONOMOUS_REFRESH_INTERVAL_MS));
    let mut last_perception: Option<LastPerceptionState> = None;
    loop {
        match select(UI_EVENT_CHANNEL.receiver().receive(), ticker.next()).await {
            Either::First(event) => {
                dispatch_ui_event(event).await;
            }
            Either::Second(()) => {
                autonomous_refresh_tick(&mut last_perception).await;
            }
        }
    }
}

/// Route a [`UiEvent`] to the appropriate handler.
async fn dispatch_ui_event(event: UiEvent) {
    match event {
        UiEvent::RotaryTurned(direction) => {
            if !ui_initialized().await {
                return;
            }
            handle_rotary_turned(direction).await;
        }
        UiEvent::RotaryButtonPressed => {
            if !ui_initialized().await {
                return;
            }
            handle_rotary_button_pressed().await;
        }
        UiEvent::RotaryButtonHoldStart => {
            if !ui_initialized().await {
                return;
            }
            handle_rotary_button_hold_start().await;
        }
        UiEvent::RotaryButtonHoldEnd => handle_rotary_button_hold_end(),
        UiEvent::TestingCompleted | UiEvent::ShowMainMenu => show_main_menu().await,
        UiEvent::CalibrationCompleted => handle_calibration_completed().await,
    }
}

/// Perception values relevant to the autonomous running display.
struct LastPerceptionState {
    /// Whether the IR sensor reports an obstacle.
    ir_detected: bool,
    /// Whether any sensor reports an obstacle.
    obstacle_detected: bool,
    /// Latest ultrasonic distance reading, if available.
    ultrasonic_reading: Option<UltrasonicReading>,
    /// Latest ultrasonic servo angle, if available.
    ultrasonic_angle: Option<f32>,
}

/// 15 Hz tick: re-render the autonomous screen only when displayed perception values change.
async fn autonomous_refresh_tick(last: &mut Option<LastPerceptionState>) {
    let snapshot = {
        let ui = UI_STATE.lock().await;
        *ui
    };

    if !matches!(snapshot.mode, UiMode::RunningAutonomous { .. }) {
        return;
    }

    let ir = perception::is_ir_obstacle_detected();
    let obs = perception::is_obstacle_detected();
    let (us_reading, us_angle) = perception::ultrasonic_sweep_snapshot().await;

    if let Some(prev) = last
        && ir == prev.ir_detected
        && obs == prev.obstacle_detected
        && us_reading == prev.ultrasonic_reading
        && us_angle == prev.ultrasonic_angle
    {
        return;
    }

    *last = Some(LastPerceptionState {
        ir_detected: ir,
        obstacle_detected: obs,
        ultrasonic_reading: us_reading,
        ultrasonic_angle: us_angle,
    });

    // Extract mode from snapshot — we already confirmed RunningAutonomous above.
    let UiMode::RunningAutonomous { mode } = snapshot.mode else {
        return;
    };
    render_autonomous_running_from_values(mode, ir, obs, us_reading, us_angle).await;
}

// ── Calibration controller ───────────────────────────────────────────────────────

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

// ── Private handlers ─────────────────────────────────────────────────────────────

/// Handle rotary encoder turns.
async fn handle_rotary_turned(direction: RotaryDirection) {
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
        | UiMode::RunningCoastAvoidDetectionTest
        | UiMode::RunningAutonomous { .. }
        | UiMode::Calibrating { .. } => {}
        UiMode::EnteringDistance { value } => {
            let new_value = match direction {
                RotaryDirection::Clockwise => value.saturating_sub(1),
                RotaryDirection::CounterClockwise => (value + 1).min(200),
            };
            let mut ui = UI_STATE.lock().await;
            if let UiMode::EnteringDistance { value: current } = &mut ui.mode {
                *current = new_value;
            }
            drop(ui);
            render_entering_distance(new_value).await;
        }
    }
}

/// Handle rotary encoder button press.
async fn handle_rotary_button_pressed() {
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
        UiMode::RunningCoastAvoidDetectionTest => handle_running_coast_avoid_detection_test_press().await,
        UiMode::RunningAutonomous { .. } => handle_ui_back().await,
        UiMode::RunningTurnsTest | UiMode::RunningStraightDriveTest | UiMode::RunningArcDriveTest => {}
        UiMode::EnteringDistance { value } => handle_distance_entry_press(value).await,
    }
}

/// Handle rotary encoder button hold start.
async fn handle_rotary_button_hold_start() {
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
const fn handle_rotary_button_hold_end() {
    // No-op for now.
}

/// Handle calibration completion by enabling exit via button press.
async fn handle_calibration_completed() {
    let mut ui = UI_STATE.lock().await;
    ui.calibration_complete = true;
}

/// Handle a UI back action based on the current mode.
async fn handle_ui_back() {
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
        UiMode::RunningCoastAvoidDetectionTest => {
            testmode::stop_coast_avoid_detection_test();
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
            set_mode(UiMode::SystemInfo { scroll_offset: 0 }).await;
        }
        crate::system::state::MenuSelection::Calibrate => {
            set_mode_mutated(UiMode::CalibrateMenu, |ui| ui.calibrate_index = 0).await;
        }
        crate::system::state::MenuSelection::DriveMode => {
            set_mode_mutated(UiMode::DriveModeMenu, |ui| ui.drive_mode_index = 0).await;
        }
        crate::system::state::MenuSelection::TestMode => {
            set_mode_mutated(UiMode::TestMenu, |ui| ui.test_index = 0).await;
        }
    }
}

/// Handle a button press while the calibration menu is active.
async fn handle_calibrate_menu_press(index: usize) {
    if let Some(selection) = calibration_selection_from_index(index) {
        set_mode_mutated(UiMode::Calibrating { kind: selection }, |ui| {
            ui.calibration_complete = false;
        })
        .await;

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
        set_mode(UiMode::RunningAutonomous { mode }).await;

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
            set_mode(UiMode::RunningTurnsTest).await;
            testmode::start_turns_test().await;
        }
        Some(TestSelection::StraightDrive) => {
            set_mode(UiMode::RunningStraightDriveTest).await;
            testmode::start_straight_drive_test().await;
        }
        Some(TestSelection::ArcDrive) => {
            set_mode(UiMode::RunningArcDriveTest).await;
            testmode::start_arc_drive_test().await;
        }
        Some(TestSelection::Imu) => {
            set_mode(UiMode::RunningImuTest).await;
            testmode::start_imu_test_mode().await;
        }
        Some(TestSelection::Imu6) => {
            set_mode(UiMode::RunningImu6Test).await;
            testmode::start_imu6_test_mode().await;
        }
        Some(TestSelection::BasicMotor) => {
            set_mode(UiMode::RunningBasicMotorTest).await;
            testmode::start_basic_motor_test_mode().await;
        }
        Some(TestSelection::IrUltrasonic) => {
            crate::task::behavior::obstacle::reset_obstacle_state().await;
            set_mode(UiMode::RunningIrUltrasonicTest).await;
            testmode::start_ir_ultrasonic_test_mode().await;
        }
        Some(TestSelection::UltrasonicSweep) => {
            set_mode(UiMode::RunningUltrasonicSweepTest).await;
            testmode::start_ultrasonic_sweep_test_mode().await;
        }
        Some(TestSelection::CoastAvoidDetection) => {
            crate::task::behavior::obstacle::reset_obstacle_state().await;
            set_mode(UiMode::RunningCoastAvoidDetectionTest).await;
            testmode::start_coast_avoid_detection_test().await;
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

/// Handle a button press while the coast-avoid detection test mode is active.
async fn handle_running_coast_avoid_detection_test_press() {
    testmode::stop_coast_avoid_detection_test();
    show_test_menu().await;
}

/// Transition to a new UI mode, lock the state, and render.
///
/// This is the canonical primitive for mode transitions. Public helpers
/// ([`show_main_menu`], [`show_test_menu`]) delegate here. For transitions
/// that also need to mutate other state fields, use [`set_mode_mutated`]
/// to keep the mutation inside the same lock.
async fn set_mode(mode: UiMode) {
    let mut ui = UI_STATE.lock().await;
    ui.mode = mode;
    let snapshot = *ui;
    drop(ui);
    render_current_ui(&snapshot).await;
}

/// Transition to a new UI mode with an additional state mutation, all
/// inside a single lock acquisition.
///
/// The closure runs before the mode is written, so it can reset
/// menu indices or clear flags that the new mode depends on.
async fn set_mode_mutated(mode: UiMode, mutate: impl FnOnce(&mut UiState)) {
    let mut ui = UI_STATE.lock().await;
    mutate(&mut ui);
    ui.mode = mode;
    let snapshot = *ui;
    drop(ui);
    render_current_ui(&snapshot).await;
}

/// Set UI state to test menu and render it.
pub async fn show_test_menu() {
    set_mode(UiMode::TestMenu).await;
}

/// Set UI state to main menu and render it.
pub async fn show_main_menu() {
    set_mode(UiMode::MainMenu).await;
}

// ── Distance calibration flow ────────────────────────────────────────────────────

/// Handle a button press while entering a distance calibration value.
async fn handle_distance_entry_press(value: u8) {
    if value == 0 {
        // Cancel — restore the pre-calibration factor and return to main menu.
        if let Some(previous) = { PREVIOUS_DISTANCE_FACTOR.lock().await.take() } {
            flash_storage::set_distance_factor(previous).await;
        }
        show_main_menu().await;
        return;
    }

    // Calibration succeeded — discard the backup.
    PREVIOUS_DISTANCE_FACTOR.lock().await.take();

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

/// Run the distance calibration procedure.
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

    // Back up the previous factor so it can be restored on cancel/error.
    let saved_factor = flash_storage::get_distance_factor().await;
    PREVIOUS_DISTANCE_FACTOR.lock().await.replace(saved_factor);
    if (saved_factor - 1.0).abs() > f32::EPSILON {
        flash_storage::set_distance_factor(1.0).await;
    }

    // Request the controller to spawn the drive task.
    DIST_CAL_CHANNEL.send(()).await;
}

/// Drive-task half of the distance calibration flow.
#[embassy_executor::task]
async fn calibration_drive_task() {
    async fn abort_calibration(message: &str) {
        // Restore the pre-calibration factor on failure.
        if let Some(previous) = { PREVIOUS_DISTANCE_FACTOR.lock().await.take() } {
            flash_storage::set_distance_factor(previous).await;
        }
        show_line(2, message).await;
        Timer::after(Duration::from_secs(2)).await;
        show_main_menu().await;
    }

    let mut queue = drive::DriveQueueBuilder::new();

    if queue
        .push_abort_on_fail(drive::DriveCommand::Drive(drive::DriveAction::DriveDistance {
            kind: drive::DriveDistanceKind::Straight { distance_cm: 150.0 },
            direction: drive::DriveDirection::Forward,
            speed: 70,
        }))
        .is_err()
    {
        abort_calibration("Queue full").await;
        return;
    }

    if queue
        .push_abort_on_fail(drive::DriveCommand::Drive(drive::DriveAction::Brake))
        .is_err()
    {
        abort_calibration("Queue full").await;
        return;
    }

    // Let the robot settle after braking, then release motors.
    let _ = queue.push(drive::DriveCommand::Drive(drive::DriveAction::Coast));

    match queue.submit().await {
        Ok(_) => {
            // Transition to entry screen.
            let mut ui = UI_STATE.lock().await;
            ui.mode = UiMode::EnteringDistance { value: 150 };
            drop(ui);
            render_entering_distance(150).await;
        }
        Err(_) => {
            abort_calibration("Queue busy").await;
        }
    }
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
