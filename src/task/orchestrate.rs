//! System orchestration
//!
//! Manages robot behavior by coordinating state changes and event handling.
//!
//! # Architecture
//! This module implements a central event loop that:
//! - Waits for system events (button presses, sensor readings, etc.)
//! - Processes events and coordinates responses across system tasks
//!
//! Each event is handled by a dedicated function for clarity and maintainability.

use core::fmt::Write;

use defmt::info;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use heapless::String;

use crate::{
    system::{
        event::{Events, wait},
        state::{self, CalibrationSelection, CalibrationStatus, MenuSelection, SYSTEM_STATE, UiMode},
    },
    task::{display, drive, flash_storage, imu_read, motor_driver, rgb_led_indicate, testing, ui_menu},
};

/// UI state owned by the orchestrator.
#[derive(Clone, Copy)]
struct UiState {
    /// Current UI mode.
    mode: UiMode,
    /// Selected index in the main menu.
    main_index: usize,
    /// Selected index in the calibration menu.
    calibrate_index: usize,
}

impl UiState {
    /// Creates the default UI state (main menu with first items selected).
    const fn new() -> Self {
        Self {
            mode: UiMode::MainMenu,
            main_index: 0,
            calibrate_index: 0,
        }
    }
}

/// Global UI state mutex owned by the orchestrator.
static UI_STATE: Mutex<CriticalSectionRawMutex, UiState> = Mutex::new(UiState::new());

/// Main coordination task that implements the system's event loop
#[embassy_executor::task]
pub async fn orchestrate() {
    info!("Orchestrator starting");

    loop {
        let event = wait().await;
        handle_event(event).await;
    }
}

/// Routes events to their respective handlers
async fn handle_event(event: Events) {
    match event {
        Events::Initialize => handle_initialize().await,
        Events::CalibrationDataLoaded(kind, data) => handle_calibration_data_loaded(kind, data).await,
        Events::ImuCalibrationFlagsLoaded(flags) => handle_imu_calibration_flags_loaded(flags).await,
        Events::OperationModeSet(mode) => handle_operation_mode_set(mode).await,
        Events::ObstacleDetected(detected) => handle_obstacle_detected(detected).await,
        Events::ObstacleAvoidanceAttempted => handle_obstacle_avoidance_attempted().await,
        Events::BatteryMeasured { level, voltage } => handle_battery_measured(level, voltage).await,
        Events::RCButtonPressed(button_id) => handle_button_pressed(button_id).await,
        Events::ButtonHoldStart(button_id) => handle_button_hold_start(button_id).await,
        Events::ButtonHoldEnd(button_id) => handle_button_hold_end(button_id).await,
        Events::RotaryTurned(direction) => handle_rotary_turned(direction).await,
        Events::RotaryButtonPressed => handle_rotary_button_pressed().await,
        Events::RotaryButtonHoldStart => handle_rotary_button_hold_start().await,
        Events::RotaryButtonHoldEnd => handle_rotary_button_hold_end(),
        Events::TestingCompleted => handle_testing_completed().await,
        Events::InactivityTimeout => handle_inactivity_timeout().await,
        Events::EncoderMeasurementTaken(measurement) => handle_encoder_measurement(measurement).await,
        Events::UltrasonicSweepReadingTaken(distance, angle) => handle_ultrasonic_sweep_reading(distance, angle).await,
        Events::ImuMeasurementTaken(measurement) => handle_imu_measurement(measurement).await,
        Events::RotationCompleted => handle_rotation_completed().await,
        Events::StartStopMotionDataCollection(start) => handle_start_stop_motion_data(start),
        Events::StartStopUltrasonicSweep(start) => handle_start_stop_ultrasonic_sweep(start).await,
        Events::CalibrationStatus {
            header,
            line1,
            line2,
            line3,
        } => handle_calibration_status(header, line1, line2, line3).await,
    }
}

/// Handle battery measurement (level and voltage)
async fn handle_battery_measured(level: u8, voltage: f32) {
    info!("Battery level measured");

    // Store battery voltage in system state for motor driver to poll
    {
        let mut state = state::SYSTEM_STATE.lock().await;
        state.battery_voltage = Some(voltage);
        state.battery_level = Some(level);
    }
    rgb_led_indicate::update_indicator(false);
}

/// Handle system initialization
async fn handle_initialize() {
    info!("System initializing");

    // Display initialization message
    display::display_update(display::DisplayAction::Clear).await;
    let mut txt: String<20> = String::new();
    let _ = write!(txt, "Initializing...");
    display::display_update(display::DisplayAction::ShowText(txt, 0)).await;

    // Request motor calibration from flash
    info!("Requesting motor calibration from flash");
    flash_storage::send_flash_command(flash_storage::FlashCommand::GetData(
        flash_storage::CalibrationKind::Motor,
    ))
    .await;

    // Request IMU calibration from flash
    info!("Requesting IMU calibration from flash");
    flash_storage::send_flash_command(flash_storage::FlashCommand::GetData(
        flash_storage::CalibrationKind::Imu,
    ))
    .await;

    // Request IMU calibration flags from flash
    info!("Requesting IMU calibration flags from flash");
    flash_storage::send_flash_command(flash_storage::FlashCommand::GetImuFlags).await;

    // Note: Initialization completes when calibration data arrives via CalibrationDataLoaded events
}

/// Check if initialization is complete and update display if so
async fn check_initialization_complete() {
    let should_show_menu = {
        let state = SYSTEM_STATE.lock().await;

        // Check if both calibrations have been checked
        if state.is_initialized() {
            info!("System initialization complete");
            info!("  Motor calibration: {:?}", state.motor_calibration_status);
            info!("  IMU calibration: {:?}", state.imu_calibration_status);
            true
        } else {
            false
        }
    };

    if should_show_menu {
        show_main_menu().await;
    }
}

/// Handle calibration data loaded from flash storage
async fn handle_calibration_data_loaded(
    kind: flash_storage::CalibrationKind,
    data: Option<flash_storage::CalibrationDataKind>,
) {
    use flash_storage::{CalibrationDataKind, CalibrationKind};

    match kind {
        CalibrationKind::Motor => {
            if let Some(CalibrationDataKind::Motor(motor_cal)) = data {
                info!(
                    "Motor calibration loaded: [{}, {}, {}, {}]",
                    motor_cal.left_front, motor_cal.left_rear, motor_cal.right_front, motor_cal.right_rear
                );

                // Update system state
                {
                    let mut state = SYSTEM_STATE.lock().await;
                    state.motor_calibration_status = CalibrationStatus::Loaded;
                }

                // Send calibration to motor driver
                motor_driver::send_motor_command(motor_driver::MotorCommand::LoadCalibration(motor_cal)).await;

                // Update display
                let mut txt: String<20> = String::new();
                let _ = write!(txt, "Calibration loaded");
                display::display_update(display::DisplayAction::ShowText(txt, 1)).await;
            } else {
                info!("No motor calibration found - using defaults");

                // Update system state
                {
                    let mut state = SYSTEM_STATE.lock().await;
                    state.motor_calibration_status = CalibrationStatus::NotAvailable;
                }

                // Update display
                let mut txt: String<20> = String::new();
                let _ = write!(txt, "Need motor calib");
                display::display_update(display::DisplayAction::ShowText(txt, 1)).await;
            }
        }
        CalibrationKind::Imu => {
            if let Some(CalibrationDataKind::Imu(imu_cal)) = data {
                info!("IMU calibration loaded from flash");

                // Update system state
                {
                    let mut state = SYSTEM_STATE.lock().await;
                    state.imu_calibration_status = CalibrationStatus::Loaded;
                }

                // Forward to IMU task
                imu_read::load_imu_calibration(imu_cal);

                // Update display
                let mut txt: String<20> = String::new();
                let _ = write!(txt, "IMU cal loaded");
                display::display_update(display::DisplayAction::ShowText(txt, 2)).await;
            } else {
                info!("No IMU calibration found - using defaults");

                // Update system state
                {
                    let mut state = SYSTEM_STATE.lock().await;
                    state.imu_calibration_status = CalibrationStatus::NotAvailable;
                    state.mag_calibration_status = CalibrationStatus::NotAvailable;
                    state.accel_calibration_status = CalibrationStatus::NotAvailable;
                    state.gyro_calibration_status = CalibrationStatus::NotAvailable;
                }

                // Update display
                let mut txt: String<20> = String::new();
                let _ = write!(txt, "Need IMU calib");
                display::display_update(display::DisplayAction::ShowText(txt, 2)).await;
            }
        }
    }

    // Check if initialization is complete
    check_initialization_complete().await;
}

/// Handle IMU calibration flags loaded from flash
async fn handle_imu_calibration_flags_loaded(flags: Option<flash_storage::ImuCalibrationFlags>) {
    {
        let mut state = SYSTEM_STATE.lock().await;
        if let Some(flags) = flags {
            state.gyro_calibration_status = if flags.gyro {
                CalibrationStatus::Loaded
            } else {
                CalibrationStatus::NotAvailable
            };
            state.accel_calibration_status = if flags.accel {
                CalibrationStatus::Loaded
            } else {
                CalibrationStatus::NotAvailable
            };
            state.mag_calibration_status = if flags.mag {
                CalibrationStatus::Loaded
            } else {
                CalibrationStatus::NotAvailable
            };
            state.imu_calibration_status = if flags.gyro && flags.accel && flags.mag {
                CalibrationStatus::Loaded
            } else {
                CalibrationStatus::NotAvailable
            };
        } else {
            state.imu_calibration_status = CalibrationStatus::NotAvailable;
            state.mag_calibration_status = CalibrationStatus::NotAvailable;
            state.accel_calibration_status = CalibrationStatus::NotAvailable;
            state.gyro_calibration_status = CalibrationStatus::NotAvailable;
        }
    }

    if ui_initialized().await {
        let ui = UI_STATE.lock().await;
        let snapshot = *ui;
        drop(ui);
        render_current_ui(&snapshot).await;
    }

    check_initialization_complete().await;
}

/// Handle operation mode changes
#[allow(clippy::unused_async)]
async fn handle_operation_mode_set(_mode: crate::system::state::OperationMode) {
    info!("Operation mode set");
    // TODO: Implement mode transition logic
    // - Start/stop autonomous drive
    // - Update LED indicators
}

/// Handle obstacle detection status changes
#[allow(clippy::unused_async)]
async fn handle_obstacle_detected(_detected: bool) {
    info!("Obstacle detection status changed");
    // TODO: Implement obstacle response
    // - Trigger avoidance in autonomous mode
    // - Update LED indicators
}

/// Handle obstacle avoidance completion
#[allow(clippy::unused_async)]
async fn handle_obstacle_avoidance_attempted() {
    info!("Obstacle avoidance attempted");
    // TODO: Implement post-avoidance logic
    // - Resume normal operation or retry
}

/// Handle button press events
#[allow(clippy::unused_async)]
async fn handle_button_pressed(_button_id: crate::system::event::RCButtonId) {
    info!("Button pressed");
    // TODO: Implement button actions
    // - Map to drive commands
    // - Signal activity tracker
}

/// Handle button hold start events
#[allow(clippy::unused_async)]
async fn handle_button_hold_start(_button_id: crate::system::event::RCButtonId) {
    info!("Button hold started");
    // TODO: Implement hold start actions
    // - Prepare for mode change
    // - Signal activity tracker
}

/// Handle button hold end events
#[allow(clippy::unused_async)]
async fn handle_button_hold_end(_button_id: crate::system::event::RCButtonId) {
    info!("Button hold ended");
    // TODO: Implement hold end actions
    // - Complete mode change
    // - Signal activity tracker
}

/// Handle rotary encoder turns
async fn handle_rotary_turned(direction: crate::system::event::RotaryDirection) {
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
            let info = build_system_info_data().await;
            let max_scroll = max_system_info_scroll(&info);
            let new_offset = match direction {
                crate::system::event::RotaryDirection::Clockwise => (scroll_offset as usize + 1).min(max_scroll),
                crate::system::event::RotaryDirection::CounterClockwise => (scroll_offset as usize).saturating_sub(1),
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

/// Handle rotary encoder button press
async fn handle_rotary_button_pressed() {
    if !ui_initialized().await {
        return;
    }

    let ui_snapshot = {
        let ui = UI_STATE.lock().await;
        *ui
    };

    match ui_snapshot.mode {
        UiMode::MainMenu => match menu_selection_from_index(ui_snapshot.main_index) {
            MenuSelection::SystemInfo => {
                let mut ui = UI_STATE.lock().await;
                ui.mode = UiMode::SystemInfo { scroll_offset: 0 };
                let snapshot = *ui;
                drop(ui);
                render_current_ui(&snapshot).await;
            }
            MenuSelection::Calibrate => {
                let mut ui = UI_STATE.lock().await;
                ui.calibrate_index = 0;
                ui.mode = UiMode::CalibrateMenu;
                let snapshot = *ui;
                drop(ui);
                render_current_ui(&snapshot).await;
            }
            MenuSelection::TestMode => {
                let mut ui = UI_STATE.lock().await;
                ui.mode = UiMode::RunningTest;
                let snapshot = *ui;
                drop(ui);
                render_current_ui(&snapshot).await;
                testing::start_testing_sequence();
                info!("Test mode selected (testing sequence started)");
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

/// Handle rotary encoder button hold start
async fn handle_rotary_button_hold_start() {
    if !ui_initialized().await {
        return;
    }

    handle_ui_back().await;
}

/// Handle rotary encoder button hold end.
const fn handle_rotary_button_hold_end() {
    // No-op for now.
}

/// Handle inactivity timeout
#[allow(clippy::unused_async)]
async fn handle_inactivity_timeout() {
    info!("Inactivity timeout");
    // TODO: Implement power saving
    // - Switch to manual mode
    // - Enter standby
}

/// Handle encoder measurements
async fn handle_encoder_measurement(measurement: crate::task::encoder_read::EncoderMeasurement) {
    // Forward encoder measurements to drive task for calibration and feedback control
    // Store in shared mutex so drive task can read latest measurement on demand
    // This ensures calibration always gets fresh data without channel overflow issues
    let _ = drive::try_send_encoder_measurement(measurement).await;
}

/// Handle ultrasonic sensor readings
async fn handle_ultrasonic_sweep_reading(distance: f64, angle: f32) {
    // Forward sweep data to display for visualization
    display::display_update(display::DisplayAction::ShowSweep(distance, angle)).await;

    // TODO: Feed data to obstacle detection
}

/// Handle IMU measurements
#[allow(clippy::unused_async)]
async fn handle_imu_measurement(measurement: crate::task::imu_read::ImuMeasurement) {
    // Forward IMU measurements to drive task for rotation control
    // Use try_send to avoid blocking orchestrator - IMU data arrives at 100Hz
    // Dropping occasional measurements is acceptable at this rate
    let _ = drive::try_send_imu_measurement(measurement);
}

/// Handle rotation completion
#[allow(clippy::unused_async)]
async fn handle_rotation_completed() {
    info!("Rotation completed");
    // TODO: Implement rotation completion logic
    // - Update display
    // - Signal next movement phase
}

/// Handle motion data collection control
fn handle_start_stop_motion_data(start: bool) {
    info!("Motion data collection control");

    // For now, "motion data collection" means IMU orientation streaming for control loops.
    // Encoders are started/stopped by the drive task depending on control mode (e.g., drift compensation),
    // so we only manage the IMU here.
    if start {
        info!("Starting IMU readings");
        imu_read::start_imu_readings();
    } else {
        info!("Stopping IMU readings");
        imu_read::stop_imu_readings();
    }
}

/// Handle ultrasonic sweep control
#[allow(clippy::unused_async)]
async fn handle_start_stop_ultrasonic_sweep(_start: bool) {
    info!("Ultrasonic sweep control");
    // TODO: Implement ultrasonic sweep control
    // - Start/stop sweep task
}

/// Handle testing completion by returning to the main menu.
async fn handle_testing_completed() {
    info!("Testing completed");
    show_main_menu().await;
}

/// Handle a UI back action based on the current mode.
async fn handle_ui_back() {
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
async fn show_main_menu() {
    let mut ui = UI_STATE.lock().await;
    ui.mode = UiMode::MainMenu;
    let snapshot = *ui;
    drop(ui);
    render_current_ui(&snapshot).await;
}

/// Render the current UI view based on the UI state.
async fn render_current_ui(state: &UiState) {
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
async fn render_test_running() {
    display::display_update(display::DisplayAction::Clear).await;
    show_line(0, "Test Mode").await;
    show_line(1, "Running...").await;
    show_line(2, "").await;
    show_line(3, "").await;
}

/// Render the calibration-in-progress screen for the selected kind.
async fn render_calibrating(kind: CalibrationSelection) {
    display::display_update(display::DisplayAction::Clear).await;
    show_line(0, "Calibrating").await;
    show_line(1, calibration_label(kind)).await;
    show_line(2, "").await;
    show_line(3, "").await;
}

/// Write a single line of text to the display.
async fn show_line(line: u8, msg: &str) {
    let mut s: String<20> = String::new();
    for ch in msg.chars() {
        if s.push(ch).is_err() {
            break;
        }
    }
    display::display_update(display::DisplayAction::ShowText(s, line)).await;
}

/// Returns true once calibration data has been queried.
async fn ui_initialized() -> bool {
    let state = SYSTEM_STATE.lock().await;
    state.is_initialized()
}

/// Build a snapshot of system info for the UI renderer.
async fn build_system_info_data() -> ui_menu::SystemInfoData {
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

/// Map a main menu index to its logical selection.
const fn menu_selection_from_index(index: usize) -> MenuSelection {
    match index {
        0 => MenuSelection::SystemInfo,
        1 => MenuSelection::Calibrate,
        _ => MenuSelection::TestMode,
    }
}

/// Map a calibration menu index to its selection.
const fn calibration_selection_from_index(index: usize) -> CalibrationSelection {
    match index {
        0 => CalibrationSelection::Motor,
        1 => CalibrationSelection::Mag,
        2 => CalibrationSelection::Accel,
        _ => CalibrationSelection::Gyro,
    }
}

/// Human-readable label for a calibration selection.
const fn calibration_label(kind: CalibrationSelection) -> &'static str {
    match kind {
        CalibrationSelection::Motor => "Motor",
        CalibrationSelection::Mag => "Mag",
        CalibrationSelection::Accel => "Accel",
        CalibrationSelection::Gyro => "Gyro",
    }
}

/// Compute the next menu index, with wrap-around.
const fn next_menu_index(current: usize, len: usize, direction: crate::system::event::RotaryDirection) -> usize {
    if len == 0 {
        return 0;
    }

    match direction {
        crate::system::event::RotaryDirection::Clockwise => (current + 1) % len,
        crate::system::event::RotaryDirection::CounterClockwise => {
            if current == 0 {
                len - 1
            } else {
                current - 1
            }
        }
    }
}

/// Compute the maximum scroll offset for the system info screen.
fn max_system_info_scroll(info: &ui_menu::SystemInfoData) -> usize {
    let line_count = ui_menu::system_info_line_count(info);
    line_count.saturating_sub(ui_menu::DISPLAY_LINES)
}

/// Handle calibration status updates
async fn handle_calibration_status(
    header: Option<heapless::String<20>>,
    line1: Option<heapless::String<20>>,
    line2: Option<heapless::String<20>>,
    line3: Option<heapless::String<20>>,
) {
    // Update display lines based on provided status
    if let Some(text) = header {
        display::display_update(display::DisplayAction::ShowText(text, 0)).await;
    }
    if let Some(text) = line1 {
        display::display_update(display::DisplayAction::ShowText(text, 1)).await;
    }
    if let Some(text) = line2 {
        display::display_update(display::DisplayAction::ShowText(text, 2)).await;
    }
    if let Some(text) = line3 {
        display::display_update(display::DisplayAction::ShowText(text, 3)).await;
    }
}
