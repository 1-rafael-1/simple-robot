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
use heapless::String;

use crate::{
    system::{
        event::{Events, wait},
        state::{CalibrationStatus, SYSTEM_STATE},
    },
    task::{display, flash_storage, motor_driver},
};

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
        Events::OperationModeSet(mode) => handle_operation_mode_set(mode).await,
        Events::ObstacleDetected(detected) => handle_obstacle_detected(detected).await,
        Events::ObstacleAvoidanceAttempted => handle_obstacle_avoidance_attempted().await,
        Events::BatteryLevelMeasured(level) => handle_battery_level_measured(level).await,
        Events::ButtonPressed(button_id) => handle_button_pressed(button_id).await,
        Events::ButtonHoldStart(button_id) => handle_button_hold_start(button_id).await,
        Events::ButtonHoldEnd(button_id) => handle_button_hold_end(button_id).await,
        Events::InactivityTimeout => handle_inactivity_timeout().await,
        Events::UltrasonicSweepReadingTaken(distance, angle) => handle_ultrasonic_sweep_reading(distance, angle).await,
        Events::ImuMeasurementTaken(measurement) => handle_imu_measurement(measurement).await,
        Events::RotationCompleted => handle_rotation_completed().await,
        Events::StartStopMotionDataCollection(start) => handle_start_stop_motion_data(start).await,
        Events::StartStopUltrasonicSweep(start) => handle_start_stop_ultrasonic_sweep(start).await,
    }
}

/// Handle system initialization
async fn handle_initialize() {
    info!("System initializing");

    // Display initialization message
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

    // Note: Initialization completes when calibration data arrives via CalibrationDataLoaded events
}

/// Check if initialization is complete and update display if so
async fn check_initialization_complete() {
    let state = SYSTEM_STATE.lock().await;

    // Check if both calibrations have been checked
    if state.is_initialized() {
        // Display final initialization status
        let mut txt: String<20> = String::new();
        let _ = write!(txt, "System Ready");
        display::display_update(display::DisplayAction::ShowText(txt, 0)).await;

        info!("System initialization complete");
        info!("  Motor calibration: {:?}", state.motor_calibration_status);
        info!("  IMU calibration: {:?}", state.imu_calibration_status);
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

                // TODO: Send to IMU task when implemented
                let _ = imu_cal; // Suppress unused warning

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

/// Handle operation mode changes
async fn handle_operation_mode_set(_mode: crate::system::state::OperationMode) {
    info!("Operation mode set");
    // TODO: Implement mode transition logic
    // - Start/stop autonomous drive
    // - Update LED indicators
}

/// Handle obstacle detection status changes
async fn handle_obstacle_detected(_detected: bool) {
    info!("Obstacle detection status changed");
    // TODO: Implement obstacle response
    // - Trigger avoidance in autonomous mode
    // - Update LED indicators
}

/// Handle obstacle avoidance completion
async fn handle_obstacle_avoidance_attempted() {
    info!("Obstacle avoidance attempted");
    // TODO: Implement post-avoidance logic
    // - Resume normal operation or retry
}

/// Handle battery level updates
async fn handle_battery_level_measured(_level: u8) {
    info!("Battery level measured");
    // TODO: Implement battery response
    // - Update LED color
    // - Trigger low battery warnings
}

/// Handle button press events
async fn handle_button_pressed(_button_id: crate::system::event::ButtonId) {
    info!("Button pressed");
    // TODO: Implement button actions
    // - Map to drive commands
    // - Signal activity tracker
}

/// Handle button hold start events
async fn handle_button_hold_start(_button_id: crate::system::event::ButtonId) {
    info!("Button hold started");
    // TODO: Implement hold start actions
    // - Prepare for mode change
    // - Signal activity tracker
}

/// Handle button hold end events
async fn handle_button_hold_end(_button_id: crate::system::event::ButtonId) {
    info!("Button hold ended");
    // TODO: Implement hold end actions
    // - Complete mode change
    // - Signal activity tracker
}

/// Handle inactivity timeout
async fn handle_inactivity_timeout() {
    info!("Inactivity timeout");
    // TODO: Implement power saving
    // - Switch to manual mode
    // - Enter standby
}

/// Handle ultrasonic sensor readings
async fn handle_ultrasonic_sweep_reading(_distance: f64, _angle: f32) {
    info!("Ultrasonic sweep reading");
    // TODO: Implement sensor data processing
    // - Update display with sweep visualization
    // - Feed data to obstacle detection
}

/// Handle IMU measurements
async fn handle_imu_measurement(_measurement: crate::task::imu_read::ImuMeasurement) {
    info!("IMU measurement taken");
    // TODO: Implement IMU data processing
    // - Send orientation feedback to drive task
    // - Update display with orientation data
}

/// Handle rotation completion
async fn handle_rotation_completed() {
    info!("Rotation completed");
    // TODO: Implement rotation completion logic
    // - Update display
    // - Signal next movement phase
}

/// Handle motion data collection control
async fn handle_start_stop_motion_data(_start: bool) {
    info!("Motion data collection control");
    // TODO: Implement sensor control
    // - Start/stop IMU readings
    // - Start/stop encoder readings
}

/// Handle ultrasonic sweep control
async fn handle_start_stop_ultrasonic_sweep(_start: bool) {
    info!("Ultrasonic sweep control");
    // TODO: Implement ultrasonic sweep control
    // - Start/stop sweep task
}
