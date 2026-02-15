//! System orchestration
//!
//! Manages robot behavior by coordinating state changes and event handling.
//!
//! # Architecture
//! This module implements a central event loop that:
//! - Waits for system events (button presses, sensor readings, etc.)
//! - Routes events to domain-specific handlers
//!
//! Each event is handled by a dedicated module for clarity and maintainability.

use defmt::info;

use crate::{
    system::event::{Events, wait},
    task::{behavior, initialization, ui},
};

/// Main coordination task that implements the system's event loop.
#[embassy_executor::task]
pub async fn orchestrate() {
    info!("Orchestrator starting");

    loop {
        let event = wait().await;
        handle_event(event).await;
    }
}

/// Routes events to their respective handlers.
async fn handle_event(event: Events) {
    match event {
        Events::Initialize => initialization::handle_initialize().await,
        Events::CalibrationDataLoaded(kind, data) => initialization::handle_calibration_data_loaded(kind, data).await,
        Events::ImuCalibrationFlagsLoaded(flags) => initialization::handle_imu_calibration_flags_loaded(flags).await,
        Events::OperationModeSet(mode) => behavior::operation_mode::handle_operation_mode_set(mode).await,
        Events::ObstacleDetected(detected) => behavior::obstacle::handle_obstacle_detected(detected).await,
        Events::ObstacleAvoidanceAttempted => behavior::obstacle::handle_obstacle_avoidance_attempted().await,
        Events::BatteryMeasured { level, voltage } => behavior::battery::handle_battery_measured(level, voltage).await,
        Events::RCButtonPressed(button_id) => behavior::input::handle_button_pressed(button_id).await,
        Events::ButtonHoldStart(button_id) => behavior::input::handle_button_hold_start(button_id).await,
        Events::ButtonHoldEnd(button_id) => behavior::input::handle_button_hold_end(button_id).await,
        Events::RotaryTurned(direction) => ui::handle_rotary_turned(direction).await,
        Events::RotaryButtonPressed => ui::handle_rotary_button_pressed().await,
        Events::RotaryButtonHoldStart => ui::handle_rotary_button_hold_start().await,
        Events::RotaryButtonHoldEnd => ui::handle_rotary_button_hold_end(),
        Events::TestingCompleted => ui::handle_testing_completed().await,
        Events::InactivityTimeout => behavior::inactivity::handle_inactivity_timeout().await,
        Events::EncoderMeasurementTaken(measurement) => {
            behavior::sensor_events::handle_encoder_measurement(measurement).await;
        }
        Events::UltrasonicSweepReadingTaken(distance, angle) => {
            behavior::sensor_events::handle_ultrasonic_sweep_reading(distance, angle).await;
        }
        Events::ImuMeasurementTaken(measurement) => {
            behavior::sensor_events::handle_imu_measurement(measurement).await;
        }
        Events::RotationCompleted => behavior::motion::handle_rotation_completed().await,
        Events::StartStopMotionDataCollection(start) => behavior::motion::handle_start_stop_motion_data(start),
        Events::StartStopUltrasonicSweep(start) => behavior::motion::handle_start_stop_ultrasonic_sweep(start).await,
        Events::CalibrationStatus {
            header,
            line1,
            line2,
            line3,
        } => initialization::handle_calibration_status(header, line1, line2, line3).await,
    }
}
