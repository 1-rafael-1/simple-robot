//! System orchestration
//!
//! Manages robot behavior by coordinating state changes and event handling.
//!
//! # Architecture
//! This module implements a central event loop that:
//! - Waits for system events (button presses, sensor readings, etc.)
//! - Routes events to domain-specific handlers
//!
//! Each event is handled by a dedicated module.

use defmt::info;

use crate::{
    system::event::{Events, wait},
    task::{
        autonomous_mode::attempt_straight_line,
        behavior, initialization,
        ui::{self, UiEvent},
    },
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
        Events::ObstacleDetected { source, detected } => {
            behavior::obstacle::handle_obstacle_detected(source, detected);
        }
        Events::ObstacleAvoidanceAttempted => behavior::obstacle::handle_obstacle_avoidance_attempted().await,
        Events::BatteryMeasured { level, voltage } => behavior::battery::handle_battery_measured(level, voltage).await,
        Events::RCButtonPressed(button_id) => behavior::input::handle_button_pressed(button_id).await,
        Events::ButtonHoldStart(button_id) => behavior::input::handle_button_hold_start(button_id).await,
        Events::ButtonHoldEnd(button_id) => behavior::input::handle_button_hold_end(button_id).await,
        Events::RotaryTurned(direction) => ui::send_ui_event(UiEvent::RotaryTurned(direction)).await,
        Events::RotaryButtonPressed => ui::send_ui_event(UiEvent::RotaryButtonPressed).await,
        Events::RotaryButtonHoldStart => ui::send_ui_event(UiEvent::RotaryButtonHoldStart).await,
        Events::RotaryButtonHoldEnd => ui::send_ui_event(UiEvent::RotaryButtonHoldEnd).await,
        Events::TestingCompleted => ui::send_ui_event(UiEvent::TestingCompleted).await,
        Events::CalibrationCompleted => ui::send_ui_event(UiEvent::CalibrationCompleted).await,
        Events::UltrasonicSweepCompleted => {
            attempt_straight_line::SWEEP_COMPLETED.signal(());
        }
        Events::CalibrationStatus {
            header,
            line1,
            line2,
            line3,
        } => initialization::handle_calibration_status(header, line1, line2, line3).await,
    }
}
