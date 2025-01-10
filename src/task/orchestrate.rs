//! System orchestration
//!
//! Manages robot behavior by coordinating state changes and event handling.
//!
//! # Architecture
//! This module implements a central event loop that:
//! - Waits for system events (button presses, sensor readings, etc.)
//! - Processes events to determine if they cause state changes
//! - Handles state transitions by coordinating other system tasks
//!
//! # State Management
//! The system has two primary operation modes:
//! - Manual: Direct RC control of the robot
//! - Autonomous: Self-driving with obstacle avoidance
//!
//! Additional states like standby and obstacle detection are managed across modes.

use crate::system::button_actions;
use crate::system::event::{send, wait, Events};
use crate::system::state;
use crate::system::state::{OperationMode, SYSTEM_STATE};
use crate::task::drive;
use crate::task::encoder_read;
use crate::task::rgb_led_indicate;
use crate::task::track_inactivity;
use crate::task::{autonomous_drive, display};
use embassy_time::Duration;

/// Main coordination task that implements the system's event loop
#[embassy_executor::task]
pub async fn orchestrate() {
    loop {
        let event = wait().await;
        if let Some(state_change) = process_event(event).await {
            handle_state_changes(state_change).await;
        }
    }
}

/// Evaluates events for state changes by comparing event data with current system state
///
/// Returns Some(event) if it caused a state change, None otherwise
async fn process_event(event: Events) -> Option<Events> {
    let mut state = SYSTEM_STATE.lock().await;

    match event {
        Events::OperationModeSet(new_mode) => {
            if state.operation_mode != new_mode {
                state.set_operation_mode(new_mode);
                Some(event)
            } else {
                None
            }
        }
        Events::ObstacleDetected(is_detected) => {
            if state.obstacle_detected != is_detected {
                state.obstacle_detected = is_detected;
                Some(event)
            } else {
                None
            }
        }
        Events::ObstacleAvoidanceAttempted => {
            // Retry avoidance if still blocked in autonomous mode
            if state.obstacle_detected && state.operation_mode == OperationMode::Autonomous {
                Some(event)
            } else {
                None
            }
        }
        Events::BatteryLevelMeasured(level) => {
            if state.battery_level != level {
                state.battery_level = level;
                Some(event)
            } else {
                None
            }
        }
        Events::ButtonPressed(_) | Events::ButtonHoldStart(_) | Events::ButtonHoldEnd(_) => {
            Some(event)
        }
        Events::InactivityTimeout => {
            if !state.standby {
                Some(event)
            } else {
                None
            }
        }
        Events::DriveCommandExecuted => Some(event),
        Events::EncoderMeasurementTaken(_) => Some(event),
        Events::UltrasonicSweepReadingTaken(_, _) => Some(event),
    }
}

/// Processes state changes by coordinating system tasks based on events
async fn handle_state_changes(event: Events) {
    match event {
        Events::OperationModeSet(new_mode) => match new_mode {
            OperationMode::Manual => {
                rgb_led_indicate::update_indicator(true);
                autonomous_drive::send_command(autonomous_drive::Command::Stop);
            }
            OperationMode::Autonomous => {
                rgb_led_indicate::update_indicator(true);
                autonomous_drive::send_command(autonomous_drive::Command::Initialize);
                autonomous_drive::send_command(autonomous_drive::Command::Start);
            }
        },
        Events::ObstacleDetected(is_detected) => {
            rgb_led_indicate::update_indicator(true);
            {
                let state = SYSTEM_STATE.lock().await;
                if state.operation_mode == OperationMode::Autonomous {
                    autonomous_drive::send_command(if is_detected {
                        autonomous_drive::Command::AvoidObstacle
                    } else {
                        autonomous_drive::Command::Start
                    });
                }
            }
        }
        Events::ObstacleAvoidanceAttempted => {
            rgb_led_indicate::update_indicator(true);
            autonomous_drive::send_command(autonomous_drive::Command::AvoidObstacle);
        }
        Events::BatteryLevelMeasured(_) => {
            rgb_led_indicate::update_indicator(false);
        }
        Events::ButtonPressed(button_id) => {
            button_actions::handle_button_action(
                button_id,
                button_actions::ButtonActionType::Press,
            )
            .await;
            track_inactivity::signal_activity();
        }
        Events::ButtonHoldStart(button_id) => {
            button_actions::handle_button_action(
                button_id,
                button_actions::ButtonActionType::HoldStart,
            )
            .await;
            track_inactivity::signal_activity();
        }
        Events::ButtonHoldEnd(button_id) => {
            button_actions::handle_button_action(
                button_id,
                button_actions::ButtonActionType::HoldEnd,
            )
            .await;
            track_inactivity::signal_activity();
        }
        Events::InactivityTimeout => {
            // Inactivity timeout occurred, set operation mode to manual and start standby mode
            send(Events::OperationModeSet(state::OperationMode::Manual)).await;
            drive::send_command(drive::Command::Drive(drive::DriveAction::Standby));
        }
        Events::DriveCommandExecuted => {
            // Drive command was executed, trigger encoder measurement
            encoder_read::request_measurement(Duration::from_millis(100));
        }
        Events::EncoderMeasurementTaken(measurement) => {
            // Send encoder feedback to drive task for speed adjustment
            drive::send_command(drive::Command::EncoderFeedback(measurement));
        }
        Events::UltrasonicSweepReadingTaken(distance, angle) => {
            // Ultrasonic sensor reading received, send distance and servo angle to display and obstacle detection
            display::request_update(display::DisplayAction::ShowText("Test", 0));
            display::request_update(display::DisplayAction::ShowSweep(distance, angle));
        }
    }
}
