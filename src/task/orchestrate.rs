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

use core::fmt::Write;

use embassy_time::Duration;
use heapless::String;
use micromath::F32Ext;

use crate::{
    system::{
        button_actions,
        event::{send_event, wait, Events},
        state,
        state::{OperationMode, SYSTEM_STATE},
    },
    task::{autonomous_drive, display, drive, encoder_read, rgb_led_indicate, track_inactivity},
};

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
        Events::ButtonPressed(_) | Events::ButtonHoldStart(_) | Events::ButtonHoldEnd(_) => Some(event),
        Events::InactivityTimeout => {
            if !state.standby {
                Some(event)
            } else {
                None
            }
        }
        Events::DriveCommandExecuted(_) => Some(event),
        Events::EncoderMeasurementTaken(_) => Some(event),
        Events::UltrasonicSweepReadingTaken(_, _) => Some(event),
        Events::ImuMeasurementTaken(_) => Some(event),
        Events::RotationCompleted => Some(event),
        Events::MotionCorrectionNeeded(_) => Some(event),
    }
}

/// Processes state changes by coordinating system tasks based on events
async fn handle_state_changes(event: Events) {
    match event {
        Events::OperationModeSet(new_mode) => match new_mode {
            OperationMode::Manual => {
                rgb_led_indicate::update_indicator(true);
                autonomous_drive::send_autonomous_command(autonomous_drive::Command::Stop);
            }
            OperationMode::Autonomous => {
                rgb_led_indicate::update_indicator(true);
                autonomous_drive::send_autonomous_command(autonomous_drive::Command::Initialize);
                autonomous_drive::send_autonomous_command(autonomous_drive::Command::Start);
            }
        },
        Events::ObstacleDetected(is_detected) => {
            rgb_led_indicate::update_indicator(true);
            {
                let state = SYSTEM_STATE.lock().await;
                if state.operation_mode == OperationMode::Autonomous {
                    autonomous_drive::send_autonomous_command(if is_detected {
                        autonomous_drive::Command::AvoidObstacle
                    } else {
                        autonomous_drive::Command::Start
                    });
                }
            }
        }
        Events::ObstacleAvoidanceAttempted => {
            rgb_led_indicate::update_indicator(true);
            autonomous_drive::send_autonomous_command(autonomous_drive::Command::AvoidObstacle);
        }
        Events::BatteryLevelMeasured(_) => {
            rgb_led_indicate::update_indicator(false);
        }
        Events::ButtonPressed(button_id) => {
            button_actions::handle_button_action(button_id, button_actions::ButtonActionType::Press).await;
            track_inactivity::signal_activity();
        }
        Events::ButtonHoldStart(button_id) => {
            button_actions::handle_button_action(button_id, button_actions::ButtonActionType::HoldStart).await;
            track_inactivity::signal_activity();
        }
        Events::ButtonHoldEnd(button_id) => {
            button_actions::handle_button_action(button_id, button_actions::ButtonActionType::HoldEnd).await;
            track_inactivity::signal_activity();
        }
        Events::InactivityTimeout => {
            // Inactivity timeout occurred, set operation mode to manual and start standby mode
            send_event(Events::OperationModeSet(state::OperationMode::Manual)).await;
            drive::send_drive_command(drive::DriveCommand::Drive(drive::DriveAction::Standby));
        }
        Events::DriveCommandExecuted(drive_action) => {
            // Drive command was executed, trigger encoder measurement
            // encoder_read::request_encoder_measurement(Duration::from_millis(100));
        }
        Events::EncoderMeasurementTaken(measurement) => {
            // Send encoder feedback to drive task for speed adjustment
            // drive::send_drive_command(drive::DriveCommand::EncoderFeedback(measurement));
        }
        Events::UltrasonicSweepReadingTaken(distance, angle) => {
            // Ultrasonic sensor reading received, send distance and servo angle to display and obstacle detection
            let mut txt: String<20> = String::new();
            let _ = write!(txt, "{}deg {}cm", angle.round(), (distance as f32).round());
            display::display_update(display::DisplayAction::ShowText(txt, 0)).await;
            display::display_update(display::DisplayAction::ShowSweep(distance, angle)).await;
        }
        Events::ImuMeasurementTaken(measurement) => {
            // drive::send_drive_command(drive::DriveCommand::ImuFeedback(measurement));
            // // update display with rotation rate
            // let mut txt: String<20> = String::new();
            // let _ = write!(txt, "Yaw: {}°", measurement.orientation.yaw);
            // display::display_update(display::DisplayAction::ShowText(txt, 2)).await;
            // let mut txt: String<20> = String::new();
            // let _ = write!(txt, "Pitch: {}°", measurement.orientation.pitch);
            // display::display_update(display::DisplayAction::ShowText(txt, 3)).await;
            // let mut txt: String<20> = String::new();
            // let _ = write!(txt, "Roll: {}°", measurement.orientation.roll);
            // display::display_update(display::DisplayAction::ShowText(txt, 4)).await;
        }
        Events::RotationCompleted => {
            // Flash LED to indicate completion
            rgb_led_indicate::update_indicator(true);

            // Update display
            let mut txt: String<20> = String::new();
            let _ = write!(txt, "Rotation Done");
            display::display_update(display::DisplayAction::ShowText(txt, 3)).await;
        }
        Events::MotionCorrectionNeeded(correction_instruction) => {
            // Correct motion based on sensor data
            // send to drive task for correction
        }
    }
}
