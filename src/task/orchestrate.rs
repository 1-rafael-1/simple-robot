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
use crate::system::event;
use crate::system::state;
use crate::system::state::{OperationMode, SYSTEM_STATE};
use crate::task::autonomous_drive;
use crate::task::drive;
use crate::task::rgb_led_indicate;
use crate::task::track_inactivity;

/// Main coordination task that implements the system's event loop
#[embassy_executor::task]
pub async fn orchestrate() {
    loop {
        let event = event::wait().await;
        if let Some(state_change) = process_event(event).await {
            handle_state_changes(state_change).await;
        }
    }
}

/// Evaluates events for state changes by comparing event data with current system state
///
/// Returns Some(event) if it caused a state change, None otherwise
///
/// State changes occur when:
/// - Operation mode changes (Manual <-> Autonomous)
/// - Obstacle detection status changes
/// - Battery level changes
/// - Button events occur
/// - Inactivity timeout occurs (only if not already in standby)
async fn process_event(event: event::Events) -> Option<event::Events> {
    let mut state = SYSTEM_STATE.lock().await;

    match event {
        event::Events::OperationModeSet(new_mode) => {
            if state.operation_mode != new_mode {
                state.set_operation_mode(new_mode);
                Some(event)
            } else {
                None
            }
        }
        event::Events::ObstacleDetected(is_detected) => {
            if state.obstacle_detected != is_detected {
                state.obstacle_detected = is_detected;
                Some(event)
            } else {
                None
            }
        }
        event::Events::ObstacleAvoidanceAttempted => {
            // Retry avoidance if still blocked in autonomous mode
            if state.obstacle_detected && state.operation_mode == OperationMode::Autonomous {
                Some(event)
            } else {
                None
            }
        }
        event::Events::BatteryLevelMeasured(level) => {
            if state.battery_level != level {
                state.battery_level = level;
                Some(event)
            } else {
                None
            }
        }
        event::Events::ButtonPressed(_)
        | event::Events::ButtonHoldStart(_)
        | event::Events::ButtonHoldEnd(_) => Some(event),
        event::Events::InactivityTimeout => {
            if !state.standby {
                Some(event)
            } else {
                None
            }
        }
    }
}

/// Processes state changes by coordinating system tasks based on events
///
/// Key behaviors:
/// - Mode changes: Updates LED indicator and manages autonomous driving
/// - Obstacle detection: Triggers avoidance in autonomous mode
/// - Button events: Handles actions and resets inactivity timer
/// - Inactivity: Switches to manual mode and enables standby
async fn handle_state_changes(event: event::Events) {
    match event {
        event::Events::OperationModeSet(new_mode) => match new_mode {
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
        event::Events::ObstacleDetected(is_detected) => {
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
        event::Events::ObstacleAvoidanceAttempted => {
            rgb_led_indicate::update_indicator(true);
            autonomous_drive::send_command(autonomous_drive::Command::AvoidObstacle);
        }
        event::Events::BatteryLevelMeasured(_) => {
            rgb_led_indicate::update_indicator(false);
        }
        event::Events::ButtonPressed(button_id) => {
            button_actions::handle_button_action(
                button_id,
                button_actions::ButtonActionType::Press,
            )
            .await;
            track_inactivity::signal_activity();
        }
        event::Events::ButtonHoldStart(button_id) => {
            button_actions::handle_button_action(
                button_id,
                button_actions::ButtonActionType::HoldStart,
            )
            .await;
            track_inactivity::signal_activity();
        }
        event::Events::ButtonHoldEnd(button_id) => {
            button_actions::handle_button_action(
                button_id,
                button_actions::ButtonActionType::HoldEnd,
            )
            .await;
            track_inactivity::signal_activity();
        }
        event::Events::InactivityTimeout => {
            event::send(event::Events::OperationModeSet(
                state::OperationMode::Manual,
            ))
            .await;
            drive::send_command(drive::Command::Standby);
        }
    }
}
