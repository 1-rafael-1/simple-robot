//! System orchestration
//!
//! Manages robot behavior by coordinating state changes and event handling.

use crate::system::activity;
use crate::system::autonomous_command;
use crate::system::button_actions;
use crate::system::drive_command;
use crate::system::event;
use crate::system::indicator;
use crate::system::state;
use crate::system::state::{OperationMode, SYSTEM_STATE};

/// Main coordination task
#[embassy_executor::task]
pub async fn orchestrate() {
    loop {
        let event = event::wait().await;
        if let Some(state_change) = process_event(event).await {
            handle_state_changes(state_change).await;
        }
    }
}

/// Evaluates events for state changes
///
/// Returns event if it caused a state change
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
        event::Events::ButtonPressed(_) | 
        event::Events::ButtonHoldStart(_) | 
        event::Events::ButtonHoldEnd(_) => Some(event),
        event::Events::InactivityTimeout => {
            if !state.standby {
                Some(event)
            } else {
                None
            }
        }
    }
}

/// Processes state changes
async fn handle_state_changes(event: event::Events) {
    match event {
        event::Events::OperationModeSet(new_mode) => match new_mode {
            OperationMode::Manual => {
                indicator::update(true);
                autonomous_command::signal(autonomous_command::Command::Stop);
            }
            OperationMode::Autonomous => {
                indicator::update(true);
                autonomous_command::signal(autonomous_command::Command::Start);
            }
        },
        event::Events::ObstacleDetected(is_detected) => {
            indicator::update(true);
            {
                let state = SYSTEM_STATE.lock().await;
                if state.operation_mode == OperationMode::Autonomous {
                    autonomous_command::signal(if is_detected {
                        autonomous_command::Command::AvoidObstacle
                    } else {
                        autonomous_command::Command::Start
                    });
                }
            }
        }
        event::Events::ObstacleAvoidanceAttempted => {
            indicator::update(true);
            autonomous_command::signal(autonomous_command::Command::AvoidObstacle);
        }
        event::Events::BatteryLevelMeasured(_) => {
            indicator::update(false);
        }
        event::Events::ButtonPressed(button_id) => {
            button_actions::handle_button_action(
                button_id,
                button_actions::ButtonActionType::Press,
            )
            .await;
            activity::signal_activity();
        }
        event::Events::ButtonHoldStart(button_id) => {
            button_actions::handle_button_action(
                button_id,
                button_actions::ButtonActionType::HoldStart,
            )
            .await;
            activity::signal_activity();
        }
        event::Events::ButtonHoldEnd(button_id) => {
            button_actions::handle_button_action(
                button_id,
                button_actions::ButtonActionType::HoldEnd,
            )
            .await;
            activity::signal_activity();
        }
        event::Events::InactivityTimeout => {
            event::send(event::Events::OperationModeSet(
                state::OperationMode::Manual,
            ))
            .await;
            drive_command::update(drive_command::Command::Standby);
        }
    }
}