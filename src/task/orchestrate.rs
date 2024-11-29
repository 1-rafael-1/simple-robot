//! System orchestration
//!
//! Manages robot behavior by coordinating state changes and event handling.

use crate::system::button_actions;
use crate::system::event;
use crate::system::state;
use crate::system::state::{OperationMode, SYSTEM_STATE};
use crate::task::autonomous_drive;
use crate::task::drive;
use crate::task::rgb_led_indicate;
use crate::task::track_inactivity;

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

/// Processes state changes
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
