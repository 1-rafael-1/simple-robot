//! Orchestrator Module
//!
//! This module contains the main orchestrator task that manages the robot's overall behavior
//! by handling system events and coordinating state changes.

use crate::system::button_actions;
use crate::system::event;
use crate::system::indicator;
use crate::system::state::{OperationMode, SYSTEM_STATE};
use defmt::info;

/// Main orchestrator task
///
/// This task continuously listens for system events, processes them, and manages state changes.
/// It serves as the central coordinator for the robot's behavior.
#[embassy_executor::task]
pub async fn orchestrate() {
    info!("Orchestrator started");
    loop {
        // wait for an event
        let event = event::wait().await;
        // process the event and if necessary, react to it
        if let Some(state_change) = process_event(event).await {
            handle_state_changes(state_change).await;
        }
    }
}

/// Processes incoming system events and determines if they result in a state change
///
/// This function processes incoming events and updates the system state accordingly.
/// It returns Some(event) if the event resulted in a state change, or None if no change occurred.
///
/// # Arguments
///
/// * `event` - The incoming system event to be processed
///
/// # Returns
///
/// * `Option<Events>` - The event that caused a state change, if any
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
        event::Events::BatteryLevelMeasured(level) => {
            if state.battery_level != level {
                state.battery_level = level;
                Some(event)
            } else {
                None
            }
        }
        event::Events::ButtonPressed(_button_id) => Some(event),
        event::Events::ButtonHoldStart(_button_id) => Some(event),
        event::Events::ButtonHoldEnd(_button_id) => Some(event),
    }
}

/// Handles state changes resulting from events
///
/// This function is called when a state change occurs, and it performs
/// the necessary actions based on the type of state change.
///
/// # Arguments
///
/// * `state_change` - The event that caused the state change
async fn handle_state_changes(event: event::Events) {
    match event {
        event::Events::OperationModeSet(new_mode) => match new_mode {
            OperationMode::Manual => {
                info!("Handling Manual mode");
                indicator::update(true);
                // TODO: Implement manual mode
            }
            OperationMode::Autonomous => {
                info!("Handling Autonomous mode");
                indicator::update(true);
                // TODO: Implement autonomous mode
            }
        },
        event::Events::ObstacleDetected(is_detected) => {
            info!("Handling obstacle detection: {}", is_detected);
            // TODO: Implement obstacle avoidance
            indicator::update(true);
        }
        event::Events::BatteryLevelMeasured(_level) => {
            indicator::update(false);
        }
        event::Events::ButtonPressed(button_id) => {
            info!("Handling button {} press", button_id);
            button_actions::handle_button_action(
                button_id,
                button_actions::ButtonActionType::Press,
            )
            .await;
        }
        event::Events::ButtonHoldStart(button_id) => {
            info!("Handling button {} hold start", button_id);
            button_actions::handle_button_action(
                button_id,
                button_actions::ButtonActionType::HoldStart,
            )
            .await;
        }
        event::Events::ButtonHoldEnd(button_id) => {
            info!("Handling button {} hold end", button_id);
            button_actions::handle_button_action(
                button_id,
                button_actions::ButtonActionType::HoldEnd,
            )
            .await;
        }
    }
}
