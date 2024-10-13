//! Orchestrator Module
//!
//! This module contains the main orchestrator task that manages the robot's overall behavior
//! by handling system events and coordinating state changes.

use crate::task::system_events::{send_system_indicator_changed, wait_for_event, Events};
use crate::task::system_state::{OperationMode, SYSTEM_STATE};
use defmt::info;

/// Main orchestrator task
///
/// This task continuously listens for system events, handles them, and manages state changes.
/// It serves as the central coordinator for the robot's behavior.
#[embassy_executor::task]
pub async fn orchestrator() {
    info!("Orchestrator started");
    loop {
        let changed_state = handle_event(wait_for_event().await).await;
        if let Some(state_change) = changed_state {
            handle_state_changes(state_change).await;
        }
    }
}

/// Handles incoming system events
///
/// This function processes incoming events and updates the system state accordingly.
/// It returns Some(event) if the state was changed, or None if no change occurred.
///
/// # Arguments
///
/// * `event` - The incoming system event to be handled
///
/// # Returns
///
/// * `Option<Events>` - The event that caused a state change, if any
async fn handle_event(event: Events) -> Option<Events> {
    let mut state = SYSTEM_STATE.lock().await;

    match event {
        Events::OperationModeSet(new_mode) => {
            if state.operation_mode != new_mode {
                state.operation_mode = new_mode;
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
        Events::BatteryLevelMeasured(level) => {
            if state.battery_level != level {
                state.battery_level = level;
                Some(event)
            } else {
                None
            }
        }
    }
}

/// Handles state changes resulting from events
///
/// This function is called when a state change occurs, and it performs
/// the necessary actions based on the type of state change.
///
/// # Arguments
///
/// * `event` - The event that caused the state change
async fn handle_state_changes(event: Events) {
    match event {
        Events::OperationModeSet(new_mode) => match new_mode {
            OperationMode::Manual => {
                info!("Handling Manual mode");
            }
            OperationMode::Autonomous => {
                info!("Handling Autonomous mode");
            }
        },
        Events::ObstacleDetected(is_detected) => {
            info!("Handling obstacle detection: {}", is_detected);
        }
        Events::BatteryLevelMeasured(_level) => {
            send_system_indicator_changed(true).await;
        }
    }
}
