use crate::task::system_messages::{Events, EVENT_CHANNEL};
use crate::task::system_state::{OperationMode, SYSTEM_STATE};
use defmt::info;

#[embassy_executor::task]
pub async fn orchestrator() {
    info!("Orchestrator started");
    let receiver = EVENT_CHANNEL.receiver();

    loop {
        let event = receiver.receive().await;
        let changed_state = handle_event(event).await;
        if let Some(state_change) = changed_state {
            handle_state_changes(state_change).await;
        }
    }
}

async fn handle_event(event: Events) -> Option<Events> {
    let mut state = SYSTEM_STATE.lock().await;

    match event {
        Events::ModeSet(new_mode) => {
            if state.operation_mode != new_mode {
                info!(
                    "Operation mode changed from {} to {}",
                    state.operation_mode, new_mode
                );
                state.operation_mode = new_mode;
                Some(event)
            } else {
                None
            }
        }
        Events::ObstacleDetected(is_detected) => {
            if state.obstacle_detected != is_detected {
                info!(
                    "Obstacle detected changed from {} to {}",
                    state.obstacle_detected, is_detected
                );
                state.obstacle_detected = is_detected;
                Some(event)
            } else {
                None
            }
        }
        Events::BatteryLevelMeasured(level) => {
            if state.battery_level != level {
                info!(
                    "Battery level changed from {} to {}",
                    state.battery_level, level
                );
                state.battery_level = level;
                Some(event)
            } else {
                None
            }
        }
    }
}

async fn handle_state_changes(event: Events) {
    match event {
        Events::ModeSet(new_mode) => {
            match new_mode {
                OperationMode::Manual => {
                    info!("Handling Manual mode");
                    // Add logic for manual mode
                }
                OperationMode::Autonomous => {
                    info!("Handling Autonomous mode");
                    // Add logic for autonomous mode
                }
            }
        }
        Events::ObstacleDetected(is_detected) => {
            info!("Handling obstacle detection: {}", is_detected);
            // Add logic for obstacle detection
        }
        Events::BatteryLevelMeasured(level) => {
            info!("Handling battery level change: {}", level);
            // Add logic for battery level changes
        }
    }
}
