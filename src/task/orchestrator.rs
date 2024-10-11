use defmt::info;
use defmt::Debug2Format;

use crate::task::system_messages::{Events, EVENT_CHANNEL};
use crate::task::system_state::{OperationMode, SYSTEM_STATE};

#[embassy_executor::task]
pub async fn orchestrator() {
    info!("Orchestrator started");

    let receiver = EVENT_CHANNEL.receiver();

    loop {
        match receiver.receive().await {
            Events::ModeSet(new_mode) => {
                let mode_changed = {
                    let mut state = SYSTEM_STATE.lock().await;
                    if state.operation_mode != new_mode {
                        info!(
                            "Operation mode changed from {:?} to {:?}",
                            Debug2Format(&state.operation_mode),
                            Debug2Format(&new_mode)
                        );
                        state.operation_mode = new_mode;
                        true
                    } else {
                        false
                    }
                };
                if mode_changed {
                    handle_state_changes().await;
                }
            }
            Events::ObstacleDetected(is_obstacle_detected) => {
                let obstacle_changed = {
                    let mut state = SYSTEM_STATE.lock().await;
                    if state.obstacle_detected != is_obstacle_detected {
                        info!(
                            "Obstacle detected changed from {:?} to {:?}",
                            Debug2Format(&state.obstacle_detected),
                            Debug2Format(&is_obstacle_detected)
                        );
                        state.obstacle_detected = is_obstacle_detected;
                        true
                    } else {
                        false
                    }
                };
                if obstacle_changed {
                    handle_state_changes().await;
                }
            }
            Events::BatteryLevelMeasured(battery_level) => {
                let battery_changed = {
                    let mut state = SYSTEM_STATE.lock().await;
                    if state.battery_level != battery_level {
                        info!(
                            "Battery level changed from {:?} to {:?}",
                            Debug2Format(&state.battery_level),
                            Debug2Format(&battery_level)
                        );
                        state.battery_level = battery_level;
                        true
                    } else {
                        false
                    }
                };
                if battery_changed {
                    handle_state_changes().await;
                }
            }
        };
    }
}

async fn handle_state_changes() {
    {
        let state = SYSTEM_STATE.lock().await;
        // Perform actions based on the current state
        match state.operation_mode {
            OperationMode::Manual => {
                info!("Handling Manual mode");
                // Add logic for manual mode
            }
            OperationMode::Autonomous => {
                info!("Handling Autonomous mode");
                // Add logic for autonomous mode
            }
        }
        // You can check other state properties and handle them here
    }
}
