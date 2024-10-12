use crate::task::system_events::{send_system_indicator_changed, wait_for_event, Events};
use crate::task::system_state::{OperationMode, SYSTEM_STATE};
use defmt::info;

#[embassy_executor::task]
pub async fn orchestrator() {
    info!("Orchestrator started");
    // let receiver = EVENT_CHANNEL.receiver();

    loop {
        // let event = wait_for_event().await;
        let changed_state = handle_event(wait_for_event().await).await;
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

async fn handle_state_changes(event: Events) {
    match event {
        Events::ModeSet(new_mode) => match new_mode {
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
