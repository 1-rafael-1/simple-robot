use defmt::info;
use defmt::Debug2Format;

use crate::task::system_messages::{Events, EVENT_CHANNEL};
use crate::task::system_state::{OperationMode, SYSTEM_STATE};

#[embassy_executor::task]
pub async fn orchestrator() {
    info!("Orchestrator started");

    let receiver = EVENT_CHANNEL.receiver();

    loop {
        let event = receiver.receive().await;
        if handle_event(event).await {
            handle_state_changes().await;
        }
    }
}

async fn handle_event(event: Events) -> bool {
    let mut state = SYSTEM_STATE.lock().await;
    match event {
        Events::ModeSet(new_mode) => {
            update_state_field(&mut state.operation_mode, new_mode, "Operation mode")
        }
        Events::ObstacleDetected(is_detected) => update_state_field(
            &mut state.obstacle_detected,
            is_detected,
            "Obstacle detected",
        ),
        Events::BatteryLevelMeasured(level) => {
            update_state_field(&mut state.battery_level, level, "Battery level")
        }
    }
}

fn update_state_field<T: PartialEq + defmt::Format>(
    field: &mut T,
    new_value: T,
    field_name: &str,
) -> bool {
    if *field != new_value {
        info!("{} changed from {} to {}", field_name, field, &new_value);
        *field = new_value;
        true
    } else {
        false
    }
}

async fn handle_state_changes() {
    let state = SYSTEM_STATE.lock().await;
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
