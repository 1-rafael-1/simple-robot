use defmt::info;
use embassy_time::{Duration, Timer};

use crate::task::system_messages::{Events, EVENT_CHANNEL};
use crate::task::system_state::SystemState;

#[embassy_executor::task]
pub async fn orchestrator() {
    info!("Orchestrator started");

    let state = SystemState::new();
    let mut receiver = EVENT_CHANNEL.receiver();

    loop {
        // Process incoming messages
        let message = match receiver.receive().await {
            Events::ModeChange(mode) => {}
            Events::ObstacleDetected(is_obstacle_detected) => {
                info!("Obstacle detected: {}", is_obstacle_detected);
            }
        };
    }
}
