use crate::task::system_messages::{Events, EVENT_WATCH};
use crate::task::system_state::SystemState;

#[embassy_executor::task]
pub async fn orchestrator() {
    let state = SystemState::new();
    let mut receiver = EVENT_WATCH.receiver().unwrap();

    loop {
        // Process incoming messages
        let message = match receiver.changed().await {
            Events::ModeChange(mode) => {}
            Events::ObstacleDetected(is_obstacle_detected) => {}
        };
    }
}
