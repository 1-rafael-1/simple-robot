use embassy_executor::Spawner;
use crate::task::orchestrator::orchestrator;

#[embassy_executor::task]
pub async fn scheduler(spawner: Spawner) {
    spawner.spawn(orchestrator(spawner)).unwrap();
    // Spawn other tasks as needed
}
