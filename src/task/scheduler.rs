use crate::task::orchestrator::orchestrator;
use embassy_executor::Spawner;

#[embassy_executor::task]
pub async fn scheduler(spawner: Spawner) {
    spawner.spawn(orchestrator(spawner)).unwrap();
    // Spawn other tasks as needed
}
