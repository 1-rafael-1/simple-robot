use crate::system::{drive_command, event, state};
use embassy_time::{Duration, Timer};
use rand::Rng;

#[embassy_executor::task]
pub async fn autonomous_drive() {
    drive_command::update(drive_command::Command::Coast);
    Timer::after(Duration::from_secs(1)).await;
    drive_command::update(drive_command::Command::Brake);

    loop {
        // drive_command::update(drive_command::Command::Forward(60));
        // Timer::after(Duration::from_secs(5)).await;
        // drive_command::update(drive_command::Command::Coast);
        // Timer::after(Duration::from_secs(1)).await;
        // drive_command::update(drive_command::Command::Brake);
        // Timer::after(Duration::from_secs(1)).await;
        // drive_command::update(drive_command::Command::Left(60));
        // Timer::after(Duration::from_secs(5)).await;
        // drive_command::update(drive_command::Command::Coast);
        // Timer::after(Duration::from_secs(1)).await;
        // drive_command::update(drive_command::Command::Right(60));
        // Timer::after(Duration::from_secs(5)).await;
        // drive_command::update(drive_command::Command::Coast);
        // Timer::after(Duration::from_secs(1)).await;
        // drive_command::update(drive_command::Command::Backward(60));
        // Timer::after(Duration::from_secs(5)).await;
        // drive_command::update(drive_command::Command::Coast);
        // Timer::after(Duration::from_secs(1)).await;
        // // Drive forward
        // drive_command::update(drive_command::Command::Forward(60));

        // // Wait for an obstacle or mode change
        // loop {
        //     match event::wait().await {
        //         event::Events::ObstacleDetected(true) => {
        //             // Obstacle detected, break the inner loop
        //             break;
        //         }
        //         event::Events::OperationModeSet(state::OperationMode::Manual) => {
        //             // Exit autonomous mode
        //             return;
        //         }
        //         _ => {} // Ignore other events
        //     }
        // }

        // // Obstacle detected, stop and back up
        // drive_command::update(drive_command::Command::Brake);
        // Timer::after(Duration::from_millis(500)).await;
        // drive_command::update(drive_command::Command::Backward(100));
        // Timer::after(Duration::from_secs(1)).await;

        // // Turn in a random direction
        // // let turn_direction = if rand::thread_rng().gen_bool(0.5) {
        // //     command::Command::Left(100)
        // // } else {
        // //     command::Command::Right(100)
        // // };
        // // command::send(turn_direction);
        // Timer::after(Duration::from_millis(500)).await;
    }
}
