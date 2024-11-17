//! Autonomous driving behavior
//!
//! Handles robot movement patterns and obstacle avoidance.

use crate::system::autonomous_command;
use crate::system::drive_command;
use crate::system::event;
use defmt::info;
use embassy_time::{Duration, Timer};
use nanorand::{Rng, WyRand};

/// Duration for obstacle backup (ms)
const BACKUP_DURATION: Duration = Duration::from_millis(1000);
/// Standard forward speed
const FORWARD_SPEED: u8 = 80;
/// Backup maneuver speed
const REVERSE_SPEED: u8 = 80;
/// Minimum turn speed
const TURN_SPEED_MIN: u8 = 80;

/// Autonomous driving control task
#[embassy_executor::task]
pub async fn autonomous_drive() {
    // Initial stop sequence
    drive_command::update(drive_command::Command::Coast);
    Timer::after(Duration::from_secs(1)).await;
    drive_command::update(drive_command::Command::Brake);

    let mut rng = WyRand::new_seed(0x1234_5678_9abc_def0);

    loop {
        match autonomous_command::wait().await {
            autonomous_command::Command::Initialize => {
                drive_command::update(drive_command::Command::Coast);
                Timer::after(Duration::from_millis(100)).await;
            }
            autonomous_command::Command::Start => {
                info!("Autonomous forward");
                drive_command::update(drive_command::Command::Forward(FORWARD_SPEED));
            }
            autonomous_command::Command::Stop => {
                info!("Autonomous stop");
                drive_command::update(drive_command::Command::Brake);
                Timer::after(Duration::from_millis(200)).await;
                continue;
            }
            autonomous_command::Command::AvoidObstacle => {
                info!("Autonomous obstacle avoid");
                // Emergency stop
                info!("emergency stop");
                drive_command::update(drive_command::Command::Brake);
                Timer::after(Duration::from_millis(500)).await;

                // Back up
                info!("backing up");
                drive_command::update(drive_command::Command::Backward(REVERSE_SPEED));
                Timer::after(BACKUP_DURATION).await;
                drive_command::update(drive_command::Command::Brake);
                Timer::after(Duration::from_millis(100)).await;

                // Random turn
                info!("turning");
                let turn_speed = rng.generate_range(TURN_SPEED_MIN..=100);
                let turn_duration = Duration::from_millis(rng.generate_range(500..=1500));
                if rng.generate_range(0..=1) == 0 {
                    drive_command::update(drive_command::Command::Left(turn_speed))
                } else {
                    drive_command::update(drive_command::Command::Right(turn_speed))
                };
                Timer::after(turn_duration).await;
                drive_command::update(drive_command::Command::Brake);
                Timer::after(Duration::from_millis(100)).await;

                // Report complete evasion (we may still or again have an obstacle)
                event::send(event::Events::ObstacleAvoidanceAttempted).await;
            }
        }
    }
}
