//! Autonomous Drive Module
//!
//! This module handles the autonomous driving behavior of the robot.
//! It responds to autonomous commands for starting, stopping, and obstacle avoidance.
//!
use crate::system::autonomous_command;
use crate::system::drive_command;
use embassy_time::{Duration, Timer};
use nanorand::{Rng, WyRand};

/// Duration to back up when obstacle is detected
const BACKUP_DURATION: Duration = Duration::from_millis(1000);
/// Standard forward speed for autonomous operation
const FORWARD_SPEED: u8 = 80;
/// Speed used when backing away from obstacles
const REVERSE_SPEED: u8 = 80;
/// Minimum turn speed to ensure effective rotation
const TURN_SPEED_MIN: u8 = 80;

/// Main autonomous driving task
///
/// This task manages the robot's autonomous movement patterns:
/// - Initializes by ensuring the robot is stopped
/// - Responds to Start/Stop commands
/// - Handles obstacle avoidance with a sequence of:
///   1. Emergency stop
///   2. Backup maneuver
///   3. Random direction turn
///   
/// Note: Forward motion after obstacle avoidance is handled by the
/// queued Start command from the orchestrator, rather than being
/// explicitly commanded here.
#[embassy_executor::task]
pub async fn autonomous_drive() {
    // stop in case we were moving
    drive_command::update(drive_command::Command::Coast);
    Timer::after(Duration::from_secs(1)).await;
    drive_command::update(drive_command::Command::Brake);

    // initialize random number generator
    let mut rng = WyRand::new_seed(0x1234_5678_9abc_def0);

    loop {
        match autonomous_command::wait().await {
            autonomous_command::Command::Start => {
                drive_command::update(drive_command::Command::Forward(FORWARD_SPEED));
            }
            autonomous_command::Command::Stop => {
                drive_command::update(drive_command::Command::Brake);
                Timer::after(Duration::from_millis(200)).await;
                continue;
            }
            autonomous_command::Command::AvoidObstacle => {
                // Emergency stop
                drive_command::update(drive_command::Command::Brake);
                Timer::after(Duration::from_millis(500)).await;

                // Back up
                drive_command::update(drive_command::Command::Backward(REVERSE_SPEED));
                Timer::after(BACKUP_DURATION).await;
                drive_command::update(drive_command::Command::Brake);
                Timer::after(Duration::from_millis(100)).await;

                // make a random turn
                let turn_speed = rng.generate_range(TURN_SPEED_MIN..=100);
                let turn_duration = Duration::from_millis(rng.generate_range(500..=1500));
                if rng.generate_range(0..=1) == 0 {
                    drive_command::update(drive_command::Command::Left(turn_speed))
                } else {
                    drive_command::update(drive_command::Command::Right(turn_speed))
                };
                Timer::after(turn_duration).await;

                // resume -> depending on what we received last while turning we either
                // - stop: if we received a stop command
                // - resume forward: if we received a start command because no more obstacle detected
                // - repeat obstacle avoidance: if we received another obstacle detected command
            }
        }
    }
}
