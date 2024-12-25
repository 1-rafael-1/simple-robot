//! Autonomous driving behavior
//!
//! Controls robot movement patterns and obstacle avoidance via commands.

use crate::system::event;
use crate::task::drive;
use defmt::info;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use nanorand::{Rng, WyRand};

/// Control signal for autonomous operations
static AUTONOMOUS_CONTROL: Signal<CriticalSectionRawMutex, Command> = Signal::new();

/// Available commands for autonomous control
#[derive(Debug, Clone)]
pub enum Command {
    /// Initialize
    Initialize,
    /// Start driving
    Start,
    /// Stop driving
    Stop,
    /// Handle detected obstacle
    AvoidObstacle,
}

/// Sends command to autonomous control
pub fn send_command(command: Command) {
    AUTONOMOUS_CONTROL.signal(command);
}

/// Waits for next command
async fn wait_command() -> Command {
    AUTONOMOUS_CONTROL.wait().await
}

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
    drive::send_command(drive::Command::Drive(drive::DriveAction::Coast));
    Timer::after(Duration::from_secs(1)).await;
    drive::send_command(drive::Command::Drive(drive::DriveAction::Brake));

    loop {
        match wait_command().await {
            Command::Initialize => {
                drive::send_command(drive::Command::Drive(drive::DriveAction::Coast));
                Timer::after(Duration::from_millis(100)).await;
            }
            Command::Start => {
                info!("Autonomous forward");
                drive::send_command(drive::Command::Drive(drive::DriveAction::Forward(
                    FORWARD_SPEED,
                )));
            }
            Command::Stop => {
                info!("Autonomous stop");
                drive::send_command(drive::Command::Drive(drive::DriveAction::Brake));
                Timer::after(Duration::from_millis(200)).await;
                continue;
            }
            Command::AvoidObstacle => {
                info!("Autonomous obstacle avoid");

                // Emergency Stop
                info!("emergency stop");
                drive::send_command(drive::Command::Drive(drive::DriveAction::Brake));
                Timer::after(Duration::from_millis(500)).await;

                // Back up
                info!("backing up");
                drive::send_command(drive::Command::Drive(drive::DriveAction::Backward(
                    REVERSE_SPEED,
                )));
                Timer::after(BACKUP_DURATION).await;
                drive::send_command(drive::Command::Drive(drive::DriveAction::Brake));
                Timer::after(Duration::from_millis(100)).await;

                // Random turn
                // Use current time since boot as seed
                let seed = Instant::now().as_micros() as u64;
                let mut rng = WyRand::new_seed(seed);

                info!("turning");
                let turn_speed = rng.generate_range(TURN_SPEED_MIN..=100);
                let turn_duration = Duration::from_millis(rng.generate_range(500..=1500));
                if rng.generate_range(0..=1) == 0 {
                    drive::send_command(drive::Command::Drive(drive::DriveAction::Left(turn_speed)))
                } else {
                    drive::send_command(drive::Command::Drive(drive::DriveAction::Right(
                        turn_speed,
                    )))
                };
                Timer::after(turn_duration).await;
                drive::send_command(drive::Command::Drive(drive::DriveAction::Brake));
                Timer::after(Duration::from_millis(100)).await;

                // Report complete evasion (we may still or again have an obstacle)
                event::send(event::Events::ObstacleAvoidanceAttempted).await;
            }
        }
    }
}
