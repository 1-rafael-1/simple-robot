//! Autonomous driving behavior
//!
//! Controls robot movement patterns and obstacle avoidance via commands.

use defmt::info;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use nanorand::{Rng, WyRand};

use crate::{
    system::event::{Events, send_event},
    task::drive::{DriveAction, DriveCommand, RotationDirection, RotationMotion, send_drive_command},
};

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
pub fn send_autonomous_command(command: Command) {
    AUTONOMOUS_CONTROL.signal(command);
}

/// Waits for next command
async fn wait() -> Command {
    AUTONOMOUS_CONTROL.wait().await
}

/// Duration for obstacle backup (ms)
const BACKUP_DURATION: Duration = Duration::from_millis(1000);
/// Standard forward speed
const FORWARD_SPEED: u8 = 80;
/// Backup maneuver speed
const REVERSE_SPEED: u8 = 80;
/// Minimum turn angle
const TURN_ANGLE_MIN: u8 = 45;
/// Maximum turn angle
const TURN_ANGLE_MAX: u8 = 180;

/// Autonomous driving control task
#[embassy_executor::task]
pub async fn autonomous_drive() {
    // Initial stop sequence
    send_drive_command(DriveCommand::Drive(DriveAction::Coast));
    Timer::after(Duration::from_secs(1)).await;
    send_drive_command(DriveCommand::Drive(DriveAction::Brake));

    loop {
        match wait().await {
            Command::Initialize => {
                send_drive_command(DriveCommand::Drive(DriveAction::Coast));
                Timer::after(Duration::from_millis(100)).await;
            }
            Command::Start => {
                info!("Autonomous forward");
                send_drive_command(DriveCommand::Drive(DriveAction::Forward(FORWARD_SPEED)));
            }
            Command::Stop => {
                info!("Autonomous stop");
                send_drive_command(DriveCommand::Drive(DriveAction::Brake));
                Timer::after(Duration::from_millis(200)).await;
                continue;
            }
            Command::AvoidObstacle => {
                info!("Autonomous obstacle avoid");

                // Emergency Stop
                info!("emergency stop");
                send_drive_command(DriveCommand::Drive(DriveAction::Brake));
                Timer::after(Duration::from_millis(500)).await;

                // Back up
                info!("backing up");
                send_drive_command(DriveCommand::Drive(DriveAction::Backward(REVERSE_SPEED)));
                Timer::after(BACKUP_DURATION).await;
                send_drive_command(DriveCommand::Drive(DriveAction::Brake));
                Timer::after(Duration::from_millis(100)).await;

                // Random turn
                // Use current time since boot as seed
                let seed = Instant::now().as_micros();
                let mut rng = WyRand::new_seed(seed);

                info!("turning");
                let turn_angle = rng.generate_range(TURN_ANGLE_MIN..=TURN_ANGLE_MAX);
                let rotation_motion = RotationMotion::Stationary;
                let rotation_direction = if rng.generate_range(0..=1) == 0 {
                    RotationDirection::CounterClockwise
                } else {
                    RotationDirection::Clockwise
                };
                send_drive_command(DriveCommand::Drive(DriveAction::RotateExact {
                    degrees: turn_angle as f32,
                    direction: rotation_direction,
                    motion: rotation_motion,
                }));

                Timer::after(Duration::from_millis(100)).await;

                // Report complete evasion (we may still or again have an obstacle)
                send_event(Events::ObstacleAvoidanceAttempted).await;
            }
        }
    }
}
