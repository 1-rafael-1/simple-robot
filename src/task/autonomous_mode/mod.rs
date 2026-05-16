//! Autonomous drive mode tasks.
//!
//! Provides on-demand autonomous mode tasks spawned via a controller task.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

pub mod coast_obstacle_avoid;

#[derive(Clone, Copy)]
/// Command sent to the autonomous mode controller.
pub(super) enum AutonomousCommand {
    /// Spawn coast-and-avoid autonomous mode task.
    CoastObstacleAvoid,
}

/// Tracks whether any autonomous mode is currently active.
static AUTONOMOUS_MODE_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Command channel for autonomous mode spawn requests.
static AUTONOMOUS_MODE_COMMAND: Channel<CriticalSectionRawMutex, AutonomousCommand, 4> = Channel::new();

/// Initialize autonomous mode support (spawns the controller task).
#[allow(clippy::unwrap_used)]
pub fn init_autonomous_mode(spawner: Spawner) {
    spawner.spawn(autonomous_mode_controller(spawner).unwrap());
}

/// Request that an autonomous mode be spawned on demand.
/// Returns true if the request was accepted.
pub(super) async fn request_start(command: AutonomousCommand) -> bool {
    if AUTONOMOUS_MODE_ACTIVE
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
        .is_err()
    {
        return false;
    }

    AUTONOMOUS_MODE_COMMAND.send(command).await;
    true
}

/// Mark the autonomous mode controller as idle again.
pub(super) fn release_autonomous_mode() {
    AUTONOMOUS_MODE_ACTIVE.store(false, Ordering::Release);
}

/// Controller task that spawns autonomous mode tasks on demand.
#[embassy_executor::task]
async fn autonomous_mode_controller(spawner: Spawner) {
    loop {
        match AUTONOMOUS_MODE_COMMAND.receive().await {
            AutonomousCommand::CoastObstacleAvoid => coast_obstacle_avoid::spawn(spawner),
        }
    }
}
