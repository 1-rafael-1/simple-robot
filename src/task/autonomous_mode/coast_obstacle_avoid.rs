//! Obstacle-avoidance autonomous drive mode: coast until an obstacle interrupts the
//! forward drive, then back up a fixed distance and turn a random angle before resuming.
//!
//! # Control flow
//!
//! ```text
//! start() ──► drive forward (DriveDistance, near-infinite)
//!                  │
//!           obstacle detected
//!           (EmergencyBrake interrupt from obstacle handler)
//!                  │
//!             Cancelled completion
//!                  │
//!          ┌── ACTIVE? ──┐
//!          No            Yes
//!          │             │
//!        brake      back up (DriveDistance, backward)
//!        exit            │
//!                   random turn (RotateExact)
//!                        │
//!                  raise ObstacleAvoidanceAttempted
//!                        │
//!                  ◄─────┘ (loop)
//! ```
//!
//! # Starting and stopping
//!
//! Call [`start`] to begin the mode and [`stop`] to request a graceful exit.
//! [`stop`] also sends an `EmergencyBrake` interrupt to unblock any active drive
//! command immediately.
//!
//! [`is_active`] can be polled by other parts of the system (e.g. the obstacle
//! behavior handler) to decide whether to issue drive interrupts.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use nanorand::{Rng, WyRand};

use crate::{
    system::{
        event::{Events, raise_event},
        state::perception,
    },
    task::{
        autonomous_mode::{self, AutonomousCommand},
        behavior::obstacle as obstacle_behavior,
        drive::{
            CompletionStatus, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind, DriveQueueBuilder,
            DriveQueueSubmitError, InterruptKind, send_drive_command, send_drive_interrupt,
            types::{DriveQueueCompletion, RotationDirection, RotationMotion},
        },
        sensors::ultrasonic::{start_ultrasonic_centered_obstacle_detect, stop_ultrasonic_measurements},
    },
};

// ── Active flag ───────────────────────────────────────────────────────────────

/// Set while the coast-and-avoid loop is running.
///
/// Checked by the obstacle behavior handler to decide whether to issue drive
/// interrupts, and by the loop itself to detect stop requests.
static ACTIVE: AtomicBool = AtomicBool::new(false);

/// Set while the forward-drive phase is active (used to gate obstacle interrupts).
static FORWARD_PHASE: AtomicBool = AtomicBool::new(false);

// ── Tuning constants ──────────────────────────────────────────────────────────

/// Forward distance used to keep the robot driving until interrupted (cm).
///
/// Set to 1 km (100,000 cm) — effectively infinite for any real-world run, but avoids
/// `f32::MAX` overflow in the distance calculations.
const MAX_FORWARD_DISTANCE_CM: f32 = 100_000.0;

/// Distance to reverse after an obstacle is detected (cm).
const BACKUP_DISTANCE_CM: f32 = 20.0;

/// Speed for forward coasting (0–100).
const FORWARD_SPEED: u8 = 80;

/// Speed for reverse backup (0–100).
const REVERSE_SPEED: u8 = 80;

/// In-place rotation speed (0–100).
const TURN_SPEED: u8 = 60;

/// Minimum random turn angle (degrees).
const TURN_ANGLE_MIN: u8 = 45;

/// Maximum random turn angle (degrees).
const TURN_ANGLE_MAX: u8 = 180;

// ── Public API ────────────────────────────────────────────────────────────────

/// Spawn the coast-and-avoid autonomous task.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner) {
    spawner.spawn(coast_obstacle_avoid_task().unwrap());
}

/// Returns `true` while the coast-and-avoid loop is running.
pub fn is_active() -> bool {
    ACTIVE.load(Ordering::Relaxed)
}

/// Returns `true` while the forward-drive phase is active.
pub fn is_forward_phase() -> bool {
    FORWARD_PHASE.load(Ordering::Relaxed)
}

/// Activate the coast-and-avoid autonomous mode.
///
/// Requests mode start through the autonomous mode controller.
pub async fn start() -> bool {
    if !autonomous_mode::request_start(AutonomousCommand::CoastObstacleAvoid).await {
        return false;
    }

    ACTIVE.store(true, Ordering::Relaxed);
    FORWARD_PHASE.store(false, Ordering::Relaxed);
    start_ultrasonic_centered_obstacle_detect();
    true
}

/// Request a graceful stop of the coast-and-avoid mode.
///
/// Clears the active flag so the loop exits after the current drive command
/// resolves, and sends an `EmergencyBrake` interrupt to unblock any in-progress
/// [`DriveDistance`] or [`RotateExact`] command immediately.
pub fn stop() {
    ACTIVE.store(false, Ordering::Relaxed);
    FORWARD_PHASE.store(false, Ordering::Relaxed);
    stop_ultrasonic_measurements();
    send_drive_interrupt(InterruptKind::EmergencyBrake);
}

// ── Task ──────────────────────────────────────────────────────────────────────

/// Coast-and-avoid autonomous drive task.
///
/// Spawned on demand by the autonomous mode controller each time the mode is
/// activated. Runs until deactivated, then returns so the task slot is freed
/// for the next activation. Call [`start`] to begin and [`stop`] to end a run.
#[embassy_executor::task]
pub async fn coast_obstacle_avoid_task() {
    info!("coast-avoid: activated");

    // Ensure a clean starting state.
    send_drive_command(DriveCommand::Drive(DriveAction::Brake)).await;
    Timer::after(Duration::from_millis(200)).await;

    // Main loop: drive forward until interrupted, then avoid, repeat.
    while ACTIVE.load(Ordering::Relaxed) {
        let status = drive_forward().await;

        if !ACTIVE.load(Ordering::Relaxed) {
            // stop() was called; exit without running avoidance.
            break;
        }

        // Only run avoidance if the forward drive was actually cancelled.
        if matches!(status, CompletionStatus::Cancelled) {
            avoid_obstacle().await;
        }
    }

    // Come to a clean stop before returning so the task slot is freed.
    send_drive_command(DriveCommand::Drive(DriveAction::Brake)).await;
    Timer::after(Duration::from_millis(200)).await;
    autonomous_mode::release_autonomous_mode();
    info!("coast-avoid: deactivated");
}

/// Helper guard to mark the forward-drive phase for obstacle gating.
struct ForwardPhaseGuard;

impl ForwardPhaseGuard {
    /// Enter the forward-drive phase.
    fn new() -> Self {
        FORWARD_PHASE.store(true, Ordering::Relaxed);
        Self
    }
}

impl Drop for ForwardPhaseGuard {
    fn drop(&mut self) {
        FORWARD_PHASE.store(false, Ordering::Relaxed);
    }
}

/// Issue a near-infinite `DriveDistance` forward and wait for it to complete
/// (either cancelled by an interrupt or, extremely unlikely, finished).
async fn drive_forward() -> CompletionStatus {
    let _forward_phase = ForwardPhaseGuard::new();

    // If an obstacle was already detected before the forward phase started
    // (e.g. IR event fired during startup), skip the drive entirely and
    // signal cancellation immediately so the avoid loop can react.
    if perception::is_obstacle_detected() {
        info!("coast-avoid: obstacle already detected, skipping forward drive");
        return CompletionStatus::Cancelled;
    }

    info!("coast-avoid: driving forward");

    let mut queue = DriveQueueBuilder::new();
    if queue
        .push(DriveCommand::Drive(DriveAction::DriveDistance {
            kind: DriveDistanceKind::Straight {
                distance_cm: MAX_FORWARD_DISTANCE_CM,
            },
            direction: DriveDirection::Forward,
            speed: FORWARD_SPEED,
        }))
        .is_err()
    {
        info!("coast-avoid: forward queue full");
        return CompletionStatus::Failed("queue full");
    }

    let completion = match queue.submit().await {
        Ok(completion) => completion,
        Err(DriveQueueSubmitError::QueueBusy) => {
            info!("coast-avoid: forward queue busy");
            return CompletionStatus::Cancelled;
        }
    };

    match completion.status {
        CompletionStatus::Cancelled => {
            info!("coast-avoid: forward drive interrupted");
        }
        CompletionStatus::Success => {
            info!("coast-avoid: forward drive finished (unexpected at max distance)");
        }
        CompletionStatus::Failed(reason) => {
            info!("coast-avoid: forward drive failed: {=str}", reason);
        }
    }

    completion.status
}

/// Return the opposite of the given rotation direction.
const fn opposite_direction(dir: RotationDirection) -> RotationDirection {
    match dir {
        RotationDirection::Clockwise => RotationDirection::CounterClockwise,
        RotationDirection::CounterClockwise => RotationDirection::Clockwise,
    }
}

// ── Avoidance helpers ─────────────────────────────────────────────────────────

/// Perform a brake via the drive queue and wait for encoder settle to confirm
/// the robot has physically stopped.
///
/// Without this the `EmergencyBrake` interrupt only brakes momentarily before the
/// reverse drive re-energises the motors, letting inertia carry the robot forward.
async fn brake_and_settle() {
    let mut brake_queue = DriveQueueBuilder::new();
    if brake_queue.push(DriveCommand::Drive(DriveAction::Brake)).is_err() {
        info!("coast-avoid: brake queue full, proceeding with avoidance");
        return;
    }
    match brake_queue.submit().await {
        Ok(completion) => match completion.status {
            CompletionStatus::Success => {
                info!("coast-avoid: encoder-settled brake confirmed, robot stopped");
            }
            CompletionStatus::Cancelled => {
                info!("coast-avoid: brake settle interrupted, proceeding with avoidance");
            }
            CompletionStatus::Failed(reason) => {
                info!("coast-avoid: brake settle failed: {=str}, proceeding", reason);
            }
        },
        Err(DriveQueueSubmitError::QueueBusy) => {
            info!("coast-avoid: brake queue busy, proceeding with avoidance");
        }
    }
}

/// Return a random turn angle between `TURN_ANGLE_MIN` and `TURN_ANGLE_MAX`.
fn random_turn_angle() -> u8 {
    let seed = Instant::now().as_micros();
    let mut rng = WyRand::new_seed(seed);
    rng.generate_range(TURN_ANGLE_MIN..=TURN_ANGLE_MAX)
}

/// Build and submit a drive queue that backs up (if `is_first`) and turns to
/// avoid an obstacle.
///
/// Returns `None` if the queue could not be built or submitted (failure logged
/// internally); returns `Some(completion)` on successful submission.
async fn build_and_submit_avoidance_queue(
    is_first: bool,
    turn_degrees: u8,
    direction: RotationDirection,
) -> Option<DriveQueueCompletion> {
    let mut queue = DriveQueueBuilder::new();

    // First iteration only: back up before turning.
    if is_first
        && queue
            .push(DriveCommand::Drive(DriveAction::DriveDistance {
                kind: DriveDistanceKind::Straight {
                    distance_cm: BACKUP_DISTANCE_CM,
                },
                direction: DriveDirection::Backward,
                speed: REVERSE_SPEED,
            }))
            .is_err()
    {
        info!("coast-avoid: avoidance queue full (backup)");
        return None;
    }

    // Turn.
    if queue
        .push(DriveCommand::Drive(DriveAction::RotateExact {
            degrees: f32::from(turn_degrees),
            direction,
            motion: RotationMotion::Stationary { speed: TURN_SPEED },
        }))
        .is_err()
    {
        info!("coast-avoid: avoidance queue full (turn)");
        return None;
    }

    match queue.submit().await {
        Ok(completion) => Some(completion),
        Err(DriveQueueSubmitError::QueueBusy) => {
            info!("coast-avoid: avoidance queue busy");
            None
        }
    }
}

/// Reset obstacle state and poll the ultrasonic sensor to determine if the
/// path is still obstructed after a turn.
async fn check_path_obstructed() -> bool {
    obstacle_behavior::reset_obstacle_state().await;
    Timer::after(Duration::from_millis(200)).await;
    perception::is_ultrasonic_obstacle_detected()
}

/// Back up a fixed distance and turn a random angle to avoid an obstacle.
///
/// On the first iteration this performs a backup followed by a turn. If the
/// path is still blocked after the turn, the function re-avoids with a turn-only
/// maneuver (using the opposite direction). Loops until the path is clear or the
/// mode is deactivated.
async fn avoid_obstacle() {
    info!("coast-avoid: obstacle avoidance maneuver");

    brake_and_settle().await;

    // Randomly choose initial turn angle and direction.
    let mut turn_degrees = random_turn_angle();
    let mut direction = {
        let seed = Instant::now().as_micros();
        let mut rng = WyRand::new_seed(seed);
        if rng.generate_range(0u8..=1u8) == 0 {
            RotationDirection::CounterClockwise
        } else {
            RotationDirection::Clockwise
        }
    };
    let mut is_first = true;

    loop {
        if !ACTIVE.load(Ordering::Relaxed) {
            info!("coast-avoid: avoidance aborted before queue (stop requested)");
            return;
        }

        info!("coast-avoid: turning {} degrees", turn_degrees);

        let Some(completion) = build_and_submit_avoidance_queue(is_first, turn_degrees, direction).await else {
            return;
        };

        match completion.status {
            CompletionStatus::Cancelled => {
                let failed_step = completion.failed_step_index.unwrap_or(0);
                info!("coast-avoid: avoidance cancelled (step {=usize})", failed_step);
                if !ACTIVE.load(Ordering::Relaxed) {
                    send_drive_interrupt(InterruptKind::EmergencyBrake);
                }
                return;
            }
            CompletionStatus::Failed(reason) => {
                let failed_step = completion.failed_step_index.unwrap_or(0);
                info!(
                    "coast-avoid: avoidance failed: {=str} (step {=usize})",
                    reason, failed_step
                );
                if !ACTIVE.load(Ordering::Relaxed) {
                    send_drive_interrupt(InterruptKind::EmergencyBrake);
                }
                return;
            }
            CompletionStatus::Success => {}
        }

        if !ACTIVE.load(Ordering::Relaxed) {
            send_drive_interrupt(InterruptKind::EmergencyBrake);
            return;
        }

        is_first = false;

        if check_path_obstructed().await {
            turn_degrees = random_turn_angle();
            direction = opposite_direction(direction);
            info!("coast-avoid: obstacle still detected — re-avoiding");
            continue;
        }

        raise_event(Events::ObstacleAvoidanceAttempted).await;
        return;
    }
}
