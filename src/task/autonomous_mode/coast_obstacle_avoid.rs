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
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use nanorand::{Rng, WyRand};

use crate::{
    system::{
        event::{Events, raise_event},
        state::perception,
    },
    task::{
        drive::{
            CompletionStatus, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind, DriveQueueBuilder,
            DriveQueueSubmitError, InterruptKind, send_drive_command, send_drive_interrupt,
            types::{RotationDirection, RotationMotion},
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

/// Signals the task to begin its loop (sent by [`start`]).
static START_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

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
/// Sets the active flag and wakes the task that is waiting on [`START_SIGNAL`].
pub fn start() {
    ACTIVE.store(true, Ordering::Relaxed);
    FORWARD_PHASE.store(false, Ordering::Relaxed);
    start_ultrasonic_centered_obstacle_detect();
    START_SIGNAL.signal(());
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
/// Spawned once at startup and waits indefinitely between activations.
/// Call [`start`] to begin and [`stop`] to end a run.
#[embassy_executor::task]
pub async fn coast_obstacle_avoid_task() {
    loop {
        // Block until start() fires the signal.
        START_SIGNAL.wait().await;
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

        // Come to a clean stop before waiting for the next start signal.
        send_drive_command(DriveCommand::Drive(DriveAction::Brake)).await;
        Timer::after(Duration::from_millis(200)).await;
        info!("coast-avoid: deactivated");
    }
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

/// Back up a fixed distance and then turn a random angle before resuming.
async fn avoid_obstacle() {
    info!("coast-avoid: obstacle avoidance maneuver");

    // Brief pause to let the emergency-brake settle.
    Timer::after(Duration::from_millis(200)).await;

    // Randomly choose a turn angle and direction
    let seed = Instant::now().as_micros();
    let mut rng = WyRand::new_seed(seed);
    let turn_degrees = rng.generate_range(TURN_ANGLE_MIN..=TURN_ANGLE_MAX);
    let direction = if rng.generate_range(0u8..=1u8) == 0 {
        RotationDirection::CounterClockwise
    } else {
        RotationDirection::Clockwise
    };

    info!("coast-avoid: turning {} degrees", turn_degrees);

    let mut queue = DriveQueueBuilder::new();
    if queue
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
        return;
    }

    if queue
        .push(DriveCommand::Drive(DriveAction::RotateExact {
            degrees: f32::from(turn_degrees),
            direction,
            motion: RotationMotion::Stationary { speed: TURN_SPEED },
        }))
        .is_err()
    {
        info!("coast-avoid: avoidance queue full (turn)");
        return;
    }

    let completion = match queue.submit().await {
        Ok(completion) => completion,
        Err(DriveQueueSubmitError::QueueBusy) => {
            info!("coast-avoid: avoidance queue busy");
            return;
        }
    };

    match completion.status {
        CompletionStatus::Cancelled => {
            let failed_step = completion.failed_step_index.unwrap_or(0);
            info!("coast-avoid: avoidance cancelled (step {=usize})", failed_step);
            return;
        }
        CompletionStatus::Failed(reason) => {
            let failed_step = completion.failed_step_index.unwrap_or(0);
            info!(
                "coast-avoid: avoidance failed: {=str} (step {=usize})",
                reason, failed_step
            );
            return;
        }
        CompletionStatus::Success => {}
    }

    if !ACTIVE.load(Ordering::Relaxed) {
        return;
    }

    Timer::after(Duration::from_millis(100)).await;

    {
        let mut state = perception::PERCEPTION_STATE.lock().await;
        state.ultrasonic_reading = None;
    }

    // Notify the rest of the system that one avoidance cycle completed.
    raise_event(Events::ObstacleAvoidanceAttempted).await;
}
