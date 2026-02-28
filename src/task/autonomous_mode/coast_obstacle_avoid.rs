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
    system::event::{Events, raise_event},
    task::drive::{
        CompletionStatus, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind, InterruptKind,
        acquire_completion_handle, completion_sender, release_completion_handle, send_drive_command,
        send_drive_command_with_completion, send_drive_interrupt,
        types::{RotationDirection, RotationMotion},
        wait_for_completion,
    },
};

// ── Active flag ───────────────────────────────────────────────────────────────

/// Set while the coast-and-avoid loop is running.
///
/// Checked by the obstacle behavior handler to decide whether to issue drive
/// interrupts, and by the loop itself to detect stop requests.
static ACTIVE: AtomicBool = AtomicBool::new(false);

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

/// Activate the coast-and-avoid autonomous mode.
///
/// Sets the active flag and wakes the task that is waiting on [`START_SIGNAL`].
pub fn start() {
    ACTIVE.store(true, Ordering::Relaxed);
    START_SIGNAL.signal(());
}

/// Request a graceful stop of the coast-and-avoid mode.
///
/// Clears the active flag so the loop exits after the current drive command
/// resolves, and sends an `EmergencyBrake` interrupt to unblock any in-progress
/// [`DriveDistance`] or [`RotateExact`] command immediately.
pub fn stop() {
    ACTIVE.store(false, Ordering::Relaxed);
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

// ── Private helpers ───────────────────────────────────────────────────────────

/// Issue a near-infinite `DriveDistance` forward and wait for it to complete
/// (either cancelled by an interrupt or, extremely unlikely, finished).
async fn drive_forward() -> CompletionStatus {
    info!("coast-avoid: driving forward");

    let Ok(handle) = acquire_completion_handle().await else {
        // Pool exhausted — log and wait briefly so the loop doesn't spin hot.
        defmt::warn!("coast-avoid: completion pool exhausted, skipping forward step");
        Timer::after(Duration::from_millis(500)).await;
        return CompletionStatus::Failed("completion pool exhausted");
    };

    let sender = completion_sender(handle);
    send_drive_command_with_completion(
        DriveCommand::Drive(DriveAction::DriveDistance {
            kind: DriveDistanceKind::Straight {
                distance_cm: MAX_FORWARD_DISTANCE_CM,
            },
            direction: DriveDirection::Forward,
            speed: FORWARD_SPEED,
        }),
        sender,
    )
    .await;

    let completion = wait_for_completion(&handle).await;
    release_completion_handle(handle).await;

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

    // ── Back up ───────────────────────────────────────────────────────────────
    info!("coast-avoid: backing up {=f32} cm", BACKUP_DISTANCE_CM);

    let Ok(handle) = acquire_completion_handle().await else {
        defmt::warn!("coast-avoid: completion pool exhausted during backup");
        Timer::after(Duration::from_millis(200)).await;
        return;
    };

    let sender = completion_sender(handle);
    send_drive_command_with_completion(
        DriveCommand::Drive(DriveAction::DriveDistance {
            kind: DriveDistanceKind::Straight {
                distance_cm: BACKUP_DISTANCE_CM,
            },
            direction: DriveDirection::Backward,
            speed: REVERSE_SPEED,
        }),
        sender,
    )
    .await;

    let completion = wait_for_completion(&handle).await;
    release_completion_handle(handle).await;

    match completion.status {
        CompletionStatus::Cancelled => {
            info!("coast-avoid: backup cancelled");
            return;
        }
        CompletionStatus::Failed(reason) => {
            info!("coast-avoid: backup failed: {=str}", reason);
        }
        CompletionStatus::Success => {}
    }

    if !ACTIVE.load(Ordering::Relaxed) {
        return;
    }

    Timer::after(Duration::from_millis(100)).await;

    // ── Random turn ───────────────────────────────────────────────────────────
    // Seed the RNG from the current uptime so successive calls differ.
    let seed = Instant::now().as_micros();
    let mut rng = WyRand::new_seed(seed);

    let turn_degrees = rng.generate_range(TURN_ANGLE_MIN..=TURN_ANGLE_MAX);
    let direction = if rng.generate_range(0u8..=1u8) == 0 {
        RotationDirection::CounterClockwise
    } else {
        RotationDirection::Clockwise
    };

    info!("coast-avoid: turning {} degrees", turn_degrees);

    let Ok(handle) = acquire_completion_handle().await else {
        defmt::warn!("coast-avoid: completion pool exhausted during turn");
        return;
    };

    let sender = completion_sender(handle);
    send_drive_command_with_completion(
        DriveCommand::Drive(DriveAction::RotateExact {
            degrees: f32::from(turn_degrees),
            direction,
            motion: RotationMotion::Stationary { speed: TURN_SPEED },
        }),
        sender,
    )
    .await;

    let completion = wait_for_completion(&handle).await;
    release_completion_handle(handle).await;

    match completion.status {
        CompletionStatus::Cancelled => {
            info!("coast-avoid: turn cancelled");
            return;
        }
        CompletionStatus::Failed(reason) => {
            info!("coast-avoid: turn failed: {=str}", reason);
        }
        CompletionStatus::Success => {}
    }

    if !ACTIVE.load(Ordering::Relaxed) {
        return;
    }

    Timer::after(Duration::from_millis(100)).await;

    // Notify the rest of the system that one avoidance cycle completed.
    raise_event(Events::ObstacleAvoidanceAttempted).await;
}
