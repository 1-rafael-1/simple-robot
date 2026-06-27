//! Attempt Straight Line autonomous drive mode: sweep, analyze gaps, drive through best gap,
//! repeat until target distance reached or blocked.
//!
//! # Control flow
//!
//! ```text
//! start(target_cm) ──► SWEEPING ──► DECIDING ──► DRIVING
//!                          ▲              │          │
//!                          │              │          ├── leg complete ──► SWEEPING
//!                          │              │          ├── obstacle interrupt ──► SWEEPING
//!                          │              │          └── target reached ──► FINISHED_REACHED
//!                          │              │
//!                          │              └── no gap >10cm ──► FINISHED_BLOCKED
//!                          │
//!                          └── stop() ──► exit (brake, release, ShowMainMenu)
//! ```

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Timer, with_timeout};

use crate::task::{
    autonomous_mode::{self, gap_analysis},
    drive::{
        CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind,
        DriveQueueBuilder, DriveQueueSubmitError, InterruptKind, send_drive_command, send_drive_interrupt,
        types::{RotationMotion, SPROCKET_CIRCUMFERENCE_CM},
    },
    sensors::ultrasonic::{self, start_ultrasonic_centered_obstacle_detect, stop_ultrasonic_measurements},
    ui::{UiEvent, send_ui_event},
};

// ── Active flag ───────────────────────────────────────────────────────────────

/// Set while the attempt-straight-line loop is running.
static ACTIVE: AtomicBool = AtomicBool::new(false);

// ── Sweep completion signal ───────────────────────────────────────────────────

/// Signalled by the orchestrator when `UltrasonicSweepCompleted` fires.
pub static SWEEP_COMPLETED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// ── Display state ─────────────────────────────────────────────────────────────

/// Shared state visible to the UI for display updates.
pub struct ModeDisplayState {
    /// Total forward progress in cm.
    pub progress_cm: f32,
    /// Target distance in cm.
    pub target_cm: u16,
    /// Accumulated drift in degrees (positive=right, negative=left).
    pub drift_deg: f32,
    /// Current state label for display.
    pub state_label: &'static str,
}

/// Public display state for the UI to read live progress/drift values.
pub static DISPLAY_STATE: Mutex<CriticalSectionRawMutex, ModeDisplayState> = Mutex::new(ModeDisplayState {
    progress_cm: 0.0,
    target_cm: 0,
    drift_deg: 0.0,
    state_label: "Idle",
});

// ── Mode entry constants ──────────────────────────────────────────────────────

/// Minimum target distance (cm).
const TARGET_MIN_CM: u16 = 100;
/// Maximum target distance (cm).
const TARGET_MAX_CM: u16 = 1000;
/// Target distance step size (cm).
const TARGET_STEP_CM: u16 = 10;
/// Preset target distance (cm).
const TARGET_PRESET_CM: u16 = 300;

// ── Tuning constants ──────────────────────────────────────────────────────────

/// Speed for forward driving through gaps (0–100).
const DRIVE_SPEED: u8 = 70;

/// Speed for in-place rotation to face gaps (0–100).
const TURN_SPEED: u8 = 60;

// ── State machine ─────────────────────────────────────────────────────────────

/// Mode state machine.
enum State {
    /// Performing a buffered ultrasonic sweep.
    Sweeping,
    /// Analyzing the sweep buffer for gaps.
    Deciding,
    /// Executing a drive leg through the chosen gap.
    Driving(gap_analysis::GapDecision),
    /// Target distance reached.
    FinishedReached,
    /// No forward path available.
    FinishedBlocked,
}

// ── Public API ────────────────────────────────────────────────────────────────

/// Spawn the attempt-straight-line autonomous task.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner, target_distance_cm: u16) {
    spawner.spawn(attempt_straight_line_task(target_distance_cm).unwrap());
}

/// Returns `true` while the mode is running.
#[allow(dead_code)]
pub fn is_active() -> bool {
    ACTIVE.load(Ordering::Relaxed)
}

/// Request a graceful stop of the mode.
pub fn stop() {
    ACTIVE.store(false, Ordering::Relaxed);
    stop_ultrasonic_measurements();
    send_drive_interrupt(InterruptKind::EmergencyBrake);
}

/// Activate the attempt-straight-line autonomous mode.
pub async fn start(target_distance_cm: u16) -> bool {
    if !autonomous_mode::request_start(autonomous_mode::AutonomousCommand::AttemptStraightLine { target_distance_cm })
        .await
    {
        return false;
    }

    ACTIVE.store(true, Ordering::Relaxed);
    true
}

/// Minimum target distance (cm).
pub const fn target_min_cm() -> u16 {
    TARGET_MIN_CM
}

/// Maximum target distance (cm).
pub const fn target_max_cm() -> u16 {
    TARGET_MAX_CM
}

/// Target distance step size (cm).
pub const fn target_step_cm() -> u16 {
    TARGET_STEP_CM
}

/// Preset target distance (cm).
pub const fn target_preset_cm() -> u16 {
    TARGET_PRESET_CM
}

// ── Task ──────────────────────────────────────────────────────────────────────

#[embassy_executor::task]
#[allow(clippy::too_many_lines)]
pub async fn attempt_straight_line_task(target_distance_cm: u16) {
    info!("attempt-straight: activated, target {} cm", target_distance_cm);

    // Ensure a clean starting state.
    send_drive_command(DriveCommand::Drive(DriveAction::Brake)).await;
    Timer::after(Duration::from_millis(200)).await;

    // Initialize display state.
    {
        let mut ds = DISPLAY_STATE.lock().await;
        ds.progress_cm = 0.0;
        ds.target_cm = target_distance_cm;
        ds.drift_deg = 0.0;
        ds.state_label = "Starting...";
    }

    let mut accumulated_drift: f32 = 0.0;
    let mut total_progress_cm: f32 = 0.0;
    let mut state = State::Sweeping;

    let exit_label = 'exit_label: loop {
        if !ACTIVE.load(Ordering::Relaxed) {
            break "Aborted";
        }

        state = match state {
            State::Sweeping => {
                {
                    let mut ds = DISPLAY_STATE.lock().await;
                    ds.state_label = "Sweeping...";
                }
                info!("attempt-straight: sweeping");
                ultrasonic::start_buffered_sweep();

                // Wait for sweep completion, polling ACTIVE so we can abort cleanly.
                loop {
                    match with_timeout(Duration::from_millis(100), SWEEP_COMPLETED.wait()).await {
                        Ok(()) => break,
                        Err(_) => {
                            if !ACTIVE.load(Ordering::Relaxed) {
                                break 'exit_label "Aborted";
                            }
                        }
                    }
                }

                if !ACTIVE.load(Ordering::Relaxed) {
                    break "Aborted";
                }

                State::Deciding
            }
            State::Deciding => {
                {
                    let mut ds = DISPLAY_STATE.lock().await;
                    ds.state_label = "Deciding...";
                }
                info!("attempt-straight: deciding");

                let remaining = f32::from(target_distance_cm) - total_progress_cm;
                let correction_angle = 80.0 - accumulated_drift;

                let buffer = ultrasonic::SWEEP_BUFFER.lock().await;
                let decision = gap_analysis::analyze_gaps(&buffer, correction_angle, remaining);
                drop(buffer);

                decision.map_or_else(
                    || {
                        info!("attempt-straight: blocked — no path");
                        State::FinishedBlocked
                    },
                    |gap| {
                        info!(
                            "attempt-straight: gap chosen, mid={}, rot={}, drive={} cm",
                            gap.servo_midpoint_deg, gap.rotation_degrees, gap.drive_distance_cm
                        );
                        State::Driving(gap)
                    },
                )
            }
            State::Driving(gap) => {
                {
                    let mut ds = DISPLAY_STATE.lock().await;
                    ds.state_label = "Driving...";
                }
                info!("attempt-straight: driving leg");

                // Build drive queue: rotate to gap midpoint, then drive straight.
                let mut queue = DriveQueueBuilder::new();

                // If rotation is needed (> 1 degree threshold to avoid jitter).
                if gap.rotation_degrees > 1.0
                    && queue
                        .push(DriveCommand::Drive(DriveAction::RotateExact {
                            degrees: gap.rotation_degrees,
                            direction: gap.rotation_direction,
                            motion: RotationMotion::Stationary { speed: TURN_SPEED },
                        }))
                        .is_err()
                {
                    info!("attempt-straight: queue full (rotate)");
                    break "Error: queue full";
                }

                if queue
                    .push(DriveCommand::Drive(DriveAction::DriveDistance {
                        kind: DriveDistanceKind::Straight {
                            distance_cm: gap.drive_distance_cm,
                        },
                        direction: DriveDirection::Forward,
                        speed: DRIVE_SPEED,
                    }))
                    .is_err()
                {
                    info!("attempt-straight: queue full (drive)");
                    break "Error: queue full";
                }

                // Set ultrasonic to center with obstacle detection for emergency
                // braking during the forward drive phase.
                start_ultrasonic_centered_obstacle_detect();

                let completion = match queue.submit().await {
                    Ok(completion) => completion,
                    Err(DriveQueueSubmitError::QueueBusy) => {
                        info!("attempt-straight: queue busy");
                        break "Error: queue busy";
                    }
                };

                // Extract partial progress from telemetry regardless of status.
                // Cancellation (EmergencyBrake) still returns DriveDistance telemetry
                // with achieved_revs from the partial leg.
                let progress = extract_progress_cm(&completion);
                total_progress_cm += progress;

                // Update accumulated drift: offset from center (80°).
                let offset = 80.0 - gap.servo_midpoint_deg;
                accumulated_drift += offset;

                // Update display state.
                {
                    let mut ds = DISPLAY_STATE.lock().await;
                    ds.progress_cm = total_progress_cm;
                    ds.drift_deg = accumulated_drift;
                }

                let leg_status = if matches!(completion.status, CompletionStatus::Success) {
                    "done"
                } else {
                    "interrupted"
                };

                info!(
                    "attempt-straight: leg {} (progress {} cm, total {} cm, drift {})",
                    leg_status, progress, total_progress_cm, accumulated_drift
                );

                if !ACTIVE.load(Ordering::Relaxed) {
                    break "Aborted";
                }

                if total_progress_cm >= f32::from(target_distance_cm) {
                    State::FinishedReached
                } else {
                    State::Sweeping
                }
            }
            State::FinishedReached => break "Target reached",
            State::FinishedBlocked => break "Blocked - finished",
        };
    };

    finish(exit_label).await;
}

/// Extract forward progress in cm from a `DriveQueueCompletion`.
///
/// Checks the telemetry variant directly (not the completion status), so partial
/// progress is extracted even for cancelled legs where `EmergencyBrake` interrupted
/// a `DriveDistance` intent — `cancellation_telemetry()` returns
/// `CompletionTelemetry::DriveDistance` with the actual achieved revolutions.
fn extract_progress_cm(completion: &crate::task::drive::types::DriveQueueCompletion) -> f32 {
    completion
        .last_step_completion
        .as_ref()
        .map_or(0.0, |last| match &last.telemetry {
            CompletionTelemetry::DriveDistance {
                achieved_left_revs,
                achieved_right_revs,
                ..
            } => {
                let avg_revs = (achieved_left_revs + achieved_right_revs) / 2.0;
                avg_revs * SPROCKET_CIRCUMFERENCE_CM
            }
            _ => 0.0,
        })
}

/// Clean up and exit the mode, showing the finish message for 2 seconds.
async fn finish(label: &'static str) {
    // Update display state with final message.
    {
        let mut ds = DISPLAY_STATE.lock().await;
        ds.state_label = label;
    }
    send_drive_command(DriveCommand::Drive(DriveAction::Brake)).await;
    Timer::after(Duration::from_millis(200)).await;
    autonomous_mode::release_autonomous_mode();

    // Wait 2 seconds to show the final message on the display.
    Timer::after(Duration::from_secs(2)).await;

    // Return to main menu.
    send_ui_event(UiEvent::ShowMainMenu).await;
    info!("attempt-straight: deactivated ({})", label);
}
