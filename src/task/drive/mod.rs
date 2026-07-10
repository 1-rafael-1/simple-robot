//! Drive system coordination and intent execution.
//!
//! This module owns the drive task and its supporting components. It coordinates
//! sensor feedback, drive intents, and motor commands without consuming the global
//! event stream directly.
//!
//! # Architecture
//!
//! The drive subsystem is structured around **intents** — state machines that own
//! a specific motion behaviour (rotation, distance, brake/coast, idle). The
//! [`dispatch`] module routes incoming commands to the correct control module.
//! Control modules own sensor setup internally via async `init()` functions;
//! teardown is declared via [`types::IntentTeardown`] descriptors.
//!
//! ```text
//! caller ──► DriveCommand ──► dispatch ──► control module init()
//!                                │              │
//!                                │         returns ActiveIntent
//!                                │              │
//!                                ▼              │
//!                          intent loop ◄────────┘
//!                                │
//!                                ▼
//!                          execute IntentTeardown (stop sensors)
//!                                │
//!                                ▼
//!                          send completion
//! ```
//!
//! # Control Flow Overview
//!
//! The drive task selects over two sources:
//!
//! - **Regular command queue**: Sequenced drive intents that may take time.
//! - **Interrupt signal**: Emergency brake/stop/cancel that preempts any intent.
//!
//! When an interrupt arrives, the task cancels the active intent via the same
//! [`IntentTeardown`] path used by normal completion, resolves its completion
//! as `Cancelled`, increments an epoch, and discards queued commands stamped
//! before the interrupt.
//!
//! # Completion Flow (queue-level)
//!
//! Queue execution is owned by the drive queue executor. It awaits per-command
//! completion internally and emits a single queue-level completion to the
//! producer.
//!
//! - **Submit and await** a queue using [`DriveQueueBuilder`].
//! - Queue completion resolves on success, failure, or cancellation.
//! - The last step's `DriveCompletion` is returned in the queue completion.
//!
//! # Data Flow
//!
//! - Commands are sent by orchestrator or other tasks.
//! - Sensor feedback flows directly from sensor tasks into dedicated channels.
//! - Motor commands are issued to `motor_driver`.
//!
//! This task does NOT consume the system event channel. Forwarding avoids
//! multiple tasks competing for events.
//!
//! # Speed Convention
//!
//! This module uses the same speed convention as `motor_driver`:
//! - **-100 to +100**: Full range of motor control
//! - **Positive values**: Forward motion
//! - **Negative values**: Backward motion
//! - **Zero**: Coast (freewheel)
//!
//! # Module Index
//!
//! ## Public surface
//! - [`api`]: Command queueing, interrupts, and internal completion wiring.
//! - [`queue`]: Queue builder and queue executor task.
//! - [`types`]: Command, telemetry, and completion types.
//!
//! ## Loop orchestration
//! - [`state`]: Drive loop data structures (`DriveLoop`, `ActiveIntent`).
//! - [`dispatch`]: `impl DriveLoop` command handlers (envelope dispatch, action handling, interrupts).
//! - [`intent`]: Active intent polling, result application, and idle stepping.
//!
//! ## Control algorithms
//! - [`rotation`]: Rotation state machine and async control step.
//! - [`distance`]: Distance state machine and async control step.
//! - [`brake_coast`]: Brake/coast settle detection.
//! - [`differential`]: Differential drive speed passthrough.
//!
//! ## Sensor infrastructure
//! - [`sensors::data`]: Static sensor feedback channels and measurement forwarding.
//! - [`sensors::control`]: IMU and encoder start/stop helpers.
//!
//! ## Calibration
//! - [`calibration`]: Motor and IMU calibration procedures.

// ── Loop orchestration ────────────────────────────────────────────────────────
mod dispatch;
mod intent;
mod state;

// ── Control algorithms ────────────────────────────────────────────────────────
mod brake_coast;
mod differential;
mod distance;
mod rotation;

// ── Sensor infrastructure ─────────────────────────────────────────────────────
mod sensors;

// ── Calibration ───────────────────────────────────────────────────────────────
mod calibration;

// ── Public API surface ────────────────────────────────────────────────────────
mod api;
mod queue;
pub mod types;

// ── Re-exports ────────────────────────────────────────────────────────────────

pub use api::{send_drive_command, send_drive_interrupt};
use brake_coast::BrakeCoastStepResult;
use distance::DistanceStepResult;
use intent::{ActiveIntentOutcome, apply_completion, poll_active_intent, step_idle};
pub use queue::{DriveQueueBuilder, drive_queue_executor};
use rotation::RotationStepResult;
pub use sensors::data::{
    clear_encoder_measurement, clear_imu_measurements, get_latest_encoder_measurement, send_mag_measurement,
    try_send_encoder_measurement, try_send_imu_measurement,
};
use state::DriveLoop;
pub use types::{
    CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind,
    DriveQueueSubmitError, ImuCalibrationKind, InterruptKind, TurnDirection,
};

/// Drive control task - coordinates motion and sensor feedback.
///
/// # Architecture
///
/// This is a high-level control task that:
/// - Receives drive commands via queue.
/// - Receives encoder feedback via channel (from encoder task).
/// - Receives IMU feedback via channel (from IMU task); orientation is calibrated when the IMU task has loaded calibration data.
/// - Receives interrupts via signal.
/// - Sends motor commands to the `motor_driver` task.
/// - Coordinates calibration procedures.
///
/// # Sensor Data Flow
///
/// Point-to-point measurements:   Sensor tasks → drive channels → Drive task
/// Semantic events:               Sensor tasks → Events → Orchestrator → Drive commands
/// Both paths converge at:        Drive task → Motor driver
///
/// IMU and encoder measurements flow directly from sensor tasks into dedicated
/// channels (`IMU_FEEDBACK_CHANNEL`, `LATEST_ENCODER_MEASUREMENT`) without
/// passing through the orchestrator or the system event channel.
#[embassy_executor::task]
pub async fn drive() {
    // Initialise per-task state.
    let mut loop_state = DriveLoop::new();

    loop {
        // Step 1: If an intent is active, poll it with higher priority than new commands.
        if let Some(outcome) = poll_active_intent(&mut loop_state).await {
            match outcome {
                ActiveIntentOutcome::Interrupt(kind) => {
                    loop_state.handle_interrupt(kind).await;
                }
                ActiveIntentOutcome::RotationStep(result) => match result {
                    RotationStepResult::InProgress => {}
                    RotationStepResult::Completed { telemetry } => {
                        apply_completion(&mut loop_state, CompletionStatus::Success, telemetry).await;
                    }
                    RotationStepResult::Failed { reason, telemetry } => {
                        apply_completion(&mut loop_state, CompletionStatus::Failed(reason), telemetry).await;
                    }
                },
                ActiveIntentOutcome::DistanceStep(result) => match result {
                    DistanceStepResult::InProgress => {}
                    DistanceStepResult::Completed { telemetry } => {
                        apply_completion(&mut loop_state, CompletionStatus::Success, telemetry).await;
                    }
                    DistanceStepResult::Failed { reason, telemetry } => {
                        apply_completion(&mut loop_state, CompletionStatus::Failed(reason), telemetry).await;
                    }
                },
                ActiveIntentOutcome::IdleElapsed => {
                    apply_completion(
                        &mut loop_state,
                        CompletionStatus::Success,
                        types::CompletionTelemetry::None,
                    )
                    .await;
                }
                ActiveIntentOutcome::BrakeCoastStep(result) => match result {
                    BrakeCoastStepResult::InProgress => {}
                    BrakeCoastStepResult::Completed => {
                        apply_completion(
                            &mut loop_state,
                            CompletionStatus::Success,
                            types::CompletionTelemetry::None,
                        )
                        .await;
                    }
                    BrakeCoastStepResult::Failed(reason) => {
                        apply_completion(
                            &mut loop_state,
                            CompletionStatus::Failed(reason),
                            types::CompletionTelemetry::None,
                        )
                        .await;
                    }
                },
            }
            continue;
        }

        // Step 2: No active intent — wait for work.
        step_idle(&mut loop_state).await;
    }
}
