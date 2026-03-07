//! Drive system coordination and intent execution.
//!
//! This module owns the drive task and its supporting components. It coordinates
//! sensor feedback, drive intents, and motor commands without consuming the global
//! event stream directly.
//!
//! # Architecture
//!
//! ```text
//! sensors::encoders     drive task               motor_driver
//! ├── Pulse counts  →   ├── Queue/interrupt   →   ├── PWM actuation
//! └── Feedback          ├── Intent lifecycle      └── Direction control
//!
//! sensors::imu      →   drive task               motor_driver
//! ├── Orientation       ├── Rotation control
//! └── Angles            └── Stabilization
//! ```
//!
//! # Control Flow Overview
//!
//! The drive task selects over two sources:
//!
//! - **Regular command queue**: Sequenced drive intents that may take time.
//! - **Interrupt signal**: Emergency brake/stop/cancel that preempts any intent.
//!
//! When an interrupt arrives, the task cancels the active intent, resolves its
//! completion (if any), increments an epoch, and discards queued commands that
//! were stamped before the interrupt.
//!
//! # Completion Flow (per-command)
//!
//! Completion handles are a fixed-size pool of one-shot completion channels.
//! A `CompletionHandle` is just an index into that pool (not a channel itself);
//! the handle stays with the caller while the `CompletionSender` is attached to
//! a command so the drive loop can resolve it.
//!
//! - **Acquire** a `CompletionHandle` from the pool (fixed-size; returns `Exhausted` immediately).
//! - **Create** a `CompletionSender` from the handle and attach it to the command.
//! - **Await** completion on the handle; the drive loop resolves it on success,
//!   failure, or cancellation (interrupts and epoch invalidation yield `Cancelled`).
//! - **Release** the handle back to the pool after you receive the result to avoid leaks.
//!
//! **Typical usage:** `acquire_completion_handle` → `completion_sender` →
//! `send_drive_command_with_completion` → `wait_for_completion` → `release_completion_handle`.
//!
//! # Data Flow
//!
//! - Commands are sent by orchestrator or other tasks.
//! - Sensor feedback is forwarded by orchestrator into dedicated channels.
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
//! - [`api`]: Command queueing and completion pool.
//! - [`types`]: Command, telemetry, and completion types.
//!
//! ## Loop orchestration
//! - [`state`]: Drive loop data structures (`DriveLoop`, `ActiveIntent`, `DriftCompensationState`).
//! - [`handlers`]: `impl DriveLoop` command handlers (envelope dispatch, action handling, interrupts).
//! - [`intent`]: Active intent polling, result application, and idle stepping.
//!
//! ## Control algorithms
//! - [`rotation`]: Rotation state machine and async control step.
//! - [`distance`]: Distance state machine and async control step.
//!
//! ## Drift compensation
//! - [`drift`]: Async drift compensation loop step.
//! - [`drift::math`]: Pure math functions (track averages, speed difference, compensation algorithm).
//!
//! ## Sensor infrastructure
//! - [`sensors::data`]: Static sensor feedback channels and measurement forwarding.
//! - [`sensors::control`]: IMU and encoder start/stop helpers.
//!
//! ## Calibration
//! - [`calibration`]: Motor and IMU calibration procedures.

// ── Loop orchestration ────────────────────────────────────────────────────────
mod handlers;
mod intent;
mod state;

// ── Control algorithms ────────────────────────────────────────────────────────
mod distance;
mod rotation;

// ── Drift compensation ────────────────────────────────────────────────────────
mod drift;

// ── Sensor infrastructure ─────────────────────────────────────────────────────
mod sensors;

// ── Calibration ───────────────────────────────────────────────────────────────
mod calibration;

// ── Public API surface ────────────────────────────────────────────────────────
mod api;
pub mod types;

// ── Re-exports ────────────────────────────────────────────────────────────────

pub use api::{
    acquire_completion_handle, completion_sender, release_completion_handle, send_drive_command,
    send_drive_command_with_completion, send_drive_interrupt, wait_for_completion,
};
use intent::{
    ActiveIntentOutcome, apply_distance_result, apply_rotation_result, poll_active_intent, step_idle_with_drift,
    step_idle_without_drift,
};
pub use sensors::data::{
    clear_encoder_measurement, get_latest_accel_measurement, get_latest_encoder_measurement,
    get_latest_gyro_measurement, get_latest_mag_measurement, send_accel_measurement, send_gyro_measurement,
    send_mag_measurement, try_send_encoder_measurement, try_send_imu_measurement,
};
use state::DriveLoop;
pub use types::{
    CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind,
    ImuCalibrationKind, InterruptKind, TurnDirection,
};

/// Drive control task - coordinates motion and sensor feedback.
///
/// # Architecture
///
/// This is a high-level control task that:
/// - Receives drive commands via queue.
/// - Receives encoder feedback via channel (from orchestrator).
/// - Receives IMU feedback via channel (from orchestrator).
/// - Receives interrupts via signal.
/// - Sends motor commands to the `motor_driver` task.
/// - Coordinates calibration procedures.
///
/// # Sensor Data Flow
///
/// Sensor tasks → Events → Orchestrator → Drive task (this) → Motor driver
///
/// The orchestrator forwards relevant sensor events to this task via dedicated
/// channels rather than having this task consume system events directly.
#[embassy_executor::task]
pub async fn drive() {
    // Initialise per-task state and the completion channel pool once.
    let mut loop_state = DriveLoop::new();
    api::init_completion_pool_once();

    loop {
        // Step 1: If an intent is active, poll it with higher priority than new commands.
        if let Some(outcome) = poll_active_intent(&mut loop_state).await {
            match outcome {
                ActiveIntentOutcome::Interrupt(kind) => {
                    loop_state.handle_interrupt(kind).await;
                }
                ActiveIntentOutcome::RotationStep(result) => {
                    apply_rotation_result(&mut loop_state, result).await;
                }
                ActiveIntentOutcome::DistanceStep(result) => {
                    apply_distance_result(&mut loop_state, result).await;
                }
            }
            continue;
        }

        // Step 2: No active intent — wait for work (with optional drift ticks).
        if loop_state.drift.enabled {
            step_idle_with_drift(&mut loop_state).await;
        } else {
            step_idle_without_drift(&mut loop_state).await;
        }
    }
}
