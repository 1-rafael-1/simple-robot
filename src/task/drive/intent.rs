//! Intent orchestration for the drive loop.
//!
//! This module wires the active-intent state machine to control-loop ticks and
//! interrupt handling. It owns the polling/apply flow and idle stepping.
//!
//! # Interrupt vs completion behavior
//!
//! - Interrupts preempt any active intent and cause its completion (if present)
//!   to resolve as `Cancelled`.
//! - The interrupt handler also increments an epoch counter. Any queued commands
//!   stamped before the interrupt are cancelled when dequeued.
//!
//! # Epoch invalidation rationale
//!
//! Epoch invalidation provides a simple, lock-free way to discard stale queued
//! commands after a preemption. Active intents are cancelled immediately, while
//! queued intents are cancelled lazily when dequeued by comparing epochs.

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};

use crate::task::drive::{
    api::{DRIVE_INTERRUPT, DRIVE_QUEUE, send_completion},
    distance::{DistanceStepResult, distance_stop_motors, run_distance_control_step},
    drift::run_drift_compensation_step,
    lifecycle::{stop_curve_imu, stop_rotation_imu},
    rotation::{RotationStepResult, run_rotation_control_step},
    state::{ActiveIntent, DriveLoop},
    types::{self, CompletionStatus, DriveCompletion},
};

/// Outcome of polling an active intent.
pub(super) enum ActiveIntentOutcome {
    /// An interrupt arrived while an intent was active.
    Interrupt(types::InterruptKind),
    /// A control step completed for the active rotation intent.
    RotationStep(RotationStepResult),
    /// A control step completed for the active distance intent.
    DistanceStep(DistanceStepResult),
}

/// Poll the active intent (if any) and return the next action to perform.
pub(super) async fn poll_active_intent(loop_state: &mut DriveLoop) -> Option<ActiveIntentOutcome> {
    match loop_state.active_intent.as_mut() {
        Some(ActiveIntent::RotateExact {
            state, started_at_ms, ..
        }) => {
            let interrupt_or_tick = select(DRIVE_INTERRUPT.wait(), rotation_tick_ms()).await;
            match interrupt_or_tick {
                Either::First(kind) => Some(ActiveIntentOutcome::Interrupt(kind)),
                Either::Second(()) => {
                    let result = run_rotation_control_step(state, *started_at_ms).await;
                    Some(ActiveIntentOutcome::RotationStep(result))
                }
            }
        }
        Some(ActiveIntent::DriveDistance { state, .. }) => {
            let interrupt_or_tick = select(DRIVE_INTERRUPT.wait(), distance_tick_ms()).await;
            match interrupt_or_tick {
                Either::First(kind) => Some(ActiveIntentOutcome::Interrupt(kind)),
                Either::Second(()) => {
                    let result = run_distance_control_step(state).await;
                    Some(ActiveIntentOutcome::DistanceStep(result))
                }
            }
        }
        None => None,
    }
}

/// Apply rotation step results to the active intent (completions + intent clearing).
pub(super) async fn apply_rotation_result(loop_state: &mut DriveLoop, result: RotationStepResult) {
    let (status, telemetry) = match result {
        RotationStepResult::InProgress => return,
        RotationStepResult::Completed { telemetry } => (CompletionStatus::Success, telemetry),
        RotationStepResult::Failed { reason, telemetry } => (CompletionStatus::Failed(reason), telemetry),
    };

    let completion = match loop_state.active_intent.as_mut() {
        Some(ActiveIntent::RotateExact { completion, .. }) => completion.take(),
        _ => None,
    };

    stop_rotation_imu().await;
    send_completion(completion, DriveCompletion { status, telemetry }).await;

    loop_state.active_intent = None;
}

/// Apply distance step results to the active intent (completions + intent clearing).
pub(super) async fn apply_distance_result(loop_state: &mut DriveLoop, result: DistanceStepResult) {
    let (status, telemetry) = match result {
        DistanceStepResult::InProgress => return,
        DistanceStepResult::Completed { telemetry } => (CompletionStatus::Success, telemetry),
        DistanceStepResult::Failed { reason, telemetry } => (CompletionStatus::Failed(reason), telemetry),
    };

    let completion = match loop_state.active_intent.as_mut() {
        Some(ActiveIntent::DriveDistance { completion, .. }) => completion.take(),
        _ => None,
    };

    distance_stop_motors().await;
    if let Some(ActiveIntent::DriveDistance { state, .. }) = loop_state.active_intent.as_ref()
        && matches!(state.kind, types::DriveDistanceKind::CurveArc { .. })
    {
        stop_curve_imu(&state.kind).await;
    }
    send_completion(completion, DriveCompletion { status, telemetry }).await;

    loop_state.active_intent = None;
}

/// Idle loop step when drift compensation is enabled.
pub(super) async fn step_idle_with_drift(loop_state: &mut DriveLoop) {
    let event_or_tick = select(
        select(DRIVE_QUEUE.receiver().receive(), DRIVE_INTERRUPT.wait()),
        drift_tick_ms(),
    )
    .await;

    match event_or_tick {
        Either::First(inner) => match inner {
            Either::First(envelope) => loop_state.handle_envelope(envelope).await,
            Either::Second(kind) => loop_state.handle_interrupt(kind).await,
        },
        Either::Second(()) => {
            run_drift_compensation_step(&mut loop_state.drift, None).await;
        }
    }
}

/// Idle loop step when drift compensation is disabled.
pub(super) async fn step_idle_without_drift(loop_state: &mut DriveLoop) {
    let command_or_interrupt = select(DRIVE_QUEUE.receiver().receive(), DRIVE_INTERRUPT.wait()).await;
    match command_or_interrupt {
        Either::First(envelope) => loop_state.handle_envelope(envelope).await,
        Either::Second(kind) => loop_state.handle_interrupt(kind).await,
    }
}

/// Drift compensation control tick interval.
async fn drift_tick_ms() {
    // Keep this modest; encoder sampling is 200ms, so 20ms is plenty to pick up new samples
    // without busy-waiting.
    Timer::after(Duration::from_millis(20)).await;
}

/// Rotation control tick interval.
async fn rotation_tick_ms() {
    // 50Hz rotation control loop tick to align with 50Hz IMU sampling.
    Timer::after(Duration::from_millis(20)).await;
}

/// Distance control tick interval.
async fn distance_tick_ms() {
    Timer::after(Duration::from_millis(types::DISTANCE_CONTROL_INTERVAL_MS)).await;
}
