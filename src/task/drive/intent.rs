//! Intent orchestration for the drive loop.
//!
//! This module wires the active-intent state machine to control-loop ticks and
//! interrupt handling. It owns the polling/apply flow and idle stepping.
//!
//! # Interrupt vs completion behaviour
//!
//! - Interrupts preempt any active intent and cause its completion (if requested)
//!   to resolve as `Cancelled`.
//! - The interrupt handler drains the queue. Any queued command that requested
//!   completion is resolved as `Cancelled`. Additional queued completion requests
//!   are dropped with a warning under the single-producer contract.
//! - The interrupt handler also increments an epoch counter. Any queued commands
//!   stamped before the interrupt are cancelled when dequeued.
//!
//! # Epoch invalidation rationale
//!
//! Epoch invalidation provides a simple, lock-free way to discard stale queued
//! commands after a preemption. Active intents are cancelled immediately, while
//! queued intents are cancelled when dequeued by comparing epochs.

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};

use crate::task::drive::{
    api::{DRIVE_INTERRUPT, DRIVE_QUEUE, send_completion},
    brake_coast::{self, BrakeCoastStepResult, run_brake_coast_step},
    dispatch::execute_intent_teardown,
    distance::{self, DistanceStepResult, run_distance_control_step},
    rotation::{self, RotationStepResult, run_rotation_control_step},
    state::DriveLoop,
    types::{self, CompletionStatus, DriveCompletion, IntentTeardown},
};

/// Outcome of polling an active intent.
pub(super) enum ActiveIntentOutcome {
    /// An interrupt arrived while an intent was active.
    Interrupt(types::InterruptKind),
    /// A control step completed for the active rotation intent.
    RotationStep(RotationStepResult),
    /// A control step completed for the active distance intent.
    DistanceStep(DistanceStepResult),
    /// The active idle intent reached its duration.
    IdleElapsed,
    /// A control step completed for the active brake/coast intent.
    BrakeCoastStep(BrakeCoastStepResult),
}

/// Poll the active intent (if any) and return the next action to perform.
pub(super) async fn poll_active_intent(loop_state: &mut DriveLoop) -> Option<ActiveIntentOutcome> {
    match loop_state.active_intent.as_mut() {
        Some(super::state::ActiveIntent::RotateExact {
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
        Some(super::state::ActiveIntent::DriveDistance { state, .. }) => {
            let interrupt_or_tick = select(DRIVE_INTERRUPT.wait(), distance_tick_ms()).await;
            match interrupt_or_tick {
                Either::First(kind) => Some(ActiveIntentOutcome::Interrupt(kind)),
                Either::Second(()) => {
                    let result = run_distance_control_step(state).await;
                    Some(ActiveIntentOutcome::DistanceStep(result))
                }
            }
        }
        Some(super::state::ActiveIntent::Idle {
            duration_ms,
            started_at_ms,
            ..
        }) => {
            let now_ms = Instant::now().as_millis();
            let elapsed_ms = now_ms.saturating_sub(*started_at_ms);
            let remaining_ms = duration_ms.saturating_sub(elapsed_ms);
            let interrupt_or_idle = select(
                DRIVE_INTERRUPT.wait(),
                Timer::after(Duration::from_millis(remaining_ms)),
            )
            .await;
            match interrupt_or_idle {
                Either::First(kind) => Some(ActiveIntentOutcome::Interrupt(kind)),
                Either::Second(()) => Some(ActiveIntentOutcome::IdleElapsed),
            }
        }
        Some(super::state::ActiveIntent::BrakeCoast { state, .. }) => {
            let interrupt_or_tick = select(DRIVE_INTERRUPT.wait(), brake_coast_tick_ms()).await;
            match interrupt_or_tick {
                Either::First(kind) => Some(ActiveIntentOutcome::Interrupt(kind)),
                Either::Second(()) => {
                    let result = run_brake_coast_step(state).await;
                    Some(ActiveIntentOutcome::BrakeCoastStep(result))
                }
            }
        }
        None => None,
    }
}

/// Apply completion: teardown sensors via descriptor, send completion, clear intent.
pub(super) async fn apply_completion(
    loop_state: &mut DriveLoop,
    status: CompletionStatus,
    telemetry: types::CompletionTelemetry,
) {
    // Extract completion_requested and teardown before taking the intent.
    let (completion_requested, teardown) = loop_state
        .active_intent
        .as_ref()
        .map_or((false, IntentTeardown::None), |i| {
            (i.completion_requested(), i.teardown())
        });

    execute_intent_teardown(teardown).await;
    send_completion(completion_requested, DriveCompletion { status, telemetry }).await;
    loop_state.active_intent = None;
}

/// Idle loop step — wait for a queued command or an interrupt.
pub(super) async fn step_idle(loop_state: &mut DriveLoop) {
    let command_or_interrupt = select(DRIVE_QUEUE.receiver().receive(), DRIVE_INTERRUPT.wait()).await;
    match command_or_interrupt {
        Either::First(envelope) => loop_state.handle_envelope(envelope).await,
        Either::Second(kind) => loop_state.handle_interrupt(kind).await,
    }
}

/// Rotation control tick interval.
async fn rotation_tick_ms() {
    Timer::after(Duration::from_millis(rotation::ROTATION_CONTROL_INTERVAL_MS)).await;
}

/// Brake/coast settle tick interval.
async fn brake_coast_tick_ms() {
    Timer::after(Duration::from_millis(brake_coast::SETTLE_INTERVAL_MS)).await;
}

/// Distance control tick interval.
async fn distance_tick_ms() {
    Timer::after(Duration::from_millis(distance::CONTROL_INTERVAL_MS)).await;
}
