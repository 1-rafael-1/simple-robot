//! Drive task public API and completion signaling.
//!
//! This module owns the command queue and a single completion channel used by
//! the drive queue executor to await per-command results.
//!
//! # Completion Semantics (single producer)
//!
//! The system assumes a single governing producer that submits one queue at a
//! time and awaits its completion before sending the next. Per-command
//! completion is internal to the drive queue executor and should not be awaited
//! directly by other tasks.
//!
//! **Typical usage:** submit a queue via `DriveQueueBuilder`.
//!
//! If a command is queued without completion, no completion event is emitted.
//!
//! # Cancellation
//!
//! Interrupts preempt the active intent. Active completion resolves as
//! `Cancelled`. Queued commands are drained and any completion requests are
//! resolved as `Cancelled`. This assumes the single governing producer does
//! not enqueue new commands during interrupt handling.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal};

use crate::task::drive::types::{DriveCommand, DriveCompletion, InterruptKind};

/// Envelope for queued drive commands.
#[derive(Debug, Clone)]
pub(super) struct DriveCommandEnvelope {
    /// Drive command to execute.
    pub(crate) command: DriveCommand,
    /// Whether this command expects a completion event.
    pub(crate) completion_requested: bool,
    /// Epoch stamped at enqueue time to cancel stale queued commands.
    /// Used to resolve completions as `Cancelled` when invalidated by interrupts.
    pub(crate) epoch: u32,
}

/// Size of the regular drive command queue.
const DRIVE_QUEUE_SIZE: usize = 16;

/// Regular command queue for drive intents.
pub(super) static DRIVE_QUEUE: Channel<CriticalSectionRawMutex, DriveCommandEnvelope, DRIVE_QUEUE_SIZE> =
    Channel::new();

/// Interrupt signal for emergency actions.
pub(super) static DRIVE_INTERRUPT: Signal<CriticalSectionRawMutex, InterruptKind> = Signal::new();

/// Epoch counter for invalidating stale queued commands after interrupts.
pub(super) static CURRENT_EPOCH: AtomicU32 = AtomicU32::new(0);

/// Single-slot channel used for per-command completion delivery.
///
/// This is intended for a single governing producer that awaits one command
/// at a time.
static COMPLETION_CHANNEL: Channel<CriticalSectionRawMutex, DriveCompletion, 1> = Channel::new();

/// Send a drive command for execution (no completion).
pub async fn send_drive_command(command: DriveCommand) {
    send_drive_command_internal(command, false).await;
}

/// Send a drive command and wait for its completion.
///
/// This is an internal helper for the drive queue executor.
pub(super) async fn complete_drive_command(command: DriveCommand) -> DriveCompletion {
    send_drive_command_internal(command, true).await;
    wait_for_completion_internal().await
}

/// Send an interrupt that preempts the active intent.
pub fn send_drive_interrupt(kind: InterruptKind) {
    DRIVE_INTERRUPT.signal(kind);
}

/// Enqueue a drive command with an optional completion request.
async fn send_drive_command_internal(command: DriveCommand, completion_requested: bool) {
    let epoch = CURRENT_EPOCH.load(Ordering::Relaxed);
    let envelope = DriveCommandEnvelope {
        command,
        completion_requested,
        epoch,
    };
    DRIVE_QUEUE.sender().send(envelope).await;
}

/// Await the completion of the most recently enqueued command that requested one.
async fn wait_for_completion_internal() -> DriveCompletion {
    COMPLETION_CHANNEL.receiver().receive().await
}

/// Send completion payload if the command requested it.
///
/// Guaranteed delivery (awaits), fails only if the receiver was dropped.
pub(super) async fn send_completion(completion_requested: bool, payload: DriveCompletion) {
    if completion_requested {
        COMPLETION_CHANNEL.sender().send(payload).await;
    }
}
