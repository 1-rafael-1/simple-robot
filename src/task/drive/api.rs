//! Drive task public API and completion pool implementation.
//!
//! This module owns the command queue and completion handle pool used by callers
//! to enqueue drive intents and await per-command results.
//!
//! # Completion Handle Semantics
//!
//! A `CompletionHandle` is an index into a fixed pool of single-slot channels.
//! The caller keeps the handle locally, attaches a `CompletionSender` to the
//! command envelope, waits for completion, then releases the handle back to the
//! pool to avoid exhaustion.
//!
//! **Typical usage:** `acquire_completion_handle` → `completion_sender` →
//! `send_drive_command_with_completion` → `wait_for_completion` →
//! `release_completion_handle`.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal};

use crate::task::drive::types::{self, DriveCommand, DriveCompletion, InterruptKind};

/// Envelope for queued drive commands.
#[derive(Debug, Clone)]
pub(super) struct DriveCommandEnvelope {
    /// Drive command to execute.
    pub(crate) command: DriveCommand,
    /// Optional completion sender for per-command completion reporting.
    pub(crate) completion: Option<types::CompletionSender>,
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

/// Number of per-command completion channels available.
const COMPLETION_POOL_SIZE: usize = 16;
/// Single-slot channel used for per-command completion delivery.
type CompletionChannel = embassy_sync::channel::Channel<CriticalSectionRawMutex, DriveCompletion, 1>;

/// Pool of completion channels used by callers to await per-command results.
static COMPLETION_CHANNELS: [CompletionChannel; COMPLETION_POOL_SIZE] = [
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
];

/// Queue of available completion channel indices.
static COMPLETION_POOL: Channel<CriticalSectionRawMutex, usize, COMPLETION_POOL_SIZE> = Channel::new();
/// One-time init guard for seeding the completion pool.
static COMPLETION_POOL_INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Handle used by callers to await per-command completion.
///
/// This is an index into a fixed pool; it is not a channel itself.
/// Keep the handle local, pass the associated `CompletionSender` with the command,
/// await completion, then release the handle back to the pool to avoid exhaustion.
#[derive(Debug, Clone, Copy)]
pub struct CompletionHandle {
    /// Index into the fixed completion channel pool.
    index: usize,
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
/// Errors returned when acquiring a completion handle.
pub enum CompletionPoolError {
    /// No completion handles are available in the pool.
    Exhausted,
}

/// Send a drive command for execution.
pub async fn send_drive_command(command: DriveCommand) {
    send_drive_command_internal(command, None).await;
}

/// Send a drive command with a per-command completion sender.
///
/// The sender is attached to the command envelope so the drive loop can resolve it.
/// Completion delivery is guaranteed because the drive loop awaits the send.
pub async fn send_drive_command_with_completion(command: DriveCommand, completion: types::CompletionSender) {
    send_drive_command_internal(command, Some(completion)).await;
}

/// Send an interrupt that preempts the active intent.
pub fn send_drive_interrupt(kind: InterruptKind) {
    DRIVE_INTERRUPT.signal(kind);
}

/// Enqueue a drive command with an optional completion sender.
async fn send_drive_command_internal(command: DriveCommand, completion: Option<types::CompletionSender>) {
    let epoch = CURRENT_EPOCH.load(Ordering::Relaxed);
    let envelope = DriveCommandEnvelope {
        command,
        completion,
        epoch,
    };
    DRIVE_QUEUE.sender().send(envelope).await;
}

/// Acquire a completion handle from the pool.
///
/// This does not wait: if the pool is empty, it returns `CompletionPoolError::Exhausted`.
/// The pool is fixed-size and initialized lazily on first use.
pub async fn acquire_completion_handle() -> Result<CompletionHandle, CompletionPoolError> {
    init_completion_pool_once();
    let index = COMPLETION_POOL
        .receiver()
        .try_receive()
        .map_err(|_| CompletionPoolError::Exhausted)?;
    Ok(CompletionHandle { index })
}

/// Release a completion handle back into the pool.
///
/// This drains any stale completion and returns the index to the pool. Call this
/// after awaiting completion to avoid leaking handles and exhausting the pool.
pub async fn release_completion_handle(handle: CompletionHandle) {
    let _ = COMPLETION_CHANNELS[handle.index].receiver().try_receive();
    COMPLETION_POOL.sender().send(handle.index).await;
}

/// Await completion for the provided handle.
///
/// This blocks until the drive loop sends a result (success, failure, or cancel).
/// Cancellation can come from an interrupt or epoch invalidation.
pub async fn wait_for_completion(handle: &CompletionHandle) -> DriveCompletion {
    COMPLETION_CHANNELS[handle.index].receiver().receive().await
}

/// Get the sender associated with a completion handle.
///
/// This is the only value that should be passed across task boundaries; the
/// `CompletionHandle` stays with the caller and is used to await completion.
pub fn completion_sender(handle: CompletionHandle) -> types::CompletionSender {
    COMPLETION_CHANNELS[handle.index].sender()
}

/// Initialize the completion pool once by seeding available indices.
pub(super) fn init_completion_pool_once() {
    if COMPLETION_POOL_INITIALIZED
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_ok()
    {
        init_completion_pool();
    }
}

/// Seed the completion pool with all channel indices.
fn init_completion_pool() {
    for (index, _) in COMPLETION_CHANNELS.iter().enumerate() {
        let _ = COMPLETION_POOL.sender().try_send(index);
    }
}

/// Send completion payload if a sender is present.
///
/// Guaranteed delivery (awaits), fails only if the receiver was dropped.
pub(super) async fn send_completion(completion: Option<types::CompletionSender>, payload: DriveCompletion) {
    if let Some(sender) = completion {
        sender.send(payload).await;
    }
}
