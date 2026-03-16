//! Drive queue builder and executor task.
//!
//! This module provides a single-queue execution model:
//! - A producer builds a queue of `DriveCommand` steps.
//! - The queue executor runs each step using `complete_drive_command`.
//! - A single queue completion is emitted with status, failure index, and the
//!   last step's `DriveCompletion` (if any).
//!
//! Only one queue may run at a time. Submitting while active yields `QueueBusy`.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use heapless::Vec;

use super::{
    api::complete_drive_command,
    types::{
        CompletionStatus, DriveCommand, DriveCompletion, DriveQueueBuildError, DriveQueueCompletion,
        DriveQueueSubmitError,
    },
};

/// Maximum number of steps allowed in a single queue.
pub const DRIVE_QUEUE_CAPACITY: usize = 10;

/// Queue submission payload.
#[derive(Debug)]
pub(super) struct DriveQueueRequest {
    /// Commands to execute in order.
    steps: Vec<DriveCommand, DRIVE_QUEUE_CAPACITY>,
}

/// Queue submission channel (single active queue rule).
static DRIVE_QUEUE_REQUESTS: Channel<CriticalSectionRawMutex, DriveQueueRequest, 1> = Channel::new();

/// Queue completion channel (single producer).
static DRIVE_QUEUE_COMPLETIONS: Channel<CriticalSectionRawMutex, DriveQueueCompletion, 1> = Channel::new();

/// Busy flag guarding single-queue execution.
static QUEUE_BUSY: AtomicBool = AtomicBool::new(false);

/// Builder for a drive queue.
#[derive(Debug, Default)]
pub struct DriveQueueBuilder {
    /// Accumulated command steps.
    steps: Vec<DriveCommand, DRIVE_QUEUE_CAPACITY>,
}

impl DriveQueueBuilder {
    /// Create a new, empty builder.
    pub const fn new() -> Self {
        Self { steps: Vec::new() }
    }

    /// Push a command onto the queue.
    pub fn push(&mut self, command: DriveCommand) -> Result<(), DriveQueueBuildError> {
        self.steps.push(command).map_err(|_| DriveQueueBuildError::Full)
    }

    /// Extend the queue with an iterator of commands.
    #[allow(dead_code)]
    pub fn extend<I>(&mut self, commands: I) -> Result<(), DriveQueueBuildError>
    where
        I: IntoIterator<Item = DriveCommand>,
    {
        for command in commands {
            self.push(command)?;
        }
        Ok(())
    }

    /// Number of queued steps.
    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        self.steps.len()
    }

    /// Returns `true` if no steps are queued.
    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.steps.is_empty()
    }

    /// Submit the queue for execution and await queue completion.
    ///
    /// Returns `QueueBusy` if another queue is already active.
    pub async fn submit(self) -> Result<DriveQueueCompletion, DriveQueueSubmitError> {
        if QUEUE_BUSY
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return Err(DriveQueueSubmitError::QueueBusy);
        }

        DRIVE_QUEUE_REQUESTS
            .sender()
            .send(DriveQueueRequest { steps: self.steps })
            .await;

        let completion = DRIVE_QUEUE_COMPLETIONS.receiver().receive().await;
        Ok(completion)
    }
}

/// Drive queue executor task.
///
/// This task owns per-step completion (`complete_drive_command`) and exposes
/// a single queue-level completion event per submission.
#[embassy_executor::task]
pub async fn drive_queue_executor() {
    loop {
        let request = DRIVE_QUEUE_REQUESTS.receiver().receive().await;

        let mut completed_steps = 0usize;
        let mut failed_step_index = None;
        let mut last_step_completion: Option<DriveCompletion> = None;
        let mut status = CompletionStatus::Success;

        for (index, command) in request.steps.into_iter().enumerate() {
            let completion = complete_drive_command(command).await;
            last_step_completion = Some(completion.clone());

            match completion.status {
                CompletionStatus::Success => {
                    completed_steps += 1;
                }
                CompletionStatus::Cancelled | CompletionStatus::Failed(_) => {
                    status = completion.status.clone();
                    failed_step_index = Some(index);
                    break;
                }
            }
        }

        DRIVE_QUEUE_COMPLETIONS
            .sender()
            .send(DriveQueueCompletion {
                status,
                failed_step_index,
                completed_steps,
                last_step_completion,
            })
            .await;

        QUEUE_BUSY.store(false, Ordering::Release);
    }
}
