//! Drive loop data structures.
//!
//! This module defines the core state types owned by the drive task's main loop.
//! It is deliberately free of behaviour — no command handling, no motor commands,
//! no sensor calls. That logic lives in [`super::dispatch`].
//!
//! # Intent lifecycle
//!
//! An intent is a state machine for one motion behaviour. It is created by a
//! control module's async `init()`, which owns sensor setup, polled by the intent
//! loop via the module's `run_step()`, and torn down via its
//! [`teardown()`](ActiveIntent::teardown) descriptor. The dispatch owns teardown
//! (stopping sensors on completion or interrupt); the control modules own the
//! runtime logic.
//!
//! Each intent carries:
//! - A controller state (`RotationState`, `DistanceDriveState`, etc.)
//! - A completion flag — whether the caller expects a [`DriveCompletion`]
//! - A teardown descriptor — which sensors to stop on completion or interrupt
//! - Cancellation telemetry — what data to report if interrupted
//!
//! Commands that complete instantly (e.g. `Differential`) are **not** intents;
//! they are fire-and-forget, handled directly by the dispatch.
//!
//! # Types
//!
//! - [`ActiveIntent`]: the currently executing drive intent, together with its
//!   completion flag, teardown descriptor, and runtime state.
//! - [`DriveLoop`]: top-level state struct — holds `standby_enabled` and the
//!   active intent (if any).

use embassy_time::Instant;

use super::{brake_coast::BrakeCoastState, rotation::RotationState};
use crate::task::drive::distance::DistanceDriveState;

/// The drive intent currently being executed by the drive task.
pub(super) enum ActiveIntent {
    /// An in-place or while-moving rotation towards a target angle.
    RotateExact {
        /// Rotation controller state (angle tracking, speed calculation).
        state: RotationState,
        /// Whether this intent should emit a completion event when finished.
        completion_requested: bool,
        /// Wall-clock start time used for duration telemetry (milliseconds).
        started_at_ms: u64,
    },
    /// Active distance drive intent with state and optional completion sender.
    DriveDistance {
        /// Distance controller state (encoder progress, curve correction).
        state: DistanceDriveState,
        /// Whether this intent should emit a completion event when finished.
        completion_requested: bool,
    },
    /// Active idle intent with a fixed duration.
    Idle {
        /// Duration to remain idle (milliseconds).
        duration_ms: u64,
        /// Wall-clock start time used for duration tracking (milliseconds).
        started_at_ms: u64,
        /// Whether this intent should emit a completion event when finished.
        completion_requested: bool,
    },
    /// Active brake/coast settle intent with encoder settle tracking.
    BrakeCoast {
        /// Brake/coast settle state.
        state: BrakeCoastState,
        /// Whether this intent should emit a completion event when finished.
        completion_requested: bool,
    },
}

/// Top-level state for the drive task's main control loop.
pub(super) struct DriveLoop {
    /// Whether the motor drivers are currently in low-power standby mode.
    pub(super) standby_enabled: bool,
    /// The intent currently being executed, if any.
    pub(super) active_intent: Option<ActiveIntent>,
}

impl DriveLoop {
    /// Create a new `DriveLoop` with standby enabled and no active intent.
    pub(super) const fn new() -> Self {
        Self {
            standby_enabled: true,
            active_intent: None,
        }
    }
}

impl ActiveIntent {
    /// Return the `IntentTeardown` descriptor for this active intent.
    pub(super) const fn teardown(&self) -> super::types::IntentTeardown {
        match self {
            Self::RotateExact { .. } => super::types::IntentTeardown::RotationImu,
            Self::DriveDistance { .. } => super::types::IntentTeardown::DistanceImuAndMotors,
            Self::Idle { .. } => super::types::IntentTeardown::None,
            Self::BrakeCoast { .. } => super::types::IntentTeardown::EncoderSettle,
        }
    }

    /// Returns true if this intent requested a completion event.
    pub(super) const fn completion_requested(&self) -> bool {
        match self {
            Self::RotateExact {
                completion_requested, ..
            }
            | Self::DriveDistance {
                completion_requested, ..
            }
            | Self::Idle {
                completion_requested, ..
            }
            | Self::BrakeCoast {
                completion_requested, ..
            } => *completion_requested,
        }
    }

    /// Returns cancellation telemetry appropriate for this intent type.
    pub(super) fn cancellation_telemetry(&self) -> super::types::CompletionTelemetry {
        match self {
            Self::RotateExact {
                state, started_at_ms, ..
            } => {
                let last_yaw_deg = state.last_yaw.unwrap_or_else(|| {
                    defmt::warn!("rotation cancelled before first IMU sample — telemetry yaw is 0");
                    0.0
                });
                let duration_ms = Instant::now().as_millis() - started_at_ms;
                super::types::CompletionTelemetry::RotateExact {
                    final_yaw_deg: last_yaw_deg,
                    angle_error_deg: -state.remaining(),
                    duration_ms,
                }
            }
            Self::DriveDistance { state, .. } => {
                let duration_ms = Instant::now().as_millis() - state.started_at_ms;
                super::types::CompletionTelemetry::DriveDistance {
                    achieved_left_revs: state.accumulated_left_revs,
                    achieved_right_revs: state.accumulated_right_revs,
                    target_left_revs: state.target_left_revs,
                    target_right_revs: state.target_right_revs,
                    duration_ms,
                }
            }
            Self::Idle { .. } | Self::BrakeCoast { .. } => super::types::CompletionTelemetry::None,
        }
    }
}
