//! Drive loop data structures.
//!
//! This module defines the core state types owned by the drive task's main loop.
//! It is deliberately free of behaviour: no command handling, no motor commands,
//! no sensor calls. That logic lives in [`super::handlers`].
//!
//! # Types
//!
//! - [`DriftCompensationState`]: tracks the encoder-based drift compensation
//!   parameters for straight-line `Differential` commands.
//! - [`ActiveIntent`]: the currently executing drive intent (rotation or
//!   distance), together with its completion sender and any runtime state.
//! - [`DriveLoop`]: the top-level state struct owned by the drive task. Holds
//!   standby status, the active intent (if any), and drift compensation state.

use super::rotation::RotationState;
use crate::task::{
    drive::{distance::DistanceDriveState, types},
    sensors::encoders::EncoderMeasurement,
};

/// Encoder-based drift compensation state for straight-line driving.
///
/// Active only while a symmetric `Differential` command is executing.
/// Disabled during rotation, distance driving, braking, and standby.
#[derive(Clone, Copy)]
pub(super) struct DriftCompensationState {
    /// Whether drift compensation is currently active.
    pub(super) enabled: bool,
    /// Base left-track speed as commanded (before compensation).
    pub(super) base_left: i8,
    /// Base right-track speed as commanded (before compensation).
    pub(super) base_right: i8,
    /// Currently applied left-track speed (after compensation adjustments).
    pub(super) adjusted_left: i8,
    /// Currently applied right-track speed (after compensation adjustments).
    pub(super) adjusted_right: i8,
    /// Timestamp of the last applied compensation update (milliseconds).
    pub(super) last_applied_ms: u64,
    /// Timestamp of the last processed encoder measurement (milliseconds).
    pub(super) last_encoder_timestamp_ms: u64,
    /// Previous encoder measurement for computing per-sample deltas.
    ///
    /// Encoder hardware counters are cumulative since the last reset, so the
    /// previous reading must be subtracted (with wraparound) to get the pulse
    /// count for the current sampling window.
    pub(super) last_encoder_measurement: Option<EncoderMeasurement>,
}

impl DriftCompensationState {
    /// Create a new `DriftCompensationState` with drift disabled and all
    /// speeds and timestamps zeroed.
    pub(super) const fn new() -> Self {
        Self {
            enabled: false,
            base_left: 0,
            base_right: 0,
            adjusted_left: 0,
            adjusted_right: 0,
            last_applied_ms: 0,
            last_encoder_timestamp_ms: 0,
            last_encoder_measurement: None,
        }
    }
}

/// The drive intent currently being executed by the drive task.
pub(super) enum ActiveIntent {
    /// An in-place or while-moving rotation towards a target angle.
    RotateExact {
        /// Rotation controller state (angle tracking, speed calculation).
        state: RotationState,
        /// Optional completion sender resolved when the rotation finishes.
        completion: Option<types::CompletionSender>,
        /// Wall-clock start time used for duration telemetry (milliseconds).
        started_at_ms: u64,
    },
    /// Active distance drive intent with state and optional completion sender.
    DriveDistance {
        /// Distance controller state (encoder progress, curve correction).
        state: DistanceDriveState,
        /// Optional completion sender resolved when the drive finishes.
        completion: Option<types::CompletionSender>,
    },
}

/// Top-level state for the drive task's main control loop.
pub(super) struct DriveLoop {
    /// Whether the motor drivers are currently in low-power standby mode.
    pub(super) standby_enabled: bool,
    /// The intent currently being executed, if any.
    pub(super) active_intent: Option<ActiveIntent>,
    /// Encoder-based drift compensation state for straight-line driving.
    pub(super) drift: DriftCompensationState,
}

impl DriveLoop {
    /// Create a new `DriveLoop` with standby enabled, no active intent, and
    /// drift compensation disabled.
    pub(super) const fn new() -> Self {
        Self {
            standby_enabled: true,
            active_intent: None,
            drift: DriftCompensationState::new(),
        }
    }
}
