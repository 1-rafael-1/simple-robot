//! Sensor lifecycle helpers for drive intents.
//!
//! This module centralizes IMU and encoder start/stop policy so that each intent
//! follows the same lifecycle rules.
//!
//! # IMU/encoder lifecycle expectations and invariants
//!
//! - Rotation intents always require IMU streaming.
//! - Curve distance intents require IMU streaming; straight distance does not.
//! - Encoder sampling is required for distance driving and drift compensation.
//! - Interrupts and intent completion must stop any active sensor streams to
//!   avoid leaking work and stale feedback into subsequent intents.

use crate::{
    system::event::{Events, raise_event},
    task::{
        drive::{feedback, types},
        sensors::{
            encoders::{self as encoder_read},
            imu::{self, AhrsFusionMode},
        },
    },
};

/// Start IMU streaming for rotation intents.
pub(super) async fn start_rotation_imu() {
    raise_event(Events::StartStopMotionDataCollection(true)).await;
}

/// Stop IMU streaming for rotation intents.
pub(super) async fn stop_rotation_imu() {
    raise_event(Events::StartStopMotionDataCollection(false)).await;
}

/// Start IMU streaming if a distance intent is a curve arc.
pub(super) async fn start_curve_imu(kind: &types::DriveDistanceKind) {
    if matches!(kind, types::DriveDistanceKind::CurveArc { .. }) {
        imu::set_ahrs_fusion_mode(AhrsFusionMode::Axis9);
        raise_event(Events::StartStopMotionDataCollection(true)).await;
    }
}

/// Stop IMU streaming if a distance intent is a curve arc.
pub(super) async fn stop_curve_imu(kind: &types::DriveDistanceKind) {
    if matches!(kind, types::DriveDistanceKind::CurveArc { .. }) {
        raise_event(Events::StartStopMotionDataCollection(false)).await;
    }
}

/// Start encoder sampling for distance intents (optionally clears cached feedback and resets counters).
pub(super) async fn start_encoder_sampling(interval_ms: u64, clear_feedback: bool) {
    if clear_feedback {
        feedback::clear_encoder_measurement().await;
    }
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms }).await;
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
}

/// Stop encoder sampling.
pub(super) async fn stop_encoder_sampling() {
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
}
