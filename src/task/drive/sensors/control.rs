//! Sensor lifecycle helpers for drive intents.
//!
//! This module is the *control plane* of the sensor infrastructure: it issues
//! commands to sensor tasks (raise system events, send encoder commands) to
//! start and stop IMU and encoder sampling as required by each drive intent.
//!
//! # Lifecycle rules
//!
//! - Rotation intents always require IMU streaming.
//! - Curve distance intents require IMU streaming; straight distance does not.
//! - Encoder sampling is required for distance driving and drift compensation.
//! - When an intent completes or is interrupted, the corresponding sensor
//!   streams must be stopped to prevent stale feedback from leaking into the
//!   next intent.
//!
//! # Relationship to `data`
//!
//! This module *commands* sensors (start/stop). The [`super::data`] module
//! *receives* their output. Together they form the full sensor infrastructure
//! used by the drive task.

use crate::{
    system::event::{Events, raise_event},
    task::{
        drive::types,
        sensors::{
            encoders::{self as encoder_read},
            imu::{self, DEFAULT_FUSION_MODE},
        },
    },
};

/// Start IMU streaming for rotation intents.
pub async fn start_rotation_imu() {
    raise_event(Events::StartStopMotionDataCollection(true)).await;
}

/// Stop IMU streaming for rotation intents.
pub async fn stop_rotation_imu() {
    raise_event(Events::StartStopMotionDataCollection(false)).await;
}

/// Start IMU streaming if a distance intent is a curve arc.
///
/// For straight distance drives this is a no-op; IMU feedback is only needed
/// to correct heading drift during curved motion.
pub async fn start_curve_imu(kind: &types::DriveDistanceKind) {
    if matches!(kind, types::DriveDistanceKind::CurveArc { .. }) {
        imu::set_ahrs_fusion_mode(DEFAULT_FUSION_MODE);
        raise_event(Events::StartStopMotionDataCollection(true)).await;
    }
}

/// Stop IMU streaming if a distance intent is a curve arc.
///
/// For straight distance drives this is a no-op.
pub async fn stop_curve_imu(kind: &types::DriveDistanceKind) {
    if matches!(kind, types::DriveDistanceKind::CurveArc { .. }) {
        raise_event(Events::StartStopMotionDataCollection(false)).await;
    }
}

/// Start encoder sampling for distance intents.
///
/// Optionally clears the cached feedback measurement so the control loop
/// waits for a fresh sample after the encoder counters are reset.
pub async fn start_encoder_sampling(interval_ms: u64, clear_feedback: bool) {
    if clear_feedback {
        super::data::clear_encoder_measurement().await;
    }
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms }).await;
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
}

/// Stop encoder sampling.
pub async fn stop_encoder_sampling() {
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
}
