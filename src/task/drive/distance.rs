//! Distance drive control loop and curve correction logic.
//!
//! This module owns the distance-control state machine used for both straight
//! drives and curve arcs. It computes encoder-based progress, ramps speed
//! near completion, and (for curves) applies an IMU-based yaw correction on
//! top of the nominal encoder-derived ratios.
//!
//! # Curve Control Model
//!
//! For a curve arc, each track follows a different radius. We compute expected
//! left/right travel using:
//!
//! - `left_arc = arc_length * (left_radius / radius)`
//! - `right_arc = arc_length * (right_radius / radius)`
//!
//! The corresponding target revolutions are `arc / sprocket_circumference`.
//! This yields base speed ratios for the inner/outer tracks.
//!
//! ## Expected yaw from encoder distance
//!
//! While driving a curve, we estimate the expected yaw from encoder distances
//! using a differential-drive approximation:
//!
//! - `expected_yaw_rad = (right_cm - left_cm) / track_width_cm`
//!
//! The IMU yaw is used **only as a correction** to keep the curve tight; the
//! encoder-derived ratios remain the primary motion plan.
//!
//! ## IMU correction and clamps
//!
//! We accumulate IMU yaw deltas (degrees) across samples and compare against the
//! expected yaw. The correction is proportional (`DISTANCE_CURVE_YAW_KP`) and
//! clamped to `DISTANCE_CURVE_YAW_MAX_CORRECTION`, then applied symmetrically:
//!
//! - `left_ratio = clamp(left_ratio - correction)`
//! - `right_ratio = clamp(right_ratio + correction)`
//!
//! This keeps the correction bounded and prevents extreme ratio distortions.
//!
//! ## Backward curve sign
//!
//! When driving backward, expected yaw is negated so that the IMU correction
//! remains consistent with the physical direction of travel.
//!
//! # IMU lifecycle
//!
//! IMU streaming is started for all distance drive intents (straight and curve)
//! and stopped when the intent completes or is interrupted. Straight drives use
//! IMU for heading correction; curve drives use IMU for yaw-based curve correction.
//!
//! # Telemetry logging
//!
//! Curve debug logs are rate-limited and compiled only when the `telemetry_logs`
//! feature is enabled; otherwise no formatting/queueing cost is incurred.

use embassy_time::Instant;
use micromath::F32Ext;

use crate::task::{
    drive::{
        drift::math as compensation,
        sensors::data::{self as feedback, IMU_FEEDBACK_CHANNEL},
        types,
    },
    motor_driver::{self, MotorCommand},
    sensors::encoders::{self as encoder_read, EncoderMeasurement},
};

/// Stall timeout during distance driving (milliseconds).
const DISTANCE_STALL_TIMEOUT_MS: u64 = 1500;

/// Result of a distance control step.
pub(super) enum DistanceStepResult {
    /// Distance drive is still in progress.
    InProgress,
    /// Distance drive completed successfully with final telemetry.
    Completed {
        /// Completion telemetry captured at success.
        telemetry: types::CompletionTelemetry,
    },
    /// Distance drive failed with a static reason and telemetry snapshot.
    Failed {
        /// Failure reason identifier.
        reason: &'static str,
        /// Completion telemetry captured at failure.
        telemetry: types::CompletionTelemetry,
    },
}

/// Distance drive control state.
pub(super) struct DistanceDriveState {
    /// Straight or curved distance specification.
    pub(super) kind: types::DriveDistanceKind,
    /// Drive direction (forward or backward).
    pub(super) direction: types::DriveDirection,
    /// Base speed magnitude (0-100).
    pub(super) base_speed: u8,
    /// Target revolutions for the left track sprocket.
    pub(super) target_left_revs: f32,
    /// Target revolutions for the right track sprocket.
    pub(super) target_right_revs: f32,
    /// Whether the left track is the inner (shorter) side for curves.
    pub(super) inner_left: Option<bool>,
    /// Target revolutions for the inner track sprocket.
    pub(super) target_inner_revs: f32,
    /// Left speed scale relative to the max target (1.0 for the dominant track).
    pub(super) left_ratio: f32,
    /// Right speed scale relative to the max target (1.0 for the dominant track).
    pub(super) right_ratio: f32,
    /// Last commanded left speed (after scaling/ramp).
    pub(super) last_left_speed: i8,
    /// Last commanded right speed (after scaling/ramp).
    pub(super) last_right_speed: i8,
    /// Accumulated left revolutions.
    pub(super) accumulated_left_revs: f32,
    /// Accumulated right revolutions.
    pub(super) accumulated_right_revs: f32,
    /// Last IMU yaw sample for curve control (degrees).
    pub(super) curve_last_yaw_deg: Option<f32>,
    /// Accumulated curve yaw delta (degrees).
    pub(super) curve_accumulated_yaw_deg: f32,
    /// IMU reference yaw captured at first valid sample for straight-line heading control.
    pub(super) reference_yaw: Option<f32>,
    /// Last time we observed forward progress (ms).
    pub(super) last_progress_ms: u64,
    /// Consecutive samples with zero progress.
    pub(super) zero_progress_samples: u32,
    /// Timestamp of the last processed encoder measurement (ms).
    pub(super) last_encoder_timestamp_ms: u64,
    /// Last time we saw a new encoder measurement (ms).
    pub(super) last_encoder_seen_ms: u64,
    /// Previous encoder measurement for computing deltas.
    pub(super) last_encoder_measurement: Option<EncoderMeasurement>,
    /// Start time (ms) used for duration telemetry.
    pub(super) started_at_ms: u64,
}

impl DistanceDriveState {
    /// Create a new distance drive state and precompute targets/ratios.
    pub(super) fn new(
        kind: types::DriveDistanceKind,
        direction: types::DriveDirection,
        base_speed: u8,
        calibration_factor: f32,
    ) -> Self {
        let (target_left_revs, target_right_revs, inner_left, target_inner_revs) = match kind {
            types::DriveDistanceKind::Straight { distance_cm } => {
                let revolutions = (distance_cm / types::SPROCKET_CIRCUMFERENCE_CM) * calibration_factor;
                (revolutions, revolutions, None, revolutions)
            }
            types::DriveDistanceKind::CurveArc {
                radius_cm,
                arc_length_cm,
                direction,
            } => {
                let half_width = types::TRACK_WIDTH_CM * 0.5;
                let inner_radius = (radius_cm - half_width).max(0.0);
                let outer_radius = radius_cm + half_width;
                let (left_radius, right_radius, inner_left) = match direction {
                    types::TurnDirection::Left => (inner_radius, outer_radius, true),
                    types::TurnDirection::Right => (outer_radius, inner_radius, false),
                };
                let safe_radius = radius_cm.max(0.001);
                let left_arc_cm = arc_length_cm * (left_radius / safe_radius);
                let right_arc_cm = arc_length_cm * (right_radius / safe_radius);
                let left_revs = (left_arc_cm / types::SPROCKET_CIRCUMFERENCE_CM) * calibration_factor;
                let right_revs = (right_arc_cm / types::SPROCKET_CIRCUMFERENCE_CM) * calibration_factor;
                let inner_revs = if inner_left { left_revs } else { right_revs };
                (left_revs, right_revs, Some(inner_left), inner_revs)
            }
        };

        let max_target = target_left_revs.max(target_right_revs);
        let left_ratio = if max_target > 0.0 {
            target_left_revs / max_target
        } else {
            1.0
        };
        let right_ratio = if max_target > 0.0 {
            target_right_revs / max_target
        } else {
            1.0
        };

        let now_ms = Instant::now().as_millis();

        Self {
            kind,
            direction,
            base_speed,
            target_left_revs,
            target_right_revs,
            inner_left,
            target_inner_revs,
            left_ratio,
            right_ratio,
            last_left_speed: 0,
            last_right_speed: 0,
            accumulated_left_revs: 0.0,
            accumulated_right_revs: 0.0,
            curve_last_yaw_deg: None,
            curve_accumulated_yaw_deg: 0.0,
            reference_yaw: None,
            last_progress_ms: now_ms,
            zero_progress_samples: 0,
            last_encoder_timestamp_ms: 0,
            last_encoder_seen_ms: now_ms,
            last_encoder_measurement: None,
            started_at_ms: now_ms,
        }
    }
}

/// Run a single step of the distance control loop.
#[allow(
    clippy::too_many_lines,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap
)]
pub(super) async fn run_distance_control_step(state: &mut DistanceDriveState) -> DistanceStepResult {
    let now_ms = Instant::now().as_millis();
    if now_ms.saturating_sub(state.last_encoder_seen_ms) >= types::DISTANCE_ENCODER_TIMEOUT_MS {
        let telemetry = types::CompletionTelemetry::DriveDistance {
            achieved_left_revs: state.accumulated_left_revs,
            achieved_right_revs: state.accumulated_right_revs,
            target_left_revs: state.target_left_revs,
            target_right_revs: state.target_right_revs,
            duration_ms: now_ms.saturating_sub(state.started_at_ms),
        };
        return DistanceStepResult::Failed {
            reason: "EncoderTimeout",
            telemetry,
        };
    }

    let Some(measurement) = feedback::get_latest_encoder_measurement().await else {
        return DistanceStepResult::InProgress;
    };

    if measurement.timestamp_ms == 0 || measurement.timestamp_ms == state.last_encoder_timestamp_ms {
        return DistanceStepResult::InProgress;
    }
    state.last_encoder_timestamp_ms = measurement.timestamp_ms;
    state.last_encoder_seen_ms = now_ms;

    if state.last_encoder_measurement.is_none() {
        state.last_encoder_measurement = Some(measurement);
        return DistanceStepResult::InProgress;
    }

    let delta_measurement = state
        .last_encoder_measurement
        .map_or(measurement, |prev| EncoderMeasurement {
            left_front: compensation::calculate_delta_u16(measurement.left_front, prev.left_front),
            left_rear: compensation::calculate_delta_u16(measurement.left_rear, prev.left_rear),
            right_front: compensation::calculate_delta_u16(measurement.right_front, prev.right_front),
            right_rear: compensation::calculate_delta_u16(measurement.right_rear, prev.right_rear),
            timestamp_ms: measurement.timestamp_ms,
        });
    state.last_encoder_measurement = Some(measurement);

    let data = compensation::calculate_track_averages(delta_measurement);
    if data.all_zero() {
        state.zero_progress_samples = state.zero_progress_samples.saturating_add(1);
        if now_ms.saturating_sub(state.last_progress_ms) >= DISTANCE_STALL_TIMEOUT_MS {
            let telemetry = types::CompletionTelemetry::DriveDistance {
                achieved_left_revs: state.accumulated_left_revs,
                achieved_right_revs: state.accumulated_right_revs,
                target_left_revs: state.target_left_revs,
                target_right_revs: state.target_right_revs,
                duration_ms: now_ms.saturating_sub(state.started_at_ms),
            };
            return DistanceStepResult::Failed {
                reason: "StallTimeout",
                telemetry,
            };
        }
        return DistanceStepResult::InProgress;
    }
    if data.has_single_motor_zero_anomaly() {
        let telemetry = types::CompletionTelemetry::DriveDistance {
            achieved_left_revs: state.accumulated_left_revs,
            achieved_right_revs: state.accumulated_right_revs,
            target_left_revs: state.target_left_revs,
            target_right_revs: state.target_right_revs,
            duration_ms: now_ms.saturating_sub(state.started_at_ms),
        };
        return DistanceStepResult::Failed {
            reason: "EncoderAnomaly",
            telemetry,
        };
    }

    state.last_progress_ms = now_ms;
    state.zero_progress_samples = 0;

    let left_revs = data.left_track_avg / types::PULSES_PER_SPROCKET_REV_F32;
    let right_revs = data.right_track_avg / types::PULSES_PER_SPROCKET_REV_F32;

    state.accumulated_left_revs += left_revs;
    state.accumulated_right_revs += right_revs;

    let mut straight_imu_sample: Option<crate::task::sensors::imu::ImuMeasurement> = None;
    if matches!(state.kind, types::DriveDistanceKind::Straight { .. }) {
        while let Ok(m) = IMU_FEEDBACK_CHANNEL.receiver().try_receive() {
            straight_imu_sample = Some(m);
        }
        if state.reference_yaw.is_none()
            && let Some(sample) = straight_imu_sample
        {
            state.reference_yaw = Some(sample.orientation.yaw);
        }
    }

    if matches!(state.kind, types::DriveDistanceKind::CurveArc { .. }) {
        let mut latest_imu: Option<crate::task::sensors::imu::ImuMeasurement> = None;
        while let Ok(m) = IMU_FEEDBACK_CHANNEL.receiver().try_receive() {
            latest_imu = Some(m);
        }

        if let Some(measurement) = latest_imu {
            if let Some(last_yaw) = state.curve_last_yaw_deg {
                let mut yaw_change = measurement.orientation.yaw - last_yaw;

                // Handle wraparound at ±180 degrees
                if yaw_change > 180.0 {
                    yaw_change -= 360.0;
                } else if yaw_change < -180.0 {
                    yaw_change += 360.0;
                }

                state.curve_accumulated_yaw_deg += yaw_change;
            }

            state.curve_last_yaw_deg = Some(measurement.orientation.yaw);
        }
    }

    let inner_progress = match state.kind {
        types::DriveDistanceKind::Straight { .. } => state.accumulated_left_revs.min(state.accumulated_right_revs),
        types::DriveDistanceKind::CurveArc { .. } => match state.inner_left {
            Some(true) => state.accumulated_left_revs,
            Some(false) => state.accumulated_right_revs,
            None => (state.accumulated_left_revs + state.accumulated_right_revs) * 0.5,
        },
    };

    let remaining = (state.target_inner_revs - inner_progress).max(0.0);

    let duration_ms = now_ms.saturating_sub(state.started_at_ms);
    let telemetry = types::CompletionTelemetry::DriveDistance {
        achieved_left_revs: state.accumulated_left_revs,
        achieved_right_revs: state.accumulated_right_revs,
        target_left_revs: state.target_left_revs,
        target_right_revs: state.target_right_revs,
        duration_ms,
    };

    if remaining <= types::DISTANCE_TOLERANCE_REVS {
        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed: 0,
            right_speed: 0,
        })
        .await;

        crate::system::state::motion::set_track_speeds(0, 0).await;

        return DistanceStepResult::Completed { telemetry };
    }

    let ramp_speed = if remaining <= types::DISTANCE_RAMP_DOWN_START_REVS {
        let factor = (remaining / types::DISTANCE_RAMP_DOWN_START_REVS).clamp(0.0, 1.0);
        let scaled = (f32::from(state.base_speed) * factor).round() as u8;
        scaled.clamp(types::DISTANCE_MIN_SPEED, types::DISTANCE_MAX_SPEED)
    } else {
        state.base_speed.min(types::DISTANCE_MAX_SPEED)
    };

    let signed_base = match state.direction {
        types::DriveDirection::Forward => ramp_speed as i8,
        types::DriveDirection::Backward => -(ramp_speed as i8),
    };

    let mut left_ratio = state.left_ratio;
    let mut right_ratio = state.right_ratio;

    if matches!(state.kind, types::DriveDistanceKind::CurveArc { .. }) && state.curve_last_yaw_deg.is_some() {
        let left_cm = state.accumulated_left_revs * types::SPROCKET_CIRCUMFERENCE_CM;
        let right_cm = state.accumulated_right_revs * types::SPROCKET_CIRCUMFERENCE_CM;
        let direction_sign = match state.direction {
            types::DriveDirection::Forward => 1.0,
            types::DriveDirection::Backward => -1.0,
        };

        let expected_yaw_rad = direction_sign * (right_cm - left_cm) / types::TRACK_WIDTH_CM;
        let actual_yaw_rad = state.curve_accumulated_yaw_deg.to_radians();
        let yaw_error = expected_yaw_rad - actual_yaw_rad;
        let correction = (types::DISTANCE_CURVE_YAW_KP * yaw_error).clamp(
            -types::DISTANCE_CURVE_YAW_MAX_CORRECTION,
            types::DISTANCE_CURVE_YAW_MAX_CORRECTION,
        );

        left_ratio = (left_ratio - correction).clamp(0.0, 1.0);
        right_ratio = (right_ratio + correction).clamp(0.0, 1.0);

        #[cfg(feature = "telemetry_logs")]
        {
            if (now_ms % 200) < 20 {
                defmt::info!(
                    "distance_curve: exp_yaw={=f32}rad act_yaw={=f32}rad err={=f32}rad corr={=f32}",
                    expected_yaw_rad,
                    actual_yaw_rad,
                    yaw_error,
                    correction
                );
            }
        }
    }

    let left_speed = (f32::from(signed_base) * left_ratio).round() as i8;
    let right_speed = (f32::from(signed_base) * right_ratio).round() as i8;

    let (adjusted_left, adjusted_right) = if matches!(state.kind, types::DriveDistanceKind::Straight { .. }) {
        if let (Some(ref_yaw), Some(sample)) = (state.reference_yaw, straight_imu_sample) {
            let current_yaw = sample.orientation.yaw;
            let mut heading_error = current_yaw - ref_yaw;
            if heading_error > 180.0 {
                heading_error -= 360.0;
            }
            if heading_error <= -180.0 {
                heading_error += 360.0;
            }
            let max_correction =
                (f32::from(ramp_speed) * types::STRAIGHT_IMU_CORRECTION_SCALE).min(types::STRAIGHT_IMU_MAX_CORRECTION);
            let correction = (types::STRAIGHT_IMU_KP * heading_error).clamp(-max_correction, max_correction);
            // Positive heading_error = drifting clockwise/right → slow right track
            // Negative heading_error = drifting counter-clockwise/left → slow left track
            let corrected_left = ((f32::from(left_speed) + correction).round() as i8).clamp(-100, 100);
            let corrected_right = ((f32::from(right_speed) - correction).round() as i8).clamp(-100, 100);
            #[cfg(feature = "telemetry_logs")]
            {
                if (now_ms % 200) < 20 {
                    defmt::info!(
                        "distance_straight: ref={=f32}° cur={=f32}° err={=f32}° corr={=f32}",
                        ref_yaw,
                        current_yaw,
                        heading_error,
                        correction,
                    );
                }
            }
            (corrected_left, corrected_right)
        } else {
            (left_speed, right_speed)
        }
    } else {
        (left_speed, right_speed)
    };

    state.last_left_speed = adjusted_left;
    state.last_right_speed = adjusted_right;

    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: adjusted_left,
        right_speed: adjusted_right,
    })
    .await;

    crate::system::state::motion::set_track_speeds(adjusted_left, adjusted_right).await;

    DistanceStepResult::InProgress
}

/// Stop both tracks and update system state (distance helpers can share this).
pub(super) async fn distance_stop_motors() {
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;

    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: 0,
        right_speed: 0,
    })
    .await;

    crate::system::state::motion::set_track_speeds(0, 0).await;
}
