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

use crate::{
    system::state,
    task::{
        drive::{
            sensors::data::{self as feedback, IMU_FEEDBACK_CHANNEL},
            types,
        },
        motor_driver::{self, MotorCommand},
        sensors::encoders::EncoderMeasurement,
    },
};

// ── Distance control constants ─────────────────────────────────────────────

/// Distance drive ramp-down begins when remaining revolutions drop below this value.
const RAMP_DOWN_START_REVS: f32 = 0.25;
/// Minimum speed during ramp-down (0-100).
const MIN_SPEED: u8 = 40;
/// Maximum speed clamp for distance driving (0-100).
pub(super) const MAX_SPEED: u8 = 100;
/// Completion tolerance for distance driving (revolutions).
pub(super) const TOLERANCE_REVS: f32 = 0.01;
/// Distance drive control interval in milliseconds.
pub(super) const CONTROL_INTERVAL_MS: u64 = 20;
/// Encoder timeout during distance driving (milliseconds).
const ENCODER_TIMEOUT_MS: u64 = 300;
/// Curve yaw correction proportional gain (radians -> ratio).
const CURVE_YAW_KP: f32 = 0.5;
/// Maximum absolute curve yaw correction applied to speed ratio.
const CURVE_YAW_MAX_CORRECTION: f32 = 0.25;
/// Proportional gain for IMU heading correction on straight drives.
const STRAIGHT_IMU_KP: f32 = 2.0;
/// Maximum absolute heading correction (speed units) before scaling.
const STRAIGHT_IMU_MAX_CORRECTION: f32 = 8.0;
/// Correction clamp scales with current ramp speed: `min(MAX, speed * SCALE)`.
const STRAIGHT_IMU_CORRECTION_SCALE: f32 = 0.15;

/// Stall timeout during distance driving (milliseconds).
const STALL_TIMEOUT_MS: u64 = 1500;

// ── Encoder delta helpers (previously in drift/math) ───────────────────────

/// 16-bit counter delta with wraparound handling.
const fn calculate_delta_u16(current: u16, previous: u16) -> u16 {
    if current >= previous {
        current - previous
    } else {
        (u16::MAX - previous).wrapping_add(current).wrapping_add(1)
    }
}

/// Snapshot of encoder pulse counts for the current sampling window,
/// together with computed per-track averages.
#[derive(Debug, Clone, Copy)]
struct TrackSpeedData {
    /// Left front encoder pulse count.
    left_front: u16,
    /// Left rear encoder pulse count.
    left_rear: u16,
    /// Right front encoder pulse count.
    right_front: u16,
    /// Right rear encoder pulse count.
    right_rear: u16,
    /// Computed average for the left track.
    left_track_avg: f32,
    /// Computed average for the right track.
    right_track_avg: f32,
    /// Originating measurement timestamp (ms).
    #[allow(dead_code)]
    timestamp_ms: u64,
}

impl TrackSpeedData {
    /// Returns true if all four motors reported zero pulses.
    const fn all_zero(&self) -> bool {
        self.left_front == 0 && self.left_rear == 0 && self.right_front == 0 && self.right_rear == 0
    }

    /// Returns true if some motors are nonzero while others are zero (anomaly).
    fn has_single_motor_zero_anomaly(&self) -> bool {
        let vals = [self.left_front, self.left_rear, self.right_front, self.right_rear];
        let any_nonzero = vals.iter().any(|&v| v != 0);
        let any_zero = vals.contains(&0);
        any_nonzero && any_zero
    }
}

/// Convert an [`EncoderMeasurement`] into [`TrackSpeedData`].
fn calculate_track_averages(measurement: EncoderMeasurement) -> TrackSpeedData {
    let left_track_avg = f32::midpoint(f32::from(measurement.left_front), f32::from(measurement.left_rear));
    let right_track_avg = f32::midpoint(f32::from(measurement.right_front), f32::from(measurement.right_rear));

    TrackSpeedData {
        left_front: measurement.left_front,
        left_rear: measurement.left_rear,
        right_front: measurement.right_front,
        right_rear: measurement.right_rear,
        left_track_avg,
        right_track_avg,
        timestamp_ms: measurement.timestamp_ms,
    }
}

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

// ── Main control step ──────────────────────────────────────────────────────────

/// Run a single step of the distance control loop.
///
/// 1. Read encoder data (timeout / anomaly checks)
/// 2. Check if target reached → complete
/// 3. Drain IMU channel, update heading or curve yaw state
/// 4. Compute ramp-down speed
/// 5. Apply IMU corrections (curve ratio + straight heading)
/// 6. Send motor commands
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, clippy::cast_possible_wrap)]
pub(super) async fn run_distance_control_step(state: &mut DistanceDriveState) -> DistanceStepResult {
    let now_ms = Instant::now().as_millis();

    // ── 1. Read encoder data ───────────────────────────────────────────────
    let _data = match read_encoder(state, now_ms).await {
        Ok(data) => data,
        Err(early) => return early,
    };

    // ── 2. Check completion ────────────────────────────────────────────────
    let inner_progress = state.inner_progress();
    let remaining = (state.target_inner_revs - inner_progress).max(0.0);

    if remaining <= TOLERANCE_REVS {
        let telemetry = state.make_telemetry(now_ms);
        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed: 0,
            right_speed: 0,
        })
        .await;
        state::motion::set_track_speeds(0, 0).await;
        return DistanceStepResult::Completed { telemetry };
    }

    // ── 3. Drain IMU channel (only after confirming not done) ───────────────
    let latest_imu = drain_imu(state);

    // ── 4. Compute ramp-down speed ──────────────────────────────────────────
    let ramp_speed = if remaining <= RAMP_DOWN_START_REVS {
        let factor = (remaining / RAMP_DOWN_START_REVS).clamp(0.0, 1.0);
        let scaled = (f32::from(state.base_speed) * factor).round() as u8;
        scaled.clamp(MIN_SPEED, MAX_SPEED)
    } else {
        state.base_speed.min(MAX_SPEED)
    };
    let signed_base = match state.direction {
        types::DriveDirection::Forward => ramp_speed as i8,
        types::DriveDirection::Backward => -(ramp_speed as i8),
    };

    // ── 5. Apply IMU corrections ────────────────────────────────────────────
    let (adjusted_left, adjusted_right) = apply_imu_corrections(state, latest_imu, signed_base, now_ms);

    // ── 6. Send motor commands ──────────────────────────────────────────────
    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: adjusted_left,
        right_speed: adjusted_right,
    })
    .await;
    state::motion::set_track_speeds(adjusted_left, adjusted_right).await;

    DistanceStepResult::InProgress
}

// ── Helper: read encoder data (with timeout / anomaly early exits) ─────────────

/// Read a fresh encoder sample, compute deltas, and check for stall/anomaly.
/// Returns the per-track averages on success, or an early-return result on failure.
#[allow(clippy::cast_possible_truncation)]
async fn read_encoder(state: &mut DistanceDriveState, now_ms: u64) -> Result<TrackSpeedData, DistanceStepResult> {
    // Timeout check.
    if now_ms.saturating_sub(state.last_encoder_seen_ms) >= ENCODER_TIMEOUT_MS {
        return Err(DistanceStepResult::Failed {
            reason: "EncoderTimeout",
            telemetry: state.make_telemetry(now_ms),
        });
    }

    // Wait for fresh measurement.
    let Some(measurement) = feedback::get_latest_encoder_measurement().await else {
        return Err(DistanceStepResult::InProgress);
    };
    if measurement.timestamp_ms == 0 || measurement.timestamp_ms == state.last_encoder_timestamp_ms {
        return Err(DistanceStepResult::InProgress);
    }
    state.last_encoder_timestamp_ms = measurement.timestamp_ms;
    state.last_encoder_seen_ms = now_ms;

    // Need two samples to compute deltas.
    if state.last_encoder_measurement.is_none() {
        state.last_encoder_measurement = Some(measurement);
        return Err(DistanceStepResult::InProgress);
    }

    // Compute deltas from cumulative counters.
    let delta_measurement = state
        .last_encoder_measurement
        .map_or(measurement, |prev| EncoderMeasurement {
            left_front: calculate_delta_u16(measurement.left_front, prev.left_front),
            left_rear: calculate_delta_u16(measurement.left_rear, prev.left_rear),
            right_front: calculate_delta_u16(measurement.right_front, prev.right_front),
            right_rear: calculate_delta_u16(measurement.right_rear, prev.right_rear),
            timestamp_ms: measurement.timestamp_ms,
        });
    state.last_encoder_measurement = Some(measurement);

    let data = calculate_track_averages(delta_measurement);

    // Stall check.
    if data.all_zero() {
        state.zero_progress_samples = state.zero_progress_samples.saturating_add(1);
        if now_ms.saturating_sub(state.last_progress_ms) >= STALL_TIMEOUT_MS {
            return Err(DistanceStepResult::Failed {
                reason: "StallTimeout",
                telemetry: state.make_telemetry(now_ms),
            });
        }
        return Err(DistanceStepResult::InProgress);
    }

    // Anomaly check.
    if data.has_single_motor_zero_anomaly() {
        return Err(DistanceStepResult::Failed {
            reason: "EncoderAnomaly",
            telemetry: state.make_telemetry(now_ms),
        });
    }

    // Accumulate progress.
    state.last_progress_ms = now_ms;
    state.zero_progress_samples = 0;
    state.accumulated_left_revs += data.left_track_avg / types::PULSES_PER_SPROCKET_REV_F32;
    state.accumulated_right_revs += data.right_track_avg / types::PULSES_PER_SPROCKET_REV_F32;

    Ok(data)
}

// ── Helper: drain IMU channel and update state ──────────────────────────────────

/// Drain the IMU feedback channel and update heading/curve state.
/// Returns the latest sample (if any) for use by correction logic.
fn drain_imu(state: &mut DistanceDriveState) -> Option<crate::task::sensors::imu::ImuMeasurement> {
    let mut latest: Option<crate::task::sensors::imu::ImuMeasurement> = None;
    while let Ok(m) = IMU_FEEDBACK_CHANNEL.receiver().try_receive() {
        latest = Some(m);
    }

    match state.kind {
        types::DriveDistanceKind::Straight { .. } => {
            if state.reference_yaw.is_none()
                && let Some(sample) = latest
            {
                state.reference_yaw = Some(sample.orientation.yaw);
            }
        }
        types::DriveDistanceKind::CurveArc { .. } => {
            if let Some(measurement) = latest {
                if let Some(last_yaw) = state.curve_last_yaw_deg {
                    let mut delta = measurement.orientation.yaw - last_yaw;
                    if delta > 180.0 {
                        delta -= 360.0;
                    } else if delta < -180.0 {
                        delta += 360.0;
                    }
                    state.curve_accumulated_yaw_deg += delta;
                }
                state.curve_last_yaw_deg = Some(measurement.orientation.yaw);
            }
        }
    }

    latest
}

// ── Helper: apply IMU-based corrections to motor speeds ────────────────────────

/// Apply curve-ratio correction or straight heading correction, returning
/// the final (left, right) motor speeds.
#[allow(clippy::cast_possible_truncation)]
fn apply_imu_corrections(
    state: &DistanceDriveState,
    latest_imu: Option<crate::task::sensors::imu::ImuMeasurement>,
    signed_base: i8,
    _now_ms: u64,
) -> (i8, i8) {
    let mut left_ratio = state.left_ratio;
    let mut right_ratio = state.right_ratio;

    // Curve-arc: adjust ratios based on yaw error.
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
        let correction = (CURVE_YAW_KP * yaw_error).clamp(-CURVE_YAW_MAX_CORRECTION, CURVE_YAW_MAX_CORRECTION);
        left_ratio = (left_ratio - correction).clamp(0.0, 1.0);
        right_ratio = (right_ratio + correction).clamp(0.0, 1.0);

        #[cfg(feature = "telemetry_logs")]
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

    let left_speed = (f32::from(signed_base) * left_ratio).round() as i8;
    let right_speed = (f32::from(signed_base) * right_ratio).round() as i8;

    // Straight: apply heading correction on top of base speeds.
    if matches!(state.kind, types::DriveDistanceKind::Straight { .. })
        && let (Some(ref_yaw), Some(sample)) = (state.reference_yaw, latest_imu)
    {
        let current_yaw = sample.orientation.yaw;
        let mut heading_error = current_yaw - ref_yaw;
        if heading_error > 180.0 {
            heading_error -= 360.0;
        } else if heading_error <= -180.0 {
            heading_error += 360.0;
        }
        let ramp_speed = signed_base.unsigned_abs();
        let max_correction = (f32::from(ramp_speed) * STRAIGHT_IMU_CORRECTION_SCALE).min(STRAIGHT_IMU_MAX_CORRECTION);
        let correction = (STRAIGHT_IMU_KP * heading_error).clamp(-max_correction, max_correction);
        let corrected_left = ((f32::from(left_speed) + correction).round() as i8).clamp(-100, 100);
        let corrected_right = ((f32::from(right_speed) - correction).round() as i8).clamp(-100, 100);

        #[cfg(feature = "telemetry_logs")]
        if (now_ms % 200) < 20 {
            defmt::info!(
                "distance_straight: ref={=f32}° cur={=f32}° err={=f32}° corr={=f32}",
                ref_yaw,
                current_yaw,
                heading_error,
                correction
            );
        }

        return (corrected_left, corrected_right);
    }

    (left_speed, right_speed)
}

// ── Helpers on DistanceDriveState ─────────────────────────────────────────────

impl DistanceDriveState {
    /// Build a telemetry snapshot from current state.
    const fn make_telemetry(&self, now_ms: u64) -> types::CompletionTelemetry {
        types::CompletionTelemetry::DriveDistance {
            achieved_left_revs: self.accumulated_left_revs,
            achieved_right_revs: self.accumulated_right_revs,
            target_left_revs: self.target_left_revs,
            target_right_revs: self.target_right_revs,
            duration_ms: now_ms.saturating_sub(self.started_at_ms),
        }
    }

    /// Compute inner-track progress (the limiting track for completion).
    fn inner_progress(&self) -> f32 {
        match self.kind {
            types::DriveDistanceKind::Straight { .. } => self.accumulated_left_revs.min(self.accumulated_right_revs),
            types::DriveDistanceKind::CurveArc { .. } => match self.inner_left {
                Some(true) => self.accumulated_left_revs,
                Some(false) => self.accumulated_right_revs,
                None => (self.accumulated_left_revs + self.accumulated_right_revs) * 0.5,
            },
        }
    }
}
