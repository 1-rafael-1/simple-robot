//! Rotation control: state machine and async control loop.
//!
//! This module is self-contained for all rotation concerns:
//!
//! - [`RotationState`]: pure state/math — tracks accumulated yaw, computes motor
//!   speeds, detects completion. No async, no I/O.
//! - [`run_rotation_control_step`]: async runner — consumes IMU samples from the
//!   feedback channel, drives `RotationState`, issues motor commands, and applies
//!   a short IMU wait per tick to smooth sampling gaps without stalling rotation.
//!   It also captures post-stop settling and can apply a single corrective pulse
//!   if the robot coasts past the target after stopping.
//!
//! # IMU dependence and timeout behavior
//!
//! Rotation relies on continuous IMU yaw samples. The loop drains all queued
//! samples each tick and uses the newest one. If no samples are available, it
//! waits briefly for a fresh sample and retries once per tick; otherwise it
//! keeps the previous motor command and tries again on the next tick. After
//! stopping, it waits briefly to capture any settling and updates telemetry.
//!
//! # Telemetry logging
//!
//! Rotation debug logs are rate-limited and compiled only when the `telemetry_logs`
//! feature is enabled; otherwise no formatting/queueing cost is incurred.

use embassy_time::{Duration, Instant, Timer};

use crate::{
    system::state::motion,
    task::{
        drive::{sensors::data::IMU_FEEDBACK_CHANNEL, types},
        motor_driver::{self, MotorCommand},
        sensors::imu::ImuMeasurement,
    },
};

// ── Pure state / math ────────────────────────────────────────────────────────

/// State tracking for precise rotation maneuvers.
///
/// Maintains rotation progress and calculates differential motor speeds
/// needed to achieve the target rotation angle using IMU feedback.
/// Contains no async code and issues no I/O; the async runner below owns that.
pub struct RotationState {
    /// Target rotation angle in degrees.
    pub(crate) target_angle: f32,
    /// Accumulated rotation so far (in degrees).
    pub(crate) accumulated_angle: f32,
    /// Last measured yaw angle (in degrees).
    pub(crate) last_yaw: Option<f32>,
    /// Last update timestamp (ms since boot).
    last_update_ms: u64,
    /// Direction of rotation.
    direction: types::RotationDirection,
    /// Type of motion during rotation.
    motion: types::RotationMotion,
    /// Base forward/backward speed when rotating while moving.
    base_speed: i8,
    /// Count of corrective direction flips detected.
    correction_flips: u8,
    /// Timestamp when overshoot correction started (ms since boot).
    correction_started_at_ms: Option<u64>,
    /// Last signed error direction for flip detection.
    last_error_sign: i8,
}

impl RotationState {
    /// Creates new rotation tracking state.
    pub const fn new(target_angle: f32, direction: types::RotationDirection, motion: types::RotationMotion) -> Self {
        let base_speed = match motion {
            types::RotationMotion::Stationary { speed: _ } => 0,
            types::RotationMotion::WhileMoving(speed) => speed,
        };

        Self {
            target_angle,
            accumulated_angle: 0.0,
            last_yaw: None,
            last_update_ms: 0,
            direction,
            motion,
            base_speed,
            correction_flips: 0,
            correction_started_at_ms: None,
            last_error_sign: 0,
        }
    }

    /// Updates rotation state with a new IMU measurement.
    ///
    /// Returns `true` if the target angle has been reached within tolerance.
    pub fn update(&mut self, measurement: &ImuMeasurement) -> bool {
        if let Some(last_yaw) = self.last_yaw {
            let mut yaw_change = measurement.orientation.yaw - last_yaw;

            // Handle wraparound at ±180 degrees.
            if yaw_change > 180.0 {
                yaw_change -= 360.0;
            } else if yaw_change < -180.0 {
                yaw_change += 360.0;
            }

            self.accumulated_angle += match self.direction {
                types::RotationDirection::Clockwise => -yaw_change,
                types::RotationDirection::CounterClockwise => yaw_change,
            };
        }

        self.last_yaw = Some(measurement.orientation.yaw);
        self.last_update_ms = measurement.timestamp_ms;

        let error_deg = self.target_angle - self.accumulated_angle;
        let error_sign = if error_deg >= 0.0 { 1 } else { -1 };
        if error_deg.abs() > types::ROTATION_CORRECTION_DEADBAND_DEG {
            if self.last_error_sign != 0 && error_sign != self.last_error_sign {
                self.correction_flips = self.correction_flips.saturating_add(1);
                if self.correction_started_at_ms.is_none() {
                    self.correction_started_at_ms = Some(measurement.timestamp_ms);
                }
            }
            self.last_error_sign = error_sign;
        }

        error_deg.abs() <= types::ROTATION_TOLERANCE_DEG
    }

    /// Calculates appropriate motor speeds for the current rotation state.
    ///
    /// Returns `(left_speed, right_speed)`.
    pub fn calculate_motor_speeds(&self) -> (i8, i8) {
        // Signed error: positive => undershoot, negative => overshoot.
        let error_deg = self.target_angle - self.accumulated_angle;
        let remaining_degrees = error_deg.abs();

        // If we overshot beyond the deadband, reverse effective direction to hunt back toward the setpoint.
        let effective_direction = if error_deg < -types::ROTATION_CORRECTION_DEADBAND_DEG {
            match self.direction {
                types::RotationDirection::Clockwise => types::RotationDirection::CounterClockwise,
                types::RotationDirection::CounterClockwise => types::RotationDirection::Clockwise,
            }
        } else {
            self.direction
        };

        let rotation_speed = match self.motion {
            types::RotationMotion::Stationary { speed } => {
                // Keep a floor of ROTATION_SPEED_MIN to overcome static friction, but
                // still taper near the target to limit overshoot.
                let requested = speed.clamp(0, 100);
                if remaining_degrees < types::ROTATION_RAMP_DOWN_START_DEG {
                    let min = types::ROTATION_SPEED_MIN;
                    let max = requested.max(min);
                    let speed_range = max - min;
                    let speed_factor = remaining_degrees / types::ROTATION_RAMP_DOWN_START_DEG;
                    Self::clamp_speed_u8(f32::from(min) + (f32::from(speed_range) * speed_factor))
                } else {
                    requested
                }
            }
            types::RotationMotion::WhileMoving(_) => {
                if remaining_degrees < types::ROTATION_RAMP_DOWN_START_DEG {
                    let speed_range = types::ROTATION_SPEED_MAX - types::ROTATION_SPEED_MIN;
                    let speed_factor = remaining_degrees / types::ROTATION_RAMP_DOWN_START_DEG;
                    Self::clamp_speed_u8(f32::from(types::ROTATION_SPEED_MIN) + (f32::from(speed_range) * speed_factor))
                } else {
                    types::ROTATION_SPEED_MAX
                }
            }
        };
        let rotation_speed_signed = i8::try_from(rotation_speed).unwrap_or(i8::MAX);

        match self.motion {
            types::RotationMotion::Stationary { speed: _ } => match effective_direction {
                types::RotationDirection::Clockwise => (rotation_speed_signed, -rotation_speed_signed),
                types::RotationDirection::CounterClockwise => (-rotation_speed_signed, rotation_speed_signed),
            },
            types::RotationMotion::WhileMoving(_) => {
                let rotation_diff = rotation_speed_signed.min(types::SPEED_DIFF_MAX);
                match effective_direction {
                    types::RotationDirection::Clockwise => {
                        (self.base_speed, (self.base_speed - rotation_diff).clamp(-100, 100))
                    }
                    types::RotationDirection::CounterClockwise => {
                        ((self.base_speed - rotation_diff).clamp(-100, 100), self.base_speed)
                    }
                }
            }
        }
    }

    /// Clamp a float speed value to the valid `u8` range 0–100.
    #[allow(clippy::cast_possible_truncation)]
    fn clamp_speed_u8(value: f32) -> u8 {
        let clamped = value.clamp(0.0, 100.0);
        if clamped.is_nan() {
            0
        } else {
            let truncated = clamped as i32;
            u8::try_from(truncated).unwrap_or(0)
        }
    }
}

// ── Async control loop ────────────────────────────────────────────────────────

/// Result of a single rotation control step.
pub(super) enum RotationStepResult {
    /// Rotation is still in progress; continue ticking.
    InProgress,
    /// Rotation completed successfully with final telemetry.
    Completed {
        /// Completion telemetry captured at success.
        telemetry: types::CompletionTelemetry,
    },
    /// Rotation failed with a static reason and telemetry snapshot.
    Failed {
        /// Failure reason identifier.
        reason: &'static str,
        /// Completion telemetry captured at failure.
        telemetry: types::CompletionTelemetry,
    },
}

/// Stop rotation motors and update the motion state.
async fn stop_rotation_motors() {
    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: 0,
        right_speed: 0,
    })
    .await;

    motion::set_track_speeds(0, 0).await;
}

/// Build a failure result with consistent telemetry from the current state.
fn rotation_failure(
    rotation_state: &RotationState,
    started_at_ms: u64,
    now_ms: u64,
    reason: &'static str,
) -> RotationStepResult {
    let accumulated = rotation_state.accumulated_angle.abs();
    let target = rotation_state.target_angle.abs();
    let last_yaw_deg = rotation_state.last_yaw.unwrap_or(0.0);
    let duration_ms = now_ms - started_at_ms;

    RotationStepResult::Failed {
        reason,
        telemetry: types::CompletionTelemetry::RotateExact {
            final_yaw_deg: last_yaw_deg,
            angle_error_deg: accumulated - target,
            duration_ms,
        },
    }
}

/// Build a success result using the final IMU measurement.
fn rotation_success(
    measurement: &ImuMeasurement,
    rotation_state: &RotationState,
    started_at_ms: u64,
) -> RotationStepResult {
    let accumulated = rotation_state.accumulated_angle.abs();
    let target = rotation_state.target_angle.abs();
    let duration_ms = Instant::now().as_millis() - started_at_ms;

    RotationStepResult::Completed {
        telemetry: types::CompletionTelemetry::RotateExact {
            final_yaw_deg: measurement.orientation.yaw,
            angle_error_deg: accumulated - target,
            duration_ms,
        },
    }
}

/// Drain the IMU feedback channel and return the newest sample since rotation start.
fn drain_latest_imu_since(started_at_ms: u64) -> Option<ImuMeasurement> {
    let mut latest: Option<ImuMeasurement> = None;
    while let Ok(m) = IMU_FEEDBACK_CHANNEL.receiver().try_receive() {
        if m.timestamp_ms >= started_at_ms {
            latest = Some(m);
        }
    }
    latest
}

/// Wait for a fresh IMU sample, then return the newest available sample.
async fn wait_for_latest_imu_since(started_at_ms: u64, wait_ms: u64) -> Option<ImuMeasurement> {
    if let Some(latest) = drain_latest_imu_since(started_at_ms) {
        return Some(latest);
    }

    Timer::after(Duration::from_millis(wait_ms)).await;
    drain_latest_imu_since(started_at_ms)
}

/// Read the newest IMU sample for this tick, waiting briefly if needed.
async fn read_rotation_measurement(started_at_ms: u64) -> Option<ImuMeasurement> {
    if let Some(measurement) = drain_latest_imu_since(started_at_ms) {
        return Some(measurement);
    }

    Timer::after(Duration::from_millis(types::ROTATION_IMU_WAIT_TIMEOUT_MS)).await;
    drain_latest_imu_since(started_at_ms)
}

/// Capture post-stop settling and optional correction pulse; returns final IMU measurement.
async fn capture_post_stop_measurement(
    rotation_state: &mut RotationState,
    started_at_ms: u64,
    measurement: ImuMeasurement,
) -> ImuMeasurement {
    let settled = wait_for_latest_imu_since(started_at_ms, types::ROTATION_POST_STOP_SETTLE_MS)
        .await
        .map_or(measurement, |settle| {
            rotation_state.update(&settle);

            #[cfg(feature = "telemetry_logs")]
            {
                let error_deg = rotation_state.target_angle - rotation_state.accumulated_angle;
                defmt::info!(
                    "rotate_exact: post_stop yaw={=f32}°, acc={=f32}°, err={=f32}°",
                    settle.orientation.yaw,
                    rotation_state.accumulated_angle.abs(),
                    error_deg
                );
            }

            settle
        });

    let error_deg = rotation_state.target_angle - rotation_state.accumulated_angle;
    let needs_correction = error_deg.abs() > types::ROTATION_TOLERANCE_DEG;
    let can_pulse = matches!(rotation_state.motion, types::RotationMotion::Stationary { .. });

    if !needs_correction || !can_pulse {
        return settled;
    }

    #[cfg(feature = "telemetry_logs")]
    {
        defmt::info!("rotate_exact: post_stop correction pulse err={=f32}°", error_deg);
    }

    let (left_speed, right_speed) = rotation_state.calculate_motor_speeds();
    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed,
        right_speed,
    })
    .await;

    motion::set_track_speeds(left_speed, right_speed).await;
    Timer::after(Duration::from_millis(types::ROTATION_POST_STOP_CORRECTION_PULSE_MS)).await;

    stop_rotation_motors().await;

    if let Some(settle) = wait_for_latest_imu_since(started_at_ms, types::ROTATION_POST_STOP_SETTLE_MS).await {
        rotation_state.update(&settle);

        #[cfg(feature = "telemetry_logs")]
        {
            let error_deg = rotation_state.target_angle - rotation_state.accumulated_angle;
            defmt::info!(
                "rotate_exact: post_pulse yaw={=f32}°, acc={=f32}°, err={=f32}°",
                settle.orientation.yaw,
                rotation_state.accumulated_angle.abs(),
                error_deg
            );
        }

        return settle;
    }

    settled
}

/// Run a single step of the rotation control loop.
///
/// Drains all queued IMU samples and uses the newest one. If no sample is
/// available, waits briefly for a fresh sample and retries once this tick.
/// If still empty, it keeps the previous motor command and tries again next tick.
pub(super) async fn run_rotation_control_step(
    rotation_state: &mut RotationState,
    started_at_ms: u64,
) -> RotationStepResult {
    let now_ms = Instant::now().as_millis();
    if now_ms - started_at_ms > types::ROTATION_TIMEOUT_MS {
        stop_rotation_motors().await;
        return rotation_failure(rotation_state, started_at_ms, now_ms, "RotateTimeout");
    }

    let Some(measurement) = read_rotation_measurement(started_at_ms).await else {
        return RotationStepResult::InProgress;
    };

    // Rate-limited debug logging (compiled out when feature is absent).
    #[cfg(feature = "telemetry_logs")]
    {
        if (measurement.timestamp_ms % 100) < 25 {
            defmt::info!(
                "rotate_exact: yaw={=f32}°, acc={=f32}°, target={=f32}°",
                measurement.orientation.yaw,
                rotation_state.accumulated_angle.abs(),
                rotation_state.target_angle.abs()
            );
        }
    }

    // Advance the state machine. If the target is reached, stop motors and report completion.
    let done = rotation_state.update(&measurement);
    if done {
        stop_rotation_motors().await;

        let final_measurement = capture_post_stop_measurement(rotation_state, started_at_ms, measurement).await;

        return rotation_success(&final_measurement, rotation_state, started_at_ms);
    }

    if rotation_state.correction_flips >= types::ROTATION_CORRECTION_MAX_FLIPS {
        stop_rotation_motors().await;
        return rotation_failure(rotation_state, started_at_ms, now_ms, "RotateCorrectionLimit");
    }

    if let Some(start_ms) = rotation_state.correction_started_at_ms
        && now_ms - start_ms > types::ROTATION_CORRECTION_TIMEOUT_MS
    {
        stop_rotation_motors().await;
        return rotation_failure(rotation_state, started_at_ms, now_ms, "RotateCorrectionTimeout");
    }

    // Still in progress — apply updated motor speeds for this tick.
    let (left_speed, right_speed) = rotation_state.calculate_motor_speeds();
    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed,
        right_speed,
    })
    .await;

    motion::set_track_speeds(left_speed, right_speed).await;

    RotationStepResult::InProgress
}
