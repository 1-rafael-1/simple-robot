//! Rotation control: state machine and async control loop.
//!
//! This module is self-contained for all rotation concerns:
//!
//! - [`RotationState`]: pure state/math — tracks accumulated yaw, computes motor
//!   speeds, detects completion. No async, no I/O.
//! - [`run_rotation_control_step`]: async runner — consumes IMU samples from the
//!   feedback channel, drives `RotationState`, issues motor commands, and applies
//!   an IMU watchdog that aborts rotation if no samples arrive for 300 ms.
//!
//! # IMU dependence and timeout behavior
//!
//! Rotation relies on continuous IMU yaw samples. The loop drains all queued
//! samples each tick and uses the newest one. If no samples are available, it
//! waits once for 300 ms and retries; continued absence is treated as a hard
//! failure to prevent uncontrolled rotation.
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

        (self.accumulated_angle - self.target_angle).abs() <= types::ROTATION_TOLERANCE_DEG
    }

    /// Calculates appropriate motor speeds for the current rotation state.
    ///
    /// Returns `(left_speed, right_speed)`.
    pub fn calculate_motor_speeds(&self) -> (i8, i8) {
        // Signed error: positive => undershoot, negative => overshoot.
        let error_deg = self.target_angle - self.accumulated_angle;
        let remaining_degrees = error_deg.abs();

        // If we overshot, reverse effective direction to hunt back toward the setpoint.
        let effective_direction = if error_deg < 0.0 {
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
                if remaining_degrees < 10.0 {
                    let min = types::ROTATION_SPEED_MIN;
                    let max = requested.max(min);
                    let speed_range = max - min;
                    let speed_factor = remaining_degrees / 10.0;
                    Self::clamp_speed_u8(f32::from(min) + (f32::from(speed_range) * speed_factor))
                } else {
                    requested
                }
            }
            types::RotationMotion::WhileMoving(_) => {
                if remaining_degrees < 10.0 {
                    let speed_range = types::ROTATION_SPEED_MAX - types::ROTATION_SPEED_MIN;
                    let speed_factor = remaining_degrees / 10.0;
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

/// Run a single step of the rotation control loop.
///
/// Drains all queued IMU samples and uses the newest one. If no sample is
/// available, waits 300 ms and retries once; a second absence is treated as a
/// hard `ImuTimeout` failure to prevent uncontrolled rotation.
pub(super) async fn run_rotation_control_step(
    rotation_state: &mut RotationState,
    started_at_ms: u64,
) -> RotationStepResult {
    // Drain the IMU feedback channel; keep only the newest sample.
    let mut latest: Option<ImuMeasurement> = None;
    while let Ok(m) = IMU_FEEDBACK_CHANNEL.receiver().try_receive() {
        latest = Some(m);
    }

    // Watchdog: no IMU data → wait 300 ms and retry once before aborting.
    let Some(measurement) = latest else {
        defmt::warn!("rotate_exact: no IMU data available (will abort after 300ms without updates)");
        Timer::after(Duration::from_millis(300)).await;

        let mut latest_after_wait: Option<ImuMeasurement> = None;
        while let Ok(m) = IMU_FEEDBACK_CHANNEL.receiver().try_receive() {
            latest_after_wait = Some(m);
        }

        if latest_after_wait.is_none() {
            defmt::warn!("rotate_exact: aborting rotation due to missing IMU data (>= 300ms)");

            motor_driver::send_motor_command(MotorCommand::SetTracks {
                left_speed: 0,
                right_speed: 0,
            })
            .await;

            motion::set_track_speeds(0, 0).await;

            let accumulated = rotation_state.accumulated_angle.abs();
            let target = rotation_state.target_angle.abs();
            let last_yaw_deg = rotation_state.last_yaw.unwrap_or(0.0);
            let duration_ms = Instant::now().as_millis() - started_at_ms;

            return RotationStepResult::Failed {
                reason: "ImuTimeout",
                telemetry: types::CompletionTelemetry::RotateExact {
                    final_yaw_deg: last_yaw_deg,
                    angle_error_deg: accumulated - target,
                    duration_ms,
                },
            };
        }

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
        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed: 0,
            right_speed: 0,
        })
        .await;

        motion::set_track_speeds(0, 0).await;

        let accumulated = rotation_state.accumulated_angle.abs();
        let target = rotation_state.target_angle.abs();
        let duration_ms = Instant::now().as_millis() - started_at_ms;

        return RotationStepResult::Completed {
            telemetry: types::CompletionTelemetry::RotateExact {
                final_yaw_deg: measurement.orientation.yaw,
                angle_error_deg: accumulated - target,
                duration_ms,
            },
        };
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
