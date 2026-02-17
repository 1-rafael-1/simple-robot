//! Rotation control loop wrapper and IMU timeout behavior.
//!
//! This module drives the in-place rotation controller (`RotationState`) using
//! IMU feedback. It applies an IMU watchdog: if no IMU samples are observed
//! for 300ms while rotating, the rotation is aborted, motors are stopped, and a
//! failure result is returned.
//!
//! # IMU dependence and timeout behavior
//!
//! Rotation relies on continuous IMU yaw samples. The loop drains all queued
//! samples each tick and uses the newest one. If no samples are available, it
//! waits once for 300ms and retries; continued absence is treated as a hard
//! failure to prevent uncontrolled rotation.
//!
//! # Telemetry logging
//!
//! Rotation debug logs are rate-limited and compiled only when the `telemetry_logs`
//! feature is enabled; otherwise no formatting/queueing cost is incurred.

use embassy_time::{Duration, Instant, Timer};

use crate::{
    system::state::SYSTEM_STATE,
    task::{
        drive::{control::RotationState, feedback::IMU_FEEDBACK_CHANNEL, types},
        motor_driver::{self, MotorCommand},
        sensors::imu::ImuMeasurement,
    },
};

/// Result of a rotation control step.
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
pub(super) async fn run_rotation_control_step(
    rotation_state: &mut RotationState,
    started_at_ms: u64,
) -> RotationStepResult {
    // Consume as many queued IMU samples as available; keep the newest one for control.
    let mut latest: Option<ImuMeasurement> = None;

    while let Ok(m) = IMU_FEEDBACK_CHANNEL.receiver().try_receive() {
        latest = Some(m);
    }

    // Simple watchdog: if we haven't seen any IMU data for 300ms while rotating, abort.
    let Some(measurement) = latest else {
        defmt::warn!("rotate_exact: no IMU data available (will abort after 300ms without updates)");
        Timer::after(Duration::from_millis(300)).await;

        // Drain again after waiting; if still empty, abort rotation.
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

            {
                let mut sys = SYSTEM_STATE.lock().await;
                sys.left_track_speed = 0;
                sys.right_track_speed = 0;
            }

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

    // Periodic debug (gated): log to confirm IMU flow + accumulation.
    // IMPORTANT: keep this behind a feature flag so we don't pay formatting/queuing costs
    // when running without an active log consumer.
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

    // Update rotation progress. When done, stop motors and emit completion event + details.
    let done = rotation_state.update(&measurement);
    if done {
        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed: 0,
            right_speed: 0,
        })
        .await;

        {
            let mut sys = SYSTEM_STATE.lock().await;
            sys.left_track_speed = 0;
            sys.right_track_speed = 0;
        }

        // Provide completion details to anyone awaiting rotation completion.
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

    // Apply updated motor speeds for this step
    let (left_speed, right_speed) = rotation_state.calculate_motor_speeds();
    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed,
        right_speed,
    })
    .await;

    let mut sys = SYSTEM_STATE.lock().await;
    sys.left_track_speed = left_speed;
    sys.right_track_speed = right_speed;

    RotationStepResult::InProgress
}
