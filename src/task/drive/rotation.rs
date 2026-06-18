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
use libm::roundf;

use crate::{
    system::state::motion,
    task::{
        drive::{
            clear_imu_measurements,
            sensors::{control as lifecycle, data::IMU_FEEDBACK_CHANNEL},
            types,
        },
        motor_driver::{self, MotorCommand},
        sensors::imu::ImuMeasurement,
    },
};

// ── Rotation control constants ─────────────────────────────────────────────

/// Maximum rotation speed (0-100%).
const SPEED_MAX: u8 = 100;
/// Minimum rotation speed to overcome friction.
const SPEED_MIN: u8 = 45;
/// Acceptable angle error in degrees.
const TOLERANCE_DEG: f32 = 0.5;
/// Overshoot deadband before applying corrective reverse (degrees).
const CORRECTION_DEADBAND_DEG: f32 = 0.5;
/// Maximum time allowed for overshoot correction before bailing (milliseconds).
const CORRECTION_TIMEOUT_MS: u64 = 4_000;
/// Rotation ramp-down begins when remaining degrees drop below this value.
const RAMP_DOWN_START_DEG: f32 = 60.0;
/// Maximum number of corrective direction flips allowed during rotation.
const CORRECTION_MAX_FLIPS: u8 = 12;
/// Safety timeout for `RotateExact` (milliseconds).
const TIMEOUT_MS: u64 = 8_000;
/// Maximum time to wait for a fresh IMU sample per tick (milliseconds).
const IMU_WAIT_TIMEOUT_MS: u64 = 20;
/// Maximum correction iterations after the initial turn stops.
const CORRECTION_MAX_ITERATIONS: u8 = 15;
/// Settle wait after each correction pulse (ms).
const CORRECTION_SETTLE_MS: u64 = 400;
/// Starting speed for the first correction pulse.
const CORRECTION_SPEED_START: u8 = 35;
/// Speed reduction per correction iteration.
const CORRECTION_SPEED_STEP: u8 = 2;
/// Minimum speed floor for correction pulses.
const CORRECTION_SPEED_MIN: u8 = 25;
/// Maximum time for a single correction iteration before giving up and moving on (ms).
const CORRECTION_ITER_TIMEOUT_MS: u64 = 400;
/// Maximum speed differential during combined motion.
const SPEED_DIFF_MAX: i8 = 30;

// ── Pure state / math ────────────────────────────────────────────────────────

/// Normalize an angle in degrees to the range (−180, +180].
fn normalize_angle(deg: f32) -> f32 {
    let d = deg % 360.0;
    if d > 180.0 {
        d - 360.0
    } else if d < -180.0 {
        d + 360.0
    } else {
        d
    }
}

/// State tracking for precise rotation maneuvers.
///
/// Maintains rotation progress and calculates differential motor speeds
/// needed to achieve the target rotation angle using IMU feedback.
/// Contains no async code and issues no I/O; the async runner below owns that.
pub struct RotationState {
    /// Target rotation angle in degrees.
    pub(crate) target_angle: f32,
    /// Last measured yaw angle (in degrees).
    pub(crate) last_yaw: Option<f32>,
    /// Absolute euler yaw at the start of the maneuver (degrees). Set on first IMU sample.
    pub(crate) start_yaw: Option<f32>,
    /// Absolute target euler yaw (degrees). Computed from `start_yaw` + direction × `target_angle`.
    pub(crate) target_yaw: Option<f32>,
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
    /// Initialise a rotation intent — clear IMU samples, zero motors, create state.
    ///
    /// Returns the `ActiveIntent` and `IntentSetup` descriptor. Motor speeds stay at
    /// zero until the first IMU sample arrives in the control step.
    pub(super) async fn init(
        degrees: f32,
        direction: types::RotationDirection,
        motion: types::RotationMotion,
        completion_requested: bool,
    ) -> (super::state::ActiveIntent, types::IntentSetup) {
        lifecycle::start_rotation_imu().await;
        clear_imu_measurements();

        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed: 0,
            right_speed: 0,
        })
        .await;
        motion::set_track_speeds(0, 0).await;

        let state = Self::new(degrees, direction, motion);
        let started_at_ms = embassy_time::Instant::now().as_millis();

        let intent = super::state::ActiveIntent::RotateExact {
            state,
            completion_requested,
            started_at_ms,
        };

        (intent, types::IntentSetup::RotationImu)
    }

    /// Creates new rotation tracking state.
    pub const fn new(target_angle: f32, direction: types::RotationDirection, motion: types::RotationMotion) -> Self {
        let base_speed = match motion {
            types::RotationMotion::Stationary { speed: _ } => 0,
            types::RotationMotion::WhileMoving(speed) => speed,
        };

        Self {
            target_angle,
            last_yaw: None,
            start_yaw: None,
            target_yaw: None,
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
        let current_yaw = measurement.orientation.yaw;

        // On the very first sample: anchor start_yaw and compute the absolute target.
        if self.start_yaw.is_none() {
            self.start_yaw = Some(current_yaw);
            self.target_yaw = Some(match self.direction {
                types::RotationDirection::Clockwise => normalize_angle(current_yaw - self.target_angle),
                types::RotationDirection::CounterClockwise => normalize_angle(current_yaw + self.target_angle),
            });
        }

        self.last_yaw = Some(current_yaw);
        self.last_update_ms = measurement.timestamp_ms;

        let remaining = self.remaining();
        let error_sign: i8 = if remaining >= 0.0 { 1 } else { -1 };
        if remaining.abs() > CORRECTION_DEADBAND_DEG {
            if self.last_error_sign != 0 && error_sign != self.last_error_sign {
                self.correction_flips = self.correction_flips.saturating_add(1);
                if self.correction_started_at_ms.is_none() {
                    self.correction_started_at_ms = Some(measurement.timestamp_ms);
                }
            }
            self.last_error_sign = error_sign;
        }

        remaining.abs() <= TOLERANCE_DEG
    }

    /// Remaining signed angle to target (degrees).
    ///
    /// Positive  → still needs to turn in the commanded direction (undershoot).
    /// Negative  → has gone past the target (overshoot).
    /// Returns `target_angle` when no IMU sample has been received yet.
    pub fn remaining(&self) -> f32 {
        let Some(current_yaw) = self.last_yaw else {
            return self.target_angle;
        };
        let Some(target_yaw) = self.target_yaw else {
            return self.target_angle;
        };
        let diff = normalize_angle(current_yaw - target_yaw);
        match self.direction {
            types::RotationDirection::Clockwise => diff,
            types::RotationDirection::CounterClockwise => -diff,
        }
    }

    /// Calculates appropriate motor speeds for the current rotation state.
    ///
    /// Returns `(left_speed, right_speed)`.
    pub fn calculate_motor_speeds(&self) -> (i8, i8) {
        // Signed error: positive => undershoot, negative => overshoot.
        let error_deg = self.remaining();
        let remaining_degrees = error_deg.abs();

        // If we overshot beyond the deadband, reverse effective direction to hunt back toward the setpoint.
        let effective_direction = if error_deg < -CORRECTION_DEADBAND_DEG {
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
                if remaining_degrees < RAMP_DOWN_START_DEG {
                    let min = SPEED_MIN;
                    let max = requested.max(min);
                    let speed_range = max - min;
                    let speed_factor = remaining_degrees / RAMP_DOWN_START_DEG;
                    Self::clamp_speed_u8(f32::from(min) + (f32::from(speed_range) * speed_factor))
                } else {
                    requested
                }
            }
            types::RotationMotion::WhileMoving(_) => {
                if remaining_degrees < RAMP_DOWN_START_DEG {
                    let speed_range = SPEED_MAX - SPEED_MIN;
                    let speed_factor = remaining_degrees / RAMP_DOWN_START_DEG;
                    Self::clamp_speed_u8(f32::from(SPEED_MIN) + (f32::from(speed_range) * speed_factor))
                } else {
                    SPEED_MAX
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
                let rotation_diff = rotation_speed_signed.min(SPEED_DIFF_MAX);
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
    let last_yaw_deg = rotation_state.last_yaw.unwrap_or(0.0);
    let duration_ms = now_ms - started_at_ms;

    RotationStepResult::Failed {
        reason,
        telemetry: types::CompletionTelemetry::RotateExact {
            final_yaw_deg: last_yaw_deg,
            angle_error_deg: -rotation_state.remaining(),
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
    let duration_ms = Instant::now().as_millis() - started_at_ms;

    RotationStepResult::Completed {
        telemetry: types::CompletionTelemetry::RotateExact {
            final_yaw_deg: measurement.orientation.yaw,
            angle_error_deg: -rotation_state.remaining(),
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

    Timer::after(Duration::from_millis(IMU_WAIT_TIMEOUT_MS)).await;
    drain_latest_imu_since(started_at_ms)
}

// ── Correction phase helpers ──────────────────────────────────────────────

/// Compute track speeds for a single correction pulse using the torque ladder.
///
/// Base speed scales proportionally with error magnitude (~6 speed-units per
/// degree), clamped between [`types::ROTATION_CORRECTION_SPEED_MIN`] and
/// [`types::ROTATION_CORRECTION_SPEED_START`], then stepped down by
/// [`types::ROTATION_CORRECTION_SPEED_STEP`] for each prior iteration.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn compute_correction_speeds(error_deg: f32, iteration: u8, direction: types::RotationDirection) -> (i8, i8) {
    let proportional = (roundf(error_deg.abs() * 6.0) as u8).clamp(CORRECTION_SPEED_MIN, CORRECTION_SPEED_START);

    let correction_speed = proportional
        .saturating_sub(iteration.saturating_mul(CORRECTION_SPEED_STEP))
        .max(CORRECTION_SPEED_MIN);

    let effective_direction = if error_deg > 0.0 {
        direction
    } else {
        match direction {
            types::RotationDirection::Clockwise => types::RotationDirection::CounterClockwise,
            types::RotationDirection::CounterClockwise => types::RotationDirection::Clockwise,
        }
    };

    let speed = i8::try_from(correction_speed).unwrap_or(i8::MAX);
    match effective_direction {
        types::RotationDirection::Clockwise => (speed, -speed),
        types::RotationDirection::CounterClockwise => (-speed, speed),
    }
}

/// Check whether the remaining angle error is within tolerance.
///
/// Returns `Some(RotationStepResult::Completed)` if converged, `None` otherwise.
fn try_correction_convergence(
    state: &RotationState,
    measurement: &ImuMeasurement,
    iteration: u8,
    started_at_ms: u64,
) -> Option<RotationStepResult> {
    let error_deg = state.remaining();
    if error_deg.abs() <= TOLERANCE_DEG {
        let yaw = state.last_yaw.unwrap_or(measurement.orientation.yaw);
        defmt::info!(
            "correction converged iter={=u8} yaw={=f32}° err={=f32}°",
            iteration,
            yaw,
            error_deg,
        );
        Some(rotation_success(measurement, state, started_at_ms))
    } else {
        None
    }
}

/// Outcome of a single correction IMU polling cycle.
enum PollOutcome {
    /// Converged to the target within tolerance.
    Converged(RotationStepResult),
    /// Overshot the target; contains the updated overshoot count.
    Overshot(u8),
    /// Timed out without reaching the target.
    TimedOut,
}

/// Poll the IMU while correction motors run.
///
/// Stops on convergence (within [`types::ROTATION_TOLERANCE_DEG`]), overshoot
/// (error sign flips relative to `iter_sign`), or a per-iteration timeout
/// ([`types::ROTATION_CORRECTION_ITER_TIMEOUT_MS`]).
async fn poll_correction_imu(
    rotation_state: &mut RotationState,
    latest_measurement: &mut ImuMeasurement,
    started_at_ms: u64,
    iteration: u8,
    iter_sign: i8,
    overshoot_count: u8,
) -> PollOutcome {
    let start_ms = Instant::now().as_millis();

    loop {
        if Instant::now().as_millis() - start_ms > CORRECTION_ITER_TIMEOUT_MS {
            stop_rotation_motors().await;
            defmt::info!(
                "correction iter={=u8} timeout yaw={=f32}° err={=f32}°",
                iteration,
                rotation_state.last_yaw.unwrap_or(latest_measurement.orientation.yaw),
                rotation_state.remaining(),
            );
            return PollOutcome::TimedOut;
        }

        if let Some(m) = drain_latest_imu_since(started_at_ms) {
            rotation_state.update(&m);
            *latest_measurement = m;
            let new_error = rotation_state.remaining();

            if new_error.abs() <= TOLERANCE_DEG {
                stop_rotation_motors().await;
                return PollOutcome::Converged(rotation_success(latest_measurement, rotation_state, started_at_ms));
            }

            let new_sign: i8 = if new_error > 0.0 { 1 } else { -1 };
            if new_sign != iter_sign {
                stop_rotation_motors().await;
                let count = overshoot_count.saturating_add(1);
                defmt::info!(
                    "correction overshoot iter={=u8} yaw={=f32}° err={=f32}° total_overshoots={=u8}",
                    iteration,
                    rotation_state.last_yaw.unwrap_or(latest_measurement.orientation.yaw),
                    new_error,
                    count,
                );
                return PollOutcome::Overshot(count);
            }
        } else {
            Timer::after(Duration::from_millis(5)).await;
        }
    }
}

/// Post-stop multi-iteration correction loop for stationary turns.
///
/// Uses a torque-ladder strategy: motor speed is stepped down by
/// `ROTATION_CORRECTION_SPEED_STEP` each iteration, starting at
/// `ROTATION_CORRECTION_SPEED_START` and floored at
/// `ROTATION_CORRECTION_SPEED_MIN`. In each iteration the motors run
/// while the IMU is polled continuously until the yaw converges, an
/// overshoot is detected, or a timeout expires.
///
/// The high starting speed reliably breaks static friction; subsequent
/// iterations trade torque for precision as the remaining error shrinks.
///
/// Up to `ROTATION_CORRECTION_MAX_ITERATIONS` attempts are made before
/// the correction is declared exhausted.
async fn run_correction_phase(
    rotation_state: &mut RotationState,
    started_at_ms: u64,
    initial_measurement: ImuMeasurement,
) -> RotationStepResult {
    // Anchor state on the pre-stop measurement.
    rotation_state.update(&initial_measurement);

    // Wait for initial settle and capture a fresh IMU sample.
    let mut latest_measurement = wait_for_latest_imu_since(started_at_ms, CORRECTION_SETTLE_MS)
        .await
        .map_or(initial_measurement, |settled| {
            rotation_state.update(&settled);
            settled
        });

    let mut overshoot_count: u8 = 0;
    let mut last_error_sign: i8 = 0;
    let mut iteration: u8 = 0;

    while iteration < CORRECTION_MAX_ITERATIONS {
        // Check convergence at loop entry (from coast between iterations).
        if let Some(result) = try_correction_convergence(rotation_state, &latest_measurement, iteration, started_at_ms)
        {
            return result;
        }

        let error_deg = rotation_state.remaining();

        // Track overshoots between iterations.
        let current_sign: i8 = if error_deg > 0.0 { 1 } else { -1 };
        if last_error_sign != 0 && current_sign != last_error_sign {
            overshoot_count = overshoot_count.saturating_add(1);
        }
        last_error_sign = current_sign;

        // Compute and apply correction speeds.
        let (left_speed, right_speed) = compute_correction_speeds(error_deg, iteration, rotation_state.direction);

        defmt::info!(
            "correction iter={=u8} yaw={=f32}° err={=f32}° l={=i8} r={=i8}",
            iteration,
            rotation_state.last_yaw.unwrap_or(latest_measurement.orientation.yaw),
            error_deg,
            left_speed,
            right_speed,
        );

        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed,
            right_speed,
        })
        .await;
        motion::set_track_speeds(left_speed, right_speed).await;

        // Poll IMU while motors run.
        match poll_correction_imu(
            rotation_state,
            &mut latest_measurement,
            started_at_ms,
            iteration,
            current_sign,
            overshoot_count,
        )
        .await
        {
            PollOutcome::Converged(result) => return result,
            PollOutcome::Overshot(count) => overshoot_count = count,
            PollOutcome::TimedOut => {}
        }

        // After stopping: settle and re-check for coast landing in tolerance.
        if let Some(settled) = wait_for_latest_imu_since(started_at_ms, CORRECTION_SETTLE_MS).await {
            rotation_state.update(&settled);
            latest_measurement = settled;
        }

        if let Some(result) = try_correction_convergence(rotation_state, &latest_measurement, iteration, started_at_ms)
        {
            return result;
        }

        iteration = iteration.saturating_add(1);
    }

    // Exhausted all correction iterations without reaching tolerance.
    let error_deg = rotation_state.remaining();
    defmt::warn!(
        "correction exhausted after {=u8} iters, final_err={=f32}°",
        iteration,
        error_deg,
    );
    RotationStepResult::Failed {
        reason: "CorrectionExhausted",
        telemetry: types::CompletionTelemetry::RotateExact {
            final_yaw_deg: latest_measurement.orientation.yaw,
            angle_error_deg: -rotation_state.remaining(),
            duration_ms: Instant::now().as_millis() - started_at_ms,
        },
    }
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
    if now_ms - started_at_ms > TIMEOUT_MS {
        stop_rotation_motors().await;
        return rotation_failure(rotation_state, started_at_ms, now_ms, "RotateTimeout");
    }

    let Some(measurement) = read_rotation_measurement(started_at_ms).await else {
        return RotationStepResult::InProgress;
    };

    let is_first_sample = rotation_state.start_yaw.is_none();

    // Advance the state machine. If the target is reached, stop motors and report completion.
    let done = rotation_state.update(&measurement);

    // Log start yaw on the very first sample (after update so target_yaw is populated).
    if is_first_sample {
        defmt::info!(
            "rotate_exact start: yaw={=f32}° target_yaw={=f32}° target_deg={=f32}°",
            measurement.orientation.yaw,
            rotation_state.target_yaw.unwrap_or(0.0),
            rotation_state.target_angle,
        );
    }

    // Rate-limited debug logging (compiled out when feature is absent).
    #[cfg(feature = "telemetry_logs")]
    {
        if (measurement.timestamp_ms % 100) < 25 {
            defmt::info!(
                "rotate_exact: yaw={=f32}° target_yaw={=f32}° remaining={=f32}°",
                measurement.orientation.yaw,
                rotation_state.target_yaw.unwrap_or(0.0),
                rotation_state.remaining(),
            );
        }
    }
    if done {
        stop_rotation_motors().await;
        if matches!(rotation_state.motion, types::RotationMotion::Stationary { .. }) {
            return run_correction_phase(rotation_state, started_at_ms, measurement).await;
        }
        return rotation_success(&measurement, rotation_state, started_at_ms);
    }

    if rotation_state.correction_flips >= CORRECTION_MAX_FLIPS {
        stop_rotation_motors().await;
        return rotation_failure(rotation_state, started_at_ms, now_ms, "RotateCorrectionLimit");
    }

    if let Some(start_ms) = rotation_state.correction_started_at_ms
        && now_ms - start_ms > CORRECTION_TIMEOUT_MS
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
