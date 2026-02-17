//! Drive system coordination and intent execution.
//!
//! This module owns the drive task and its supporting components. It coordinates
//! sensor feedback, drive intents, and motor commands without consuming the global
//! event stream directly.
//!
//! # Architecture
//!
//! ```text
//! sensors::encoders     drive task               motor_driver
//! ├── Pulse counts  →   ├── Queue/interrupt   →   ├── PWM actuation
//! └── Feedback          ├── Intent lifecycle      └── Direction control
//!
//! sensors::imu      →   drive task               motor_driver
//! ├── Orientation       ├── Rotation control
//! └── Angles            └── Stabilization
//! ```
//!
//! # Control Flow Overview
//!
//! The drive task selects over two sources:
//!
//! - **Regular command queue**: Sequenced drive intents that may take time.
//! - **Interrupt signal**: Emergency brake/stop/cancel that preempts any intent.
//!
//! When an interrupt arrives, the task cancels the active intent, resolves its
//! completion (if any), increments an epoch, and discards queued commands that
//! were stamped before the interrupt.
//!
//! # Completion Flow (per-command)
//!
//! Completion handles are a fixed-size pool of one-shot completion channels.
//! A `CompletionHandle` is just an index into that pool (not a channel itself);
//! the handle stays with the caller while the `CompletionSender` is attached to
//! a command so the drive loop can resolve it.
//!
//! - **Acquire** a `CompletionHandle` from the pool (fixed-size; returns `Exhausted` immediately).
//! - **Create** a `CompletionSender` from the handle and attach it to the command.
//! - **Await** completion on the handle; the drive loop resolves it on success,
//!   failure, or cancellation (interrupts and epoch invalidation yield `Cancelled`).
//! - **Release** the handle back to the pool after you receive the result to avoid leaks.
//!
//! **Typical usage:** `acquire_completion_handle` → `completion_sender` →
//! `send_drive_command_with_completion` → `wait_for_completion` → `release_completion_handle`.
//!
//! # Data Flow
//!
//! - Commands are sent by orchestrator or other tasks.
//! - Sensor feedback is forwarded by orchestrator into dedicated channels.
//! - Motor commands are issued to `motor_driver`.
//!
//! This task does NOT consume the system event channel. Forwarding avoids
//! multiple tasks competing for events.
//!
//! # Speed Convention
//!
//! This module uses the same speed convention as `motor_driver`:
//! - **-100 to +100**: Full range of motor control
//! - **Positive values**: Forward motion
//! - **Negative values**: Backward motion
//! - **Zero**: Coast (freewheel)
//!
//! # Module Index
//!
//! - [`control`]: Intent execution and control-loop state machines.
//! - [`feedback`]: Sensor input channels and measurement forwarding.
//! - [`compensation`]: Drift compensation helpers.
//! - [`calibration`]: Motor and IMU calibration procedures.
//! - [`types`]: Command, telemetry, and completion types.

#![allow(clippy::too_many_arguments)]

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use micromath::F32Ext;

use crate::{
    system::{
        event::{Events, raise_event},
        state::SYSTEM_STATE,
    },
    task::{
        drive::feedback::IMU_FEEDBACK_CHANNEL,
        motor_driver::{self, MotorCommand},
        sensors::{
            encoders::{self as encoder_read, EncoderMeasurement},
            imu::{AhrsFusionMode, ImuMeasurement, set_ahrs_fusion_mode},
        },
    },
};

// Submodules
mod calibration;
mod compensation;
mod control;
pub mod feedback;
pub mod types;

// Re-export public API
// Internal imports
use calibration::{run_imu_calibration, run_motor_calibration};
use control::RotationState;
pub use feedback::{
    send_accel_measurement, send_gyro_measurement, send_mag_measurement, try_send_encoder_measurement,
    try_send_imu_measurement,
};
pub use types::{
    CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveCompletion, DriveDirection,
    DriveDistanceKind, ImuCalibrationKind, InterruptKind, TurnDirection,
};

/// Envelope for queued drive commands.
#[derive(Debug, Clone)]
struct DriveCommandEnvelope {
    /// Drive command to execute.
    command: DriveCommand,
    /// Optional completion sender for per-command completion reporting.
    completion: Option<types::CompletionSender>,
    /// Epoch stamped at enqueue time to cancel stale queued commands.
    /// Used to resolve completions as `Cancelled` when invalidated by interrupts.
    epoch: u32,
}

/// Size of the regular drive command queue.
const DRIVE_QUEUE_SIZE: usize = 16;

/// Regular command queue for drive intents.
static DRIVE_QUEUE: Channel<CriticalSectionRawMutex, DriveCommandEnvelope, DRIVE_QUEUE_SIZE> = Channel::new();

/// Interrupt signal for emergency actions.
static DRIVE_INTERRUPT: Signal<CriticalSectionRawMutex, InterruptKind> = Signal::new();

/// Epoch counter for invalidating stale queued commands after interrupts.
static CURRENT_EPOCH: AtomicU32 = AtomicU32::new(0);

/// Number of per-command completion channels available.
const COMPLETION_POOL_SIZE: usize = 16;
/// Single-slot channel used for per-command completion delivery.
type CompletionChannel = embassy_sync::channel::Channel<CriticalSectionRawMutex, DriveCompletion, 1>;

/// Pool of completion channels used by callers to await per-command results.
static COMPLETION_CHANNELS: [CompletionChannel; COMPLETION_POOL_SIZE] = [
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
    CompletionChannel::new(),
];

/// Queue of available completion channel indices.
static COMPLETION_POOL: Channel<CriticalSectionRawMutex, usize, COMPLETION_POOL_SIZE> = Channel::new();
/// One-time init guard for seeding the completion pool.
static COMPLETION_POOL_INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Handle used by callers to await per-command completion.
///
/// This is an index into a fixed pool; it is not a channel itself.
/// Keep the handle local, pass the associated `CompletionSender` with the command,
/// await completion, then release the handle back to the pool to avoid exhaustion.
#[derive(Debug, Clone, Copy)]
pub struct CompletionHandle {
    /// Index into the fixed completion channel pool.
    index: usize,
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
/// Errors returned when acquiring a completion handle.
pub enum CompletionPoolError {
    /// No completion handles are available in the pool.
    Exhausted,
}

/// Send a drive command for execution.
pub async fn send_drive_command(command: DriveCommand) {
    send_drive_command_internal(command, None).await;
}

/// Send a drive command with a per-command completion sender.
///
/// The sender is attached to the command envelope so the drive loop can resolve it.
/// Completion delivery is guaranteed because the drive loop awaits the send.
pub async fn send_drive_command_with_completion(command: DriveCommand, completion: types::CompletionSender) {
    send_drive_command_internal(command, Some(completion)).await;
}

/// Send an interrupt that preempts the active intent.
pub fn send_drive_interrupt(kind: InterruptKind) {
    DRIVE_INTERRUPT.signal(kind);
}

/// Enqueue a drive command with an optional completion sender.
async fn send_drive_command_internal(command: DriveCommand, completion: Option<types::CompletionSender>) {
    let epoch = CURRENT_EPOCH.load(Ordering::Relaxed);
    let envelope = DriveCommandEnvelope {
        command,
        completion,
        epoch,
    };
    DRIVE_QUEUE.sender().send(envelope).await;
}

/// Acquire a completion handle from the pool.
///
/// This does not wait: if the pool is empty, it returns `CompletionPoolError::Exhausted`.
/// The pool is fixed-size and initialized lazily on first use.
pub async fn acquire_completion_handle() -> Result<CompletionHandle, CompletionPoolError> {
    init_completion_pool_once();
    let index = COMPLETION_POOL
        .receiver()
        .try_receive()
        .map_err(|_| CompletionPoolError::Exhausted)?;
    Ok(CompletionHandle { index })
}

/// Release a completion handle back into the pool.
///
/// This drains any stale completion and returns the index to the pool. Call this
/// after awaiting completion to avoid leaking handles and exhausting the pool.
pub async fn release_completion_handle(handle: CompletionHandle) {
    let _ = COMPLETION_CHANNELS[handle.index].receiver().try_receive();
    COMPLETION_POOL.sender().send(handle.index).await;
}

/// Await completion for the provided handle.
///
/// This blocks until the drive loop sends a result (success, failure, or cancel).
/// Cancellation can come from an interrupt or epoch invalidation.
pub async fn wait_for_completion(handle: &CompletionHandle) -> DriveCompletion {
    COMPLETION_CHANNELS[handle.index].receiver().receive().await
}

/// Get the sender associated with a completion handle.
///
/// This is the only value that should be passed across task boundaries; the
/// `CompletionHandle` stays with the caller and is used to await completion.
pub fn completion_sender(handle: CompletionHandle) -> types::CompletionSender {
    COMPLETION_CHANNELS[handle.index].sender()
}

/// Initialize the completion pool once by seeding available indices.
fn init_completion_pool_once() {
    if COMPLETION_POOL_INITIALIZED
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_ok()
    {
        init_completion_pool();
    }
}

/// Seed the completion pool with all channel indices.
fn init_completion_pool() {
    for (index, _) in COMPLETION_CHANNELS.iter().enumerate() {
        let _ = COMPLETION_POOL.sender().try_send(index);
    }
}

/// Send completion payload if a sender is present.
///
/// Guaranteed delivery (awaits), fails only if the receiver was dropped.
async fn send_completion(completion: Option<types::CompletionSender>, payload: DriveCompletion) {
    if let Some(sender) = completion {
        sender.send(payload).await;
    }
}

/// Drift compensation state (encoder-based straight-line correction)
#[derive(Clone, Copy)]
struct DriftCompensationState {
    /// Whether drift compensation is currently active
    enabled: bool,
    /// Base speeds as commanded (before compensation)
    base_left: i8,
    /// Base speeds as commanded (before compensation)
    base_right: i8,
    /// Currently applied left speed (after compensation)
    adjusted_left: i8,
    /// Currently applied right speed (after compensation)
    adjusted_right: i8,
    /// Timestamp of the last applied compensation update (ms)
    last_applied_ms: u64,
    /// Timestamp of the last processed encoder measurement (ms)
    last_encoder_timestamp_ms: u64,
    /// Previous encoder measurement for computing per-sample deltas.
    /// Encoder hardware counters are cumulative since last reset, so we must
    /// subtract the previous reading to get pulses for the current window.
    last_encoder_measurement: Option<EncoderMeasurement>,
}

impl DriftCompensationState {
    /// Create a new `DriftCompensationState` with default values (disabled, zero speeds, no timestamps)
    const fn new() -> Self {
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

/// Rotation control tick interval
async fn drift_tick_ms() {
    // Keep this modest; encoder sampling is 200ms, so 20ms is plenty to pick up new samples
    // without busy-waiting.
    Timer::after(Duration::from_millis(20)).await;
}

/// Rotation control tick interval
async fn rotation_tick_ms() {
    // 50Hz rotation control loop tick to align with 50Hz IMU sampling.
    Timer::after(Duration::from_millis(20)).await;
}

/// Distance control tick interval
async fn distance_tick_ms() {
    Timer::after(Duration::from_millis(types::DISTANCE_CONTROL_INTERVAL_MS)).await;
}

/// Stall timeout during distance driving (milliseconds).
const DISTANCE_STALL_TIMEOUT_MS: u64 = 750;

/// Result of a rotation control step.
enum RotationStepResult {
    /// Rotation is still in progress; continue ticking.
    InProgress,
    /// Rotation completed successfully with final telemetry.
    Completed {
        /// Completion telemetry captured at success.
        telemetry: CompletionTelemetry,
    },
    /// Rotation failed with a static reason and telemetry snapshot.
    Failed {
        /// Failure reason identifier.
        reason: &'static str,
        /// Completion telemetry captured at failure.
        telemetry: CompletionTelemetry,
    },
}

/// Result of a distance control step.
enum DistanceStepResult {
    /// Distance drive is still in progress.
    InProgress,
    /// Distance drive completed successfully with final telemetry.
    Completed {
        /// Completion telemetry captured at success.
        telemetry: CompletionTelemetry,
    },
    /// Distance drive failed with a static reason and telemetry snapshot.
    Failed {
        /// Failure reason identifier.
        reason: &'static str,
        /// Completion telemetry captured at failure.
        telemetry: CompletionTelemetry,
    },
}

/// Distance drive control state.
struct DistanceDriveState {
    /// Straight or curved distance specification.
    kind: types::DriveDistanceKind,
    /// Drive direction (forward or backward).
    direction: types::DriveDirection,
    /// Base speed magnitude (0-100).
    base_speed: u8,
    /// Target revolutions for the left track sprocket.
    target_left_revs: f32,
    /// Target revolutions for the right track sprocket.
    target_right_revs: f32,
    /// Whether the left track is the inner (shorter) side for curves.
    inner_left: Option<bool>,
    /// Target revolutions for the inner track sprocket.
    target_inner_revs: f32,
    /// Left speed scale relative to the max target (1.0 for the dominant track).
    left_ratio: f32,
    /// Right speed scale relative to the max target (1.0 for the dominant track).
    right_ratio: f32,
    /// Last commanded left speed (after scaling/ramp).
    last_left_speed: i8,
    /// Last commanded right speed (after scaling/ramp).
    last_right_speed: i8,
    /// Accumulated left revolutions.
    accumulated_left_revs: f32,
    /// Accumulated right revolutions.
    accumulated_right_revs: f32,
    /// Last IMU yaw sample for curve control (degrees).
    curve_last_yaw_deg: Option<f32>,
    /// Accumulated curve yaw delta (degrees).
    curve_accumulated_yaw_deg: f32,
    /// Last time we observed forward progress (ms).
    last_progress_ms: u64,
    /// Consecutive samples with zero progress.
    zero_progress_samples: u32,
    /// Timestamp of the last processed encoder measurement (ms).
    last_encoder_timestamp_ms: u64,
    /// Last time we saw a new encoder measurement (ms).
    last_encoder_seen_ms: u64,
    /// Previous encoder measurement for computing deltas.
    last_encoder_measurement: Option<EncoderMeasurement>,
    /// Start time (ms) used for duration telemetry.
    started_at_ms: u64,
}

impl DistanceDriveState {
    /// Create a new distance drive state and precompute targets/ratios.
    fn new(kind: types::DriveDistanceKind, direction: types::DriveDirection, base_speed: u8) -> Self {
        let (target_left_revs, target_right_revs, inner_left, target_inner_revs) = match kind {
            types::DriveDistanceKind::Straight { revolutions } => (revolutions, revolutions, None, revolutions),
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
                let left_revs = left_arc_cm / types::SPROCKET_CIRCUMFERENCE_CM;
                let right_revs = right_arc_cm / types::SPROCKET_CIRCUMFERENCE_CM;
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
            last_progress_ms: now_ms,
            zero_progress_samples: 0,
            last_encoder_timestamp_ms: 0,
            last_encoder_seen_ms: now_ms,
            last_encoder_measurement: None,
            started_at_ms: now_ms,
        }
    }
}

/// Run a single step of the rotation control loop.
async fn run_rotation_control_step(rotation_state: &mut RotationState, started_at_ms: u64) -> RotationStepResult {
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
                telemetry: CompletionTelemetry::RotateExact {
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
            telemetry: CompletionTelemetry::RotateExact {
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

/// Run a single step of the distance control loop.
#[allow(
    clippy::too_many_lines,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap
)]
async fn run_distance_control_step(state: &mut DistanceDriveState) -> DistanceStepResult {
    let now_ms = Instant::now().as_millis();
    if now_ms.saturating_sub(state.last_encoder_seen_ms) >= types::DISTANCE_ENCODER_TIMEOUT_MS {
        let telemetry = CompletionTelemetry::DriveDistance {
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

    let data = crate::task::drive::compensation::calculate_track_averages(delta_measurement);
    if data.all_zero() {
        state.zero_progress_samples = state.zero_progress_samples.saturating_add(1);
        if now_ms.saturating_sub(state.last_progress_ms) >= DISTANCE_STALL_TIMEOUT_MS {
            let telemetry = CompletionTelemetry::DriveDistance {
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
        let telemetry = CompletionTelemetry::DriveDistance {
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

    if matches!(state.kind, types::DriveDistanceKind::CurveArc { .. }) {
        let mut latest_imu: Option<ImuMeasurement> = None;
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
    let telemetry = CompletionTelemetry::DriveDistance {
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

        let mut sys = SYSTEM_STATE.lock().await;
        sys.left_track_speed = 0;
        sys.right_track_speed = 0;
        drop(sys);

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

    let mut adjusted_left = left_speed;
    let mut adjusted_right = right_speed;
    if matches!(state.kind, types::DriveDistanceKind::Straight { .. }) {
        distance_apply_compensation(left_speed, right_speed, &mut adjusted_left, &mut adjusted_right, data);
    }

    state.last_left_speed = adjusted_left;
    state.last_right_speed = adjusted_right;

    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: adjusted_left,
        right_speed: adjusted_right,
    })
    .await;

    let mut sys = SYSTEM_STATE.lock().await;
    sys.left_track_speed = adjusted_left;
    sys.right_track_speed = adjusted_right;

    DistanceStepResult::InProgress
}

/// Stop both tracks and update system state (distance helpers can share this).
async fn distance_stop_motors() {
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;

    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: 0,
        right_speed: 0,
    })
    .await;

    let mut sys = SYSTEM_STATE.lock().await;
    sys.left_track_speed = 0;
    sys.right_track_speed = 0;
}

/// Apply drift compensation to scaled left/right targets for distance driving.
fn distance_apply_compensation(
    base_left: i8,
    base_right: i8,
    adjusted_left: &mut i8,
    adjusted_right: &mut i8,
    data: crate::task::drive::compensation::TrackSpeedData,
) {
    if data.all_zero() || data.has_single_motor_zero_anomaly() {
        return;
    }

    let diff_percent = crate::task::drive::compensation::calculate_speed_difference(&data);
    let action =
        crate::task::drive::compensation::determine_compensation(diff_percent, *adjusted_left, *adjusted_right);

    let (new_left, new_right) =
        crate::task::drive::compensation::apply_compensation_action(action, *adjusted_left, *adjusted_right);

    if new_left != *adjusted_left || new_right != *adjusted_right {
        *adjusted_left = new_left.clamp(-100, 100);
        *adjusted_right = new_right.clamp(-100, 100);
    } else {
        // Keep base values if no adjustment is needed.
        *adjusted_left = base_left;
        *adjusted_right = base_right;
    }
}

/// Run a single step of the drift compensation loop
async fn run_drift_compensation_step(drift: &mut DriftCompensationState, rotation_state: Option<&RotationState>) {
    if !drift.enabled || rotation_state.is_some() {
        return;
    }

    if let Some(measurement) = feedback::get_latest_encoder_measurement().await {
        // Only process each encoder sample once
        if measurement.timestamp_ms == 0 || measurement.timestamp_ms == drift.last_encoder_timestamp_ms {
            return;
        }
        drift.last_encoder_timestamp_ms = measurement.timestamp_ms;

        // Compute per-sample deltas from cumulative hardware counters.
        // The encoder task publishes cumulative counts since last reset, so we
        // subtract the previous reading (with wraparound handling) to get pulses
        // for the current sampling window only.
        // First sample after start/reset: treat cumulative as delta (counters
        // were just reset, so this is correct for the first window).
        let delta_measurement = drift
            .last_encoder_measurement
            .map_or(measurement, |prev| EncoderMeasurement {
                left_front: compensation::calculate_delta_u16(measurement.left_front, prev.left_front),
                left_rear: compensation::calculate_delta_u16(measurement.left_rear, prev.left_rear),
                right_front: compensation::calculate_delta_u16(measurement.right_front, prev.right_front),
                right_rear: compensation::calculate_delta_u16(measurement.right_rear, prev.right_rear),
                timestamp_ms: measurement.timestamp_ms,
            });
        drift.last_encoder_measurement = Some(measurement);

        let data = crate::task::drive::compensation::calculate_track_averages(delta_measurement);

        // If nothing moved (very low speed or stopped), ignore the sample.
        if data.all_zero() {
            return;
        }

        // If we detect an anomaly (e.g., one encoder stuck at zero), disable compensation.
        if data.has_single_motor_zero_anomaly() {
            drift.enabled = false;
            encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
            return;
        }

        let diff_percent = crate::task::drive::compensation::calculate_speed_difference(&data);

        let action = crate::task::drive::compensation::determine_compensation(
            diff_percent,
            drift.adjusted_left,
            drift.adjusted_right,
        );

        let (new_left, new_right) = crate::task::drive::compensation::apply_compensation_action(
            action,
            drift.adjusted_left,
            drift.adjusted_right,
        );

        // Only send motor update if anything actually changes.
        if new_left != drift.adjusted_left || new_right != drift.adjusted_right {
            let prev_left = drift.adjusted_left;
            let prev_right = drift.adjusted_right;

            drift.adjusted_left = new_left;
            drift.adjusted_right = new_right;
            drift.last_applied_ms = data.timestamp_ms;

            info!(
                "drift_comp: diff={}% action={:?} speeds L:{}->{} R:{}->{} (ctrs lf:{} lr:{} rf:{} rr:{})",
                diff_percent,
                action,
                prev_left,
                drift.adjusted_left,
                prev_right,
                drift.adjusted_right,
                data.left_front,
                data.left_rear,
                data.right_front,
                data.right_rear
            );

            motor_driver::send_motor_command(MotorCommand::SetTracks {
                left_speed: drift.adjusted_left,
                right_speed: drift.adjusted_right,
            })
            .await;

            let mut state = SYSTEM_STATE.lock().await;
            state.left_track_speed = drift.adjusted_left;
            state.right_track_speed = drift.adjusted_right;
        }
    }
}

/// Active intent being executed by the drive task.
enum ActiveIntent {
    /// Active in-place rotation intent with state and optional completion sender.
    RotateExact {
        /// Rotation controller state for the in-progress turn.
        state: RotationState,
        /// Optional completion sender for the issued command.
        completion: Option<types::CompletionSender>,
        /// Start time (ms) used for duration telemetry.
        started_at_ms: u64,
    },
    /// Active distance drive intent with state and optional completion sender.
    DriveDistance {
        /// Distance controller state for the in-progress drive.
        state: DistanceDriveState,
        /// Optional completion sender for the issued command.
        completion: Option<types::CompletionSender>,
    },
}

/// State for the main drive control loop
struct DriveLoop {
    /// Whether the system is currently in standby mode (motors disabled)
    standby_enabled: bool,
    /// Active intent, if any
    active_intent: Option<ActiveIntent>,
    /// State for encoder-based drift compensation (straight-line correction)
    drift: DriftCompensationState,
}

impl DriveLoop {
    /// Create a new `DriveLoop` with default state (standby enabled, no active intent, drift compensation disabled)
    const fn new() -> Self {
        Self {
            standby_enabled: true,
            active_intent: None,
            drift: DriftCompensationState::new(),
        }
    }

    /// Handle a dequeued command envelope, honoring epoch cancellation.
    async fn handle_envelope(&mut self, envelope: DriveCommandEnvelope) {
        let current_epoch = CURRENT_EPOCH.load(Ordering::Relaxed);
        if envelope.epoch != current_epoch {
            send_completion(
                envelope.completion,
                DriveCompletion {
                    status: CompletionStatus::Cancelled,
                    telemetry: CompletionTelemetry::None,
                },
            )
            .await;
            return;
        }

        let completion = envelope.completion;
        match envelope.command {
            DriveCommand::Drive(action) => {
                self.handle_drive_action(action, completion).await;
            }
            DriveCommand::RunMotorCalibration => {
                info!("Starting motor calibration procedure");
                run_motor_calibration().await;
                info!("Motor calibration procedure completed");
                send_completion(
                    completion,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: CompletionTelemetry::None,
                    },
                )
                .await;
            }
            DriveCommand::RunImuCalibration(kind) => {
                info!("Starting IMU calibration procedure");
                run_imu_calibration(kind).await;
                info!("IMU calibration procedure completed");
                send_completion(
                    completion,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: CompletionTelemetry::None,
                    },
                )
                .await;
            }
        }
    }

    /// Handle a `DriveAction` command by executing the corresponding motor commands and state updates.
    async fn handle_drive_action(&mut self, action: DriveAction, completion: Option<types::CompletionSender>) {
        // Wake from standby if movement requested
        if self.standby_enabled {
            match &action {
                DriveAction::SetSpeed { .. } | DriveAction::RotateExact { .. } | DriveAction::DriveDistance { .. } => {
                    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true }).await;
                    self.standby_enabled = false;
                    Timer::after(Duration::from_millis(100)).await;
                }
                _ => {}
            }
        }

        match action {
            DriveAction::SetSpeed { left, right } => {
                self.handle_set_speed(left, right).await;
                send_completion(
                    completion,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: CompletionTelemetry::None,
                    },
                )
                .await;
            }
            DriveAction::RotateExact {
                degrees,
                direction,
                motion,
            } => {
                self.handle_rotate_exact(degrees, direction, motion, completion).await;
            }
            DriveAction::DriveDistance { kind, direction, speed } => {
                self.handle_drive_distance(kind, direction, speed, completion).await;
            }
            DriveAction::Coast => {
                self.handle_coast().await;
                send_completion(
                    completion,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: CompletionTelemetry::None,
                    },
                )
                .await;
            }
            DriveAction::Brake => {
                self.handle_brake().await;
                send_completion(
                    completion,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: CompletionTelemetry::None,
                    },
                )
                .await;
            }
            DriveAction::Standby => {
                self.handle_standby().await;
                send_completion(
                    completion,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: CompletionTelemetry::None,
                    },
                )
                .await;
            }
        }
    }

    /// Handle a `SetSpeed` command by applying drift compensation logic and sending motor commands.
    async fn handle_set_speed(&mut self, left: i8, right: i8) {
        // Clamp speeds to valid range
        let left_clamped = left.clamp(-100, 100);
        let right_clamped = right.clamp(-100, 100);

        // Detect straight-line driving (speeds equal or very close)
        // Enable drift compensation only in that mode.
        if (left_clamped - right_clamped).abs() <= 2 && left_clamped != 0 {
            self.drift.enabled = true;
            self.drift.base_left = left_clamped;
            self.drift.base_right = right_clamped;
            self.drift.adjusted_left = left_clamped;
            self.drift.adjusted_right = right_clamped;

            // Start encoder sampling at configured interval and reset counters.
            // Delta computation happens in run_drift_compensation_step using
            // stored previous readings (last_encoder_measurement).
            self.drift.last_encoder_measurement = None;
            encoder_read::send_command(encoder_read::EncoderCommand::Start {
                interval_ms: crate::task::drive::types::DRIFT_COMPENSATION_INTERVAL_MS,
            })
            .await;
            encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
        } else {
            // Turning/differential or stop: disable compensation and stop encoder sampling
            self.drift.enabled = false;
            encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
        }

        // Apply command (initially base, then adjusted as feedback arrives)
        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed: self.drift.adjusted_left,
            right_speed: self.drift.adjusted_right,
        })
        .await;

        // Update system state directly
        let mut state = SYSTEM_STATE.lock().await;
        state.left_track_speed = self.drift.adjusted_left;
        state.right_track_speed = self.drift.adjusted_right;
    }

    /// Handle a `RotateExact` command by initializing rotation state and starting IMU-based control.
    async fn handle_rotate_exact(
        &mut self,
        degrees: f32,
        direction: types::RotationDirection,
        motion: types::RotationMotion,
        completion: Option<types::CompletionSender>,
    ) {
        // Request IMU start for closed-loop rotation control.
        // The orchestrator handles this event and starts IMU streaming.
        raise_event(Events::StartStopMotionDataCollection(true)).await;

        let rotation_state = RotationState::new(degrees, direction, motion);
        let started_at_ms = Instant::now().as_millis();

        // Apply initial motor speeds for rotation
        let (left_speed, right_speed) = rotation_state.calculate_motor_speeds();
        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed,
            right_speed,
        })
        .await;

        // Update system state
        let mut state = SYSTEM_STATE.lock().await;
        state.left_track_speed = left_speed;
        state.right_track_speed = right_speed;
        drop(state);

        self.active_intent = Some(ActiveIntent::RotateExact {
            state: rotation_state,
            completion,
            started_at_ms,
        });
    }

    /// Handle a `DriveDistance` command by initializing encoder-based control.
    #[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation)]
    async fn handle_drive_distance(
        &mut self,
        kind: types::DriveDistanceKind,
        direction: types::DriveDirection,
        speed: u8,
        completion: Option<types::CompletionSender>,
    ) {
        // Disable drift loop; distance control applies compensation internally.
        self.drift.enabled = false;

        let (target_left_revs, target_right_revs, target_inner_revs) = match kind {
            types::DriveDistanceKind::Straight { revolutions } => (revolutions, revolutions, revolutions),
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
                let left_revs = left_arc_cm / types::SPROCKET_CIRCUMFERENCE_CM;
                let right_revs = right_arc_cm / types::SPROCKET_CIRCUMFERENCE_CM;
                let inner_revs = if inner_left { left_revs } else { right_revs };
                (left_revs, right_revs, inner_revs)
            }
        };

        if target_inner_revs <= types::DISTANCE_TOLERANCE_REVS {
            send_completion(
                completion,
                DriveCompletion {
                    status: CompletionStatus::Success,
                    telemetry: CompletionTelemetry::DriveDistance {
                        achieved_left_revs: 0.0,
                        achieved_right_revs: 0.0,
                        target_left_revs,
                        target_right_revs,
                        duration_ms: 0,
                    },
                },
            )
            .await;
            return;
        }

        if matches!(kind, types::DriveDistanceKind::CurveArc { .. }) {
            set_ahrs_fusion_mode(AhrsFusionMode::Axis9);
            raise_event(Events::StartStopMotionDataCollection(true)).await;
        }

        feedback::clear_encoder_measurement().await;
        encoder_read::send_command(encoder_read::EncoderCommand::Start {
            interval_ms: types::DISTANCE_CONTROL_INTERVAL_MS,
        })
        .await;
        encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;

        let mut state = DistanceDriveState::new(kind, direction, speed);

        let base_speed = speed.min(types::DISTANCE_MAX_SPEED);
        let signed_base = match direction {
            types::DriveDirection::Forward => base_speed as i8,
            types::DriveDirection::Backward => -(base_speed as i8),
        };

        let left_speed = (f32::from(signed_base) * state.left_ratio).round() as i8;
        let right_speed = (f32::from(signed_base) * state.right_ratio).round() as i8;

        state.last_left_speed = left_speed;
        state.last_right_speed = right_speed;

        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed,
            right_speed,
        })
        .await;

        let mut sys = SYSTEM_STATE.lock().await;
        sys.left_track_speed = left_speed;
        sys.right_track_speed = right_speed;
        drop(sys);

        self.active_intent = Some(ActiveIntent::DriveDistance { state, completion });
    }

    /// Handle a `Coast` command by disabling compensation, stopping encoder sampling, and sending coast command.
    async fn handle_coast(&mut self) {
        info!("coast");

        // Disable compensation on stop/coast and stop encoder sampling
        self.drift.enabled = false;
        encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;

        motor_driver::send_motor_command(MotorCommand::CoastAll).await;

        // Update system state
        let mut state = SYSTEM_STATE.lock().await;
        state.left_track_speed = 0;
        state.right_track_speed = 0;
    }

    /// Handle a `Brake` command by disabling compensation, stopping encoder sampling, and sending brake command.
    async fn handle_brake(&mut self) {
        info!("brake");

        // Disable compensation on brake and stop encoder sampling
        self.drift.enabled = false;
        encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;

        motor_driver::send_motor_command(MotorCommand::BrakeAll).await;

        // Update system state
        let mut state = SYSTEM_STATE.lock().await;
        state.left_track_speed = 0;
        state.right_track_speed = 0;
    }

    /// Handle a `Standby` command by disabling compensation, stopping encoder sampling, and disabling motors.
    async fn handle_standby(&mut self) {
        // Disable compensation in standby and stop encoder sampling
        self.drift.enabled = false;
        encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;

        if !self.standby_enabled {
            motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            Timer::after(Duration::from_millis(100)).await;
            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(100)).await;
            motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
            self.standby_enabled = true;

            // Update system state
            let mut state = SYSTEM_STATE.lock().await;
            state.left_track_speed = 0;
            state.right_track_speed = 0;
        }
    }

    /// Handle an interrupt by applying motor action, cancelling the active intent only,
    /// and bumping the epoch to invalidate queued commands.
    /// Only the active intent’s completion is resolved here; queued commands are cancelled
    /// when dequeued due to epoch mismatch.
    async fn handle_interrupt(&mut self, kind: InterruptKind) {
        match kind {
            InterruptKind::EmergencyBrake => {
                motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            }
            InterruptKind::Stop | InterruptKind::CancelCurrent => {
                motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            }
        }

        // Disable compensation and stop encoder sampling
        self.drift.enabled = false;
        encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;

        // Cancel active intent
        if let Some(intent) = self.active_intent.take() {
            match intent {
                ActiveIntent::RotateExact {
                    state,
                    completion,
                    started_at_ms,
                } => {
                    let accumulated = state.accumulated_angle.abs();
                    let target = state.target_angle.abs();
                    let last_yaw_deg = state.last_yaw.unwrap_or(0.0);
                    let duration_ms = Instant::now().as_millis() - started_at_ms;

                    raise_event(Events::StartStopMotionDataCollection(false)).await;
                    send_completion(
                        completion,
                        DriveCompletion {
                            status: CompletionStatus::Cancelled,
                            telemetry: CompletionTelemetry::RotateExact {
                                final_yaw_deg: last_yaw_deg,
                                angle_error_deg: accumulated - target,
                                duration_ms,
                            },
                        },
                    )
                    .await;
                }
                ActiveIntent::DriveDistance { state, completion } => {
                    let duration_ms = Instant::now().as_millis() - state.started_at_ms;

                    if matches!(state.kind, types::DriveDistanceKind::CurveArc { .. }) {
                        raise_event(Events::StartStopMotionDataCollection(false)).await;
                    }

                    send_completion(
                        completion,
                        DriveCompletion {
                            status: CompletionStatus::Cancelled,
                            telemetry: CompletionTelemetry::DriveDistance {
                                achieved_left_revs: state.accumulated_left_revs,
                                achieved_right_revs: state.accumulated_right_revs,
                                target_left_revs: state.target_left_revs,
                                target_right_revs: state.target_right_revs,
                                duration_ms,
                            },
                        },
                    )
                    .await;
                }
            }
        }

        // Invalidate queued commands
        CURRENT_EPOCH.fetch_add(1, Ordering::Relaxed);

        let mut state = SYSTEM_STATE.lock().await;
        state.left_track_speed = 0;
        state.right_track_speed = 0;
    }
}

/// Drive control task - coordinates motion and sensor feedback
///
/// # Architecture
///
/// This is a high-level control task that:
/// - Receives drive commands via queue
/// - Receives encoder feedback via channel (from orchestrator)
/// - Receives IMU feedback via command (from orchestrator)
/// - Receives interrupts via signal
/// - Sends motor commands to `motor_driver` task
/// - Coordinates calibration procedures
///
/// # Sensor Data Flow
///
/// Sensor tasks → Events → Orchestrator → Drive task (this) → Motor driver
///
/// The orchestrator forwards relevant sensor events to this task via dedicated
/// channels rather than having this task consume system events directly.
#[embassy_executor::task]
pub async fn drive() {
    // Initialize per-task state and the completion channel pool once.
    let mut loop_state = DriveLoop::new();
    init_completion_pool_once();

    loop {
        // Step 1: If an intent is active, poll it with higher priority than new commands.
        if let Some(outcome) = poll_active_intent(&mut loop_state).await {
            match outcome {
                ActiveIntentOutcome::Interrupt(kind) => {
                    loop_state.handle_interrupt(kind).await;
                }
                ActiveIntentOutcome::RotationStep(result) => {
                    apply_rotation_result(&mut loop_state, result).await;
                }
                ActiveIntentOutcome::DistanceStep(result) => {
                    apply_distance_result(&mut loop_state, result).await;
                }
            }
            continue;
        }

        // Step 2: No active intent — wait for work (with optional drift ticks).
        if loop_state.drift.enabled {
            step_idle_with_drift(&mut loop_state).await;
        } else {
            step_idle_without_drift(&mut loop_state).await;
        }
    }
}

/// Outcome of polling an active intent.
enum ActiveIntentOutcome {
    /// An interrupt arrived while an intent was active.
    Interrupt(InterruptKind),
    /// A control step completed for the active rotation intent.
    RotationStep(RotationStepResult),
    /// A control step completed for the active distance intent.
    DistanceStep(DistanceStepResult),
}

/// Poll the active intent (if any) and return the next action to perform.
async fn poll_active_intent(loop_state: &mut DriveLoop) -> Option<ActiveIntentOutcome> {
    match loop_state.active_intent.as_mut() {
        Some(ActiveIntent::RotateExact {
            state, started_at_ms, ..
        }) => {
            let interrupt_or_tick = select(DRIVE_INTERRUPT.wait(), rotation_tick_ms()).await;
            match interrupt_or_tick {
                Either::First(kind) => Some(ActiveIntentOutcome::Interrupt(kind)),
                Either::Second(()) => {
                    let result = run_rotation_control_step(state, *started_at_ms).await;
                    Some(ActiveIntentOutcome::RotationStep(result))
                }
            }
        }
        Some(ActiveIntent::DriveDistance { state, .. }) => {
            let interrupt_or_tick = select(DRIVE_INTERRUPT.wait(), distance_tick_ms()).await;
            match interrupt_or_tick {
                Either::First(kind) => Some(ActiveIntentOutcome::Interrupt(kind)),
                Either::Second(()) => {
                    let result = run_distance_control_step(state).await;
                    Some(ActiveIntentOutcome::DistanceStep(result))
                }
            }
        }
        None => None,
    }
}

/// Apply rotation step results to the active intent (completions + intent clearing).
async fn apply_rotation_result(loop_state: &mut DriveLoop, result: RotationStepResult) {
    let (status, telemetry) = match result {
        RotationStepResult::InProgress => return,
        RotationStepResult::Completed { telemetry } => (CompletionStatus::Success, telemetry),
        RotationStepResult::Failed { reason, telemetry } => (CompletionStatus::Failed(reason), telemetry),
    };

    let completion = match loop_state.active_intent.as_mut() {
        Some(ActiveIntent::RotateExact { completion, .. }) => completion.take(),
        _ => None,
    };

    raise_event(Events::StartStopMotionDataCollection(false)).await;
    send_completion(completion, DriveCompletion { status, telemetry }).await;

    loop_state.active_intent = None;
}

/// Apply distance step results to the active intent (completions + intent clearing).
async fn apply_distance_result(loop_state: &mut DriveLoop, result: DistanceStepResult) {
    let (status, telemetry) = match result {
        DistanceStepResult::InProgress => return,
        DistanceStepResult::Completed { telemetry } => (CompletionStatus::Success, telemetry),
        DistanceStepResult::Failed { reason, telemetry } => (CompletionStatus::Failed(reason), telemetry),
    };

    let stop_motion_data = match loop_state.active_intent.as_ref() {
        Some(ActiveIntent::DriveDistance { state, .. }) => {
            matches!(state.kind, types::DriveDistanceKind::CurveArc { .. })
        }
        _ => false,
    };

    let completion = match loop_state.active_intent.as_mut() {
        Some(ActiveIntent::DriveDistance { completion, .. }) => completion.take(),
        _ => None,
    };

    distance_stop_motors().await;
    if stop_motion_data {
        raise_event(Events::StartStopMotionDataCollection(false)).await;
    }
    send_completion(completion, DriveCompletion { status, telemetry }).await;

    loop_state.active_intent = None;
}

/// Idle loop step when drift compensation is enabled.
async fn step_idle_with_drift(loop_state: &mut DriveLoop) {
    let event_or_tick = select(
        select(DRIVE_QUEUE.receiver().receive(), DRIVE_INTERRUPT.wait()),
        drift_tick_ms(),
    )
    .await;

    match event_or_tick {
        Either::First(inner) => match inner {
            Either::First(envelope) => loop_state.handle_envelope(envelope).await,
            Either::Second(kind) => loop_state.handle_interrupt(kind).await,
        },
        Either::Second(()) => {
            run_drift_compensation_step(&mut loop_state.drift, None).await;
        }
    }
}

/// Idle loop step when drift compensation is disabled.
async fn step_idle_without_drift(loop_state: &mut DriveLoop) {
    let command_or_interrupt = select(DRIVE_QUEUE.receiver().receive(), DRIVE_INTERRUPT.wait()).await;
    match command_or_interrupt {
        Either::First(envelope) => loop_state.handle_envelope(envelope).await,
        Either::Second(kind) => loop_state.handle_interrupt(kind).await,
    }
}
