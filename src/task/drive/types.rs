//! Type definitions and constants for drive control

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Receiver, Sender},
};

/// Drift compensation sample interval in milliseconds
pub const DRIFT_COMPENSATION_INTERVAL_MS: u64 = 200;

/// Speed difference tolerance (percent)
pub const DRIFT_TOLERANCE_PERCENT: f32 = 1.0;

/// Compensation adjustment gain (0.0-1.0)
pub const DRIFT_COMPENSATION_GAIN: f32 = 0.5;

/// Maximum compensation per adjustment (percent points of speed command)
pub const DRIFT_COMPENSATION_MAX: i8 = 5;

/// IMU calibration target selection.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum ImuCalibrationKind {
    /// Calibrate gyroscope (stationary).
    Gyro,
    /// Calibrate accelerometer (stationary).
    Accel,
    /// Calibrate magnetometer (manual rotation).
    Mag,
    /// Calibrate gyroscope, accelerometer, and magnetometer (full sequence).
    Full,
}

/// Combined motor control and sensor feedback commands
#[derive(Debug, Clone)]
pub enum DriveCommand {
    /// Movement and control commands
    Drive(DriveAction),
    /// Run motor calibration procedure
    RunMotorCalibration,
    /// Run IMU calibration procedure
    RunImuCalibration(ImuCalibrationKind),
}

/// Motion control commands with associated parameters
#[derive(Debug, Clone, PartialEq)]
pub enum DriveAction {
    /// Set motor speeds directly using `motor_driver` convention
    ///
    /// Speed range: -100 (full backward) to +100 (full forward)
    /// - Positive values: Forward motion
    /// - Negative values: Backward motion
    /// - Zero: Coast (freewheel)
    ///
    /// This directly maps to `motor_driver`'s `SetTracks` command.
    SetSpeed {
        /// Left track speed (-100 to +100)
        left: i8,
        /// Right track speed (-100 to +100)
        right: i8,
    },
    /// Drive a fixed distance using encoder feedback and drift compensation.
    DriveDistance {
        /// Straight or curved distance specification.
        kind: DriveDistanceKind,
        /// Drive direction (forward or backward).
        direction: DriveDirection,
        /// Target speed magnitude (0-100).
        speed: u8,
    },
    /// Active electrical braking
    Brake,
    /// Passive stop (freewheeling)
    Coast,
    /// Enter low-power standby mode
    Standby,
    /// Precise rotation with optional forward/backward motion
    RotateExact {
        /// Rotation angle in degrees
        degrees: f32,
        /// Rotation direction
        direction: RotationDirection,
        /// Combined motion type
        motion: RotationMotion,
    },
}

/// Drive direction for distance-based commands
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum DriveDirection {
    /// Forward motion
    Forward,
    /// Backward motion
    Backward,
}

/// Turn direction for curved motion.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum TurnDirection {
    /// Left turn (counter-clockwise yaw).
    Left,
    /// Right turn (clockwise yaw).
    Right,
}

/// Distance command specification
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DriveDistanceKind {
    /// Straight-line distance in centimeters.
    Straight {
        /// Target distance along the centerline (cm).
        distance_cm: f32,
    },
    /// Curved distance specified by centerline radius and arc length (centimeters).
    CurveArc {
        /// Radius from robot centerline (cm).
        radius_cm: f32,
        /// Desired arc length along the robot centerline (cm).
        arc_length_cm: f32,
        /// Turn direction (left/right).
        direction: TurnDirection,
    },
}

/// Interrupt commands that preempt active drive intents.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum InterruptKind {
    /// Immediately brake all motors.
    EmergencyBrake,
    /// Stop motors without braking.
    Stop,
    /// Cancel the active intent without issuing a stop/brake.
    CancelCurrent,
}

/// Completion status for long-running commands.
///
/// Used by per-command completion handles to report final outcomes.
#[derive(Debug, Clone, Eq, PartialEq)]
pub enum CompletionStatus {
    /// Command finished successfully.
    Success,
    /// Command was cancelled by an interrupt or newer epoch.
    Cancelled,
    /// Command failed with a reason string.
    Failed(&'static str),
}

/// Completion telemetry for long-running commands.
///
/// Used by per-command completion handles to return command-specific details.
#[derive(Debug, Clone, PartialEq)]
pub enum CompletionTelemetry {
    /// No telemetry payload.
    None,
    /// Telemetry for `RotateExact`.
    RotateExact {
        /// IMU yaw at completion (degrees).
        final_yaw_deg: f32,
        /// Signed error (degrees): achieved - target.
        angle_error_deg: f32,
        /// Duration in milliseconds.
        duration_ms: u64,
    },
    /// Telemetry for `DriveDistance`.
    DriveDistance {
        /// Achieved revolutions (left track sprocket).
        achieved_left_revs: f32,
        /// Achieved revolutions (right track sprocket).
        achieved_right_revs: f32,
        /// Target revolutions (left track sprocket).
        target_left_revs: f32,
        /// Target revolutions (right track sprocket).
        target_right_revs: f32,
        /// Duration in milliseconds.
        duration_ms: u64,
    },
}

/// Generic completion payload for drive commands.
#[derive(Debug, Clone, PartialEq)]
pub struct DriveCompletion {
    /// Overall status.
    pub status: CompletionStatus,
    /// Command-specific telemetry.
    pub telemetry: CompletionTelemetry,
}

/// One-shot completion sender type.
///
/// Obtain via a completion handle from the pool; do not construct channels manually.
/// Pass this to `send_drive_command_with_completion`.
pub type CompletionSender = Sender<'static, CriticalSectionRawMutex, DriveCompletion, 1>;

/// One-shot completion receiver type.
///
/// Managed by the completion handle pool; do not construct manually.
/// Callers should await via `wait_for_completion`.
pub type CompletionReceiver = Receiver<'static, CriticalSectionRawMutex, DriveCompletion, 1>;

/// Rotation direction for precise turning
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum RotationDirection {
    /// Clockwise rotation (right turn)
    Clockwise,
    /// Counter-clockwise rotation (left turn)
    CounterClockwise,
}

/// Combined motion options during rotation
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum RotationMotion {
    /// Rotate in place at the given rotation speed (0-100)
    ///
    /// This is the "speed differential" input for turns-in-place:
    /// the controller will command left/right tracks as equal-and-opposite
    /// using this value as the magnitude.
    Stationary {
        /// Rotation speed (0-100)
        speed: u8,
    },

    /// Rotate while moving at specified speed (-100 to +100)
    /// Positive = forward, Negative = backward
    WhileMoving(i8),
}

// Control parameters
/// Number of encoder pulses per motor shaft revolution
pub const PULSES_PER_REV: u32 = 8;
/// Gear ratio from motor shaft to track sprocket
pub const GEAR_RATIO_MOTOR_TO_SPROCKET: u32 = 120;
/// Number of encoder pulses per track sprocket revolution
pub const PULSES_PER_SPROCKET_REV: u32 = PULSES_PER_REV * GEAR_RATIO_MOTOR_TO_SPROCKET;
/// Number of encoder pulses per track sprocket revolution (float).
#[allow(clippy::cast_precision_loss)]
pub const PULSES_PER_SPROCKET_REV_F32: f32 = PULSES_PER_SPROCKET_REV as f32;
/// Distance between left/right track centerlines (cm).
pub const TRACK_WIDTH_CM: f32 = 14.5;
/// Effective drive sprocket circumference with track installed (cm).
pub const SPROCKET_CIRCUMFERENCE_CM: f32 = 19.01;
/// Distance drive ramp-down begins when remaining revolutions drop below this value.
pub const DISTANCE_RAMP_DOWN_START_REVS: f32 = 0.25;
/// Minimum speed during ramp-down (0-100).
pub const DISTANCE_MIN_SPEED: u8 = 20;
/// Maximum speed clamp for distance driving (0-100).
pub const DISTANCE_MAX_SPEED: u8 = 100;
/// Completion tolerance for distance driving (revolutions).
pub const DISTANCE_TOLERANCE_REVS: f32 = 0.01;
/// Distance drive control interval in milliseconds.
pub const DISTANCE_CONTROL_INTERVAL_MS: u64 = 20;
/// Encoder timeout during distance driving (milliseconds).
pub const DISTANCE_ENCODER_TIMEOUT_MS: u64 = 300;
/// Curve yaw correction proportional gain (radians -> ratio).
///
/// This scales the yaw error (expected vs IMU yaw delta, in radians) into a speed
/// ratio adjustment. Higher values correct curvature more aggressively but can
/// introduce oscillation. Start low and tune on hardware.
pub const DISTANCE_CURVE_YAW_KP: f32 = 0.5;
/// Maximum absolute curve yaw correction applied to speed ratio.
///
/// This clamps the ratio adjustment applied to left/right speeds to avoid
/// destabilizing the base curve profile. For example, `0.25` limits correction
/// to ±25% of the commanded ratio before ramping/clamping.
pub const DISTANCE_CURVE_YAW_MAX_CORRECTION: f32 = 0.25;
/// Maximum rotation speed (0-100%)
pub const ROTATION_SPEED_MAX: u8 = 50;
/// Minimum rotation speed to overcome friction
pub const ROTATION_SPEED_MIN: u8 = 20;
/// Acceptable angle error in degrees
///
/// Target: ~0.5° for higher-precision turns-in-place.
pub const ROTATION_TOLERANCE_DEG: f32 = 0.5;
/// Maximum speed differential during combined motion
pub const SPEED_DIFF_MAX: i8 = 30;

// Motor calibration constants
/// Calibration test speed for individual motors (100% forward)
pub const CALIBRATION_SPEED_INDIVIDUAL: i8 = 100;
/// Calibration test speed for track verification (60% forward)
pub const CALIBRATION_SPEED_TRACK: i8 = 60;
/// Calibration measurement duration in milliseconds
pub const CALIBRATION_SAMPLE_DURATION_MS: u64 = 10_000;
/// Coast time between calibration tests in milliseconds
pub const CALIBRATION_COAST_DURATION_MS: u64 = 500;
