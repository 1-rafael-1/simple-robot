//! Type definitions and constants for drive control

/// Drift compensation sample interval in milliseconds
pub const DRIFT_COMPENSATION_INTERVAL_MS: u64 = 200;

/// Speed difference tolerance (percent)
pub const DRIFT_TOLERANCE_PERCENT: f32 = 1.0;

/// Compensation adjustment gain (0.0-1.0)
pub const DRIFT_COMPENSATION_GAIN: f32 = 0.5;

/// Maximum compensation per adjustment (percent points of speed command)
pub const DRIFT_COMPENSATION_MAX: i8 = 5;

/// Combined motor control and sensor feedback commands
#[derive(Debug, Clone)]
pub enum DriveCommand {
    /// Movement and control commands
    Drive(DriveAction),
    /// Run motor calibration procedure
    RunMotorCalibration,
    /// Run IMU calibration procedure
    RunImuCalibration,
}

/// Motion control commands with associated parameters
#[derive(Debug, Clone, PartialEq)]
pub enum DriveAction {
    /// Set motor speeds directly using motor_driver convention
    ///
    /// Speed range: -100 (full backward) to +100 (full forward)
    /// - Positive values: Forward motion
    /// - Negative values: Backward motion
    /// - Zero: Coast (freewheel)
    ///
    /// This directly maps to motor_driver's SetTracks command.
    SetSpeed {
        /// Left track speed (-100 to +100)
        left: i8,
        /// Right track speed (-100 to +100)
        right: i8,
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

/// Rotation direction for precise turning
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RotationDirection {
    /// Clockwise rotation (right turn)
    Clockwise,
    /// Counter-clockwise rotation (left turn)
    CounterClockwise,
}

/// Combined motion options during rotation
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RotationMotion {
    /// Rotate in place at the given rotation speed (0-100)
    ///
    /// This is the "speed differential" input for turns-in-place:
    /// the controller will command left/right tracks as equal-and-opposite
    /// using this value as the magnitude.
    Stationary { speed: u8 },

    /// Rotate while moving at specified speed (-100 to +100)
    /// Positive = forward, Negative = backward
    WhileMoving(i8),
}

// Control parameters
/// Number of encoder pulses per motor shaft revolution
pub const PULSES_PER_REV: u32 = 8;
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
