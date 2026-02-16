//! Rotation control for precise turning maneuvers
//!
//! This module only computes rotation state and motor speeds.
//! Completion is resolved by the drive loop when a rotation step finishes.

use crate::task::{
    drive::types::{
        ROTATION_SPEED_MAX, ROTATION_SPEED_MIN, ROTATION_TOLERANCE_DEG, RotationDirection, RotationMotion,
        SPEED_DIFF_MAX,
    },
    sensors::imu::ImuMeasurement,
};

/// State tracking for precise rotation maneuvers
///
/// Maintains rotation progress and calculates differential motor speeds
/// needed to achieve the target rotation angle using IMU feedback.
pub struct RotationState {
    /// Target rotation angle in degrees
    pub(crate) target_angle: f32,
    /// Accumulated rotation so far (in degrees)
    pub(crate) accumulated_angle: f32,
    /// Last measured yaw angle (in degrees)
    pub(crate) last_yaw: Option<f32>,
    /// Last update timestamp
    last_update_ms: u64,
    /// Direction of rotation
    direction: RotationDirection,
    /// Type of motion during rotation
    motion: RotationMotion,
    /// Base speed before rotation adjustment
    base_speed: i8,
}

impl RotationState {
    /// Creates new rotation tracking state
    pub const fn new(target_angle: f32, direction: RotationDirection, motion: RotationMotion) -> Self {
        let base_speed = match motion {
            RotationMotion::Stationary { speed: _ } => 0,
            RotationMotion::WhileMoving(speed) => speed,
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

    /// Updates rotation state with new IMU measurement
    /// Returns true if target angle has been reached
    pub fn update(&mut self, measurement: &ImuMeasurement) -> bool {
        // Calculate yaw change since last reading
        if let Some(last_yaw) = self.last_yaw {
            let mut yaw_change = measurement.orientation.yaw - last_yaw;

            // Handle wraparound at ±180 degrees
            if yaw_change > 180.0 {
                yaw_change -= 360.0;
            } else if yaw_change < -180.0 {
                yaw_change += 360.0;
            }

            // Accumulate based on desired direction
            self.accumulated_angle += match self.direction {
                RotationDirection::Clockwise => -yaw_change,
                RotationDirection::CounterClockwise => yaw_change,
            };
        }

        self.last_yaw = Some(measurement.orientation.yaw);
        self.last_update_ms = measurement.timestamp_ms;

        // Check if we've reached target angle within tolerance
        //
        // NOTE: ROTATION_TOLERANCE_DEG is used as the "tight" tolerance; set this to ~0.5°
        // for higher-precision turns.
        (self.accumulated_angle - self.target_angle).abs() <= ROTATION_TOLERANCE_DEG
    }

    /// Clamp speed value to valid u8 range (0-100)
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

    /// Calculates appropriate motor speeds for current rotation state
    /// Returns (`left_speed`, `right_speed`)
    pub fn calculate_motor_speeds(&self) -> (i8, i8) {
        // Signed error: positive => undershoot (need more rotation), negative => overshoot.
        let error_deg = self.target_angle - self.accumulated_angle;
        let remaining_degrees = error_deg.abs();

        // Overshoot correction:
        // If we overshot (error < 0), reverse motor directions to "turn back" until within tolerance.
        // This keeps the bot hunting around the setpoint rather than spinning forever past it.
        let effective_direction = if error_deg < 0.0 {
            match self.direction {
                RotationDirection::Clockwise => RotationDirection::CounterClockwise,
                RotationDirection::CounterClockwise => RotationDirection::Clockwise,
            }
        } else {
            self.direction
        };

        let rotation_speed = match self.motion {
            RotationMotion::Stationary { speed } => {
                // For high precision turns, avoid tapering too low near the target; keep a floor.
                // We still reduce speed near the target to limit oscillation, but never below ROTATION_SPEED_MIN.
                let requested = speed.clamp(0, 100);
                if remaining_degrees < 10.0 {
                    let min = ROTATION_SPEED_MIN;
                    let max = requested.max(min);
                    let speed_range = max - min;
                    let speed_factor = remaining_degrees / 10.0;
                    Self::clamp_speed_u8(f32::from(min) + (f32::from(speed_range) * speed_factor))
                } else {
                    requested
                }
            }
            RotationMotion::WhileMoving(_) => {
                // Default profile for combined motion rotations
                if remaining_degrees < 10.0 {
                    // Linear interpolation between min and max speed
                    let speed_range = ROTATION_SPEED_MAX - ROTATION_SPEED_MIN;
                    let speed_factor = remaining_degrees / 10.0;
                    Self::clamp_speed_u8(f32::from(ROTATION_SPEED_MIN) + (f32::from(speed_range) * speed_factor))
                } else {
                    ROTATION_SPEED_MAX
                }
            }
        };
        let rotation_speed_signed = i8::try_from(rotation_speed).unwrap_or(i8::MAX);

        match self.motion {
            RotationMotion::Stationary { speed: _ } => {
                // Simple differential drive for in-place rotation (with overshoot reversal)
                match effective_direction {
                    RotationDirection::Clockwise => (rotation_speed_signed, -rotation_speed_signed),
                    RotationDirection::CounterClockwise => (-rotation_speed_signed, rotation_speed_signed),
                }
            }
            RotationMotion::WhileMoving(_) => {
                // Combine base motion with rotation
                let rotation_diff = rotation_speed_signed.min(SPEED_DIFF_MAX);

                match effective_direction {
                    RotationDirection::Clockwise => {
                        (self.base_speed, (self.base_speed - rotation_diff).clamp(-100, 100))
                    }
                    RotationDirection::CounterClockwise => {
                        ((self.base_speed - rotation_diff).clamp(-100, 100), self.base_speed)
                    }
                }
            }
        }
    }

    /// Returns the base speed for maintaining motion after rotation
    pub const fn continuation_speed(&self) -> Option<i8> {
        match self.motion {
            RotationMotion::Stationary { speed: _ } => None,
            RotationMotion::WhileMoving(_) => Some(self.base_speed),
        }
    }
}

/// Detect if robot is performing a stationary rotation (turn in place)
///
/// Returns true if left and right tracks have equal but opposite speeds.
pub const fn is_turning_in_place(left_speed: i8, right_speed: i8) -> bool {
    left_speed == -right_speed && left_speed != 0
}
