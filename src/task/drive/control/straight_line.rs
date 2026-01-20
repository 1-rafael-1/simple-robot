//! Straight-line motion control with drift correction

/// Tracks straight-line motion correction
pub(crate) struct StraightLineState {
    /// Initial yaw angle when straight motion started
    target_yaw: f32,
    /// Previous yaw error for derivative calculation
    last_error: Option<f32>,
    /// Last update timestamp for derivative calculation
    last_update_ms: u32,
}

impl StraightLineState {
    /// Maximum correction strength (as percentage of base speed)
    const MAX_CORRECTION: f32 = 0.2; // 20% max speed difference
    /// Proportional control factor
    const P_FACTOR: f32 = 0.1;
    /// Derivative control factor
    const D_FACTOR: f32 = 0.05;

    pub fn new(initial_yaw: f32) -> Self {
        Self {
            target_yaw: initial_yaw,
            last_error: None,
            last_update_ms: 0,
        }
    }

    /// Calculates motor speed corrections to maintain straight line
    /// Returns (left_adjustment, right_adjustment) as factors to multiply with base speed
    pub fn calculate_correction(&mut self, current_yaw: f32, timestamp_ms: u32) -> (f32, f32) {
        let mut yaw_error = current_yaw - self.target_yaw;

        // Handle wraparound at ±180 degrees
        if yaw_error > 180.0 {
            yaw_error -= 360.0;
        } else if yaw_error < -180.0 {
            yaw_error += 360.0;
        }

        // Calculate P term
        let p_correction = yaw_error * Self::P_FACTOR;

        // Calculate D term (rate of change of error)
        let d_correction = if let Some(last_error) = self.last_error {
            let dt = (timestamp_ms - self.last_update_ms) as f32 / 1000.0;
            if dt > 0.0 {
                let error_rate = (yaw_error - last_error) / dt;
                error_rate * Self::D_FACTOR
            } else {
                0.0
            }
        } else {
            0.0
        };

        // Update state for next calculation
        self.last_error = Some(yaw_error);
        self.last_update_ms = timestamp_ms;

        // Combine P and D terms and clamp to max correction
        let correction = (p_correction + d_correction).clamp(-Self::MAX_CORRECTION, Self::MAX_CORRECTION);

        if correction > 0.0 {
            // Drifting right, speed up right motor
            (1.0 - correction, 1.0 + correction)
        } else {
            // Drifting left, speed up left motor
            (1.0 + correction.abs(), 1.0 - correction.abs())
        }
    }
}
