//! Tilt compensation for motor control on inclines

/// Motor power adjustment based on tilt
pub(crate) struct TiltCompensation {
    /// Maximum power adjustment factor
    max_adjustment: f32,
    /// Maximum tilt angle in degrees
    max_tilt_angle: f32,
}

impl TiltCompensation {
    /// Creates a new tilt compensation instance
    /// with the given maximum adjustment and tilt angle
    pub fn new(max_adjustment: f32, max_tilt_angle: f32) -> Self {
        Self {
            max_adjustment,
            max_tilt_angle,
        }
    }

    /// Calculate power adjustment factor based on tilt angle
    pub fn calculate_adjustment(&mut self, tilt_degrees: f32) -> f32 {
        (tilt_degrees / self.max_tilt_angle).clamp(-1.0, 1.0) * self.max_adjustment
    }
}
