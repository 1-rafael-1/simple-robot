//! Track state tracking and control interface

use super::tilt::TiltCompensation;
use crate::{
    system::state::SYSTEM_STATE,
    task::motor_driver::{self, MotorCommand, Track},
};

/// Track state tracking (simplified - actual control via `motor_driver` task)
pub struct TrackState {
    /// Current motor speed setting (-100 to +100)
    current_speed: i8,
    /// Current direction (true = forward, false = backward)
    forward: bool,
}

impl TrackState {
    /// Creates a new track state with default values
    pub const fn new() -> Self {
        Self {
            current_speed: 0,
            forward: true,
        }
    }

    /// Converts encoder pulses to motor shaft RPM
    /// RPM is calculated for the motor shaft before the 120:1 gear reduction
    pub fn calculate_rpm(&self, pulses: u16, elapsed_ms: u32) -> f32 {
        use crate::task::drive::types::PULSES_PER_REV;

        if elapsed_ms == 0 {
            return 0.0;
        }

        let hz = (f64::from(pulses) * 1000.0) / f64::from(elapsed_ms);
        let motor_rpm = (hz / f64::from(PULSES_PER_REV)) * 60.0;
        #[allow(clippy::cast_precision_loss)]
        #[allow(clippy::cast_possible_truncation)]
        let motor_rpm = motor_rpm as f32;
        if self.forward { motor_rpm } else { -motor_rpm }
    }

    /// Sets motor speed and direction (-100 to +100)
    pub async fn set_speed(&mut self, track: Track, speed: i8) {
        self.current_speed = speed;
        self.forward = speed >= 0;

        motor_driver::send_motor_command(MotorCommand::SetTrack { track, speed }).await;

        // Update system state
        let mut state = SYSTEM_STATE.lock().await;
        match track {
            Track::Left => state.left_track_speed = speed,
            Track::Right => state.right_track_speed = speed,
        }
    }

    /// Sets motor speed with tilt compensation
    pub async fn set_speed_with_tilt(&mut self, track: Track, base_speed: i8, tilt_degrees: f32) {
        let mut tilt_compensation = TiltCompensation::new(0.3, 45.0);
        let tilt_adjustment = tilt_compensation.calculate_adjustment(tilt_degrees);
        let adjusted = f32::from(base_speed) * (1.0 + tilt_adjustment);
        #[allow(clippy::cast_possible_truncation)]
        let adjusted_speed = libm::roundf(adjusted).clamp(-100.0, 100.0) as i8;
        self.set_speed(track, adjusted_speed).await;
    }

    /// Actively stops motor using electrical braking
    pub async fn brake(&mut self, track: Track) {
        self.current_speed = 0;
        motor_driver::send_motor_command(MotorCommand::SetTrack { track, speed: 0 }).await;
        motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
    }

    /// Stops motor by letting it spin freely
    pub async fn coast(&mut self, track: Track) {
        self.current_speed = 0;
        motor_driver::send_motor_command(MotorCommand::SetTrack { track, speed: 0 }).await;
    }

    /// Returns current motor speed setting (-100 to +100)
    pub const fn current_speed(&self) -> i8 {
        self.current_speed
    }
}
