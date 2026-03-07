//! Perception state module.
//!
//! Holds obstacle detection flags and ultrasonic sweep data in a dedicated mutex.
//!
//! Lock order (when multiple state mutexes are needed):
//! 1) `POWER_STATE`
//! 2) `CALIBRATION_STATE`
//! 3) `PERCEPTION_STATE`
//! 4) `MOTION_STATE`

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use crate::system::event::UltrasonicReading;

/// Global perception state protected by a mutex.
pub static PERCEPTION_STATE: Mutex<CriticalSectionRawMutex, PerceptionState> = Mutex::new(PerceptionState {
    obstacle_detected: false,
    ir_obstacle_detected: false,
    ultrasonic_obstacle_detected: false,
    ultrasonic_reading: None,
    ultrasonic_angle_deg: None,
});

/// Perception-related state shared across the system.
pub struct PerceptionState {
    /// Obstacle detection status (combined).
    pub obstacle_detected: bool,
    /// IR obstacle detection status.
    pub ir_obstacle_detected: bool,
    /// Ultrasonic obstacle detection status.
    pub ultrasonic_obstacle_detected: bool,
    /// Latest ultrasonic reading status, if available.
    pub ultrasonic_reading: Option<UltrasonicReading>,
    /// Latest ultrasonic angle reading (degrees), if available.
    pub ultrasonic_angle_deg: Option<f32>,
}
