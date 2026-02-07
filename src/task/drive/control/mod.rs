//! Control systems for drive task
//!
//! This module contains all the control algorithms and state machines
//! used for precise motion control, including rotation, straight-line
//! driving, tilt compensation, and motor state management.

pub mod motor_state;
pub mod rotation;
pub mod straight_line;
pub mod tilt;

// Re-export commonly used types for convenience
pub use rotation::RotationState;
