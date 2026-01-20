//! Control systems for drive task
//!
//! This module contains all the control algorithms and state machines
//! used for precise motion control, including rotation, straight-line
//! driving, tilt compensation, and motor state management.

pub(crate) mod motor_state;
pub(crate) mod rotation;
pub(crate) mod straight_line;
pub(crate) mod tilt;

// Re-export commonly used types for convenience
pub(crate) use motor_state::TrackState;
pub(crate) use rotation::{RotationState, is_turning_in_place};
pub(crate) use straight_line::StraightLineState;
pub(crate) use tilt::TiltCompensation;
