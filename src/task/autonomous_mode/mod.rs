//! Autonomous drive mode tasks.
//!
//! Each sub-module implements one self-contained autonomous behaviour.
//! A mode is started and stopped via its public [`start`] / [`stop`] functions
//! and runs as a long-lived Embassy task that waits between activations.

pub mod coast_obstacle_avoid;
