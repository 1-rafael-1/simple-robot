//! Sensor infrastructure for the drive task.
//!
//! This module groups the two distinct sensor concerns used by the drive task:
//!
//! - [`data`]: Static feedback channels and measurement-forwarding functions
//!   (the *data plane* — measurements arriving from sensor tasks into the drive task).
//! - [`control`]: Sensor lifecycle helpers that start and stop IMU and encoder
//!   sampling (the *control plane* — commands going out to sensor tasks).
//!

pub(super) mod control;
pub(super) mod data;
