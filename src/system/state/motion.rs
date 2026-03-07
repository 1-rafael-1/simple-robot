//! Motion state module.
//!
//! Holds track speed telemetry in a dedicated mutex, plus
//! atomic mirrors for high-frequency readers (e.g., IMU).
//!
//! Lock order (when multiple state mutexes are needed):
//! 1) `POWER_STATE`
//! 2) `CALIBRATION_STATE`
//! 3) `PERCEPTION_STATE`
//! 4) `MOTION_STATE`
//!
//! UI state is separate and does not participate in this order.

use core::sync::atomic::{AtomicI8, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

/// Atomic mirror of the left track speed (for high-frequency, lock-free reads).
pub static LEFT_TRACK_SPEED_ATOMIC: AtomicI8 = AtomicI8::new(0);
/// Atomic mirror of the right track speed (for high-frequency, lock-free reads).
pub static RIGHT_TRACK_SPEED_ATOMIC: AtomicI8 = AtomicI8::new(0);

/// Global motion state protected by a mutex.
pub static MOTION_STATE: Mutex<CriticalSectionRawMutex, MotionState> = Mutex::new(MotionState {
    left_track_speed: 0,
    right_track_speed: 0,
});

/// Motion-related state shared across the system.
pub struct MotionState {
    /// Current left track speed (-100 to +100).
    pub left_track_speed: i8,
    /// Current right track speed (-100 to +100).
    pub right_track_speed: i8,
}

/// Sets both track speeds and updates the atomic mirrors.
pub async fn set_track_speeds(left: i8, right: i8) {
    {
        let mut state = MOTION_STATE.lock().await;
        state.left_track_speed = left;
        state.right_track_speed = right;
        LEFT_TRACK_SPEED_ATOMIC.store(left, Ordering::Relaxed);
        RIGHT_TRACK_SPEED_ATOMIC.store(right, Ordering::Relaxed);
    }
}

/// Reads the latest track speeds from the atomic mirrors.
pub fn get_track_speeds_atomic() -> (i8, i8) {
    (
        LEFT_TRACK_SPEED_ATOMIC.load(Ordering::Relaxed),
        RIGHT_TRACK_SPEED_ATOMIC.load(Ordering::Relaxed),
    )
}
