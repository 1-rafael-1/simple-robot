//! Perception state module.
//!
//! Owns obstacle detection state and ultrasonic readings behind a single interface.
//! All access goes through public accessor functions; the mutex and struct are private.
//!
//! Lock order (when multiple state mutexes are needed):
//! 1) power state mutex (use power module accessors)
//! 2) `CALIBRATION_STATE`
//! 3) perception mutex (private — use accessor functions)
//! 4) `MOTION_STATE`

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use crate::system::event::UltrasonicReading;

// ── Atomics (lock-free fast path for reads) ────────────────────────────────────

/// IR obstacle detection flag — updated by `set_ir_obstacle`, read lock-free.
static IR_DETECTED: AtomicBool = AtomicBool::new(false);
/// Ultrasonic obstacle detection flag — updated by `set_ultrasonic_obstacle`, read lock-free.
static ULTRASONIC_DETECTED: AtomicBool = AtomicBool::new(false);
/// Combined obstacle flag — `IR_DETECTED || ULTRASONIC_DETECTED`.
/// Recomputed atomically by every mutating accessor, read lock-free.
static COMBINED_DETECTED: AtomicBool = AtomicBool::new(false);

// ── Mutex (full picture: readings, angle) ─────────────────────────────────────

/// Mutex-guarded state holding ultrasonic readings.
static STATE: Mutex<CriticalSectionRawMutex, PerceptionState> = Mutex::new(PerceptionState {
    ultrasonic_reading: None,
    ultrasonic_angle_deg: None,
});

/// Internal state behind the mutex — readings and angle.
struct PerceptionState {
    /// Latest ultrasonic reading, if available.
    ultrasonic_reading: Option<UltrasonicReading>,
    /// Latest ultrasonic servo angle (degrees), if available.
    ultrasonic_angle_deg: Option<f32>,
}

// ── Change detection ──────────────────────────────────────────────────────────

/// Outcome of a setter that may change the combined obstacle flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChangeDetected {
    /// The obstacle flag did not change.
    NoChange,
    /// The combined obstacle flag transitioned from false → true.
    ChangedToDetected,
    /// The combined obstacle flag transitioned from true → false.
    ChangedToCleared,
}

// ── Public accessors (lock-free) ──────────────────────────────────────────────

/// Set the IR obstacle flag. Returns whether the combined obstacle state changed.
pub fn set_ir_obstacle(detected: bool) -> ChangeDetected {
    let old_combined = COMBINED_DETECTED.load(Ordering::Relaxed);
    IR_DETECTED.store(detected, Ordering::Relaxed);
    recompute_combined();
    let new_combined = COMBINED_DETECTED.load(Ordering::Relaxed);
    change_detected(old_combined, new_combined)
}

/// Set the ultrasonic obstacle flag. Returns whether the combined obstacle state changed.
pub fn set_ultrasonic_obstacle(detected: bool) -> ChangeDetected {
    let old_combined = COMBINED_DETECTED.load(Ordering::Relaxed);
    ULTRASONIC_DETECTED.store(detected, Ordering::Relaxed);
    recompute_combined();
    let new_combined = COMBINED_DETECTED.load(Ordering::Relaxed);
    change_detected(old_combined, new_combined)
}

/// Returns `true` if any obstacle sensor (IR or ultrasonic) reports an obstacle.
/// Lock-free — safe to call from any context.
pub fn is_obstacle_detected() -> bool {
    COMBINED_DETECTED.load(Ordering::Relaxed)
}

/// Returns `true` if the IR sensor reports an obstacle.
/// Lock-free — safe to call from any context.
pub fn is_ir_obstacle_detected() -> bool {
    IR_DETECTED.load(Ordering::Relaxed)
}

/// Returns `true` if the ultrasonic sensor reports an obstacle.
/// Lock-free — safe to call from any context.
pub fn is_ultrasonic_obstacle_detected() -> bool {
    ULTRASONIC_DETECTED.load(Ordering::Relaxed)
}

// ── Public accessors (async — touch the mutex) ────────────────────────────────

/// Store an ultrasonic reading and angle in the mutex.
///
/// Obstacle classification and atomic flag updates are handled by the event-bus
/// path (see [`set_ultrasonic_obstacle`]); this function is a pure data store.
pub async fn set_ultrasonic_reading(reading: Option<UltrasonicReading>, angle_deg: f32) {
    let mut state = STATE.lock().await;
    state.ultrasonic_reading = reading;
    state.ultrasonic_angle_deg = Some(angle_deg);
}

/// Clear ultrasonic reading, angle, and obstacle flag (e.g. when exiting a test mode).
pub async fn clear_ultrasonic_data() {
    ULTRASONIC_DETECTED.store(false, Ordering::Relaxed);
    recompute_combined();

    let mut state = STATE.lock().await;
    state.ultrasonic_reading = None;
    state.ultrasonic_angle_deg = None;
}

/// Reset all obstacle flags and ultrasonic data to defaults.
pub async fn reset_all() {
    IR_DETECTED.store(false, Ordering::Relaxed);
    ULTRASONIC_DETECTED.store(false, Ordering::Relaxed);
    COMBINED_DETECTED.store(false, Ordering::Relaxed);

    let mut state = STATE.lock().await;
    state.ultrasonic_reading = None;
    state.ultrasonic_angle_deg = None;
}

/// Return the latest ultrasonic reading and servo angle from a single lock
/// acquisition, so the two values are from the same measurement.
pub async fn ultrasonic_sweep_snapshot() -> (Option<UltrasonicReading>, Option<f32>) {
    let state = STATE.lock().await;
    (state.ultrasonic_reading, state.ultrasonic_angle_deg)
}

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Recompute the combined obstacle flag from IR and ultrasonic atomics.
fn recompute_combined() {
    let combined = IR_DETECTED.load(Ordering::Relaxed) || ULTRASONIC_DETECTED.load(Ordering::Relaxed);
    COMBINED_DETECTED.store(combined, Ordering::Relaxed);
}

/// Map (old, new) combined flag states to a `ChangeDetected` variant.
const fn change_detected(old_combined: bool, new_combined: bool) -> ChangeDetected {
    match (old_combined, new_combined) {
        (false, true) => ChangeDetected::ChangedToDetected,
        (true, false) => ChangeDetected::ChangedToCleared,
        _ => ChangeDetected::NoChange,
    }
}
