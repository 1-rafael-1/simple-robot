//! Perception state module.
//!
//! Owns obstacle detection state and ultrasonic readings behind a single interface.
//! All access goes through public accessor functions; the mutex and struct are private.
//!
//! Lock order (when multiple state mutexes are needed):
//! 1) `POWER_STATE`
//! 2) `CALIBRATION_STATE`
//! 3) perception mutex (private — use accessor functions)
//! 4) `MOTION_STATE`

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use crate::system::event::UltrasonicReading;

// ── Atomics (lock-free fast path for reads) ────────────────────────────────────

/// IR obstacle detection flag — updated by `set_ir_obstacle`, read lock-free.
static IR_DETECTED: AtomicBool = AtomicBool::new(false);
/// Ultrasonic obstacle detection flag — updated by `set_ultrasonic_obstacle` and
/// `set_ultrasonic_reading`, read lock-free.
static ULTRASONIC_DETECTED: AtomicBool = AtomicBool::new(false);
/// Combined obstacle flag — `IR_DETECTED || ULTRASONIC_DETECTED`.
/// Recomputed atomically by every mutating accessor, read lock-free.
static COMBINED_DETECTED: AtomicBool = AtomicBool::new(false);

// ── Mutex (full picture: readings, angle, threshold) ──────────────────────────

/// Default obstacle distance threshold (cm). Used by `set_ultrasonic_reading` to
/// auto-detect obstacles from raw distance readings.
///
/// Matches the ultrasonic sensor task's `ULTRASONIC_OBSTACLE_THRESHOLD_CM` (15 cm)
/// so lock-free readers see the same obstacle state the event system produces.
const DEFAULT_OBSTACLE_THRESHOLD_CM: f64 = 15.0;

/// Mutex-guarded state holding ultrasonic readings and threshold.
static STATE: Mutex<CriticalSectionRawMutex, PerceptionState> = Mutex::new(PerceptionState {
    ultrasonic_reading: None,
    ultrasonic_angle_deg: None,
    obstacle_threshold_cm: DEFAULT_OBSTACLE_THRESHOLD_CM,
});

/// Internal state behind the mutex — readings, angle, and threshold.
struct PerceptionState {
    /// Latest ultrasonic reading, if available.
    ultrasonic_reading: Option<UltrasonicReading>,
    /// Latest ultrasonic servo angle (degrees), if available.
    ultrasonic_angle_deg: Option<f32>,
    /// Current obstacle distance threshold (cm).
    obstacle_threshold_cm: f64,
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

/// Store an ultrasonic reading and angle, auto-applying the current obstacle
/// threshold to update the ultrasonic obstacle flag.
pub async fn set_ultrasonic_reading(reading: Option<UltrasonicReading>, angle_deg: f32) {
    let obstacle = match reading {
        Some(UltrasonicReading::Distance(cm)) => {
            let threshold = { STATE.lock().await.obstacle_threshold_cm };
            cm <= threshold
        }
        _ => false,
    };

    {
        let mut state = STATE.lock().await;
        state.ultrasonic_reading = reading;
        state.ultrasonic_angle_deg = Some(angle_deg);
    }

    let old_combined = COMBINED_DETECTED.load(Ordering::Relaxed);
    ULTRASONIC_DETECTED.store(obstacle, Ordering::Relaxed);
    recompute_combined();
    let _new_combined = COMBINED_DETECTED.load(Ordering::Relaxed);
    // Note: no ChangeDetected return here — the ultrasonic task's own
    // ObstacleDetected event handles reaction. This path just keeps the
    // atomic mirrors correct for lock-free readers.
    let _ = old_combined;
}

/// Clear ultrasonic reading and angle (e.g. when exiting a test mode).
pub async fn clear_ultrasonic_data() {
    let mut state = STATE.lock().await;
    state.ultrasonic_reading = None;
    state.ultrasonic_angle_deg = None;
}

/// Reset all obstacle flags, ultrasonic data, and threshold to defaults.
pub async fn reset_all() {
    IR_DETECTED.store(false, Ordering::Relaxed);
    ULTRASONIC_DETECTED.store(false, Ordering::Relaxed);
    COMBINED_DETECTED.store(false, Ordering::Relaxed);

    let mut state = STATE.lock().await;
    state.ultrasonic_reading = None;
    state.ultrasonic_angle_deg = None;
    state.obstacle_threshold_cm = DEFAULT_OBSTACLE_THRESHOLD_CM;
}

/// Return a copy of the latest ultrasonic reading, if any.
pub async fn ultrasonic_reading_copy() -> Option<UltrasonicReading> {
    STATE.lock().await.ultrasonic_reading
}

/// Override the obstacle detection distance threshold (cm).
/// Distance readings ≤ this value are considered obstacles by `set_ultrasonic_reading`.
#[allow(dead_code)]
pub async fn set_obstacle_threshold(cm: f64) {
    let mut state = STATE.lock().await;
    state.obstacle_threshold_cm = cm;
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
