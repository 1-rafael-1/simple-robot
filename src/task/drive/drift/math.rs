//! Pure drift-compensation mathematics.
//!
//! This module contains only stateless, synchronous functions and types that
//! implement the encoder-based drift-compensation algorithm. There is no async
//! code and no I/O here; all of that lives in the parent [`super`] module
//! (`drift/mod.rs`), which owns the async control-loop step and calls into
//! these functions for the actual computation.
//!
//! # Responsibilities
//!
//! - Convert raw [`EncoderMeasurement`] deltas into per-track averages ([`TrackSpeedData`]).
//! - Compute the percentage speed difference between the left and right tracks.
//! - Decide what correction to apply ([`CompensationAction`]).
//! - Apply a correction action to current motor-speed commands.
//! - Provide a 16-bit counter delta helper with wraparound handling.
//!
//! # Usage
//!
//! Callers are expected to:
//! 1. Compute per-sample deltas from cumulative hardware counters using
//!    [`calculate_delta_u16`] before passing values to [`calculate_track_averages`].
//! 2. Call [`calculate_speed_difference`] on the resulting [`TrackSpeedData`].
//! 3. Call [`determine_compensation`] with the difference and current speed commands.
//! 4. Apply the returned [`CompensationAction`] via [`apply_compensation_action`].

use crate::task::drive::types::{DRIFT_COMPENSATION_GAIN, DRIFT_COMPENSATION_MAX, DRIFT_TOLERANCE_PERCENT};

// ── Types ─────────────────────────────────────────────────────────────────────

/// Which track (left or right) is being referenced.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Track {
    /// Left track.
    Left,
    /// Right track.
    Right,
}

/// Snapshot of encoder pulse counts for the current sampling window,
/// together with computed per-track averages.
///
/// **Important**: the pulse counts stored here must already be per-window
/// *deltas*, not raw cumulative hardware counters. Use [`calculate_delta_u16`]
/// to compute deltas from consecutive cumulative readings before constructing
/// this value via [`calculate_track_averages`].
#[derive(Debug, Clone, Copy)]
pub struct TrackSpeedData {
    /// Delta pulse count for the left-front motor in the current sampling window.
    pub left_front: u16,
    /// Delta pulse count for the left-rear motor in the current sampling window.
    pub left_rear: u16,
    /// Delta pulse count for the right-front motor in the current sampling window.
    pub right_front: u16,
    /// Delta pulse count for the right-rear motor in the current sampling window.
    pub right_rear: u16,
    /// Computed average pulse count for the left track (`(left_front + left_rear) / 2`).
    pub left_track_avg: f32,
    /// Computed average pulse count for the right track (`(right_front + right_rear) / 2`).
    pub right_track_avg: f32,
    /// Timestamp of the originating `EncoderMeasurement` (milliseconds since boot).
    pub timestamp_ms: u64,
}

impl TrackSpeedData {
    /// Returns `true` if all four motor encoders reported zero pulses this window.
    ///
    /// This typically means the robot is stationary or at very low speed; the
    /// drift-compensation loop should skip the sample.
    pub const fn all_zero(&self) -> bool {
        self.left_front == 0 && self.left_rear == 0 && self.right_front == 0 && self.right_rear == 0
    }

    /// Returns `true` if at least one motor is zero while others are non-zero.
    ///
    /// This pattern suggests a faulty or unplugged encoder. The compensation
    /// loop should treat this as an anomaly and disable itself.
    pub fn has_single_motor_zero_anomaly(&self) -> bool {
        let vals = [self.left_front, self.left_rear, self.right_front, self.right_rear];
        let any_nonzero = vals.iter().any(|&v| v != 0);
        let any_zero = vals.contains(&0);
        any_nonzero && any_zero
    }
}

/// Drift-compensation decision to apply to the current motor commands.
///
/// The policy prefers *increasing* the slower track if doing so stays within
/// the ±100 speed limit; otherwise it *decreases* the faster track.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CompensationAction {
    /// Within tolerance — no adjustment needed.
    None,
    /// Increase the left track by N speed points.
    IncreaseLeft(i8),
    /// Increase the right track by N speed points.
    IncreaseRight(i8),
    /// Decrease the left track by N speed points.
    DecreaseLeft(i8),
    /// Decrease the right track by N speed points.
    DecreaseRight(i8),
}

// ── Pure functions ────────────────────────────────────────────────────────────

/// Convert an [`EncoderMeasurement`] into [`TrackSpeedData`] with per-track averages.
///
/// The caller must supply per-window *delta* values (not raw cumulative counters).
/// Use [`calculate_delta_u16`] to compute deltas from consecutive cumulative readings.
pub fn calculate_track_averages(measurement: crate::task::sensors::encoders::EncoderMeasurement) -> TrackSpeedData {
    let left_front = measurement.left_front;
    let left_rear = measurement.left_rear;
    let right_front = measurement.right_front;
    let right_rear = measurement.right_rear;

    // Convert each u16 to f32 before summing to avoid u16 overflow and to
    // sidestep precision-loss concerns: u16::MAX (65535) < 2^23, so every
    // u16 value is representable exactly in f32's 23-bit mantissa.
    let left_track_avg = f32::midpoint(f32::from(left_front), f32::from(left_rear));
    let right_track_avg = f32::midpoint(f32::from(right_front), f32::from(right_rear));

    TrackSpeedData {
        left_front,
        left_rear,
        right_front,
        right_rear,
        left_track_avg,
        right_track_avg,
        timestamp_ms: measurement.timestamp_ms,
    }
}

/// Compute the percentage speed difference between the left and right tracks.
///
/// Uses per-track averages from [`TrackSpeedData`].
///
/// Formula: `(left − right) / max(left, right) × 100`
///
/// Sign convention:
/// - **Positive** → left track is faster.
/// - **Negative** → right track is faster.
///
/// Returns `0.0` if both tracks are at (or near) zero to avoid division by zero.
pub fn calculate_speed_difference(data: &TrackSpeedData) -> f32 {
    let left = data.left_track_avg;
    let right = data.right_track_avg;
    let denom = left.max(right);
    if denom <= 0.0 {
        return 0.0;
    }
    ((left - right) / denom) * 100.0
}

/// Determine what compensation to apply based on the measured speed difference
/// and the current left/right speed commands.
///
/// # Policy
///
/// 1. If `|diff_percent| ≤ DRIFT_TOLERANCE_PERCENT`, return [`CompensationAction::None`].
/// 2. Identify the slow and fast tracks from the sign of `diff_percent`.
/// 3. Compute adjustment magnitude proportional to the error, capped to
///    [`DRIFT_COMPENSATION_MAX`].
/// 4. Prefer *increasing* the slow track if it stays within ±100 without
///    crossing zero. Otherwise *decrease* the fast track.
///
/// # Reverse-driving symmetry
///
/// For negative speeds, "increase" means moving further from zero (more
/// negative), and "decrease" means moving toward zero, preserving sign in
/// both cases. This keeps compensation symmetric between forward and reverse.
///
/// # Preconditions
///
/// Speed commands are expected to be in the range `[-100, 100]`.
pub fn determine_compensation(diff_percent: f32, left_speed: i8, right_speed: i8) -> CompensationAction {
    if diff_percent.abs() <= DRIFT_TOLERANCE_PERCENT {
        return CompensationAction::None;
    }

    // diff > 0 ⇒ left faster ⇒ right is the slow track.
    // diff < 0 ⇒ right faster ⇒ left is the slow track.
    let (slow_track, fast_track) = if diff_percent > 0.0 {
        (Track::Right, Track::Left)
    } else {
        (Track::Left, Track::Right)
    };

    // Adjustment magnitude: proportional gain, rounded to the nearest integer,
    // clamped to [1, DRIFT_COMPENSATION_MAX].
    let raw = diff_percent.abs() * DRIFT_COMPENSATION_GAIN;
    // Round-to-nearest without std: add 0.5 then truncate (raw ≥ 0 here).
    #[allow(clippy::cast_possible_truncation)]
    let mut adjustment = (raw + 0.5) as i8;
    if adjustment < 1 {
        adjustment = 1;
    }
    adjustment = adjustment.clamp(0, DRIFT_COMPENSATION_MAX);

    if adjustment == 0 {
        return CompensationAction::None;
    }

    let slow_speed = match slow_track {
        Track::Left => left_speed,
        Track::Right => right_speed,
    };
    let fast_speed = match fast_track {
        Track::Left => left_speed,
        Track::Right => right_speed,
    };

    // Prefer increasing the slow side if it stays within bounds without sign flip.
    if can_increase_magnitude_within_bounds(slow_speed, adjustment) {
        match slow_track {
            Track::Left => CompensationAction::IncreaseLeft(adjustment),
            Track::Right => CompensationAction::IncreaseRight(adjustment),
        }
    } else {
        // Fall back to decreasing the fast side toward zero.
        let adj = clamp_decrease_magnitude_amount_within_bounds(fast_speed, adjustment);
        if adj <= 0 {
            CompensationAction::None
        } else {
            match fast_track {
                Track::Left => CompensationAction::DecreaseLeft(adj),
                Track::Right => CompensationAction::DecreaseRight(adj),
            }
        }
    }
}

/// Apply a [`CompensationAction`] to the current motor commands.
///
/// Returns `(new_left, new_right)` clamped to `[-100, 100]`.
pub fn apply_compensation_action(action: CompensationAction, left_speed: i8, right_speed: i8) -> (i8, i8) {
    let (mut l, mut r) = (left_speed, right_speed);

    match action {
        CompensationAction::None => {}
        CompensationAction::IncreaseLeft(adj) => l = increase_magnitude_clamped(l, adj),
        CompensationAction::IncreaseRight(adj) => r = increase_magnitude_clamped(r, adj),
        CompensationAction::DecreaseLeft(adj) => l = decrease_magnitude_clamped(l, adj),
        CompensationAction::DecreaseRight(adj) => r = decrease_magnitude_clamped(r, adj),
    }

    (l, r)
}

/// 16-bit counter delta with wraparound handling.
///
/// Use this to convert two consecutive cumulative hardware counter readings
/// into the number of pulses that occurred in that window, even when the
/// counter wraps around `u16::MAX`.
///
/// # Example
/// ```text
/// previous = 65530, current = 3  →  delta = 9
/// ```
pub const fn calculate_delta_u16(current: u16, previous: u16) -> u16 {
    if current >= previous {
        current - previous
    } else {
        (u16::MAX - previous).wrapping_add(current).wrapping_add(1)
    }
}

// ── Private helpers ───────────────────────────────────────────────────────────

/// Add `delta` to `value`, clamped to `[-100, 100]`.
#[allow(clippy::cast_possible_truncation)]
fn add_clamped(value: i8, delta: i8) -> i8 {
    (i16::from(value) + i16::from(delta)).clamp(-100, 100) as i8
}

/// Subtract `delta` from `value`, clamped to `[-100, 100]`.
#[allow(clippy::cast_possible_truncation)]
fn sub_clamped(value: i8, delta: i8) -> i8 {
    (i16::from(value) - i16::from(delta)).clamp(-100, 100) as i8
}

/// Increase the magnitude of `value` by `delta`, preserving sign and clamping
/// to `[-100, 100]`.
///
/// - `value = +40, delta = 5` → `+45`
/// - `value = -40, delta = 5` → `-45`
fn increase_magnitude_clamped(value: i8, delta: i8) -> i8 {
    if delta <= 0 {
        return value.clamp(-100, 100);
    }
    if value >= 0 {
        add_clamped(value, delta)
    } else {
        sub_clamped(value, delta)
    }
}

/// Decrease the magnitude of `value` by `delta` (move toward zero), preserving
/// sign and clamping to `[-100, 100]`.
///
/// - `value = +40, delta = 5` → `+35`
/// - `value = -40, delta = 5` → `-35`
fn decrease_magnitude_clamped(value: i8, delta: i8) -> i8 {
    if delta <= 0 {
        return value.clamp(-100, 100);
    }
    if value >= 0 {
        sub_clamped(value, delta)
    } else {
        add_clamped(value, delta)
    }
}

/// Returns `true` if increasing the magnitude of `value` by `delta` stays
/// within `±100` without crossing zero.
const fn can_increase_magnitude_within_bounds(value: i8, delta: i8) -> bool {
    if delta <= 0 {
        return true;
    }
    if value >= 0 {
        (value as i16 + delta as i16) <= 100
    } else {
        (value as i16 - delta as i16) >= -100
    }
}

/// Clamp how much the magnitude of `value` can be decreased toward zero,
/// ensuring the result does not cross zero.
///
/// - Forward (`value ≥ 0`): max decrease = `value` (result ≥ 0).
/// - Reverse (`value < 0`): max decrease = `−value` (result ≤ 0).
#[allow(clippy::cast_possible_truncation)]
fn clamp_decrease_magnitude_amount_within_bounds(value: i8, desired_delta: i8) -> i8 {
    if desired_delta <= 0 {
        return 0;
    }
    let max_allowed = if value >= 0 {
        i16::from(value)
    } else {
        (-i16::from(value)).max(0)
    };
    i16::from(desired_delta).min(max_allowed).max(0) as i8
}
