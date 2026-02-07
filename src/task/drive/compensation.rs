//! Encoder-based drift compensation for straight-line driving.
//!
//! This module is intentionally self-contained:
//! - It computes per-track average pulse counts from an `EncoderMeasurement`
//! - It computes a % difference between tracks
//! - It determines an action to compensate without exceeding +/-100 motor command limits
//!
//! Design notes:
//! - The current encoder task resets counters at the start of sampling and publishes
//!   absolute counts since last reset. In that mode, you can treat the measurement
//!   values as deltas for the sampling window.
//! - If your encoder task ever changes to publish cumulative counts, you can use
//!   `calculate_delta_u16()` with stored previous readings.

use crate::task::{
    drive::types::{DRIFT_COMPENSATION_GAIN, DRIFT_COMPENSATION_MAX, DRIFT_TOLERANCE_PERCENT},
    encoder_read::EncoderMeasurement,
};

/// Which track (left/right) we are referring to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Track {
    /// Left track
    Left,
    /// Right track
    Right,
}

/// Snapshot of encoder pulse counts for the current sampling window,
/// with computed per-track averages.
#[derive(Debug, Clone, Copy)]
pub struct TrackSpeedData {
    /// Raw pulse counts for each motor in the current sampling window.
    pub left_front: u16,
    /// Raw pulse counts for each motor in the current sampling window.
    pub left_rear: u16,
    /// Raw pulse counts for each motor in the current sampling window.
    pub right_front: u16,
    /// Raw pulse counts for each motor in the current sampling window.
    pub right_rear: u16,
    /// Computed average pulse count for the left track (average of left front and rear).
    pub left_track_avg: f32,
    /// Computed average pulse count for the right track (average of right front and rear).
    pub right_track_avg: f32,
    /// Timestamp of the measurement (copied from `EncoderMeasurement`).
    pub timestamp_ms: u64,
}

impl TrackSpeedData {
    /// True if all four motors reported zero pulses for this sample window.
    pub const fn all_zero(&self) -> bool {
        self.left_front == 0 && self.left_rear == 0 && self.right_front == 0 && self.right_rear == 0
    }

    /// True if any motor is zero while others are non-zero (possible failure / unplugged encoder).
    pub fn has_single_motor_zero_anomaly(&self) -> bool {
        let vals = [self.left_front, self.left_rear, self.right_front, self.right_rear];

        let any_nonzero = vals.iter().any(|&v| v != 0);
        let any_zero = vals.contains(&0);

        any_nonzero && any_zero
    }
}

/// Drift compensation decision to apply to current motor commands.
///
/// Note: Your plan prefers decreasing the faster track if you can't increase the slower one.
/// This enum supports both, but the policy is implemented by `determine_compensation()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CompensationAction {
    /// Within tolerance; do nothing.
    None,
    /// Increase left track by N (speed points).
    IncreaseLeft(i8),
    /// Increase right track by N (speed points).
    IncreaseRight(i8),
    /// Decrease left track by N (speed points).
    DecreaseLeft(i8),
    /// Decrease right track by N (speed points).
    DecreaseRight(i8),
}

/// Convert an `EncoderMeasurement` (which is assumed to represent pulses for the window)
/// into `TrackSpeedData` with per-track averages.
///
/// If you later need wraparound handling for cumulative counts, use `calculate_delta_u16()`
/// externally and feed deltas here instead.
pub fn calculate_track_averages(measurement: EncoderMeasurement) -> TrackSpeedData {
    let left_front = measurement.left_front;
    let left_rear = measurement.left_rear;
    let right_front = measurement.right_front;
    let right_rear = measurement.right_rear;

    let left_track_avg = f32::from(left_front + left_rear) / 2.0;
    let right_track_avg = f32::from(right_front + right_rear) / 2.0;

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

/// Percentage difference between left and right track speeds, using per-track averages.
///
/// Returned value is in percent, in range roughly [-100, 100] when one side is zero (but
/// we guard the divide-by-zero case).
///
/// Formula from plan:
/// `(left - right) / max(left, right) * 100.0`
///
/// Sign:
/// - Positive => left faster
/// - Negative => right faster
pub fn calculate_speed_difference(data: &TrackSpeedData) -> f32 {
    let left = data.left_track_avg;
    let right = data.right_track_avg;

    let denom = left.max(right);
    if denom <= 0.0 {
        // Both are zero (or effectively zero). No meaningful difference.
        return 0.0;
    }

    ((left - right) / denom) * 100.0
}

/// Determine what compensation should be applied based on the measured difference and
/// the *current* left/right speed commands (typically already-adjusted).
///
/// Policy:
/// - If within `DRIFT_TOLERANCE_PERCENT`, do nothing.
/// - Identify slow vs fast track from `diff_percent`.
/// - Compute adjustment magnitude: proportional to error, capped to `DRIFT_COMPENSATION_MAX`.
/// - Prefer increasing the slow track *if* it would not exceed +/-100 *and* won't cross zero.
/// - Otherwise decrease the fast track.
///
/// Reverse-driving symmetry:
/// - For negative speeds, "increase" means increasing the magnitude (more negative),
///   i.e. moving *away* from zero while preserving sign.
/// - Similarly, "decrease" means reducing magnitude (toward zero), preserving sign.
/// - This keeps compensation behavior symmetric between forward and reverse motion.
///
/// Important:
/// - Speed commands are assumed to be in [-100, 100].
pub fn determine_compensation(diff_percent: f32, left_speed: i8, right_speed: i8) -> CompensationAction {
    if diff_percent.abs() <= DRIFT_TOLERANCE_PERCENT {
        return CompensationAction::None;
    }

    // Determine which track is slower/faster based on sign convention.
    // diff > 0 => left faster => right is slow
    // diff < 0 => right faster => left is slow
    let (slow_track, fast_track) = if diff_percent > 0.0 {
        (Track::Right, Track::Left)
    } else {
        (Track::Left, Track::Right)
    };

    // Adjustment magnitude:
    // proportional gain (default 0.5 => 2% diff -> 1 speed point)
    // capped to DRIFT_COMPENSATION_MAX
    let raw = diff_percent.abs() * DRIFT_COMPENSATION_GAIN;
    // Avoid `f32::round()` to stay no_std-friendly (core-only). We want an integer step count.
    // `raw` is non-negative here, so "+ 0.5" is a round-to-nearest approximation.
    #[allow(clippy::cast_possible_truncation)]
    let mut adjustment = (raw + 0.5) as i8;

    // Ensure at least 1 step for non-trivial diffs, but never exceed max.
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

    // Prefer increasing slow side if it stays within bounds AND preserves direction (no sign flip).
    // For forward motion: +speed increases toward +100.
    // For reverse motion: -speed increases in magnitude toward -100.
    if can_increase_magnitude_within_bounds(slow_speed, adjustment) {
        match slow_track {
            Track::Left => CompensationAction::IncreaseLeft(adjustment),
            Track::Right => CompensationAction::IncreaseRight(adjustment),
        }
    } else {
        // Fall back to decreasing fast side (toward zero), also bounded.
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

/// 16-bit counter delta with wraparound handling.
///
/// This is useful if you have cumulative counters and store the previous sample.
///
/// Example:
/// - previous = 65530, current = 3 => delta = 9
pub const fn calculate_delta_u16(current: u16, previous: u16) -> u16 {
    if current >= previous {
        current - previous
    } else {
        (u16::MAX - previous).wrapping_add(current).wrapping_add(1)
    }
}

/// Apply a `CompensationAction` to current motor commands.
///
/// Returns `(new_left, new_right)` clamped to [-100, 100].
pub fn apply_compensation_action(action: CompensationAction, left_speed: i8, right_speed: i8) -> (i8, i8) {
    let (mut l, mut r) = (left_speed, right_speed);

    match action {
        CompensationAction::None => {}
        // Symmetric behavior: "Increase" increases magnitude while preserving sign
        CompensationAction::IncreaseLeft(adj) => {
            l = increase_magnitude_clamped(l, adj);
        }
        CompensationAction::IncreaseRight(adj) => {
            r = increase_magnitude_clamped(r, adj);
        }
        // Symmetric behavior: "Decrease" decreases magnitude (toward zero) while preserving sign
        CompensationAction::DecreaseLeft(adj) => {
            l = decrease_magnitude_clamped(l, adj);
        }
        CompensationAction::DecreaseRight(adj) => {
            r = decrease_magnitude_clamped(r, adj);
        }
    }

    (l, r)
}

/// Add `delta` to `value` while clamping to [-100, 100].
#[allow(clippy::cast_possible_truncation)]
fn add_clamped(value: i8, delta: i8) -> i8 {
    i16::from(value + delta).clamp(-100, 100) as i8
}

/// Subtract `delta` from `value` while clamping to [-100, 100].
#[allow(clippy::cast_possible_truncation)]
fn sub_clamped(value: i8, delta: i8) -> i8 {
    i16::from(value - delta).clamp(-100, 100) as i8
}

/// Increase speed magnitude by `delta` while preserving sign (symmetric forward/reverse).
///
/// Examples:
/// - value=+40, delta=5 => +45
/// - value=-40, delta=5 => -45
fn increase_magnitude_clamped(value: i8, delta: i8) -> i8 {
    if delta <= 0 {
        return value.clamp(-100, 100);
    }
    if value >= 0 {
        add_clamped(value, delta)
    } else {
        // More negative => larger magnitude
        sub_clamped(value, delta)
    }
}

/// Decrease speed magnitude by `delta` (toward zero) while preserving sign.
///
/// Examples:
/// - value=+40, delta=5 => +35
/// - value=-40, delta=5 => -35
fn decrease_magnitude_clamped(value: i8, delta: i8) -> i8 {
    if delta <= 0 {
        return value.clamp(-100, 100);
    }
    if value >= 0 {
        sub_clamped(value, delta)
    } else {
        // Less negative => closer to zero
        add_clamped(value, delta)
    }
}

/// True if increasing magnitude by `delta` is possible without exceeding limits
/// AND without crossing zero.
const fn can_increase_magnitude_within_bounds(value: i8, delta: i8) -> bool {
    if delta <= 0 {
        return true;
    }
    if value >= 0 {
        // Forward: value + delta <= 100
        (value as i16 + delta as i16) <= 100
    } else {
        // Reverse: value - delta >= -100
        (value as i16 - delta as i16) >= -100
    }
}

/// Clamp how much we can decrease magnitude of `value` by (move toward zero) without crossing zero.
///
/// For forward (+):
/// - we do value - delta, require >= 0 => delta <= value
///
/// For reverse (-):
/// - we do value + delta, require <= 0 => delta <= -value
#[allow(clippy::cast_possible_truncation)]
fn clamp_decrease_magnitude_amount_within_bounds(value: i8, desired_delta: i8) -> i8 {
    if desired_delta <= 0 {
        return 0;
    }
    let max_allowed = if value >= 0 {
        i16::from(value)
    } else {
        -(i16::from(value)).max(0)
    };
    i16::from(desired_delta).min(max_allowed).max(0) as i8
}
