//! Differential drive speed management — fire-and-forget with optional drift correction.
//!
//! Called by the dispatch for `DriveAction::Differential` commands. Applies
//! drift-compensation math for symmetric (straight-line) commands and passes
//! speeds through unchanged for differential/turning commands.

use crate::task::drive::drift_math;

/// Apply drift-compensated speeds for a differential command.
///
/// For symmetric commands (|left - right| ≤ 2, at least one non-zero), reads
/// the latest encoder measurement and applies drift math to equalize track
/// speeds. For asymmetric commands, returns the speeds unchanged.
///
/// Returns (`adjusted_left`, `adjusted_right`) clamped to [-100, 100].
pub(super) async fn set_speeds(left: i8, right: i8) -> (i8, i8) {
    let left = left.clamp(-100, 100);
    let right = right.clamp(-100, 100);

    // Only apply drift math for symmetric (straight-line) commands.
    if (left - right).abs() > 2 || left == 0 {
        return (left, right);
    }

    // Read latest encoder data if available.
    let Some(measurement) = super::sensors::data::get_latest_encoder_measurement().await else {
        return (left, right);
    };

    if measurement.timestamp_ms == 0 {
        return (left, right);
    }

    // Compute per-track averages and check for anomalies.
    let data = drift_math::calculate_track_averages(measurement);
    if data.all_zero() || data.has_single_motor_zero_anomaly() {
        return (left, right);
    }

    let diff_percent = drift_math::calculate_speed_difference(&data);
    let action = drift_math::determine_compensation(diff_percent, left, right);
    drift_math::apply_compensation_action(action, left, right)
}
