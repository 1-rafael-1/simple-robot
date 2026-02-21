//! Drift compensation loop for straight-line driving.
//!
//! This module monitors encoder deltas during straight, constant-speed motion
//! and applies small speed corrections to reduce track drift. It is disabled
//! during rotation or when encoder anomalies are detected.

use defmt::info;

use crate::{
    system::state::SYSTEM_STATE,
    task::{
        drive::{compensation, feedback, state::DriftCompensationState},
        motor_driver::{self, MotorCommand},
        sensors::encoders::{self as encoder_read, EncoderMeasurement},
    },
};

/// Run a single step of the drift compensation loop.
pub(super) async fn run_drift_compensation_step(
    drift: &mut DriftCompensationState,
    rotation_state: Option<&crate::task::drive::rotation::RotationState>,
) {
    if !drift.enabled || rotation_state.is_some() {
        return;
    }

    if let Some(measurement) = feedback::get_latest_encoder_measurement().await {
        // Only process each encoder sample once.
        if measurement.timestamp_ms == 0 || measurement.timestamp_ms == drift.last_encoder_timestamp_ms {
            return;
        }
        drift.last_encoder_timestamp_ms = measurement.timestamp_ms;

        // Compute per-sample deltas from cumulative hardware counters.
        // The encoder task publishes cumulative counts since last reset, so we
        // subtract the previous reading (with wraparound handling) to get pulses
        // for the current sampling window only.
        // First sample after start/reset: treat cumulative as delta (counters
        // were just reset, so this is correct for the first window).
        let delta_measurement = drift
            .last_encoder_measurement
            .map_or(measurement, |prev| EncoderMeasurement {
                left_front: compensation::calculate_delta_u16(measurement.left_front, prev.left_front),
                left_rear: compensation::calculate_delta_u16(measurement.left_rear, prev.left_rear),
                right_front: compensation::calculate_delta_u16(measurement.right_front, prev.right_front),
                right_rear: compensation::calculate_delta_u16(measurement.right_rear, prev.right_rear),
                timestamp_ms: measurement.timestamp_ms,
            });
        drift.last_encoder_measurement = Some(measurement);

        let data = compensation::calculate_track_averages(delta_measurement);

        // If nothing moved (very low speed or stopped), ignore the sample.
        if data.all_zero() {
            return;
        }

        // If we detect an anomaly (e.g., one encoder stuck at zero), disable compensation.
        if data.has_single_motor_zero_anomaly() {
            drift.enabled = false;
            encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
            return;
        }

        let diff_percent = compensation::calculate_speed_difference(&data);

        let action = compensation::determine_compensation(diff_percent, drift.adjusted_left, drift.adjusted_right);

        let (new_left, new_right) =
            compensation::apply_compensation_action(action, drift.adjusted_left, drift.adjusted_right);

        // Only send motor update if anything actually changes.
        if new_left != drift.adjusted_left || new_right != drift.adjusted_right {
            let prev_left = drift.adjusted_left;
            let prev_right = drift.adjusted_right;

            drift.adjusted_left = new_left;
            drift.adjusted_right = new_right;
            drift.last_applied_ms = data.timestamp_ms;

            info!(
                "drift_comp: diff={}% action={:?} speeds L:{}->{} R:{}->{} (ctrs lf:{} lr:{} rf:{} rr:{})",
                diff_percent,
                action,
                prev_left,
                drift.adjusted_left,
                prev_right,
                drift.adjusted_right,
                data.left_front,
                data.left_rear,
                data.right_front,
                data.right_rear
            );

            motor_driver::send_motor_command(MotorCommand::SetTracks {
                left_speed: drift.adjusted_left,
                right_speed: drift.adjusted_right,
            })
            .await;

            let mut state = SYSTEM_STATE.lock().await;
            state.left_track_speed = drift.adjusted_left;
            state.right_track_speed = drift.adjusted_right;
        }
    }
}
