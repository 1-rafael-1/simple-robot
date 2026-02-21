//! Encoder-based drift compensation loop for straight-line driving.
//!
//! This module owns the async control-loop step that monitors encoder deltas
//! during straight, constant-speed motion and applies small speed corrections
//! to keep both tracks running at equal speed.
//!
//! # Structure
//!
//! - [`run_drift_compensation_step`]: the async entry point called by the drive
//!   task's idle loop. It reads the latest encoder measurement, computes deltas,
//!   and decides whether to adjust motor speeds.
//! - [`math`]: pure, synchronous math functions and types used by the step above
//!   (track averages, speed difference, compensation decision, delta helpers).
//!
//! # When drift compensation is active
//!
//! Compensation is enabled only during straight-line `Differential` commands
//! where both tracks are commanded at equal (or near-equal) speed. It is
//! disabled during rotation, distance driving (which applies its own internal
//! compensation), braking, coasting, and standby.
//!
//! # Encoder delta handling
//!
//! The encoder task publishes cumulative hardware PWM counter values since the
//! last explicit reset. This module stores the previous measurement and computes
//! per-sample deltas with wraparound handling each tick via
//! [`math::calculate_delta_u16`].

pub(super) mod math;

use defmt::info;

use self::math as compensation;
use crate::{
    system::state::SYSTEM_STATE,
    task::{
        drive::{sensors::data as feedback, state::DriftCompensationState},
        motor_driver::{self, MotorCommand},
        sensors::encoders::{self as encoder_read, EncoderMeasurement},
    },
};

/// Run a single step of the drift compensation loop.
///
/// This is called by the drive task's idle loop at a modest tick rate
/// (see `drift_tick_ms` in `intent`). It is a no-op when:
/// - `drift.enabled` is `false`, or
/// - a rotation is in progress (detected via the passed `rotation_state`).
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
        // subtract the previous reading (with wraparound) to get pulses for the
        // current sampling window only.
        // First sample after start/reset: treat cumulative as delta (counters
        // were just reset at that point, so this is correct for the first window).
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

        // If nothing moved (very low speed or stopped), skip this sample.
        if data.all_zero() {
            return;
        }

        // If one encoder reads zero while others are non-zero, disable compensation
        // to avoid making corrections based on faulty data.
        if data.has_single_motor_zero_anomaly() {
            drift.enabled = false;
            encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
            return;
        }

        let diff_percent = compensation::calculate_speed_difference(&data);
        let action = compensation::determine_compensation(diff_percent, drift.adjusted_left, drift.adjusted_right);
        let (new_left, new_right) =
            compensation::apply_compensation_action(action, drift.adjusted_left, drift.adjusted_right);

        // Only send a motor update when something actually changes.
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
