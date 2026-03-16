//! Brake/Coast settle control logic for the drive loop.

use embassy_time::{Duration, Instant};

use crate::task::{
    drive::{sensors::data::get_latest_encoder_measurement, types},
    sensors::encoders::EncoderMeasurement,
};

/// State for brake/coast settle detection.
#[derive(Debug, Clone, Copy)]
pub(super) struct BrakeCoastState {
    /// Deadline for encoder settle detection.
    deadline: Instant,
    /// Consecutive zero-delta samples observed.
    consecutive: u8,
    /// Previous encoder measurement for delta computation.
    last_measurement: Option<EncoderMeasurement>,
    /// Timestamp of the last processed encoder measurement (ms).
    last_timestamp_ms: u64,
}

impl BrakeCoastState {
    /// Create a new settle state with timeout applied.
    pub(super) fn new() -> Self {
        Self {
            deadline: Instant::now() + Duration::from_millis(types::BRAKE_COAST_SETTLE_TIMEOUT_MS),
            consecutive: 0,
            last_measurement: None,
            last_timestamp_ms: 0,
        }
    }
}

/// Outcome of one brake/coast settle control step.
pub(super) enum BrakeCoastStepResult {
    /// Still waiting for encoder settle.
    InProgress,
    /// Encoder settle criteria met.
    Completed,
    /// Encoder settle failed with a reason string.
    Failed(&'static str),
}

/// Run one brake/coast settle control step.
pub(super) async fn run_brake_coast_step(state: &mut BrakeCoastState) -> BrakeCoastStepResult {
    if Instant::now() >= state.deadline {
        return BrakeCoastStepResult::Failed("encoder settle timeout");
    }

    if let Some(measurement) = get_latest_encoder_measurement()
        .await
        .filter(|measurement| measurement.timestamp_ms != 0 && measurement.timestamp_ms != state.last_timestamp_ms)
    {
        state.last_timestamp_ms = measurement.timestamp_ms;

        if let Some(previous) = state.last_measurement {
            let delta_left = u32::from(measurement.left_front.wrapping_sub(previous.left_front))
                + u32::from(measurement.left_rear.wrapping_sub(previous.left_rear));
            let delta_right = u32::from(measurement.right_front.wrapping_sub(previous.right_front))
                + u32::from(measurement.right_rear.wrapping_sub(previous.right_rear));

            if delta_left == 0 && delta_right == 0 {
                state.consecutive = state.consecutive.saturating_add(1);
                if state.consecutive >= types::BRAKE_COAST_SETTLE_CONSECUTIVE_SAMPLES {
                    state.last_measurement = Some(measurement);
                    return BrakeCoastStepResult::Completed;
                }
            } else {
                state.consecutive = 0;
            }
        }

        state.last_measurement = Some(measurement);
    }

    BrakeCoastStepResult::InProgress
}
