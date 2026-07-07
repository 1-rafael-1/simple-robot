//! Brake/Coast settle control logic.
//!
//! Handles the `Brake` and `Coast` drive actions. After issuing the motor
//! command (brake or freewheel), the control loop waits for encoder readings
//! to settle — N consecutive zero-delta samples within a timeout window.
//!
//! # Lifecycle
//!
//! - `init()` creates the settle state, starts encoder sampling, and returns
//!   an `ActiveIntent`. The dispatch stores the intent directly.
//! - `run_brake_coast_step()` checks for settle or timeout each tick.
//! - Completion teardown uses an `IntentTeardown::EncoderSettle` descriptor to
//!   stop encoder sampling.
//!
//! # Constants
//!
//! - `SETTLE_INTERVAL_MS` (100 ms): time between settle checks
//! - `SETTLE_CONSECUTIVE_SAMPLES` (3): consecutive zero-delta readings to declare settled
//! - `SETTLE_TIMEOUT_MS` (2000 ms): maximum time before failing

use embassy_time::{Duration, Instant};

use crate::task::{
    drive::sensors::{control as lifecycle, data::get_latest_encoder_measurement},
    sensors::encoders::EncoderMeasurement,
};

/// Encoder settle interval for brake/coast completion (milliseconds).
pub(super) const SETTLE_INTERVAL_MS: u64 = 100;
/// Consecutive zero-delta samples required to declare settled.
const SETTLE_CONSECUTIVE_SAMPLES: u8 = 3;
/// Maximum time allowed to settle before failing completion (milliseconds).
const SETTLE_TIMEOUT_MS: u64 = 2000;

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
    /// Initialise a brake/coast settle intent.
    ///
    /// Starts encoder sampling and returns the `ActiveIntent`.
    pub(super) async fn init(completion_requested: bool) -> super::state::ActiveIntent {
        lifecycle::start_encoder_sampling(SETTLE_INTERVAL_MS, true).await;

        let state = Self::new();
        super::state::ActiveIntent::BrakeCoast {
            state,
            completion_requested,
        }
    }

    /// Create a new settle state with timeout applied.
    pub(super) fn new() -> Self {
        Self {
            deadline: Instant::now() + Duration::from_millis(SETTLE_TIMEOUT_MS),
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
                if state.consecutive >= SETTLE_CONSECUTIVE_SAMPLES {
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
