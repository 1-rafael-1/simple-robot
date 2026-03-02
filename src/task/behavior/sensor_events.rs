//! Sensor event behavior handlers.
//!
//! Forwards sensor readings to the appropriate subsystems.

use crate::{
    system::state::{SYSTEM_STATE, UiMode},
    task::{
        drive,
        io::display,
        sensors::{encoders, imu},
    },
};

/// Handle encoder measurements.
pub fn handle_encoder_measurement(measurement: encoders::EncoderMeasurement) {
    // Forward encoder measurements to drive task for calibration and feedback control.
    // Store in shared mutex so drive task can read latest measurement on demand.
    // This ensures calibration always gets fresh data without channel overflow issues.
    let _ = drive::try_send_encoder_measurement(measurement);
}

/// Handle ultrasonic sensor readings.
pub async fn handle_ultrasonic_sweep_reading(distance: f64, angle: f32) {
    {
        let mut state = SYSTEM_STATE.lock().await;
        #[allow(clippy::cast_possible_truncation)]
        let distance_cm = distance as f32;
        state.ultrasonic_distance_cm = Some(distance_cm);
        state.ultrasonic_angle_deg = Some(angle);
    }

    // Forward sweep data to display for visualization unless IR/US test mode owns the screen.
    let ui_mode = {
        let ui = crate::task::ui::state::UI_STATE.lock().await;
        ui.mode
    };
    if !matches!(ui_mode, UiMode::RunningIrUltrasonicTest) {
        display::display_update(display::DisplayAction::ShowSweep(distance, angle)).await;
    }

    // TODO: Feed data to obstacle detection.
}

/// Handle IMU measurements.
pub fn handle_imu_measurement(measurement: imu::ImuMeasurement) {
    // Forward IMU measurements to drive task for rotation control.
    // Use try_send to avoid blocking orchestrator - IMU data arrives at 100Hz.
    // Dropping occasional measurements is acceptable at this rate.
    let _ = drive::try_send_imu_measurement(measurement);
}
