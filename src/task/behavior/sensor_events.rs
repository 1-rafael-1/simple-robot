//! Sensor event behavior handlers.
//!
//! Forwards sensor readings to the appropriate subsystems.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_time::Instant;
use heapless::String;

use crate::{
    system::{event::UltrasonicReading, state::perception},
    task::{
        drive,
        io::display,
        sensors::{encoders, imu},
        ui::{self, state::UiMode},
    },
};

/// Throttle UI refresh while autonomous mode is running (ms).
const AUTONOMOUS_UI_REFRESH_INTERVAL_MS: u32 = 500;
/// Last timestamp (ms) when an autonomous UI refresh was emitted.
static LAST_AUTONOMOUS_REFRESH_MS: AtomicU32 = AtomicU32::new(0);

/// Handle encoder measurements.
pub fn handle_encoder_measurement(measurement: encoders::EncoderMeasurement) {
    // Forward encoder measurements to drive task for calibration and feedback control.
    // Store in shared mutex so drive task can read latest measurement on demand.
    // This ensures calibration always gets fresh data without channel overflow issues.
    let _ = drive::try_send_encoder_measurement(measurement);
}

/// Handle ultrasonic sensor readings.
#[allow(clippy::cast_possible_truncation)]
pub async fn handle_ultrasonic_sweep_reading(reading: UltrasonicReading, angle: f32) {
    {
        let mut state = perception::PERCEPTION_STATE.lock().await;
        state.ultrasonic_reading = Some(reading);
        state.ultrasonic_angle_deg = Some(angle);
    }

    // Forward sweep data to the display only while the sweep test owns the screen.
    let ui_mode = {
        let ui = crate::task::ui::state::UI_STATE.lock().await;
        ui.mode
    };
    if matches!(ui_mode, UiMode::RunningUltrasonicSweepTest) {
        let mut header: String<20> = String::new();
        match reading {
            UltrasonicReading::Distance(distance) => {
                let _ = core::fmt::write(&mut header, format_args!("US:{distance:>5.1} A:{angle:>4.1}"));
                display::display_update(display::DisplayAction::ShowSweep(Some(distance), angle)).await;
            }
            UltrasonicReading::Timeout => {
                let _ = core::fmt::write(&mut header, format_args!("US:timeout A:{angle:>4.1}"));
                display::display_update(display::DisplayAction::ShowSweep(None, angle)).await;
            }
            UltrasonicReading::Error => {
                let _ = core::fmt::write(&mut header, format_args!("US:error A:{angle:>4.1}"));
            }
        }
        display::display_update(display::DisplayAction::ShowText(header, 0)).await;
    }

    if matches!(ui_mode, UiMode::RunningAutonomous { .. }) {
        // Use a wrapping 32-bit millisecond counter so long uptimes don't break the throttle.
        let now_ms = Instant::now().as_millis() as u32;
        let last_ms = LAST_AUTONOMOUS_REFRESH_MS.load(Ordering::Relaxed);
        if now_ms.wrapping_sub(last_ms) >= AUTONOMOUS_UI_REFRESH_INTERVAL_MS {
            LAST_AUTONOMOUS_REFRESH_MS.store(now_ms, Ordering::Relaxed);
            ui::refresh().await;
        }
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
