//! Turns test mode task.
//!
//! Validates in-place rotation accuracy at a range of speeds.
//!
//! # Test Sequence
//!
//! 1. Load calibration from flash (if available)
//! 2. Wait 5 seconds
//! 3. Perform a series of in-place 90° turns at speeds: 40, 60, 80, 100
//!
//! During the run, the OLED shows per-turn telemetry:
//! - Line 0: "TURN TEST"
//! - Line 1: "TGT:xx.x ACT:xx.x"
//! - Line 2: "YAW:xxx.x"
//! - Line 3: "DEV: xx.x"
//!
//! IMPORTANT: The last turn's telemetry is left on the OLED after completion
//! to support inspection without a debugger.

use core::{
    fmt::write,
    sync::atomic::{AtomicBool, Ordering},
};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use heapless::String;

use super::{TestCommand, release_testmode, request_start};
use crate::{
    system::event::{Events, raise_event},
    task::{
        drive::{
            CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveQueueBuilder,
            types::{DriveQueueBuildError, RotationDirection, RotationMotion},
        },
        io::display::{DisplayAction, display_update},
        sensors::imu::{DmpFusionMode, set_dmp_fusion_mode},
    },
};

/// Tracks whether the turns test is currently active.
static TURNS_TEST_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Request the turns test to start.
pub async fn start_turns_test() {
    if TURNS_TEST_ACTIVE.swap(true, Ordering::Relaxed) {
        return;
    }

    if !request_start(TestCommand::Turns).await {
        TURNS_TEST_ACTIVE.store(false, Ordering::Relaxed);
    }
}

/// Spawn the turns test task via the controller.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner) {
    spawner.spawn(turns_test_task().unwrap());
}

#[embassy_executor::task]
async fn turns_test_task() {
    run_turns_test().await;
    release_testmode();
    TURNS_TEST_ACTIVE.store(false, Ordering::Relaxed);
    raise_event(Events::TestingCompleted).await;
}

/// Run the in-place turns test.
#[allow(clippy::too_many_lines)]
async fn run_turns_test() {
    async fn show_line(line: u8, msg: &str) {
        let mut s: String<20> = String::new();
        let _ = s.push_str(msg);
        display_update(DisplayAction::ShowText(s, line)).await;
    }

    async fn show_turn_status(target_deg: f32, current_deg: f32, yaw_deg: f32, deviation_deg: f32) {
        show_line(0, "TURN TEST").await;

        {
            let mut s: String<20> = String::new();
            let _ = write(&mut s, format_args!("TGT:{target_deg:>4.1} ACT:{current_deg:>4.1}"));
            display_update(DisplayAction::ShowText(s, 1)).await;
        }

        {
            let mut s: String<20> = String::new();
            let _ = write(&mut s, format_args!("YAW:{yaw_deg:>6.1}"));
            display_update(DisplayAction::ShowText(s, 2)).await;
        }

        {
            let mut s: String<20> = String::new();
            let _ = write(&mut s, format_args!("DEV:{deviation_deg:>6.1}"));
            display_update(DisplayAction::ShowText(s, 3)).await;
        }
    }

    fn build_turn_queue(target_deg: f32, test_speeds: &[u8]) -> Result<DriveQueueBuilder, DriveQueueBuildError> {
        let mut queue = DriveQueueBuilder::new();

        for speed in test_speeds {
            queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::RotateExact {
                degrees: target_deg,
                direction: RotationDirection::Clockwise,
                motion: RotationMotion::Stationary { speed: *speed },
            }))?;
            queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::Coast))?;
        }

        queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::Idle { duration_ms: 1000 }))?;

        Ok(queue)
    }

    // Clear display at start of test.
    display_update(DisplayAction::Clear).await;
    show_line(0, "TURN TEST").await;
    show_line(1, "Initializing...").await;
    show_line(2, "").await;
    show_line(3, "").await;

    // Send initialization event to orchestrator (only if not already initialized).
    Timer::after(Duration::from_millis(100)).await;
    if !crate::task::ui::ui_initialized().await {
        raise_event(Events::Initialize).await;
    }

    // Force 6-axis fusion (gyro + accel) to avoid magnetometer yaw issues.
    defmt::info!("🧪 TURNS: Setting IMU DMP fusion mode to Axis6");
    set_dmp_fusion_mode(DmpFusionMode::Axis6);
    show_line(1, "Fusion: Axis6").await;
    Timer::after(Duration::from_millis(250)).await;

    // Countdown before driving.
    defmt::info!("🧪 TURNS: Waiting 10 seconds before driving...");
    show_line(1, "Starting in 5s").await;
    Timer::after(Duration::from_secs(5)).await;

    // Turn in place: 90°, varying rotation speeds.
    let target_deg: f32 = 90.0;
    let test_speeds: [u8; 4] = [40, 60, 80, 100];

    for speed in test_speeds {
        defmt::info!("🧪 TURNS: Queue turn 90° at speed {=u8}", speed);

        show_line(0, "TURN TEST").await;
        {
            let mut s: String<20> = String::new();
            let _ = write(&mut s, format_args!("SPD:{speed:>3} TGT:{target_deg:>4.1}"));
            display_update(DisplayAction::ShowText(s, 1)).await;
        }
        show_line(2, "Queueing...").await;
        show_line(3, "").await;
    }

    let Ok(turn_queue) = build_turn_queue(target_deg, &test_speeds) else {
        defmt::warn!("🧪 TURNS: turn queue full");
        return;
    };

    let Ok(turn_completion) = turn_queue.submit().await else {
        defmt::warn!("🧪 TURNS: turn queue busy");
        return;
    };

    if let Some(completion) = turn_completion.last_step_completion {
        let (final_yaw_deg, angle_error_deg, status) = if let CompletionTelemetry::RotateExact {
            final_yaw_deg,
            angle_error_deg,
            ..
        } = completion.telemetry
        {
            (final_yaw_deg, angle_error_deg, completion.status)
        } else {
            defmt::warn!("🧪 TURNS: Unexpected completion telemetry");
            (0.0, 0.0, completion.status)
        };

        // error = achieved - target, so achieved = target + error.
        let achieved_deg = target_deg + angle_error_deg;

        show_turn_status(target_deg, achieved_deg, final_yaw_deg, angle_error_deg).await;

        let status_str = match status {
            CompletionStatus::Success => "Success",
            CompletionStatus::Cancelled => "Cancelled",
            CompletionStatus::Failed(_) => "Failed",
        };
        defmt::info!(
            "🧪 TURNS: batch complete: status={=str}, final_yaw={=f32}°, angle_error={=f32}°",
            status_str,
            final_yaw_deg,
            angle_error_deg
        );

        if let CompletionStatus::Failed(reason) = status {
            defmt::warn!("🧪 TURNS: batch failed: {=str}", reason);
        }
    }

    // Leave the last telemetry on the OLED for inspection.
    defmt::info!("🧪 TURNS: complete (OLED showing last telemetry)");
}
