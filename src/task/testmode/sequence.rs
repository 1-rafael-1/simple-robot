//! Development testing sequence
//!
//! This module contains the on-demand test sequence used for development and debugging.
//! It is spawned only when requested and exits when the sequence completes.
//!
//! # Current Test Sequence
//!
//! 1. Load calibration from flash (if available)
//! 2. Wait 10 seconds
//! 3. Perform a series of in-place turns at different speeds
//! 4. Drive forward 50 cm (straight, encoder-based)
//! 5. Drive backward 50 cm (straight, encoder-based)
//! 6. Drive a 360° circle at 1m radius (curve arc)
//!
//! During each turn, we update the OLED with telemetry:
//! - Line 0: "TURN TEST"
//! - Line 1: "TGT:xx.x ACT:xx.x"
//! - Line 2: "YAW:xxx.x"
//! - Line 3: "DEV: xx.x"
//!
//! IMPORTANT: We do NOT overwrite the OLED at the end of the sequence.
//! The last turn's telemetry remains visible to support testing without a debugger attached.

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
            CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind,
            DriveQueueBuilder, TurnDirection,
            types::{DriveQueueBuildError, RotationDirection, RotationMotion},
        },
        io::display::{DisplayAction, display_update},
        sensors::imu::{AhrsFusionMode, set_ahrs_fusion_mode},
    },
};

/// Tracks whether the test sequence is currently active.
static TEST_SEQUENCE_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Request the test sequence to start.
pub async fn start_testing_sequence() {
    if TEST_SEQUENCE_ACTIVE.swap(true, Ordering::Relaxed) {
        return;
    }

    if !request_start(TestCommand::Sequence).await {
        TEST_SEQUENCE_ACTIVE.store(false, Ordering::Relaxed);
    }
}

/// Spawn the test sequence task via the controller.
pub(super) fn spawn(spawner: Spawner) {
    spawner.must_spawn(testing_sequence_task());
}

#[embassy_executor::task]
async fn testing_sequence_task() {
    run_testing_sequence().await;
    release_testmode();
    TEST_SEQUENCE_ACTIVE.store(false, Ordering::Relaxed);
    raise_event(Events::TestingCompleted).await;
}

/// Main testing sequence.
///
/// This is the entry point for all development tests. Modify as needed.
#[allow(clippy::too_many_lines)]
async fn run_testing_sequence() {
    async fn show_line(line: u8, msg: &str) {
        let mut s: String<20> = String::new();
        // Truncate to 20 chars to match the display action contract.
        let _ = s.push_str(msg);
        display_update(DisplayAction::ShowText(s, line)).await;
    }

    async fn show_turn_status(target_deg: f32, current_deg: f32, yaw_deg: f32, deviation_deg: f32) {
        show_line(0, "TURN TEST").await;

        {
            let mut s: String<20> = String::new();
            // Example: "TGT:45.0 ACT:12.3"
            let _ = write(&mut s, format_args!("TGT:{target_deg:>4.1} ACT:{current_deg:>4.1}"));
            display_update(DisplayAction::ShowText(s, 1)).await;
        }

        {
            let mut s: String<20> = String::new();
            // Example: "YAW:-123.4"
            let _ = write(&mut s, format_args!("YAW:{yaw_deg:>6.1}"));
            display_update(DisplayAction::ShowText(s, 2)).await;
        }

        {
            let mut s: String<20> = String::new();
            // Example: "DEV:  -1.6"
            let _ = write(&mut s, format_args!("DEV:{deviation_deg:>6.1}"));
            display_update(DisplayAction::ShowText(s, 3)).await;
        }
    }

    fn build_turn_queue(target_deg: f32, test_speeds: &[u8]) -> Result<DriveQueueBuilder, DriveQueueBuildError> {
        let mut queue = DriveQueueBuilder::new();

        for speed in test_speeds {
            queue.push(DriveCommand::Drive(DriveAction::RotateExact {
                degrees: target_deg,
                direction: RotationDirection::Clockwise,
                motion: RotationMotion::Stationary { speed: *speed },
            }))?;
            queue.push(DriveCommand::Drive(DriveAction::Coast))?;
        }

        queue.push(DriveCommand::Drive(DriveAction::Idle { duration_ms: 1000 }))?;

        Ok(queue)
    }

    fn build_distance_queue(circle_arc_cm: f32) -> Result<DriveQueueBuilder, DriveQueueBuildError> {
        let mut queue = DriveQueueBuilder::new();

        queue.push(DriveCommand::Drive(DriveAction::DriveDistance {
            kind: DriveDistanceKind::Straight { distance_cm: 50.0 },
            direction: DriveDirection::Forward,
            speed: 70,
        }))?;

        queue.push(DriveCommand::Drive(DriveAction::DriveDistance {
            kind: DriveDistanceKind::Straight { distance_cm: 50.0 },
            direction: DriveDirection::Backward,
            speed: 70,
        }))?;

        queue.push(DriveCommand::Drive(DriveAction::DriveDistance {
            kind: DriveDistanceKind::CurveArc {
                radius_cm: 100.0,
                arc_length_cm: circle_arc_cm,
                direction: TurnDirection::Left,
            },
            direction: DriveDirection::Forward,
            speed: 60,
        }))?;

        Ok(queue)
    }

    // Clear display at start of test.
    display_update(DisplayAction::Clear).await;
    show_line(0, "TURN TEST").await;
    show_line(1, "Initializing...").await;
    show_line(2, "").await;
    show_line(3, "").await;

    // Send initialization event to orchestrator (only if not already initialized)
    Timer::after(Duration::from_millis(100)).await;
    if !crate::task::ui::ui_initialized().await {
        raise_event(Events::Initialize).await;
    }

    // Wait for system to stabilize and for orchestrator to load calibration from flash
    // (orchestrator automatically loads calibration on Initialize event)
    defmt::info!("🧪 TEST: Waiting for system initialization and calibration loading...");
    Timer::after(Duration::from_secs(3)).await;

    // Force 6-axis fusion (gyro + accel) for turn testing.
    // This avoids magnetometer-related yaw issues while validating the control flow.
    defmt::info!("🧪 TEST: Setting IMU AHRS fusion mode to Axis6");
    set_ahrs_fusion_mode(AhrsFusionMode::Axis6);
    show_line(1, "Fusion: Axis6").await;
    Timer::after(Duration::from_millis(250)).await;

    // Wait before driving
    defmt::info!("🧪 TEST: Waiting 10 seconds before driving...");
    show_line(1, "Starting in 10s").await;
    Timer::after(Duration::from_secs(10)).await;

    // Turn in place: 90 degrees, varying rotation speeds.
    let target_deg: f32 = 90.0;
    let test_speeds: [u8; 4] = [40, 60, 80, 100];

    for speed in test_speeds {
        defmt::info!("🧪 TEST: Queue turn 90° at speed {}", speed);

        // Pre-turn display (queueing only; completion reported after batch)
        show_line(0, "TURN TEST").await;
        {
            let mut s: String<20> = String::new();
            let _ = write(&mut s, format_args!("SPD:{speed:>3} TGT:{target_deg:>4.1}"));
            display_update(DisplayAction::ShowText(s, 1)).await;
        }
        show_line(2, "Queueing...").await;
        show_line(3, "").await;
    }

    // make the queue of turns
    let Ok(turn_queue) = build_turn_queue(target_deg, &test_speeds) else {
        defmt::warn!("🧪 TEST: turn queue full");
        return;
    };

    // and submit it for execution
    let Ok(turn_completion) = turn_queue.submit().await else {
        defmt::warn!("🧪 TEST: turn queue busy");
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
            defmt::warn!("🧪 TEST: Unexpected completion telemetry");
            (0.0, 0.0, completion.status)
        };

        // The controller reports normalized error:
        // error = achieved - target, so achieved = target + error.
        let achieved_deg = target_deg + angle_error_deg;

        // Update display with the final telemetry for the last queued turn.
        show_turn_status(target_deg, achieved_deg, final_yaw_deg, angle_error_deg).await;

        let status_str = match status {
            CompletionStatus::Success => "Success",
            CompletionStatus::Cancelled => "Cancelled",
            CompletionStatus::Failed(_) => "Failed",
        };
        defmt::info!(
            "🧪 TEST: Turn batch complete: status={=str}, final_yaw={=f32}°, angle_error={=f32}°",
            status_str,
            final_yaw_deg,
            angle_error_deg
        );

        if let CompletionStatus::Failed(reason) = status {
            defmt::warn!("🧪 TEST: Turn batch failed: {=str}", reason);
        }
    }

    // Straight distance runs after turns (encoder-based).
    show_line(0, "DIST TEST").await;
    show_line(1, "FWD/REV/ARC").await;
    show_line(2, "Queueing...").await;
    show_line(3, "").await;

    // Full circle curve test: 1m radius, 360° arc (centerline).
    defmt::info!("🧪 TEST: Curve circle 360° at radius 1m");
    let circle_arc_cm = 2.0 * core::f32::consts::PI * 100.0;

    let Ok(distance_queue) = build_distance_queue(circle_arc_cm) else {
        defmt::warn!("🧪 TEST: distance queue full");
        return;
    };

    let Ok(distance_completion) = distance_queue.submit().await else {
        defmt::warn!("🧪 TEST: distance queue busy");
        return;
    };

    if let Some(completion) = distance_completion.last_step_completion
        && let CompletionTelemetry::DriveDistance {
            achieved_left_revs,
            achieved_right_revs,
            ..
        } = completion.telemetry
    {
        let status_str = match completion.status {
            CompletionStatus::Success => "Success",
            CompletionStatus::Cancelled => "Cancelled",
            CompletionStatus::Failed(_) => "Failed",
        };
        defmt::info!(
            "🧪 TEST: Distance batch complete: status={=str} left={=f32} right={=f32}",
            status_str,
            achieved_left_revs,
            achieved_right_revs
        );
    }

    // Do not overwrite the OLED here; keep the last test's telemetry visible.
    defmt::info!("🧪 TEST: Sequence complete (OLED left showing last test telemetry)");
}
