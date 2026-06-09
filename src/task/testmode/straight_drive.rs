//! Straight drive test mode task.
//!
//! Validates encoder-based straight-line distance driving.
//!
//! # Test Sequence
//!
//! 1. Load calibration from flash (if available)
//! 2. Wait 10 seconds
//! 3. Drive forward 50 cm at speed 70
//! 4. Drive backward 50 cm at speed 70
//!
//! Completion telemetry (achieved left/right revs and status) is logged
//! after the queue drains.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use heapless::String;

use super::{TestCommand, release_testmode, request_start};
use crate::{
    system::event::{Events, raise_event},
    task::{
        drive::{
            CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind,
            DriveQueueBuilder, types::DriveQueueBuildError,
        },
        io::display::{DisplayAction, display_update},
        sensors::imu::{DmpFusionMode, set_dmp_fusion_mode},
    },
};

/// Tracks whether the straight drive test is currently active.
static STRAIGHT_DRIVE_TEST_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Request the straight drive test to start.
pub async fn start_straight_drive_test() {
    if STRAIGHT_DRIVE_TEST_ACTIVE.swap(true, Ordering::Relaxed) {
        return;
    }

    if !request_start(TestCommand::StraightDrive).await {
        STRAIGHT_DRIVE_TEST_ACTIVE.store(false, Ordering::Relaxed);
    }
}

/// Spawn the straight drive test task via the controller.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner) {
    spawner.spawn(straight_drive_test_task().unwrap());
}

#[embassy_executor::task]
async fn straight_drive_test_task() {
    run_straight_drive_test().await;
    release_testmode();
    STRAIGHT_DRIVE_TEST_ACTIVE.store(false, Ordering::Relaxed);
    raise_event(Events::TestingCompleted).await;
}

/// Run the straight-line distance test.
#[allow(clippy::too_many_lines)]
async fn run_straight_drive_test() {
    async fn show_line(line: u8, msg: &str) {
        let mut s: String<20> = String::new();
        let _ = s.push_str(msg);
        display_update(DisplayAction::ShowText(s, line)).await;
    }

    fn build_straight_queue() -> Result<DriveQueueBuilder, DriveQueueBuildError> {
        let mut queue = DriveQueueBuilder::new();

        queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::DriveDistance {
            kind: DriveDistanceKind::Straight { distance_cm: 150.0 },
            direction: DriveDirection::Forward,
            speed: 70,
        }))?;

        queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::Brake))?;

        queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::Idle { duration_ms: 500 }))?;

        queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::DriveDistance {
            kind: DriveDistanceKind::Straight { distance_cm: 150.0 },
            direction: DriveDirection::Backward,
            speed: 70,
        }))?;

        queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::Brake))?;

        queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::Idle { duration_ms: 500 }))?;

        queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::Coast))?;

        Ok(queue)
    }

    // Clear display at start of test.
    display_update(DisplayAction::Clear).await;
    show_line(0, "DIST TEST").await;
    show_line(1, "Initializing...").await;
    show_line(2, "").await;
    show_line(3, "").await;

    // Send initialization event to orchestrator (only if not already initialized).
    Timer::after(Duration::from_millis(100)).await;
    if !crate::task::ui::ui_initialized().await {
        raise_event(Events::Initialize).await;
    }

    // Wait for system to stabilize and calibration to load.
    defmt::info!("🧪 DIST: Waiting for system initialization and calibration loading...");
    Timer::after(Duration::from_secs(3)).await;

    // Force 6-axis fusion (gyro + accel).
    defmt::info!("🧪 DIST: Setting IMU DMP fusion mode to Axis6");
    set_dmp_fusion_mode(DmpFusionMode::Axis6);
    show_line(1, "Fusion: Axis6").await;
    Timer::after(Duration::from_millis(250)).await;

    // Countdown before driving.
    let wait_s = 5u64;
    defmt::info!("🧪 DIST: Waiting {} seconds before driving...", wait_s);
    show_line(1, "Starting in a few s").await;
    Timer::after(Duration::from_secs(wait_s)).await;

    // Queue forward + backward 50 cm.
    show_line(0, "DIST TEST").await;
    show_line(1, "FWD then REV").await;
    show_line(2, "Queueing...").await;
    show_line(3, "").await;

    let Ok(queue) = build_straight_queue() else {
        defmt::warn!("🧪 DIST: queue full");
        return;
    };

    let Ok(completion) = queue.submit().await else {
        defmt::warn!("🧪 DIST: queue busy");
        return;
    };

    if let Some(step) = completion.last_step_completion
        && let CompletionTelemetry::DriveDistance {
            achieved_left_revs,
            achieved_right_revs,
            ..
        } = step.telemetry
    {
        let status_str = match step.status {
            CompletionStatus::Success => "Success",
            CompletionStatus::Cancelled => "Cancelled",
            CompletionStatus::Failed(_) => "Failed",
        };
        defmt::info!(
            "🧪 DIST: complete: status={=str} left={=f32} right={=f32}",
            status_str,
            achieved_left_revs,
            achieved_right_revs
        );
    }

    defmt::info!("🧪 DIST: Straight drive test complete");
}
