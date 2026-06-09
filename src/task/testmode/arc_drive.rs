//! Arc drive test mode task.
//!
//! Validates curve arc driving by executing a 360° circle at 1 m radius.
//!
//! # Test Sequence
//!
//! 1. Load calibration from flash (if available)
//! 2. Wait 10 seconds
//! 3. Drive a 360° left arc at radius 100 cm, speed 60, forward direction
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
            DriveQueueBuilder, TurnDirection, types::DriveQueueBuildError,
        },
        io::display::{DisplayAction, display_update},
        sensors::imu::{DmpFusionMode, set_dmp_fusion_mode},
    },
};

/// Tracks whether the arc drive test is currently active.
static ARC_DRIVE_TEST_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Request the arc drive test to start.
pub async fn start_arc_drive_test() {
    if ARC_DRIVE_TEST_ACTIVE.swap(true, Ordering::Relaxed) {
        return;
    }

    if !request_start(TestCommand::ArcDrive).await {
        ARC_DRIVE_TEST_ACTIVE.store(false, Ordering::Relaxed);
    }
}

/// Spawn the arc drive test task via the controller.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner) {
    spawner.spawn(arc_drive_test_task().unwrap());
}

#[embassy_executor::task]
async fn arc_drive_test_task() {
    run_arc_drive_test().await;
    release_testmode();
    ARC_DRIVE_TEST_ACTIVE.store(false, Ordering::Relaxed);
    raise_event(Events::TestingCompleted).await;
}

/// Run the arc drive test.
#[allow(clippy::too_many_lines)]
async fn run_arc_drive_test() {
    async fn show_line(line: u8, msg: &str) {
        let mut s: String<20> = String::new();
        let _ = s.push_str(msg);
        display_update(DisplayAction::ShowText(s, line)).await;
    }

    fn build_arc_queue() -> Result<DriveQueueBuilder, DriveQueueBuildError> {
        let mut queue = DriveQueueBuilder::new();

        // 360° circle: arc length = 2π × radius.
        let circle_arc_cm = 2.0 * core::f32::consts::PI * 100.0;

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
    show_line(0, "ARC TEST").await;
    show_line(1, "Initializing...").await;
    show_line(2, "").await;
    show_line(3, "").await;

    // Send initialization event to orchestrator (only if not already initialized).
    Timer::after(Duration::from_millis(100)).await;
    if !crate::task::ui::ui_initialized().await {
        raise_event(Events::Initialize).await;
    }

    // Wait for system to stabilize and calibration to load.
    defmt::info!("🧪 ARC: Waiting for system initialization and calibration loading...");
    Timer::after(Duration::from_secs(3)).await;

    // Force 6-axis fusion (gyro + accel).
    defmt::info!("🧪 ARC: Setting IMU DMP fusion mode to Axis6");
    set_dmp_fusion_mode(DmpFusionMode::Axis6);
    show_line(1, "Fusion: Axis6").await;
    Timer::after(Duration::from_millis(250)).await;

    // Countdown before driving.
    defmt::info!("🧪 ARC: Waiting 10 seconds before driving...");
    show_line(1, "Starting in 10s").await;
    Timer::after(Duration::from_secs(10)).await;

    // Queue the 360° arc.
    show_line(0, "ARC TEST").await;
    show_line(1, "360 deg, r=100cm").await;
    show_line(2, "Queueing...").await;
    show_line(3, "").await;

    defmt::info!("🧪 ARC: Curve circle 360° at radius 1m");

    let Ok(queue) = build_arc_queue() else {
        defmt::warn!("🧪 ARC: queue full");
        return;
    };

    let Ok(completion) = queue.submit().await else {
        defmt::warn!("🧪 ARC: queue busy");
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
            "🧪 ARC: complete: status={=str} left={=f32} right={=f32}",
            status_str,
            achieved_left_revs,
            achieved_right_revs
        );
    }

    defmt::info!("🧪 ARC: Arc drive test complete");
}
