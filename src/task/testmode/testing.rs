//! Development testing task
//!
//! This module contains temporary test sequences for development and debugging.
//! These are meant to be modified frequently during development and should not
//! be considered part of the production codebase.
//!
//! # Current Test Sequence
//!
//! 1. Load calibration from flash (if available)
//! 2. Wait 10 seconds
//! 3. Perform a series of in-place turns at different speeds
//!
//! During each turn, we update the OLED with telemetry:
//! - Line 0: "TURN TEST"
//! - Line 1: "TGT:xx.x ACT:xx.x"
//! - Line 2: "YAW:xxx.x"
//! - Line 3: "DEV: xx.x"
//!
//! IMPORTANT: We do NOT overwrite the OLED at the end of the sequence.
//! The last turn's telemetry remains visible to support testing without a debugger attached.
//!
//! # Usage
//!
//! Call `init_testing()` from main to spawn the test task.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use heapless::String;

use crate::{
    system::event::{Events, raise_event},
    task::{
        drive::{DriveAction, DriveCommand, ImuCalibrationKind, send_drive_command},
        io::display::{DisplayAction, display_update},
        sensors::imu::{AhrsFusionMode, set_ahrs_fusion_mode},
    },
};

/// Signal used to trigger the test sequence on demand.
static TEST_START_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Request the test sequence to start.
pub fn start_testing_sequence() {
    TEST_START_SIGNAL.signal(());
}

/// Initialize testing task (idle until `start_testing_sequence` is called).
pub fn init_testing(spawner: embassy_executor::Spawner) {
    spawner.must_spawn(testing_sequence_runner());
}

/// Main testing sequence runner (waits for on-demand start signals).
#[embassy_executor::task]
async fn testing_sequence_runner() {
    loop {
        TEST_START_SIGNAL.wait().await;
        run_testing_sequence().await;
        raise_event(Events::TestingCompleted).await;
    }
}

/// Main testing sequence.
///
/// This is the entry point for all development tests. Modify as needed.
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
            let _ = core::fmt::write(&mut s, format_args!("TGT:{target_deg:>4.1} ACT:{current_deg:>4.1}"));
            display_update(DisplayAction::ShowText(s, 1)).await;
        }

        {
            let mut s: String<20> = String::new();
            // Example: "YAW:-123.4"
            let _ = core::fmt::write(&mut s, format_args!("YAW:{yaw_deg:>6.1}"));
            display_update(DisplayAction::ShowText(s, 2)).await;
        }

        {
            let mut s: String<20> = String::new();
            // Example: "DEV:  -1.6"
            let _ = core::fmt::write(&mut s, format_args!("DEV:{deviation_deg:>6.1}"));
            display_update(DisplayAction::ShowText(s, 3)).await;
        }
    }

    // Clear display at start of test.
    display_update(DisplayAction::Clear).await;
    show_line(0, "TURN TEST").await;
    show_line(1, "Initializing...").await;
    show_line(2, "").await;
    show_line(3, "").await;

    // Send initialization event to orchestrator
    Timer::after(Duration::from_millis(100)).await;
    raise_event(Events::Initialize).await;

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

    // Turn in place: 45 degrees, varying rotation speeds.
    let target_deg: f32 = 90.0;
    let test_speeds: [u8; 4] = [40, 60, 80, 100];

    for speed in test_speeds {
        defmt::info!("🧪 TEST: Turn in place 90° at speed {}", speed);

        // Pre-turn display
        show_line(0, "TURN TEST").await;
        {
            let mut s: String<20> = String::new();
            let _ = core::fmt::write(&mut s, format_args!("SPD:{speed:>3} TGT:{target_deg:>4.1}"));
            display_update(DisplayAction::ShowText(s, 1)).await;
        }
        show_line(2, "Turning...").await;
        show_line(3, "").await;

        // Ensure we're stopped before the next turn
        send_drive_command(DriveCommand::Drive(DriveAction::SetSpeed { left: 0, right: 0 }));
        Timer::after(Duration::from_millis(500)).await;

        // Start the turn
        send_drive_command(DriveCommand::Drive(DriveAction::RotateExact {
            degrees: target_deg,
            direction: crate::task::drive::types::RotationDirection::Clockwise,
            motion: crate::task::drive::types::RotationMotion::Stationary { speed },
        }));

        // Await completion (no global event consumption)
        let completion = crate::task::drive::wait_for_rotation_completed().await;

        // The controller reports normalized error:
        // error = achieved - target, so achieved = target + error.
        let achieved_deg = target_deg + completion.angle_error_deg;

        // Update display with the final telemetry for this turn.
        show_turn_status(
            target_deg,
            achieved_deg,
            completion.final_yaw_deg,
            completion.angle_error_deg,
        )
        .await;

        defmt::info!(
            "🧪 TEST: Rotation complete: final_yaw={=f32}°, angle_error={=f32}°",
            completion.final_yaw_deg,
            completion.angle_error_deg
        );

        // Stop after each turn
        send_drive_command(DriveCommand::Drive(DriveAction::Coast));
        Timer::after(Duration::from_millis(750)).await;
    }

    // Do not overwrite the OLED here; keep the last turn's telemetry visible.
    defmt::info!("🧪 TEST: Sequence complete (OLED left showing last turn telemetry)");
}

/// Auto-start calibration sequence (LEGACY - keep for reference)
///
/// This was the old test sequence that automatically triggered motor calibration
/// first, then IMU calibration. Motor calibration must run first because IMU
/// calibration uses calibrated motor commands to measure interference at actual
/// operational speeds.
///
/// Uncomment and use this if you need to run full calibration during development.
#[allow(dead_code)]
#[embassy_executor::task]
async fn auto_calibration_sequence() {
    // Send initialization event
    Timer::after(Duration::from_millis(100)).await;
    raise_event(Events::Initialize).await;

    // Wait for system to initialize
    Timer::after(Duration::from_secs(3)).await;

    defmt::info!("🤖 AUTO-CALIBRATION: Starting MOTOR calibration in 2 seconds...");
    Timer::after(Duration::from_secs(2)).await;

    defmt::info!("🤖 AUTO-CALIBRATION: Triggering motor calibration now!");
    send_drive_command(DriveCommand::RunMotorCalibration);

    // Wait for motor calibration to complete (~50s) plus delay
    Timer::after(Duration::from_secs(80)).await;

    defmt::info!("🤖 AUTO-CALIBRATION: Starting IMU calibration in 10 seconds...");
    Timer::after(Duration::from_secs(10)).await;

    defmt::info!("🤖 AUTO-CALIBRATION: Triggering IMU calibration now!");
    send_drive_command(DriveCommand::RunImuCalibration(ImuCalibrationKind::Full));
}
