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
//! 3. Drive straight at speed 60 for 10 seconds
//!
//! # Usage
//!
//! Call `init_testing()` from main to spawn the test task.

use embassy_time::{Duration, Timer};

use crate::system::event::{Events, send_event};
use crate::task::drive::{DriveAction, DriveCommand, send_drive_command};

/// Initialize testing task
pub fn init_testing(spawner: &embassy_executor::Spawner) {
    spawner.must_spawn(testing_sequence());
}

/// Main testing sequence
///
/// This is the entry point for all development tests. Modify as needed.
#[embassy_executor::task]
async fn testing_sequence() {
    // Send initialization event to orchestrator
    Timer::after(Duration::from_millis(100)).await;
    send_event(Events::Initialize).await;

    // Wait for system to stabilize and for orchestrator to load calibration from flash
    // (orchestrator automatically loads calibration on Initialize event)
    defmt::info!("🧪 TEST: Waiting for system initialization and calibration loading...");
    Timer::after(Duration::from_secs(3)).await;

    // Perform motor calibration first to fix broken calibration data
    // defmt::info!("🧪 TEST: Running motor calibration...");
    // send_drive_command(DriveCommand::RunMotorCalibration);

    // Wait for calibration to complete (takes ~150 seconds)
    // defmt::info!("🧪 TEST: Waiting for calibration to complete...");
    // Timer::after(Duration::from_secs(160)).await;

    // Wait before driving
    defmt::info!("🧪 TEST: Waiting 10 seconds before driving...");
    Timer::after(Duration::from_secs(10)).await;

    // Drive straight ahead at speed 60 for 10 seconds
    defmt::info!("🧪 TEST: Driving straight at speed 60 for 40 seconds");
    send_drive_command(DriveCommand::Drive(DriveAction::SetSpeed { left: 100, right: 100 }));

    // Let it drive for 10 seconds
    Timer::after(Duration::from_secs(20)).await;

    // Stop
    defmt::info!("🧪 TEST: Stopping");
    send_drive_command(DriveCommand::Drive(DriveAction::SetSpeed { left: 0, right: 0 }));

    defmt::info!("🧪 TEST: Sequence complete");
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
    send_event(Events::Initialize).await;

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
    send_drive_command(DriveCommand::RunImuCalibration);
}
