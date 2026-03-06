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
//!
//! # Usage
//!
//! Call `init_testing()` from main to spawn the test task.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use heapless::String;

use crate::{
    system::{
        event::{Events, raise_event},
        state::SYSTEM_STATE,
    },
    task::{
        drive::{
            CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind,
            ImuCalibrationKind, TurnDirection, acquire_completion_handle, completion_sender,
            get_latest_accel_measurement, get_latest_gyro_measurement, get_latest_mag_measurement,
            release_completion_handle, send_drive_command, send_drive_command_with_completion, wait_for_completion,
        },
        io::display::{DisplayAction, display_update},
        sensors::{
            imu::{
                AhrsFusionMode, Orientation, get_latest_orientation, set_ahrs_fusion_mode, start_imu_readings,
                stop_imu_readings,
            },
            ultrasonic::{start_ultrasonic_fixed, start_ultrasonic_sweep, stop_ultrasonic_sweep},
        },
    },
};

/// Signal used to trigger the test sequence on demand.
static TEST_START_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signal used to start the IMU test mode.
static IMU_TEST_START_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signal used to stop the IMU test mode.
static IMU_TEST_STOP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Tracks whether the IMU test mode is active.
static IMU_TEST_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Signal used to start the IR + ultrasonic test mode.
static IR_ULTRASONIC_TEST_START_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signal used to stop the IR + ultrasonic test mode.
static IR_ULTRASONIC_TEST_STOP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Tracks whether the IR + ultrasonic test mode is active.
static IR_ULTRASONIC_TEST_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Signal used to start the ultrasonic sweep test mode.
static ULTRASONIC_SWEEP_TEST_START_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signal used to stop the ultrasonic sweep test mode.
static ULTRASONIC_SWEEP_TEST_STOP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Tracks whether the ultrasonic sweep test mode is active.
static ULTRASONIC_SWEEP_TEST_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Request the test sequence to start.
pub fn start_testing_sequence() {
    TEST_START_SIGNAL.signal(());
}

/// Request the IMU test mode to start.
pub fn start_imu_test_mode() {
    IMU_TEST_ACTIVE.store(true, Ordering::Relaxed);
    IMU_TEST_START_SIGNAL.signal(());
}

/// Request the IMU test mode to stop.
pub fn stop_imu_test_mode() {
    IMU_TEST_ACTIVE.store(false, Ordering::Relaxed);
    IMU_TEST_STOP_SIGNAL.signal(());
}

/// Request the IR + ultrasonic test mode to start.
pub fn start_ir_ultrasonic_test_mode() {
    IR_ULTRASONIC_TEST_ACTIVE.store(true, Ordering::Relaxed);
    IR_ULTRASONIC_TEST_START_SIGNAL.signal(());
}

/// Request the IR + ultrasonic test mode to stop.
pub fn stop_ir_ultrasonic_test_mode() {
    IR_ULTRASONIC_TEST_ACTIVE.store(false, Ordering::Relaxed);
    IR_ULTRASONIC_TEST_STOP_SIGNAL.signal(());
}

/// Request the ultrasonic sweep test mode to start.
pub fn start_ultrasonic_sweep_test_mode() {
    ULTRASONIC_SWEEP_TEST_ACTIVE.store(true, Ordering::Relaxed);
    ULTRASONIC_SWEEP_TEST_START_SIGNAL.signal(());
}

/// Request the ultrasonic sweep test mode to stop.
pub fn stop_ultrasonic_sweep_test_mode() {
    ULTRASONIC_SWEEP_TEST_ACTIVE.store(false, Ordering::Relaxed);
    ULTRASONIC_SWEEP_TEST_STOP_SIGNAL.signal(());
}

/// Initialize testing task (idle until `start_testing_sequence` is called).
pub fn init_testing(spawner: embassy_executor::Spawner) {
    spawner.must_spawn(testing_sequence_runner());
    spawner.must_spawn(imu_test_runner());
    spawner.must_spawn(ir_ultrasonic_test_runner());
    spawner.must_spawn(ultrasonic_sweep_test_runner());
}

/// Ultrasonic sweep test mode runner (keeps sweep active while selected).
#[embassy_executor::task]
async fn ultrasonic_sweep_test_runner() {
    loop {
        ULTRASONIC_SWEEP_TEST_START_SIGNAL.wait().await;
        start_ultrasonic_sweep();

        // Clear any pending stop signal so the next test doesn't end immediately.
        while ULTRASONIC_SWEEP_TEST_STOP_SIGNAL.signaled() {
            ULTRASONIC_SWEEP_TEST_STOP_SIGNAL.wait().await;
        }

        loop {
            match select(
                ULTRASONIC_SWEEP_TEST_STOP_SIGNAL.wait(),
                Timer::after(Duration::from_millis(100)),
            )
            .await
            {
                Either::First(()) => {
                    ULTRASONIC_SWEEP_TEST_ACTIVE.store(false, Ordering::Relaxed);
                    stop_ultrasonic_sweep();
                    display_update(DisplayAction::Clear).await;
                    break;
                }
                Either::Second(()) => {
                    if !ULTRASONIC_SWEEP_TEST_ACTIVE.load(Ordering::Relaxed) {
                        stop_ultrasonic_sweep();
                        display_update(DisplayAction::Clear).await;
                        break;
                    }

                    let reading = {
                        let state = SYSTEM_STATE.lock().await;
                        state.ultrasonic_reading
                    };

                    let header = {
                        let mut s: String<20> = String::new();
                        match reading {
                            Some(crate::system::event::UltrasonicReading::Distance(cm)) => {
                                let _ = core::fmt::write(&mut s, format_args!("US: {cm:>6.1} cm"));
                            }
                            Some(crate::system::event::UltrasonicReading::Timeout) => {
                                let _ = s.push_str("US: timeout");
                            }
                            Some(crate::system::event::UltrasonicReading::Error) => {
                                let _ = s.push_str("US: error");
                            }
                            None => {
                                let _ = s.push_str("US: ---- cm");
                            }
                        }
                        s
                    };
                    display_update(DisplayAction::ShowText(header, 0)).await;
                }
            }
        }
    }
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

/// IMU test mode runner (updates display at 50Hz while active).
#[embassy_executor::task]
async fn imu_test_runner() {
    loop {
        IMU_TEST_START_SIGNAL.wait().await;
        set_ahrs_fusion_mode(AhrsFusionMode::Axis9);
        start_imu_readings();
        display_update(DisplayAction::Clear).await;

        let mut tick: u32 = 0;
        let mut missing: u32 = 0;

        // Clear any pending stop signal so the next test doesn't end immediately.
        while IMU_TEST_STOP_SIGNAL.signaled() {
            IMU_TEST_STOP_SIGNAL.wait().await;
        }

        loop {
            match select(IMU_TEST_STOP_SIGNAL.wait(), Timer::after(Duration::from_millis(20))).await {
                Either::First(()) => {
                    IMU_TEST_ACTIVE.store(false, Ordering::Relaxed);
                    stop_imu_readings();
                    break;
                }
                Either::Second(()) => {
                    if !IMU_TEST_ACTIVE.load(Ordering::Relaxed) {
                        stop_imu_readings();
                        break;
                    }

                    let orientation = get_latest_orientation().await;
                    let accel = get_latest_accel_measurement().await;
                    let gyro = get_latest_gyro_measurement().await;
                    let mag = get_latest_mag_measurement().await;

                    if orientation.is_none() && accel.is_none() && gyro.is_none() && mag.is_none() {
                        missing = missing.saturating_add(1);
                    } else {
                        missing = 0;
                    }

                    tick = tick.wrapping_add(1);
                    // Throttle display updates to avoid I2C starvation of the IMU and port expander.
                    if tick.is_multiple_of(5) {
                        let header = format_orientation_line(orientation);
                        display_update(DisplayAction::ShowText(header, 0)).await;

                        let line1 = format_axis_line("A", accel);
                        display_update(DisplayAction::ShowText(line1, 1)).await;

                        let line2 = format_axis_line("G", gyro);
                        display_update(DisplayAction::ShowText(line2, 2)).await;

                        let line3 = format_axis_line("M", mag);
                        display_update(DisplayAction::ShowText(line3, 3)).await;
                    }

                    if tick.is_multiple_of(50) {
                        defmt::debug!(
                            "IMU test data: ori={} accel={} gyro={} mag={} missing={}",
                            orientation.is_some(),
                            accel.is_some(),
                            gyro.is_some(),
                            mag.is_some(),
                            missing
                        );
                    }
                }
            }
        }
    }
}

/// IR + ultrasonic test mode runner (updates display at 10Hz while active).
#[embassy_executor::task]
async fn ir_ultrasonic_test_runner() {
    loop {
        IR_ULTRASONIC_TEST_START_SIGNAL.wait().await;
        start_ultrasonic_fixed(80.0);
        display_update(DisplayAction::Clear).await;

        // Clear any pending stop signal so the next test doesn't end immediately.
        while IR_ULTRASONIC_TEST_STOP_SIGNAL.signaled() {
            IR_ULTRASONIC_TEST_STOP_SIGNAL.wait().await;
        }

        loop {
            match select(
                IR_ULTRASONIC_TEST_STOP_SIGNAL.wait(),
                Timer::after(Duration::from_millis(100)),
            )
            .await
            {
                Either::First(()) => {
                    IR_ULTRASONIC_TEST_ACTIVE.store(false, Ordering::Relaxed);
                    stop_ultrasonic_sweep();
                    break;
                }
                Either::Second(()) => {
                    if !IR_ULTRASONIC_TEST_ACTIVE.load(Ordering::Relaxed) {
                        stop_ultrasonic_sweep();
                        break;
                    }

                    let (ir_detected, reading) = {
                        let state = SYSTEM_STATE.lock().await;
                        (state.obstacle_detected, state.ultrasonic_reading)
                    };

                    let header = {
                        let mut s: String<20> = String::new();
                        let _ = s.push_str("IR+US Test");
                        s
                    };
                    display_update(DisplayAction::ShowText(header, 0)).await;

                    let ir_line = if ir_detected { "IR: obstacle" } else { "IR: nothing" };
                    let mut line1: String<20> = String::new();
                    let _ = line1.push_str(ir_line);
                    display_update(DisplayAction::ShowText(line1, 1)).await;

                    let line2 = {
                        let mut s: String<20> = String::new();
                        match reading {
                            Some(crate::system::event::UltrasonicReading::Distance(cm)) => {
                                let _ = core::fmt::write(&mut s, format_args!("US: {cm:>6.1} cm"));
                            }
                            Some(crate::system::event::UltrasonicReading::Timeout) => {
                                let _ = s.push_str("US: timeout");
                            }
                            Some(crate::system::event::UltrasonicReading::Error) => {
                                let _ = s.push_str("US: error");
                            }
                            None => {
                                let _ = s.push_str("US: ---- cm");
                            }
                        }
                        s
                    };
                    display_update(DisplayAction::ShowText(line2, 2)).await;

                    let mut line3: String<20> = String::new();
                    let _ = line3.push_str("Press to exit");
                    display_update(DisplayAction::ShowText(line3, 3)).await;
                }
            }
        }
    }
}

/// Format an IMU orientation line (Euler angles).
fn format_orientation_line(orientation: Option<Orientation>) -> String<20> {
    let mut s: String<20> = String::new();
    match orientation {
        Some(ori) => {
            let _ = core::fmt::write(
                &mut s,
                format_args!("E {:.1}/{:.1}/{:.1}", ori.yaw, ori.pitch, ori.roll),
            );
        }
        None => {
            let _ = core::fmt::write(&mut s, format_args!("E --.-/--.-/--.-"));
        }
    }
    s
}

/// Format a 3-axis sensor line (accel/gyro/mag).
fn format_axis_line(prefix: &str, data: Option<nalgebra::Vector3<f32>>) -> String<20> {
    let mut s: String<20> = String::new();
    match data {
        Some(v) => {
            let _ = core::fmt::write(&mut s, format_args!("{prefix} {:.1}/{:.1}/{:.1}", v.x, v.y, v.z));
        }
        None => {
            let _ = core::fmt::write(&mut s, format_args!("{prefix} --.-/--.-/--.-"));
        }
    }
    s
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
        send_drive_command(DriveCommand::Drive(DriveAction::Coast)).await;
        Timer::after(Duration::from_millis(500)).await;

        let Ok(mut completion_handle) = acquire_completion_handle().await else {
            defmt::warn!("🧪 TEST: completion pool exhausted; skipping turn");
            continue;
        };
        let sender = completion_sender(completion_handle);

        // Start the turn
        send_drive_command_with_completion(
            DriveCommand::Drive(DriveAction::RotateExact {
                degrees: target_deg,
                direction: crate::task::drive::types::RotationDirection::Clockwise,
                motion: crate::task::drive::types::RotationMotion::Stationary { speed },
            }),
            sender,
        )
        .await;

        // Await completion (no global event consumption)
        let completion = wait_for_completion(&completion_handle).await;
        release_completion_handle(completion_handle).await;

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

        // Update display with the final telemetry for this turn.
        show_turn_status(target_deg, achieved_deg, final_yaw_deg, angle_error_deg).await;

        let status_str = match status {
            CompletionStatus::Success => "Success",
            CompletionStatus::Cancelled => "Cancelled",
            CompletionStatus::Failed(_) => "Failed",
        };
        defmt::info!(
            "🧪 TEST: Rotation complete: status={=str}, final_yaw={=f32}°, angle_error={=f32}°",
            status_str,
            final_yaw_deg,
            angle_error_deg
        );

        if let CompletionStatus::Failed(reason) = status {
            defmt::warn!("🧪 TEST: Rotation failed: {=str}", reason);
        }

        // Stop after each turn
        send_drive_command(DriveCommand::Drive(DriveAction::Coast)).await;
        Timer::after(Duration::from_millis(750)).await;
    }

    // Straight distance runs after turns (encoder-based).
    show_line(0, "DIST TEST").await;
    show_line(1, "FWD 50 CM").await;
    show_line(2, "").await;
    show_line(3, "").await;

    let Ok(mut completion_handle) = acquire_completion_handle().await else {
        defmt::warn!("🧪 TEST: completion pool exhausted; skipping forward distance");
        return;
    };
    let sender = completion_sender(completion_handle);

    send_drive_command_with_completion(
        DriveCommand::Drive(DriveAction::DriveDistance {
            kind: DriveDistanceKind::Straight { distance_cm: 50.0 },
            direction: DriveDirection::Forward,
            speed: 60,
        }),
        sender,
    )
    .await;

    let completion = wait_for_completion(&completion_handle).await;
    release_completion_handle(completion_handle).await;

    if let CompletionTelemetry::DriveDistance {
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
            "🧪 TEST: Forward distance complete: status={=str} left={=f32} right={=f32}",
            status_str,
            achieved_left_revs,
            achieved_right_revs
        );
    }

    send_drive_command(DriveCommand::Drive(DriveAction::Coast)).await;
    Timer::after(Duration::from_millis(750)).await;

    show_line(1, "REV 50 CM").await;

    let Ok(completion_handle) = acquire_completion_handle().await else {
        defmt::warn!("🧪 TEST: completion pool exhausted; skipping reverse distance");
        return;
    };
    let sender = completion_sender(completion_handle);

    send_drive_command_with_completion(
        DriveCommand::Drive(DriveAction::DriveDistance {
            kind: DriveDistanceKind::Straight { distance_cm: 50.0 },
            direction: DriveDirection::Backward,
            speed: 60,
        }),
        sender,
    )
    .await;

    let completion = wait_for_completion(&completion_handle).await;
    release_completion_handle(completion_handle).await;

    if let CompletionTelemetry::DriveDistance {
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
            "🧪 TEST: Reverse distance complete: status={=str} left={=f32} right={=f32}",
            status_str,
            achieved_left_revs,
            achieved_right_revs
        );
    }

    send_drive_command(DriveCommand::Drive(DriveAction::Coast)).await;
    Timer::after(Duration::from_millis(750)).await;

    // Full circle curve test: 1m radius, 360° arc (centerline).
    show_line(0, "CIRCLE TEST").await;
    show_line(1, "R=100cm 360").await;
    show_line(2, "FWD LEFT").await;
    show_line(3, "").await;

    defmt::info!("🧪 TEST: Curve circle 360° at radius 1m");
    let circle_arc_cm = 2.0 * core::f32::consts::PI * 100.0;

    let Ok(mut completion_handle) = acquire_completion_handle().await else {
        defmt::warn!("🧪 TEST: completion pool exhausted; skipping circle test");
        return;
    };
    let sender = completion_sender(completion_handle);

    send_drive_command_with_completion(
        DriveCommand::Drive(DriveAction::DriveDistance {
            kind: DriveDistanceKind::CurveArc {
                radius_cm: 100.0,
                arc_length_cm: circle_arc_cm,
                direction: TurnDirection::Left,
            },
            direction: DriveDirection::Forward,
            speed: 50,
        }),
        sender,
    )
    .await;

    let completion = wait_for_completion(&completion_handle).await;
    release_completion_handle(completion_handle).await;

    if let CompletionTelemetry::DriveDistance {
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
            "🧪 TEST: Circle complete: status={=str} left={=f32} right={=f32}",
            status_str,
            achieved_left_revs,
            achieved_right_revs
        );
    }

    send_drive_command(DriveCommand::Drive(DriveAction::Coast)).await;
    Timer::after(Duration::from_millis(750)).await;

    // Do not overwrite the OLED here; keep the last test's telemetry visible.
    defmt::info!("🧪 TEST: Sequence complete (OLED left showing last test telemetry)");
}
