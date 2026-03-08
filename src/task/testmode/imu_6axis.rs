//! On-demand IMU test mode task.
//!
//! Spawns an IMU telemetry task when requested and exits on stop.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use heapless::String;

use super::{TestCommand, release_testmode, request_start};
use crate::task::{
    io::display::{DisplayAction, display_update},
    sensors::imu::{
        AhrsFusionMode, Orientation, get_latest_calibrated_accel, get_latest_calibrated_gyro, get_latest_orientation,
        get_latest_raw_accel, get_latest_raw_gyro, set_ahrs_fusion_mode, start_imu_readings, stop_imu_readings,
    },
};

/// Signal used to stop the IMU 6-axis test mode.
static IMU6_TEST_STOP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Tracks whether the IMU 6-axis test mode is active.
static IMU6_TEST_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Request the IMU 6-axis test mode to start (spawns the task on demand).
pub async fn start_imu6_test_mode() {
    if IMU6_TEST_ACTIVE.swap(true, Ordering::Relaxed) {
        return;
    }

    if !request_start(TestCommand::Imu6).await {
        IMU6_TEST_ACTIVE.store(false, Ordering::Relaxed);
    }
}

/// Request the IMU 6-axis test mode to stop.
pub fn stop_imu6_test_mode() {
    IMU6_TEST_ACTIVE.store(false, Ordering::Relaxed);
    IMU6_TEST_STOP_SIGNAL.signal(());
}

/// Spawn the IMU 6-axis test task via the controller.
pub(super) fn spawn(spawner: Spawner) {
    spawner.must_spawn(imu6_test_task());
}

/// IMU test mode runner (updates display at 50Hz while active).

#[embassy_executor::task]
async fn imu6_test_task() {
    start_imu_readings();
    Timer::after(Duration::from_millis(30)).await;
    set_ahrs_fusion_mode(AhrsFusionMode::Axis6);
    display_update(DisplayAction::Clear).await;

    let mut tick: u32 = 0;
    let mut missing: u32 = 0;

    // Clear any pending stop signal so the next test doesn't end immediately.
    while IMU6_TEST_STOP_SIGNAL.signaled() {
        IMU6_TEST_STOP_SIGNAL.wait().await;
    }

    loop {
        match select(IMU6_TEST_STOP_SIGNAL.wait(), Timer::after(Duration::from_millis(20))).await {
            Either::First(()) => break,
            Either::Second(()) => {
                if !IMU6_TEST_ACTIVE.load(Ordering::Relaxed) {
                    break;
                }

                let orientation = get_latest_orientation().await;
                let accel = get_latest_calibrated_accel().await;
                let gyro = get_latest_calibrated_gyro().await;
                let raw_accel = get_latest_raw_accel().await;
                let raw_gyro = get_latest_raw_gyro().await;

                if orientation.is_none() && accel.is_none() && gyro.is_none() {
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

                    let mut line3: String<20> = String::new();
                    let _ = line3.push_str("6-axis");
                    display_update(DisplayAction::ShowText(line3, 3)).await;
                }

                if tick.is_multiple_of(50) {
                    defmt::debug!(
                        "IMU6 test data: ori={} accel={} gyro={} missing={}",
                        orientation.is_some(),
                        accel.is_some(),
                        gyro.is_some(),
                        missing
                    );

                    if let (Some(a), Some(g)) = (raw_accel, raw_gyro) {
                        defmt::debug!(
                            "IMU6 raw: a=({=f32},{=f32},{=f32}) g=({=f32},{=f32},{=f32})",
                            a.x,
                            a.y,
                            a.z,
                            g.x,
                            g.y,
                            g.z
                        );
                    } else {
                        defmt::debug!("IMU6 raw: accel={} gyro={}", raw_accel.is_some(), raw_gyro.is_some());
                    }
                }
            }
        }
    }

    stop_imu_readings();
    release_testmode();
    IMU6_TEST_ACTIVE.store(false, Ordering::Relaxed);
}

/// Format an IMU orientation line (Euler angles).
fn format_orientation_line(orientation: Option<Orientation>) -> String<20> {
    let mut s: String<20> = String::new();
    match orientation {
        Some(ori) => {
            let _ = core::fmt::write(
                &mut s,
                format_args!("E y{:.1}/p{:.1}/r{:.1}", ori.yaw, ori.pitch, ori.roll),
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
            let _ = core::fmt::write(&mut s, format_args!("{prefix} x{:.1}/y{:.1}/z{:.1}", v.x, v.y, v.z));
        }
        None => {
            let _ = core::fmt::write(&mut s, format_args!("{prefix} --.-/--.-/--.-"));
        }
    }
    s
}
