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
    drive::{get_latest_accel_measurement, get_latest_gyro_measurement, get_latest_mag_measurement},
    io::display::{DisplayAction, display_update},
    sensors::imu::{
        AhrsFusionMode, Orientation, get_latest_orientation, set_ahrs_fusion_mode, start_imu_readings,
        stop_imu_readings,
    },
};

/// Signal used to stop the IMU test mode.
static IMU_TEST_STOP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Tracks whether the IMU test mode is active.
static IMU_TEST_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Request the IMU test mode to start (spawns the task on demand).
pub async fn start_imu_test_mode() {
    if IMU_TEST_ACTIVE.swap(true, Ordering::Relaxed) {
        return;
    }

    request_start(TestCommand::Imu).await;
}

/// Request the IMU test mode to stop.
pub fn stop_imu_test_mode() {
    IMU_TEST_ACTIVE.store(false, Ordering::Relaxed);
    IMU_TEST_STOP_SIGNAL.signal(());
}

/// IMU test mode runner (updates display at 50Hz while active).
/// Spawn the IMU test task via the controller.
pub(super) fn spawn(spawner: Spawner) {
    spawner.must_spawn(imu_test_task());
}

#[embassy_executor::task]
async fn imu_test_task() {
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
            Either::First(()) => break,
            Either::Second(()) => {
                if !IMU_TEST_ACTIVE.load(Ordering::Relaxed) {
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

    stop_imu_readings();
    display_update(DisplayAction::Clear).await;
    release_testmode();
    IMU_TEST_ACTIVE.store(false, Ordering::Relaxed);
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
