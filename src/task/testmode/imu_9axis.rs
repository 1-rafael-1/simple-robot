//! On-demand IMU test mode task.
//!
//! Spawns an IMU telemetry task when requested and exits on stop.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use heapless::String;
use micromath::F32Ext;

use super::{TestCommand, release_testmode, request_start};
use crate::task::{
    io::display::{DisplayAction, display_update},
    sensors::imu::{
        AhrsFusionMode, Orientation, get_latest_calibrated_accel, get_latest_calibrated_gyro,
        get_latest_calibrated_mag, get_latest_orientation, get_latest_raw_accel, get_latest_raw_gyro,
        get_latest_raw_mag, set_ahrs_fusion_mode, start_imu_readings, stop_imu_readings,
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

    if !request_start(TestCommand::Imu).await {
        IMU_TEST_ACTIVE.store(false, Ordering::Relaxed);
    }
}

/// Request the IMU test mode to stop.
pub fn stop_imu_test_mode() {
    IMU_TEST_ACTIVE.store(false, Ordering::Relaxed);
    IMU_TEST_STOP_SIGNAL.signal(());
}

/// Spawn the IMU test task via the controller.
pub(super) fn spawn(spawner: Spawner) {
    spawner.must_spawn(imu_test_task());
}

/// IMU test mode runner (updates display at 50Hz while active).

#[embassy_executor::task]
async fn imu_test_task() {
    start_imu_readings();
    Timer::after(Duration::from_millis(30)).await;
    set_ahrs_fusion_mode(AhrsFusionMode::Axis9);
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
                let accel = get_latest_calibrated_accel().await;
                let gyro = get_latest_calibrated_gyro().await;
                let mag = get_latest_calibrated_mag().await;
                let raw_accel = get_latest_raw_accel().await;
                let raw_gyro = get_latest_raw_gyro().await;
                let raw_mag = get_latest_raw_mag().await;

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

                    let line3 = format_axis_line("M9", mag);
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

                    if let (Some(a), Some(g), Some(m)) = (raw_accel, raw_gyro, raw_mag) {
                        log_mag_diagnostics(a, g, m, orientation);
                    } else {
                        defmt::debug!(
                            "IMU raw: accel={} gyro={} mag={}",
                            raw_accel.is_some(),
                            raw_gyro.is_some(),
                            raw_mag.is_some()
                        );
                        defmt::debug!("IMU mag gate: |m|=n/a use_mag=false");
                    }
                }
            }
        }
    }

    stop_imu_readings();
    release_testmode();
    IMU_TEST_ACTIVE.store(false, Ordering::Relaxed);
}

/// Log raw IMU vectors and heading remap diagnostics.
#[allow(clippy::similar_names)]
fn log_mag_diagnostics(
    accel: nalgebra::Vector3<f32>,
    gyro: nalgebra::Vector3<f32>,
    mag: nalgebra::Vector3<f32>,
    orientation: Option<Orientation>,
) {
    defmt::debug!(
        "IMU raw: a=({=f32},{=f32},{=f32}) g=({=f32},{=f32},{=f32}) m=({=f32},{=f32},{=f32})",
        accel.x,
        accel.y,
        accel.z,
        gyro.x,
        gyro.y,
        gyro.z,
        mag.x,
        mag.y,
        mag.z
    );
    let mag_norm = mag.norm();
    let use_mag = mag_norm > 20.0 && mag_norm < 200.0;
    defmt::debug!("IMU mag gate: |m|={=f32} use_mag={}", mag_norm, use_mag);

    let headings_xy = [
        mag.y.atan2(mag.x).to_degrees(),
        mag.x.atan2(mag.y).to_degrees(),
        mag.y.atan2(-mag.x).to_degrees(),
        (-mag.y).atan2(mag.x).to_degrees(),
        (-mag.y).atan2(-mag.x).to_degrees(),
        mag.x.atan2(-mag.y).to_degrees(),
        (-mag.x).atan2(mag.y).to_degrees(),
        (-mag.x).atan2(-mag.y).to_degrees(),
    ];

    defmt::debug!(
        "IMU mag remap headings: xy={=f32} yx={=f32} -x,y={=f32} x,-y={=f32} -x,-y={=f32} -y,x={=f32} y,-x={=f32} -y,-x={=f32}",
        headings_xy[0],
        headings_xy[1],
        headings_xy[2],
        headings_xy[3],
        headings_xy[4],
        headings_xy[5],
        headings_xy[6],
        headings_xy[7]
    );

    let headings_xz = [
        mag.z.atan2(mag.x).to_degrees(),
        mag.x.atan2(mag.z).to_degrees(),
        mag.z.atan2(-mag.x).to_degrees(),
        (-mag.z).atan2(mag.x).to_degrees(),
        (-mag.z).atan2(-mag.x).to_degrees(),
        mag.x.atan2(-mag.z).to_degrees(),
        (-mag.x).atan2(mag.z).to_degrees(),
        (-mag.x).atan2(-mag.z).to_degrees(),
    ];

    defmt::debug!(
        "IMU mag remap headings xz-plane: xz={=f32} zx={=f32} -x,z={=f32} x,-z={=f32} -x,-z={=f32} -z,x={=f32} z,-x={=f32} -z,-x={=f32}",
        headings_xz[0],
        headings_xz[1],
        headings_xz[2],
        headings_xz[3],
        headings_xz[4],
        headings_xz[5],
        headings_xz[6],
        headings_xz[7]
    );

    let headings_yz = [
        mag.z.atan2(mag.y).to_degrees(),
        mag.y.atan2(mag.z).to_degrees(),
        mag.z.atan2(-mag.y).to_degrees(),
        (-mag.z).atan2(mag.y).to_degrees(),
        (-mag.z).atan2(-mag.y).to_degrees(),
        mag.y.atan2(-mag.z).to_degrees(),
        (-mag.y).atan2(mag.z).to_degrees(),
        (-mag.y).atan2(-mag.z).to_degrees(),
    ];

    defmt::debug!(
        "IMU mag remap headings yz-plane: yz={=f32} zy={=f32} -y,z={=f32} y,-z={=f32} -y,-z={=f32} -z,y={=f32} z,-y={=f32} -z,-y={=f32}",
        headings_yz[0],
        headings_yz[1],
        headings_yz[2],
        headings_yz[3],
        headings_yz[4],
        headings_yz[5],
        headings_yz[6],
        headings_yz[7]
    );

    let raw_heading = headings_xy[0];

    let roll = accel.y.atan2(accel.z);
    let pitch = (-accel.x).atan2((accel.y * accel.y + accel.z * accel.z).sqrt());
    let mx2 = mag.x * pitch.cos() + mag.z * pitch.sin();
    let my2 = mag.x * roll.sin() * pitch.sin() + mag.y * roll.cos() - mag.z * roll.sin() * pitch.cos();
    let tilt_heading = my2.atan2(mx2).to_degrees();

    if let Some(ori) = orientation {
        defmt::debug!(
            "IMU heading: raw={=f32} tilt={=f32} ahrs_yaw={=f32}",
            raw_heading,
            tilt_heading,
            ori.yaw
        );
    } else {
        defmt::debug!(
            "IMU heading: raw={=f32} tilt={=f32} ahrs_yaw=n/a",
            raw_heading,
            tilt_heading
        );
    }
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
