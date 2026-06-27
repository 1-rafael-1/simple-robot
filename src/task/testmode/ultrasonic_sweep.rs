//! On-demand ultrasonic sweep test mode task.
//!
//! Spawns a sweep telemetry task when requested and exits on stop.
//! The spawned task owns its own display rendering at 10 Hz, reading
//! perception state directly — matching the IMU test pattern.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use heapless::String;

use super::{TestCommand, release_testmode, request_start};
use crate::{
    system::state::perception,
    task::{
        io::display::{self, DisplayAction},
        sensors::ultrasonic::{self, start_ultrasonic_sweep, stop_ultrasonic_measurements},
    },
};

/// Signal used to stop the ultrasonic sweep test mode.
static ULTRASONIC_SWEEP_TEST_STOP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Tracks whether the ultrasonic sweep test mode is active.
static ULTRASONIC_SWEEP_TEST_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Request the ultrasonic sweep test mode to start (spawns the task on demand).
pub async fn start_ultrasonic_sweep_test_mode() {
    if ULTRASONIC_SWEEP_TEST_ACTIVE.swap(true, Ordering::Relaxed) {
        return;
    }

    if !request_start(TestCommand::UltrasonicSweep).await {
        ULTRASONIC_SWEEP_TEST_ACTIVE.store(false, Ordering::Relaxed);
    }
}

/// Request the ultrasonic sweep test mode to stop.
pub fn stop_ultrasonic_sweep_test_mode() {
    ULTRASONIC_SWEEP_TEST_ACTIVE.store(false, Ordering::Relaxed);
    ULTRASONIC_SWEEP_TEST_STOP_SIGNAL.signal(());
}

/// Spawn the ultrasonic sweep test task via the controller.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner) {
    spawner.spawn(ultrasonic_sweep_test_task().unwrap());
}

/// Ultrasonic sweep test mode runner with built-in 10 Hz display loop.
#[embassy_executor::task]
async fn ultrasonic_sweep_test_task() {
    start_ultrasonic_sweep();
    display::display_update(DisplayAction::Clear).await;

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
            Either::First(()) => break,
            Either::Second(()) => {
                if !ULTRASONIC_SWEEP_TEST_ACTIVE.load(Ordering::Relaxed) {
                    break;
                }

                // Read current servo angle for the sweep line
                let (_reading, angle) = perception::ultrasonic_sweep_snapshot().await;

                // Only render when we have an angle
                if let Some(a) = angle {
                    // Compute nearest and farthest obstacle distances for header
                    // before the display update so the header read happens first.
                    let mut header: String<20> = String::new();
                    let mut nearest: Option<f64> = None;
                    let mut farthest: Option<f64> = None;
                    let points = ultrasonic::sweep_buffer_points().await;
                    for p in &points {
                        if nearest.is_none_or(|n| p.distance_cm < n) {
                            nearest = Some(p.distance_cm);
                        }
                        if farthest.is_none_or(|f| p.distance_cm > f) {
                            farthest = Some(p.distance_cm);
                        }
                    }

                    match (nearest, farthest) {
                        (Some(n), Some(f)) => {
                            let _ = core::fmt::write(&mut header, format_args!("n{n:.0}cm | f{f:.0}cm"));
                        }
                        _ => {
                            let _ = header.push_str("nNone | fNone");
                        }
                    }

                    display::display_update(DisplayAction::ShowText(header, 0)).await;
                    display::display_update(DisplayAction::ShowSweepFromBuffer { current_angle: a }).await;
                }
            }
        }
    }

    stop_ultrasonic_measurements();
    perception::clear_ultrasonic_data().await;
    release_testmode();
    ULTRASONIC_SWEEP_TEST_ACTIVE.store(false, Ordering::Relaxed);
}
