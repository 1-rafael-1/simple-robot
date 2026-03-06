//! On-demand ultrasonic sweep test mode task.
//!
//! Spawns a sweep telemetry task when requested and exits on stop.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};

use super::{TestCommand, release_testmode, request_start};
use crate::{
    system::state::SYSTEM_STATE,
    task::sensors::ultrasonic::{start_ultrasonic_sweep, stop_ultrasonic_measurements},
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
pub(super) fn spawn(spawner: Spawner) {
    spawner.must_spawn(ultrasonic_sweep_test_task());
}

/// Ultrasonic sweep test mode runner.
#[embassy_executor::task]
async fn ultrasonic_sweep_test_task() {
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
            Either::First(()) => break,
            Either::Second(()) => {
                if !ULTRASONIC_SWEEP_TEST_ACTIVE.load(Ordering::Relaxed) {
                    break;
                }
            }
        }
    }

    stop_ultrasonic_measurements();
    {
        let mut state = SYSTEM_STATE.lock().await;
        state.ultrasonic_reading = None;
        state.ultrasonic_angle_deg = None;
    }
    release_testmode();
    ULTRASONIC_SWEEP_TEST_ACTIVE.store(false, Ordering::Relaxed);
}
