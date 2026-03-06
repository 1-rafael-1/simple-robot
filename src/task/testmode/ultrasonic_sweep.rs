//! On-demand ultrasonic sweep test mode task.
//!
//! Spawns a sweep telemetry task when requested and exits on stop.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use heapless::String;

use super::{TestCommand, request_start};
use crate::{
    system::state::SYSTEM_STATE,
    task::{
        io::display::{DisplayAction, display_update},
        sensors::ultrasonic::{start_ultrasonic_sweep, stop_ultrasonic_sweep},
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

    request_start(TestCommand::UltrasonicSweep).await;
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

/// Ultrasonic sweep test mode runner (updates display at 10Hz while active).
#[embassy_executor::task]
async fn ultrasonic_sweep_test_task() {
    start_ultrasonic_sweep();
    display_update(DisplayAction::Clear).await;

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

    stop_ultrasonic_sweep();
    display_update(DisplayAction::Clear).await;
    ULTRASONIC_SWEEP_TEST_ACTIVE.store(false, Ordering::Relaxed);
}
