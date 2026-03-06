//! On-demand IR + ultrasonic test mode task.
//!
//! Spawns a display telemetry task when requested and exits on stop.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use heapless::String;

use super::{TestCommand, release_testmode, request_start};
use crate::{
    system::state::SYSTEM_STATE,
    task::{
        io::display::{DisplayAction, display_update},
        sensors::ultrasonic::{start_ultrasonic_fixed, stop_ultrasonic_sweep},
    },
};

/// Signal used to stop the IR + ultrasonic test mode.
static IR_ULTRASONIC_TEST_STOP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Tracks whether the IR + ultrasonic test mode is active.
static IR_ULTRASONIC_TEST_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Request the IR + ultrasonic test mode to start (spawns the task on demand).
pub async fn start_ir_ultrasonic_test_mode() {
    if IR_ULTRASONIC_TEST_ACTIVE.swap(true, Ordering::Relaxed) {
        return;
    }

    request_start(TestCommand::IrUltrasonic).await;
}

/// Request the IR + ultrasonic test mode to stop.
pub fn stop_ir_ultrasonic_test_mode() {
    IR_ULTRASONIC_TEST_ACTIVE.store(false, Ordering::Relaxed);
    IR_ULTRASONIC_TEST_STOP_SIGNAL.signal(());
}

/// Spawn the IR + ultrasonic test task via the controller.
pub(super) fn spawn(spawner: Spawner) {
    spawner.must_spawn(ir_ultrasonic_test_task());
}

/// IR + ultrasonic test mode runner (updates display at 10Hz while active).
#[embassy_executor::task]
async fn ir_ultrasonic_test_task() {
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
            Either::First(()) => break,
            Either::Second(()) => {
                if !IR_ULTRASONIC_TEST_ACTIVE.load(Ordering::Relaxed) {
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

    stop_ultrasonic_sweep();
    display_update(DisplayAction::Clear).await;
    {
        let mut state = SYSTEM_STATE.lock().await;
        state.ultrasonic_reading = None;
        state.ultrasonic_angle_deg = None;
    }
    release_testmode();
    IR_ULTRASONIC_TEST_ACTIVE.store(false, Ordering::Relaxed);
}
