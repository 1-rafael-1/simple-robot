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
    system::{event::UltrasonicReading, state::perception},
    task::{
        io::display::{DisplayAction, display_update},
        sensors::ultrasonic::{start_ultrasonic_sweep, stop_ultrasonic_measurements},
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
    display_update(DisplayAction::Clear).await;

    // Clear any pending stop signal so the next test doesn't end immediately.
    while ULTRASONIC_SWEEP_TEST_STOP_SIGNAL.signaled() {
        ULTRASONIC_SWEEP_TEST_STOP_SIGNAL.wait().await;
    }

    // Track the last rendered angle so we don't send duplicate ShowSweep
    // calls at the same servo position (which flips the display direction
    // tracker and wipes newly added points).
    let mut last_render_angle: Option<f32> = None;

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

                // Read latest perception data and render sweep display.
                let (reading, angle) = perception::ultrasonic_sweep_snapshot().await;

                // Only send ShowSweep when the angle actually changed.
                // At 10 Hz the display timer often ticks twice per ultrasonic
                // sweep step; a duplicate call at the same angle causes the
                // direction tracker to flip and the retain filter to erase
                // the point that was just added.
                if last_render_angle != angle {
                    render_sweep_display(reading, angle).await;
                    last_render_angle = angle;
                }
            }
        }
    }

    stop_ultrasonic_measurements();
    perception::clear_ultrasonic_data().await;
    release_testmode();
    ULTRASONIC_SWEEP_TEST_ACTIVE.store(false, Ordering::Relaxed);
}

/// Render the ultrasonic sweep display from the latest perception data.
async fn render_sweep_display(reading: Option<UltrasonicReading>, angle: Option<f32>) {
    // Radar sweep graphic — needs both distance and angle.
    match (reading, angle) {
        (Some(UltrasonicReading::Distance(distance)), Some(a)) => {
            display_update(DisplayAction::ShowSweep(Some(distance), a)).await;
        }
        (Some(UltrasonicReading::Timeout) | None, Some(a)) => {
            display_update(DisplayAction::ShowSweep(None, a)).await;
        }
        _ => {} // No angle yet — skip sweep graphic.
    }

    let mut header: String<20> = String::new();
    match (reading, angle) {
        (Some(UltrasonicReading::Distance(distance)), Some(a)) => {
            let _ = core::fmt::write(&mut header, format_args!("US:{distance:>5.1} A:{a:>3.0}"));
        }
        (Some(UltrasonicReading::Timeout), Some(a)) => {
            let _ = core::fmt::write(&mut header, format_args!("US:timeout A:{a:>3.0}"));
        }
        (Some(UltrasonicReading::Error), _) => {
            let _ = header.push_str("US:error");
        }
        _ => {
            let _ = header.push_str("US:---- cm");
        }
    }
    display_update(DisplayAction::ShowText(header, 0)).await;
}
