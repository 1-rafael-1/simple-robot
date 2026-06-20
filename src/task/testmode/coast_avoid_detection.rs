//! Coast-avoid obstacle-detection event-chain test mode.
//!
//! Spawned on demand, this test mirrors the exact sensor configuration and
//! perception-state reads that coast-avoid uses during the forward-drive phase.
//! It lets you verify that IR→perception→combined→would-trigger-EmergencyBrake
//! works while the robot is stationary (no motor PWM noise).
//!
//! If IR detection works here but not during actual coast-avoid driving, the
//! culprit is almost certainly electrical noise from the motors coupling into
//! the IR sensor signal path (schottky OR → CD4049 → pulldown → port expander).

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use heapless::String;

use super::{TestCommand, release_testmode, request_start};
use crate::{
    system::state::perception,
    task::{
        io::display::{DisplayAction, display_update},
        motor_driver::{self, MotorCommand},
        sensors::ultrasonic::{start_ultrasonic_centered_obstacle_detect, stop_ultrasonic_measurements},
    },
};

/// Signal used to stop the coast-avoid detection test mode.
static COAST_AVOID_DETECTION_STOP: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Tracks whether the test mode is active.
static COAST_AVOID_DETECTION_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Request the coast-avoid detection test mode to start.
pub async fn start_coast_avoid_detection_test() {
    if COAST_AVOID_DETECTION_ACTIVE.swap(true, Ordering::Relaxed) {
        return;
    }

    if !request_start(TestCommand::CoastAvoidDetection).await {
        COAST_AVOID_DETECTION_ACTIVE.store(false, Ordering::Relaxed);
    }
}

/// Request the coast-avoid detection test mode to stop.
pub fn stop_coast_avoid_detection_test() {
    COAST_AVOID_DETECTION_ACTIVE.store(false, Ordering::Relaxed);
    COAST_AVOID_DETECTION_STOP.signal(());
}

/// Spawn the test task via the controller.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner) {
    spawner.spawn(coast_avoid_detection_task().unwrap());
}

/// Coast-avoid forward-drive speed used during the test (matches `FORWARD_SPEED`).
const TEST_SPEED: u8 = 80;

/// Coast-avoid detection test — mirrors the exact sensor config used during
/// the forward-drive phase and shows which sensors are detecting, what the
/// combined flag is, and whether coast-avoid would have issued `EmergencyBrake`.
#[embassy_executor::task]
async fn coast_avoid_detection_task() {
    // Mirror coast-avoid sensor setup: centered ultrasonic with obstacle detection.
    start_ultrasonic_centered_obstacle_detect();
    display_update(DisplayAction::Clear).await;

    // Spin motors at coast-avoid forward speed to reproduce motor PWM noise
    // conditions.  **Hold or block the robot** so it does not drive away.
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true }).await;
    Timer::after(Duration::from_millis(100)).await;
    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: TEST_SPEED.cast_signed(),
        right_speed: TEST_SPEED.cast_signed(),
    })
    .await;

    // Clear any pending stop signal.
    while COAST_AVOID_DETECTION_STOP.signaled() {
        COAST_AVOID_DETECTION_STOP.wait().await;
    }

    let mut last_combined = false;

    loop {
        match select(
            COAST_AVOID_DETECTION_STOP.wait(),
            Timer::after(Duration::from_millis(100)),
        )
        .await
        {
            Either::First(()) => break,
            Either::Second(()) => {
                if !COAST_AVOID_DETECTION_ACTIVE.load(Ordering::Relaxed) {
                    break;
                }

                let ir = perception::is_ir_obstacle_detected();
                let us = perception::is_ultrasonic_obstacle_detected();
                let combined = perception::is_obstacle_detected();

                // Log state transitions.
                if combined != last_combined {
                    if combined {
                        info!(
                            "coast-avoid-detection-test: OBSTACLE — IR={} US={} — would trigger EmergencyBrake",
                            ir, us
                        );
                    } else {
                        info!("coast-avoid-detection-test: clear — IR={} US={}", ir, us);
                    }
                    last_combined = combined;
                }

                // Header line.
                {
                    let mut s: String<20> = String::new();
                    let _ = s.push_str("C-A Detect Test");
                    display_update(DisplayAction::ShowText(s, 0)).await;
                }

                // IR status.
                {
                    let mut s: String<20> = String::new();
                    if ir {
                        let _ = s.push_str("IR: OBSTACLE");
                    } else {
                        let _ = s.push_str("IR: clear");
                    }
                    display_update(DisplayAction::ShowText(s, 1)).await;
                }

                // US status + combined + action.
                {
                    let mut s: String<20> = String::new();
                    if us {
                        let _ = s.push_str("US: OBSTACLE");
                    } else {
                        let _ = s.push_str("US: clear");
                    }
                    display_update(DisplayAction::ShowText(s, 2)).await;
                }

                // Combined: shows whether EmergencyBrake would fire.
                {
                    let mut s: String<20> = String::new();
                    if combined {
                        let _ = s.push_str("=> BRAKE!");
                    } else {
                        let _ = s.push_str("=> coast");
                    }
                    display_update(DisplayAction::ShowText(s, 3)).await;
                }
            }
        }
    }

    stop_ultrasonic_measurements();
    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
    crate::system::state::perception::clear_ultrasonic_data().await;
    release_testmode();
    COAST_AVOID_DETECTION_ACTIVE.store(false, Ordering::Relaxed);
}
