//! Basic motor test mode task.
//!
//! Spins each motor at 50% and displays its encoder count.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use heapless::String;

use super::{TestCommand, release_testmode, request_start};
use crate::task::{
    drive::{clear_encoder_measurement, get_latest_encoder_measurement},
    io::display::{DisplayAction, display_update},
    motor_driver::{self, Motor, MotorCommand, Track},
    sensors::encoders::{self, EncoderCommand},
};

/// Signal used to stop the basic motor test mode.
static BASIC_MOTOR_TEST_STOP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Tracks whether the basic motor test mode is active.
static BASIC_MOTOR_TEST_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Request the basic motor test mode to start (spawns the task on demand).
pub async fn start_basic_motor_test_mode() {
    if BASIC_MOTOR_TEST_ACTIVE.swap(true, Ordering::Relaxed) {
        return;
    }

    if !request_start(TestCommand::BasicMotor).await {
        BASIC_MOTOR_TEST_ACTIVE.store(false, Ordering::Relaxed);
    }
}

/// Request the basic motor test mode to stop.
pub fn stop_basic_motor_test_mode() {
    BASIC_MOTOR_TEST_ACTIVE.store(false, Ordering::Relaxed);
    BASIC_MOTOR_TEST_STOP_SIGNAL.signal(());
}

/// Spawn the basic motor test task via the controller.
pub(super) fn spawn(spawner: Spawner) {
    match basic_motor_test_task() {
        Ok(token) => spawner.spawn(token),
        Err(err) => {
            defmt::warn!("Failed to spawn basic motor test task: {:?}", err);
        }
    }
}

#[derive(Clone, Copy)]
/// Encoder source for the motor under test.
enum EncoderKind {
    /// Left front encoder counter.
    LeftFront,
    /// Left rear encoder counter.
    LeftRear,
    /// Right front encoder counter.
    RightFront,
    /// Right rear encoder counter.
    RightRear,
}

#[derive(Clone, Copy)]
/// Motor configuration for the test loop.
struct MotorSpec {
    /// Display name for the motor.
    name: &'static str,
    /// Track side (left/right).
    track: Track,
    /// Motor position (front/rear).
    motor: Motor,
    /// Encoder channel associated with this motor.
    encoder: EncoderKind,
}

/// Basic motor test mode runner.
#[allow(clippy::too_many_lines)]
#[embassy_executor::task]
async fn basic_motor_test_task() {
    display_update(DisplayAction::Clear).await;

    // Clear any pending stop signal so the next test doesn't end immediately.
    while BASIC_MOTOR_TEST_STOP_SIGNAL.signaled() {
        BASIC_MOTOR_TEST_STOP_SIGNAL.wait().await;
    }

    // Enable both motor drivers before running the test.
    motor_driver::send_motor_command(MotorCommand::SetDriverEnable {
        track: Track::Left,
        enabled: true,
    })
    .await;
    motor_driver::send_motor_command(MotorCommand::SetDriverEnable {
        track: Track::Right,
        enabled: true,
    })
    .await;
    Timer::after(Duration::from_millis(10)).await;

    // Ensure encoders are sampling and clean.
    encoders::send_command(EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(100)).await;
    encoders::send_command(EncoderCommand::Reset).await;
    Timer::after(Duration::from_millis(100)).await;
    clear_encoder_measurement().await;
    encoders::send_command(EncoderCommand::Start { interval_ms: 50 }).await;
    Timer::after(Duration::from_millis(100)).await;

    let motors = [
        MotorSpec {
            name: "Left Front",
            track: Track::Left,
            motor: Motor::Front,
            encoder: EncoderKind::LeftFront,
        },
        MotorSpec {
            name: "Left Rear",
            track: Track::Left,
            motor: Motor::Rear,
            encoder: EncoderKind::LeftRear,
        },
        MotorSpec {
            name: "Right Front",
            track: Track::Right,
            motor: Motor::Front,
            encoder: EncoderKind::RightFront,
        },
        MotorSpec {
            name: "Right Rear",
            track: Track::Right,
            motor: Motor::Rear,
            encoder: EncoderKind::RightRear,
        },
    ];

    'test: loop {
        for motor in motors {
            if !BASIC_MOTOR_TEST_ACTIVE.load(Ordering::Relaxed) {
                break 'test;
            }

            let mut header: String<20> = String::new();
            let _ = header.push_str("Basic Motor Test");
            display_update(DisplayAction::ShowText(header, 0)).await;

            let mut line1: String<20> = String::new();
            let _ = line1.push_str(motor.name);
            display_update(DisplayAction::ShowText(line1, 1)).await;

            let mut line3: String<20> = String::new();
            let _ = line3.push_str("Press to exit");
            display_update(DisplayAction::ShowText(line3, 3)).await;

            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(100)).await;
            encoders::send_command(EncoderCommand::Reset).await;
            Timer::after(Duration::from_millis(100)).await;
            clear_encoder_measurement().await;

            motor_driver::send_motor_command(MotorCommand::SetSpeed {
                track: motor.track,
                motor: motor.motor,
                speed: 50,
            })
            .await;

            for _ in 0..20 {
                match select(
                    BASIC_MOTOR_TEST_STOP_SIGNAL.wait(),
                    Timer::after(Duration::from_millis(100)),
                )
                .await
                {
                    Either::First(()) => break 'test,
                    Either::Second(()) => {
                        if !BASIC_MOTOR_TEST_ACTIVE.load(Ordering::Relaxed) {
                            break 'test;
                        }

                        let count =
                            get_latest_encoder_measurement()
                                .await
                                .map_or(0, |measurement| match motor.encoder {
                                    EncoderKind::LeftFront => measurement.left_front,
                                    EncoderKind::LeftRear => measurement.left_rear,
                                    EncoderKind::RightFront => measurement.right_front,
                                    EncoderKind::RightRear => measurement.right_rear,
                                });

                        let mut line2: String<20> = String::new();
                        let _ = core::fmt::write(&mut line2, format_args!("ENC: {count:>6}"));
                        display_update(DisplayAction::ShowText(line2, 2)).await;
                    }
                }
            }

            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(200)).await;
        }
    }

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    encoders::send_command(EncoderCommand::Stop).await;
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
    release_testmode();
    BASIC_MOTOR_TEST_ACTIVE.store(false, Ordering::Relaxed);
}
