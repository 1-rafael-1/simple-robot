//! Testing task modules.
//!
//! Provides on-demand test mode tasks spawned via a controller task.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

pub mod basic_motor;
pub mod imu_6axis;
pub mod imu_9axis;
pub mod ir_ultrasonic;
pub mod sequence;
pub mod ultrasonic_sweep;

pub use basic_motor::{start_basic_motor_test_mode, stop_basic_motor_test_mode};
pub use imu_6axis::{start_imu6_test_mode, stop_imu6_test_mode};
pub use imu_9axis::{start_imu_test_mode, stop_imu_test_mode};
pub use ir_ultrasonic::{start_ir_ultrasonic_test_mode, stop_ir_ultrasonic_test_mode};
pub use sequence::start_testing_sequence;
pub use ultrasonic_sweep::{start_ultrasonic_sweep_test_mode, stop_ultrasonic_sweep_test_mode};

#[derive(Clone, Copy)]
/// Command sent to the testmode controller.
pub(super) enum TestCommand {
    /// Spawn the combined test sequence.
    Sequence,
    /// Spawn the IMU telemetry test.
    Imu,
    /// Spawn the IMU 6-axis telemetry test.
    Imu6,
    /// Spawn the IR + ultrasonic test.
    IrUltrasonic,
    /// Spawn the ultrasonic sweep test.
    UltrasonicSweep,
    /// Spawn the basic motor test.
    BasicMotor,
}

/// Tracks whether any testmode is currently active.
static TESTMODE_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Command channel for testmode spawn requests.
static TESTMODE_COMMAND: Channel<CriticalSectionRawMutex, TestCommand, 4> = Channel::new();

/// Initialize testmode support (spawns the controller task).
pub fn init_testing(spawner: Spawner) {
    spawner.must_spawn(testmode_controller(spawner));
}

/// Request that a test be spawned on demand.
/// Returns true if the request was accepted.
pub(super) async fn request_start(command: TestCommand) -> bool {
    if TESTMODE_ACTIVE
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
        .is_err()
    {
        return false;
    }

    TESTMODE_COMMAND.send(command).await;
    true
}

/// Mark the testmode controller as idle again.
pub(super) fn release_testmode() {
    TESTMODE_ACTIVE.store(false, Ordering::Release);
}

/// Controller task that spawns test tasks on demand.
#[embassy_executor::task]
async fn testmode_controller(spawner: Spawner) {
    loop {
        match TESTMODE_COMMAND.receive().await {
            TestCommand::Sequence => sequence::spawn(spawner),
            TestCommand::Imu => imu_9axis::spawn(spawner),
            TestCommand::Imu6 => imu_6axis::spawn(spawner),
            TestCommand::IrUltrasonic => ir_ultrasonic::spawn(spawner),
            TestCommand::UltrasonicSweep => ultrasonic_sweep::spawn(spawner),
            TestCommand::BasicMotor => basic_motor::spawn(spawner),
        }
    }
}
