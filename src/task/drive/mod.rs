//! High-level drive control and coordination
//!
//! This module implements the control layer that coordinates between sensing and actuation:
//!
//! # Architecture
//!
//! ```text
//! encoder_read          drive (THIS)          motor_driver
//! ├── Sensing       →   ├── Calibration   →   ├── PWM actuation
//! └── Pulse counts      ├── Motion control    └── Direction control
//!                       ├── Feedback loops
//!                       └── Coordination
//!
//! imu_read          →   drive             →   motor_driver
//! ├── Orientation       ├── Rotation
//! └── Angles            └── Stabilization
//! ```
//!
//! # Responsibilities
//!
//! 1. **Motor Calibration**: Orchestrates the calibration procedure by coordinating
//!    encoder readings with motor commands and saving results to flash
//!
//! 2. **Motion Control**: Implements high-level driving behaviors:
//!    - Direct speed control using motor_driver's -100 to +100 convention
//!    - Differential steering (different left/right speeds)
//!    - Precise rotation using IMU feedback
//!    - Combined rotation + motion maneuvers
//!
//! 3. **Feedback Processing**: Receives sensor data from orchestrator and uses it for:
//!    - Speed adjustments based on encoder feedback
//!    - Rotation control using gyroscope/IMU data
//!    - Tilt compensation for inclines
//!    - Straight-line correction
//!
//! 4. **Task Coordination**: Sends commands to lower-level tasks:
//!    - `motor_driver`: PWM speed and direction commands
//!    - `encoder_read`: Start/stop/reset commands
//!    - `flash_storage`: Save calibration data
//!
//! # Speed Convention
//!
//! This module uses the same speed convention as `motor_driver`:
//! - **-100 to +100**: Full range of motor control
//! - **Positive values**: Forward motion
//! - **Negative values**: Backward motion
//! - **Zero**: Coast (freewheel)
//!
//! No unnecessary abstraction - the motor_driver's interface is clean and simple.
//!
//! # Data Flow
//!
//! ## Commands (from orchestrator or other high-level tasks)
//! ```rust
//! // Direct speed control
//! drive::send_drive_command(DriveCommand::Drive(
//!     DriveAction::SetSpeed { left: 50, right: 50 }
//! )).await;
//!
//! // Differential steering (turn right)
//! drive::send_drive_command(DriveCommand::Drive(
//!     DriveAction::SetSpeed { left: 60, right: 40 }
//! )).await;
//!
//! // Calibration
//! drive::send_drive_command(DriveCommand::RunMotorCalibration).await;
//! ```
//!
//! ## Sensor Feedback (from orchestrator)
//! ```rust
//! // Encoder measurements (forwarded by orchestrator from encoder_read events)
//! drive::send_encoder_measurement(measurement).await;
//!
//! // IMU measurements are forwarded by orchestrator into the drive feedback subsystem.
//! // The drive task can poll them on a timer tick when implementing rotation/tilt control.
//! ```
//!
//! # Important: Event Channel Usage
//!
//! This task does NOT directly consume system events. All sensor data is forwarded
//! by the orchestrator via dedicated channels (`send_encoder_measurement()`) to
//! prevent multiple tasks from competing for events on the system event channel.
//!
//! # Control Modes
//!
//! - **Direct Control**: Manual speed commands (-100 to +100 per track)
//! - **Calibration Mode**: Automated motor matching procedure
//! - **Rotation Control**: IMU-based precise turning
//! - **Feedback Control**: Encoder-based speed adjustment (TODO)
//! - **Straight-Line**: IMU-based drift correction (TODO)

#![allow(clippy::too_many_arguments)]

use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};

use crate::{
    system::state::SYSTEM_STATE,
    task::{
        encoder_read,
        motor_driver::{self, MotorCommand},
    },
};

// Submodules
mod calibration;
mod compensation;
mod control;
pub mod feedback;
pub mod types;

// Re-export public API
// Internal imports
use calibration::{run_imu_calibration, run_motor_calibration};
use control::RotationState;
pub use feedback::{
    send_accel_measurement, send_gyro_measurement, send_mag_measurement, try_send_encoder_measurement,
    try_send_imu_measurement,
};
pub use types::{DriveAction, DriveCommand};

/// Command signal for drive control
///
/// Used by orchestrator and other high-level tasks to send drive commands
static DRIVE_CONTROL: Signal<CriticalSectionRawMutex, DriveCommand> = Signal::new();

/// Send a drive command for execution
///
/// Non-blocking signal delivery for drive commands
pub fn send_drive_command(command: DriveCommand) {
    DRIVE_CONTROL.signal(command);
}

/// Wait for next drive command (internal use)
async fn wait() -> DriveCommand {
    DRIVE_CONTROL.wait().await
}

/// Drive control task - coordinates motion and sensor feedback
///
/// # Architecture
///
/// This is a high-level control task that:
/// - Receives drive commands via signal
/// - Receives encoder feedback via channel (from orchestrator)
/// - Receives IMU feedback via command (from orchestrator)
/// - Sends motor commands to motor_driver task
/// - Coordinates calibration procedures
///
/// # Sensor Data Flow
///
/// Sensor tasks → Events → Orchestrator → Drive task (this) → Motor driver
///
/// The orchestrator forwards relevant sensor events to this task via dedicated
/// channels rather than having this task consume system events directly.
#[embassy_executor::task]
pub async fn drive() {
    // Motor driver standby control - now handled by port_expander task via motor_driver
    let mut standby_enabled = true;

    // Rotation state tracking
    let mut rotation_state: Option<RotationState> = None;

    // Drift compensation state (encoder-based straight-line correction)
    #[derive(Clone, Copy)]
    struct DriftCompensationState {
        enabled: bool,
        base_left: i8,
        base_right: i8,
        adjusted_left: i8,
        adjusted_right: i8,
        last_applied_ms: u32,
        last_encoder_timestamp_ms: u32,
    }

    impl DriftCompensationState {
        fn new() -> Self {
            Self {
                enabled: false,
                base_left: 0,
                base_right: 0,
                adjusted_left: 0,
                adjusted_right: 0,
                last_applied_ms: 0,
                last_encoder_timestamp_ms: 0,
            }
        }
    }

    async fn drift_tick_ms() {
        // Keep this modest; encoder sampling is 200ms, so 20ms is plenty to pick up new samples
        // without busy-waiting.
        Timer::after(Duration::from_millis(20)).await;
    }

    async fn run_drift_compensation_step(drift: &mut DriftCompensationState, rotation_state: &Option<RotationState>) {
        if !drift.enabled || rotation_state.is_some() {
            return;
        }

        if let Some(measurement) = feedback::get_latest_encoder_measurement().await {
            // Only process each encoder sample once
            if measurement.timestamp_ms == 0 || measurement.timestamp_ms == drift.last_encoder_timestamp_ms {
                return;
            }
            drift.last_encoder_timestamp_ms = measurement.timestamp_ms;

            let data = crate::task::drive::compensation::calculate_track_averages(measurement);

            // If nothing moved (very low speed or stopped), ignore the sample.
            if data.all_zero() {
                return;
            }

            // If we detect an anomaly (e.g., one encoder stuck at zero), disable compensation.
            if data.has_single_motor_zero_anomaly() {
                drift.enabled = false;
                encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
                return;
            }

            let diff_percent = crate::task::drive::compensation::calculate_speed_difference(&data);

            let action = crate::task::drive::compensation::determine_compensation(
                diff_percent,
                drift.adjusted_left,
                drift.adjusted_right,
            );

            let (new_left, new_right) = crate::task::drive::compensation::apply_compensation_action(
                action,
                drift.adjusted_left,
                drift.adjusted_right,
            );

            // Only send motor update if anything actually changes.
            if new_left != drift.adjusted_left || new_right != drift.adjusted_right {
                let prev_left = drift.adjusted_left;
                let prev_right = drift.adjusted_right;

                drift.adjusted_left = new_left;
                drift.adjusted_right = new_right;
                drift.last_applied_ms = data.timestamp_ms;

                info!(
                    "drift_comp: diff={}% action={:?} speeds L:{}->{} R:{}->{} (ctrs lf:{} lr:{} rf:{} rr:{})",
                    diff_percent,
                    action,
                    prev_left,
                    drift.adjusted_left,
                    prev_right,
                    drift.adjusted_right,
                    data.left_front,
                    data.left_rear,
                    data.right_front,
                    data.right_rear
                );

                motor_driver::send_motor_command(MotorCommand::SetTracks {
                    left_speed: drift.adjusted_left,
                    right_speed: drift.adjusted_right,
                })
                .await;

                let mut state = SYSTEM_STATE.lock().await;
                state.left_track_speed = drift.adjusted_left;
                state.right_track_speed = drift.adjusted_right;
            }
        }
    }

    let mut drift = DriftCompensationState::new();

    loop {
        let command_or_tick = select(wait(), drift_tick_ms()).await;

        match command_or_tick {
            Either::First(command) => match command {
                DriveCommand::Drive(action) => {
                    // Wake from standby if movement requested
                    if standby_enabled {
                        match action {
                            DriveAction::SetSpeed { .. } | DriveAction::RotateExact { .. } => {
                                motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true })
                                    .await;
                                standby_enabled = false;
                                Timer::after(Duration::from_millis(100)).await;
                            }
                            _ => {}
                        }
                    }

                    // Clear rotation state unless this is a rotation command
                    if !matches!(action, DriveAction::RotateExact { .. }) {
                        // rotation_state = None;
                    }

                    // Execute drive action
                    match action {
                        DriveAction::SetSpeed { left, right } => {
                            // Clamp speeds to valid range
                            let left_clamped = left.clamp(-100, 100);
                            let right_clamped = right.clamp(-100, 100);

                            // Detect straight-line driving (speeds equal or very close)
                            // Enable drift compensation only in that mode.
                            if (left_clamped - right_clamped).abs() <= 2 && left_clamped != 0 {
                                drift.enabled = true;
                                drift.base_left = left_clamped;
                                drift.base_right = right_clamped;
                                drift.adjusted_left = left_clamped;
                                drift.adjusted_right = right_clamped;

                                // Start encoder sampling at configured interval and reset counters so
                                // measurements represent per-window deltas.
                                encoder_read::send_command(encoder_read::EncoderCommand::Start {
                                    interval_ms: crate::task::drive::types::DRIFT_COMPENSATION_INTERVAL_MS,
                                })
                                .await;
                                encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
                            } else {
                                // Turning/differential or stop: disable compensation and stop encoder sampling
                                drift.enabled = false;
                                encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
                            }

                            // Apply command (initially base, then adjusted as feedback arrives)
                            motor_driver::send_motor_command(MotorCommand::SetTracks {
                                left_speed: drift.adjusted_left,
                                right_speed: drift.adjusted_right,
                            })
                            .await;

                            // Update system state directly
                            let mut state = SYSTEM_STATE.lock().await;
                            state.left_track_speed = drift.adjusted_left;
                            state.right_track_speed = drift.adjusted_right;
                        }
                        DriveAction::RotateExact {
                            degrees,
                            direction,
                            motion,
                        } => {
                            // Initialize new rotation state
                            rotation_state = Some(RotationState::new(degrees, direction, motion));

                            // Apply initial motor speeds for rotation
                            let (left_speed, right_speed) = rotation_state.as_ref().unwrap().calculate_motor_speeds();

                            // Use SetTracks for atomic update
                            motor_driver::send_motor_command(MotorCommand::SetTracks {
                                left_speed,
                                right_speed,
                            })
                            .await;

                            // Update system state
                            let mut state = SYSTEM_STATE.lock().await;
                            state.left_track_speed = left_speed;
                            state.right_track_speed = right_speed;
                        }
                        DriveAction::Coast => {
                            info!("coast");

                            // Disable compensation on stop/coast and stop encoder sampling
                            drift.enabled = false;
                            encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;

                            motor_driver::send_motor_command(MotorCommand::CoastAll).await;

                            // Update system state
                            let mut state = SYSTEM_STATE.lock().await;
                            state.left_track_speed = 0;
                            state.right_track_speed = 0;
                        }
                        DriveAction::Brake => {
                            info!("brake");

                            // Disable compensation on brake and stop encoder sampling
                            drift.enabled = false;
                            encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;

                            motor_driver::send_motor_command(MotorCommand::BrakeAll).await;

                            // Update system state
                            let mut state = SYSTEM_STATE.lock().await;
                            state.left_track_speed = 0;
                            state.right_track_speed = 0;
                        }
                        DriveAction::Standby => {
                            // Disable compensation in standby and stop encoder sampling
                            drift.enabled = false;
                            encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;

                            if !standby_enabled {
                                motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
                                Timer::after(Duration::from_millis(100)).await;
                                motor_driver::send_motor_command(MotorCommand::CoastAll).await;
                                Timer::after(Duration::from_millis(100)).await;
                                motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false })
                                    .await;
                                standby_enabled = true;

                                // Update system state
                                let mut state = SYSTEM_STATE.lock().await;
                                state.left_track_speed = 0;
                                state.right_track_speed = 0;
                            }
                        }
                    }

                    // Drive command executed
                }

                // IMU feedback is no longer delivered as a DriveCommand.
                // When implementing rotation/tilt control, poll IMU measurements from the
                // drive feedback subsystem on a timer tick (similar to drift compensation).
                DriveCommand::RunMotorCalibration => {
                    info!("Starting motor calibration procedure");
                    run_motor_calibration().await;
                    info!("Motor calibration procedure completed");
                }

                DriveCommand::RunImuCalibration => {
                    info!("Starting IMU calibration procedure");
                    run_imu_calibration().await;
                    info!("IMU calibration procedure completed");
                }
            },
            Either::Second(()) => {
                run_drift_compensation_step(&mut drift, &rotation_state).await;
            }
        }
    }
}
