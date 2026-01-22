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
//! // IMU measurements (forwarded by orchestrator from imu_read events)
//! drive::send_drive_command(DriveCommand::ImuFeedback(measurement)).await;
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
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};

use crate::task::motor_driver::{self, MotorCommand, Track};

// Submodules
mod calibration;
mod control;
pub mod feedback;
pub mod types;

// Re-export public API
// Internal imports
use calibration::{run_imu_calibration, run_motor_calibration};
use control::{RotationState, TrackState};
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
    // Initialize track state tracking
    let mut left_track = TrackState::new();
    let mut right_track = TrackState::new();

    // Motor driver standby control - now handled by port_expander task via motor_driver
    let mut standby_enabled = true;

    // Rotation state tracking
    let mut rotation_state: Option<RotationState>;

    // Add straight-line tracking state
    // let straight_line_state: Option<StraightLineState> = None;

    loop {
        // Process any pending commands
        let command = wait().await;

        match command {
            DriveCommand::Drive(action) => {
                // Wake from standby if movement requested
                if standby_enabled {
                    match action {
                        DriveAction::SetSpeed { .. } | DriveAction::RotateExact { .. } => {
                            motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true }).await;
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

                        left_state.set_speed(Track::Left, left_clamped).await;
                        right_state.set_speed(Track::Right, right_clamped).await;
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

                        left_state.set_speed(Track::Left, left_speed).await;
                        right_state.set_speed(Track::Right, right_speed).await;
                    }
                    DriveAction::Coast => {
                        info!("coast");
                        left_state.coast(Track::Left).await;
                        right_state.coast(Track::Right).await;
                    }
                    DriveAction::Brake => {
                        info!("brake");
                        left_state.brake(Track::Left).await;
                        right_state.brake(Track::Right).await;
                    }
                    DriveAction::Standby => {
                        if !standby_enabled {
                            left_state.brake(Track::Left).await;
                            right_state.brake(Track::Right).await;
                            Timer::after(Duration::from_millis(100)).await;
                            left_state.coast(Track::Left).await;
                            right_state.coast(Track::Right).await;
                            Timer::after(Duration::from_millis(100)).await;
                            motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false })
                                .await;
                            standby_enabled = true;
                        }
                    }
                }

                // Drive command executed
            }

            DriveCommand::EncoderFeedback(_measurement) => {
                // // Skip encoder feedback during precise rotation
                // if rotation_state.is_some() {
                //     continue;
                // }

                // // Calculate motor RPMs
                // let left_rpm = left.calculate_rpm(measurement.left.pulse_count, measurement.left.elapsed_ms);
                // let right_rpm = right.calculate_rpm(measurement.right.pulse_count, measurement.right.elapsed_ms);

                // // Apply speed adjustments if motors are running
                // let left_speed = left.current_speed();
                // let right_speed = right.current_speed();

                // if left_speed != 0 || right_speed != 0 {
                //     // Compare raw motor RPMs since encoders are on motor shaft
                //     let rpm_ratio = if left_rpm != 0.0 { right_rpm / left_rpm } else { 1.0 };

                //     // Calculate how far we are from perfect ratio
                //     let ratio_error = (1.0 - rpm_ratio).abs();

                //     // Only adjust if error is above threshold
                //     if ratio_error > 0.05 {
                //         // 5% tolerance
                //         // Variable correction factor based on error magnitude
                //         let correction_factor = if ratio_error > 0.2 {
                //             0.8 // Aggressive correction when far off
                //         } else {
                //             0.4 // Fine adjustment when closer
                //         };

                //         let adjustment = 1.0 + ((1.0 - rpm_ratio) * correction_factor);
                //         let adjusted_speed = (right_speed as f32 * adjustment).clamp(-100.0, 100.0) as i8;
                //         right.set_speed(adjusted_speed).unwrap();

                //         // Signal that we're still adjusting
                //         event::send_event(Events::DriveCommandExecuted).await;
                //     }
                // }
            }

            DriveCommand::ImuFeedback(_measurement) => {
                // // Part 1: Tilt Compensation for Forward/Backward Motion
                // // --------------------------------------------------
                // // Only apply tilt compensation when:
                // // - Not in a rotation maneuver
                // // - Motors are running
                // // - Both motors at same speed (pure forward/backward motion)
                // //
                // // Example scenario:
                // // Robot driving forward at 50% speed encounters a 10° incline
                // // - Base speed: 50
                // // - Tilt: +10° (positive = climbing)
                // // - Adjustment factor: ~0.067 (10° / 45° * 0.3)
                // // - New speed: 50 * (1 + 0.067) = 53
                // // This increases power to maintain speed while climbing
                // if !rotation_state.is_some() && (left.current_speed() != 0 || right.current_speed() != 0) {
                //     if left.current_speed() == right.current_speed() {
                //         left.set_speed_with_tilt(left.current_speed(), measurement.orientation.pitch)
                //             .unwrap();
                //         right
                //             .set_speed_with_tilt(right.current_speed(), measurement.orientation.pitch)
                //             .unwrap();
                //     }
                // }

                // // Part 2: Rotation Control
                // // -----------------------
                // // Handle active rotation maneuvers using yaw measurements
                // if let Some(rot_state) = rotation_state.as_mut() {
                //     // Check if target angle reached using yaw tracking
                //     // Example: 90° clockwise turn
                //     // - Target: 90°
                //     // - Current accumulated: 88°
                //     // - Tolerance: ±2°
                //     // - Status: Almost complete, using reduced speed
                //     if rot_state.update(&measurement) {
                //         // Target angle reached
                //         info!("rotation complete");

                //         // Handle post-rotation motion:
                //         // 1. For combined movements (e.g., "rotate while moving forward")
                //         //    continue with the base motion including tilt compensation
                //         // 2. For stationary rotations, stop completely
                //         //
                //         // Example: After 90° turn while moving forward at 30%
                //         // - continue_speed returns Some(30)
                //         // - Apply tilt compensation to maintain 30% on any incline
                //         if let Some(continue_speed) = rot_state.continuation_speed() {
                //             left.set_speed_with_tilt(continue_speed, measurement.orientation.pitch)
                //                 .unwrap();
                //             right
                //                 .set_speed_with_tilt(continue_speed, measurement.orientation.pitch)
                //                 .unwrap();
                //         } else {
                //             // Stationary rotation complete - brake both motors
                //             left.brake().unwrap();
                //             right.brake().unwrap();
                //         }

                //         rotation_state = None;
                //         event::send_event(Events::RotationCompleted).await;
                //     } else {
                //         // Rotation still in progress
                //         // Calculate differential speeds based on:
                //         // - Remaining angle
                //         // - Current motion type (stationary or moving)
                //         // - Direction of rotation
                //         //
                //         // Example: Mid-way through 90° clockwise turn
                //         // - Accumulated: 45°
                //         // - Remaining: 45°
                //         // - Speed: ROTATION_SPEED_MAX (50%)
                //         // - Results in: Left=+50%, Right=-50% for stationary turn
                //         let (left_speed, right_speed) = rot_state.calculate_motor_speeds();
                //         left.set_speed(left_speed).unwrap();
                //         right.set_speed(right_speed).unwrap();
                //     }
                // }

                // // Part 3: Straight-Line Motion Correction
                // // --------------------------------------
                // // Adjust motor speeds to maintain straight line motion

                // // Check if we should initialize straight-line tracking
                // if straight_line_state.is_none()
                //     && left.current_speed() == right.current_speed()
                //     && left.current_speed() != 0
                // {
                //     // Starting new straight-line motion
                //     info!("Starting straight line control at yaw: {}", measurement.orientation.yaw);
                //     straight_line_state = Some(StraightLineState::new(measurement.orientation.yaw));
                // }

                // // Apply straight-line correction if active
                // if let Some(straight_state) = straight_line_state.as_mut() {
                //     if left.current_speed() == right.current_speed() && left.current_speed() != 0 {
                //         let (left_adj, right_adj) =
                //             straight_state.calculate_correction(measurement.orientation.yaw, measurement.timestamp_ms);

                //         let base_speed = left.current_speed();
                //         let left_corrected = (base_speed as f32 * left_adj) as i8;
                //         let right_corrected = (base_speed as f32 * right_adj) as i8;

                //         info!(
                //             "Straight correction L:{} R:{} (base:{})",
                //             left_corrected, right_corrected, base_speed
                //         );

                //         left.set_speed_with_tilt(left_corrected, measurement.orientation.pitch)
                //             .unwrap();
                //         right
                //             .set_speed_with_tilt(right_corrected, measurement.orientation.pitch)
                //             .unwrap();
                //     } else {
                //         // No longer in straight-line motion
                //         straight_line_state = None;
                //     }
                // }
            }

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
        }
    }
}
