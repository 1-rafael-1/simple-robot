//! Motor and motion control with encoder and gyroscope feedback
//!
//! Implements robot movement control using:
//! - TB6612FNG dual motor driver
//! - DFRobot FIT0450 DC motors with quadrature encoders
//! - MPU-9250 IMU for precise rotation control
//!
//! The control system supports multiple operation modes:
//! 1. Direct control with torque bias for manual steering
//! 2. Precise rotation control using gyroscope feedback
//! 3. Combined motion with simultaneous rotation and forward/backward movement
//!
//! Motion commands are processed asynchronously while maintaining smooth
//! transitions between different movement states.

use core::convert::Infallible;

use defmt::info;
use embassy_rp::{
    gpio::{self},
    pwm::{self, Config, Pwm, PwmError},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Timer};
use tb6612fng::{DriveCommand as DriverCommand, MotorError};

use crate::{
    system::{
        event::{self, Events},
        resources::MotorDriverResources,
    },
    task::{encoder_read::EncoderMeasurement, imu_read::ImuMeasurement},
};

/// Dispatches drive commands to the motor control task
static DRIVE_CONTROL: Signal<CriticalSectionRawMutex, DriveCommand> = Signal::new();

/// Combined motor control and sensor feedback commands
#[derive(Debug, Clone)]
pub enum DriveCommand {
    /// Movement and control commands
    Drive(DriveAction),
    /// Encoder feedback for speed adjustment
    EncoderFeedback(EncoderMeasurement),
    /// Gyroscope feedback for rotation control
    ImuFeedback(ImuMeasurement),
}

/// Motion control commands with associated parameters
#[derive(Debug, Clone, PartialEq)]
pub enum DriveAction {
    /// Differential steering by reducing torque on one side
    TorqueBias {
        /// Which motor side to reduce power on
        reduce_side: MotorSide,
        /// Amount of power reduction (0-100%)
        bias_amount: u8,
    },
    /// Move forward at specified speed (0-100%)
    Forward(u8),
    /// Move backward at specified speed (0-100%)
    Backward(u8),
    /// Active electrical braking
    Brake,
    /// Passive stop (freewheeling)
    Coast,
    /// Enter low-power standby mode
    Standby,
    /// Precise rotation with optional forward/backward motion
    RotateExact {
        /// Rotation angle in degrees
        degrees: f32,
        /// Rotation direction
        direction: RotationDirection,
        /// Combined motion type
        motion: RotationMotion,
    },
}

/// Motor side selection for differential steering
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorSide {
    /// Left side motor
    Left,
    /// Right side motor
    Right,
}

/// Rotation direction for precise turning
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RotationDirection {
    /// Clockwise rotation (right turn)
    Clockwise,
    /// Counter-clockwise rotation (left turn)
    CounterClockwise,
}

/// Combined motion options during rotation
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RotationMotion {
    /// Rotate in place
    Stationary,
    /// Rotate while maintaining forward motion
    WhileForward(u8),
    /// Rotate while maintaining backward motion
    WhileBackward(u8),
}

// Control parameters
/// Number of encoder pulses per motor shaft revolution
const PULSES_PER_REV: u32 = 8;
/// Maximum rotation speed (0-100%)
const ROTATION_SPEED_MAX: u8 = 50;
/// Minimum rotation speed to overcome friction
const ROTATION_SPEED_MIN: u8 = 20;
/// Acceptable angle error in degrees
const ROTATION_TOLERANCE_DEG: f32 = 2.0;
/// Maximum speed differential during combined motion
const SPEED_DIFF_MAX: i8 = 30;

/// Queues a drive command for execution
pub fn send_drive_command(command: DriveCommand) {
    DRIVE_CONTROL.signal(command);
}

/// Blocks until next motor command is available
async fn wait() -> DriveCommand {
    DRIVE_CONTROL.wait().await
}

/// Left motor controller protected by mutex.
pub static LEFT_MOTOR: Mutex<CriticalSectionRawMutex, Option<Motor>> = Mutex::new(None);

/// Right motor controller protected by mutex
pub static RIGHT_MOTOR: Mutex<CriticalSectionRawMutex, Option<Motor>> = Mutex::new(None);

/// Tracks state during precise rotation maneuvers
struct RotationState {
    /// Target rotation angle in degrees
    target_angle: f32,
    /// Accumulated rotation so far (in degrees)
    accumulated_angle: f32,
    /// Last measured yaw angle (in degrees)
    last_yaw: Option<f32>,
    /// Last update timestamp
    last_update_ms: u32,
    /// Direction of rotation
    direction: RotationDirection,
    /// Type of motion during rotation
    motion: RotationMotion,
    /// Base speed before rotation adjustment
    base_speed: i8,
}

impl RotationState {
    /// Creates new rotation tracking state
    fn new(target_angle: f32, direction: RotationDirection, motion: RotationMotion) -> Self {
        let base_speed = match motion {
            RotationMotion::Stationary => 0,
            RotationMotion::WhileForward(speed) => speed as i8,
            RotationMotion::WhileBackward(speed) => -(speed as i8),
        };

        Self {
            target_angle,
            accumulated_angle: 0.0,
            last_yaw: None,
            last_update_ms: 0,
            direction,
            motion,
            base_speed,
        }
    }

    /// Updates rotation state with new IMU measurement
    /// Returns true if target angle has been reached
    fn update(&mut self, measurement: &ImuMeasurement) -> bool {
        // Calculate yaw change since last reading
        if let Some(last_yaw) = self.last_yaw {
            let mut yaw_change = measurement.orientation.yaw - last_yaw;

            // Handle wraparound at ±180 degrees
            if yaw_change > 180.0 {
                yaw_change -= 360.0;
            } else if yaw_change < -180.0 {
                yaw_change += 360.0;
            }

            // Accumulate based on desired direction
            self.accumulated_angle += match self.direction {
                RotationDirection::Clockwise => -yaw_change,
                RotationDirection::CounterClockwise => yaw_change,
            };
        }

        self.last_yaw = Some(measurement.orientation.yaw);
        self.last_update_ms = measurement.timestamp_ms;

        // Check if we've reached target angle within tolerance
        (self.accumulated_angle - self.target_angle).abs() <= ROTATION_TOLERANCE_DEG
    }

    /// Calculates appropriate motor speeds for current rotation state
    /// Returns (left_speed, right_speed)
    fn calculate_motor_speeds(&self) -> (i8, i8) {
        // Calculate rotation component based on remaining angle
        let remaining_degrees = (self.target_angle - self.accumulated_angle).abs();
        let rotation_speed = if remaining_degrees < 10.0 {
            // Linear interpolation between min and max speed
            let speed_range = ROTATION_SPEED_MAX - ROTATION_SPEED_MIN;
            let speed_factor = remaining_degrees / 10.0;
            (ROTATION_SPEED_MIN as f32 + (speed_range as f32 * speed_factor)) as u8
        } else {
            ROTATION_SPEED_MAX
        };

        match self.motion {
            RotationMotion::Stationary => {
                // Simple differential drive for in-place rotation
                match self.direction {
                    RotationDirection::Clockwise => (rotation_speed as i8, -(rotation_speed as i8)),
                    RotationDirection::CounterClockwise => (-(rotation_speed as i8), rotation_speed as i8),
                }
            }
            RotationMotion::WhileForward(_) | RotationMotion::WhileBackward(_) => {
                // Combine base motion with rotation
                let rotation_diff = rotation_speed.min(SPEED_DIFF_MAX as u8) as i8;

                match self.direction {
                    RotationDirection::Clockwise => {
                        (self.base_speed, (self.base_speed - rotation_diff).clamp(-100, 100))
                    }
                    RotationDirection::CounterClockwise => {
                        ((self.base_speed - rotation_diff).clamp(-100, 100), self.base_speed)
                    }
                }
            }
        }
    }

    /// Returns the base speed for maintaining motion after rotation
    fn continuation_speed(&self) -> Option<i8> {
        match self.motion {
            RotationMotion::Stationary => None,
            RotationMotion::WhileForward(_) | RotationMotion::WhileBackward(_) => Some(self.base_speed),
        }
    }
}

/// Motor power adjustment based on tilt
struct TiltCompensation {
    /// Maximum power adjustment factor
    max_adjustment: f32,
    /// Maximum tilt angle in degrees
    max_tilt_angle: f32,
}

impl TiltCompensation {
    /// Creates a new tilt compensation instance
    /// with the given maximum adjustment and tilt angle
    fn new(max_adjustment: f32, max_tilt_angle: f32) -> Self {
        Self {
            max_adjustment,
            max_tilt_angle,
        }
    }

    /// Calculate power adjustment factor based on tilt angle
    fn calculate_adjustment(&mut self, tilt_degrees: f32) -> f32 {
        (tilt_degrees / self.max_tilt_angle).clamp(-1.0, 1.0) * self.max_adjustment
    }
}

/// Tracks straight-line motion correction
struct StraightLineState {
    /// Initial yaw angle when straight motion started
    target_yaw: f32,
    /// Previous yaw error for derivative calculation
    last_error: Option<f32>,
    /// Last update timestamp for derivative calculation
    last_update_ms: u32,
}

impl StraightLineState {
    /// Maximum correction strength (as percentage of base speed)
    const MAX_CORRECTION: f32 = 0.2; // 20% max speed difference
    /// Proportional control factor
    const P_FACTOR: f32 = 0.1;
    /// Derivative control factor
    const D_FACTOR: f32 = 0.05;

    fn new(initial_yaw: f32) -> Self {
        Self {
            target_yaw: initial_yaw,
            last_error: None,
            last_update_ms: 0,
        }
    }

    /// Calculates motor speed corrections to maintain straight line
    /// Returns (left_adjustment, right_adjustment) as factors to multiply with base speed
    fn calculate_correction(&mut self, current_yaw: f32, timestamp_ms: u32) -> (f32, f32) {
        let mut yaw_error = current_yaw - self.target_yaw;

        // Handle wraparound at ±180 degrees
        if yaw_error > 180.0 {
            yaw_error -= 360.0;
        } else if yaw_error < -180.0 {
            yaw_error += 360.0;
        }

        // Calculate P term
        let p_correction = yaw_error * Self::P_FACTOR;

        // Calculate D term (rate of change of error)
        let d_correction = if let Some(last_error) = self.last_error {
            let dt = (timestamp_ms - self.last_update_ms) as f32 / 1000.0;
            if dt > 0.0 {
                let error_rate = (yaw_error - last_error) / dt;
                error_rate * Self::D_FACTOR
            } else {
                0.0
            }
        } else {
            0.0
        };

        // Update state for next calculation
        self.last_error = Some(yaw_error);
        self.last_update_ms = timestamp_ms;

        // Combine P and D terms and clamp to max correction
        let correction = (p_correction + d_correction).clamp(-Self::MAX_CORRECTION, Self::MAX_CORRECTION);

        if correction > 0.0 {
            // Drifting right, speed up right motor
            (1.0 - correction, 1.0 + correction)
        } else {
            // Drifting left, speed up left motor
            (1.0 + correction.abs(), 1.0 - correction.abs())
        }
    }
}

/// Single motor control interface
pub struct Motor {
    motor: tb6612fng::Motor<gpio::Output<'static>, gpio::Output<'static>, Pwm<'static>>,
    forward: bool,
    current_speed: i8,
}

impl Motor {
    /// Creates a new motor instance with the given GPIO and PWM resources
    fn new(
        fwd: gpio::Output<'static>,
        bckw: gpio::Output<'static>,
        pwm: Pwm<'static>,
    ) -> Result<Self, MotorError<Infallible, Infallible, PwmError>> {
        Ok(Self {
            motor: tb6612fng::Motor::new(fwd, bckw, pwm)?,
            forward: true,
            current_speed: 0,
        })
    }

    /// Converts encoder pulses to motor shaft RPM
    /// RPM is calculated for the motor shaft before the 120:1 gear reduction
    fn calculate_rpm(&self, pulses: u16, elapsed_ms: u32) -> f32 {
        let hz = (pulses as f32 * 1000.0) / elapsed_ms as f32;
        let motor_rpm = (hz / PULSES_PER_REV as f32) * 60.0;
        if self.forward { motor_rpm } else { -motor_rpm }
    }

    /// Sets motor speed and direction (-100 to +100)
    fn set_speed(&mut self, speed: i8) -> Result<(), MotorError<Infallible, Infallible, PwmError>> {
        self.current_speed = speed;
        self.forward = speed >= 0;

        match speed {
            s if s > 0 => self.motor.drive(DriverCommand::Forward(s as u8))?,
            s if s < 0 => self.motor.drive(DriverCommand::Backward(-s as u8))?,
            _ => self.motor.drive(DriverCommand::Stop)?,
        }
        Ok(())
    }

    /// Sets motor speed with tilt compensation
    fn set_speed_with_tilt(
        &mut self,
        base_speed: i8,
        tilt_degrees: f32,
    ) -> Result<(), MotorError<Infallible, Infallible, PwmError>> {
        let mut tilt_compensation = TiltCompensation::new(0.3, 45.0);
        let tilt_adjustment = tilt_compensation.calculate_adjustment(tilt_degrees);
        let adjusted_speed = (base_speed as f32 * (1.0 + tilt_adjustment)) as i8;
        self.set_speed(adjusted_speed.clamp(-100, 100))
    }

    /// Actively stops motor using electrical braking
    fn brake(&mut self) -> Result<(), MotorError<Infallible, Infallible, PwmError>> {
        self.current_speed = 0;
        self.motor.drive(DriverCommand::Brake)
    }

    /// Stops motor by letting it spin freely
    fn coast(&mut self) -> Result<(), MotorError<Infallible, Infallible, PwmError>> {
        self.current_speed = 0;
        self.motor.drive(DriverCommand::Stop)
    }

    /// Returns current motor speed setting (-100 to +100)
    pub fn current_speed(&self) -> i8 {
        self.current_speed
    }
}

/// Initializes both motors with configured PWM (10kHz) and GPIO pins
fn setup_motors(
    d: MotorDriverResources,
) -> Result<(Motor, Motor, gpio::Output<'static>), MotorError<Infallible, Infallible, PwmError>> {
    // Configure PWM for 10kHz operation
    let desired_freq_hz = 10_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    let mut pwm_config = Config::default();
    pwm_config.divider = divider.into();
    pwm_config.top = period;

    // Initialize left motor with its pins
    let left_fwd = gpio::Output::new(d.left_forward_pin, gpio::Level::Low);
    let left_bckw = gpio::Output::new(d.left_backward_pin, gpio::Level::Low);
    let left_pwm = pwm::Pwm::new_output_a(d.left_slice, d.left_pwm_pin, pwm_config.clone());
    let left_motor = Motor::new(left_fwd, left_bckw, left_pwm)?;

    // Initialize right motor with its pins
    let right_fwd = gpio::Output::new(d.right_forward_pin, gpio::Level::Low);
    let right_bckw = gpio::Output::new(d.right_backward_pin, gpio::Level::Low);
    let right_pwm = pwm::Pwm::new_output_b(d.right_slice, d.right_pwm_pin, pwm_config);
    let right_motor = Motor::new(right_fwd, right_bckw, right_pwm)?;

    // Initialize standby control pin
    let standby = gpio::Output::new(d.standby_pin, gpio::Level::Low);

    Ok((left_motor, right_motor, standby))
}

/// Detects if robot is performing a stationary rotation
fn is_turning_in_place(left_speed: i8, right_speed: i8) -> bool {
    left_speed == -right_speed && left_speed != 0
}

/// Primary motor control task that processes movement commands and sensor feedback
#[embassy_executor::task]
pub async fn drive(d: MotorDriverResources) {
    // Initialize motors
    let (left_motor, right_motor, stby) = setup_motors(d).unwrap();

    // Initialize mutexes
    critical_section::with(|_| {
        *LEFT_MOTOR.try_lock().unwrap() = Some(left_motor);
        *RIGHT_MOTOR.try_lock().unwrap() = Some(right_motor);
    });

    // Motor driver standby control
    let mut standby = stby;
    let mut standby_enabled = true;

    // Rotation state tracking
    let mut rotation_state: Option<RotationState>;

    // Lock motors for the entire system runtime
    let mut left = LEFT_MOTOR.lock().await;
    let mut right = RIGHT_MOTOR.lock().await;
    let left = left.as_mut().unwrap();
    let right = right.as_mut().unwrap();

    // Add straight-line tracking state
    // let straight_line_state: Option<StraightLineState> = None;

    loop {
        // Process any pending commands
        let command = wait().await;

        match command {
            DriveCommand::Drive(action) => {
                let left_speed_cmd = left.current_speed();
                let right_speed_cmd = right.current_speed();

                // Wake from standby if movement requested
                if standby_enabled {
                    match action {
                        DriveAction::Forward(_)
                        | DriveAction::Backward(_)
                        | DriveAction::TorqueBias { .. }
                        | DriveAction::RotateExact { .. } => {
                            standby.set_high();
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
                    DriveAction::Forward(speed) => {
                        // Stop first if currently moving backward
                        if left_speed_cmd < 0 || right_speed_cmd < 0 {
                            info!("in conflicting motion, stopping");
                            send_drive_command(DriveCommand::Drive(DriveAction::Coast));
                        } else {
                            let new_speed = (left_speed_cmd + speed as i8).clamp(0, 100);
                            left.set_speed(new_speed).unwrap();
                            right.set_speed(new_speed).unwrap();
                        }
                    }
                    DriveAction::Backward(speed) => {
                        // Stop first if currently moving forward
                        if left_speed_cmd > 0 || right_speed_cmd > 0 {
                            info!("in conflicting motion, stopping");
                            send_drive_command(DriveCommand::Drive(DriveAction::Coast));
                        } else {
                            let new_speed = (-left_speed_cmd + speed as i8).clamp(0, 100);
                            let neg_speed = -new_speed;
                            left.set_speed(neg_speed).unwrap();
                            right.set_speed(neg_speed).unwrap();
                        }
                    }
                    DriveAction::TorqueBias {
                        reduce_side,
                        bias_amount,
                    } => {
                        if left_speed_cmd > right_speed_cmd {
                            // We're in an opposite bias, need to counter it
                            if is_turning_in_place(left_speed_cmd, right_speed_cmd) {
                                info!("stopping turn-in-place maneuver");
                                send_drive_command(DriveCommand::Drive(DriveAction::Coast));
                            } else {
                                // If turning while moving, restore original motion
                                let original_speed = (left_speed_cmd + right_speed_cmd) / 2;
                                left.set_speed(original_speed).unwrap();
                                right.set_speed(original_speed).unwrap();
                            }
                        } else {
                            // Apply new torque bias
                            match reduce_side {
                                MotorSide::Left => {
                                    let new_left_speed = (left_speed_cmd - bias_amount as i8).clamp(-100, 100);
                                    let target_right_speed = (right_speed_cmd + bias_amount as i8).clamp(-100, 100);
                                    left.set_speed(new_left_speed).unwrap();
                                    right.set_speed(target_right_speed).unwrap();
                                }
                                MotorSide::Right => {
                                    let new_left_speed = (left_speed_cmd + bias_amount as i8).clamp(-100, 100);
                                    let target_right_speed = (right_speed_cmd - bias_amount as i8).clamp(-100, 100);
                                    left.set_speed(new_left_speed).unwrap();
                                    right.set_speed(target_right_speed).unwrap();
                                }
                            }
                        }
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

                        left.set_speed(left_speed).unwrap();
                        right.set_speed(right_speed).unwrap();
                    }
                    DriveAction::Coast => {
                        info!("coast");
                        left.coast().unwrap();
                        right.coast().unwrap();
                    }
                    DriveAction::Brake => {
                        info!("brake");
                        left.brake().unwrap();
                        right.brake().unwrap();
                    }
                    DriveAction::Standby => {
                        if !standby_enabled {
                            left.brake().unwrap();
                            right.brake().unwrap();
                            Timer::after(Duration::from_millis(100)).await;
                            left.coast().unwrap();
                            right.coast().unwrap();
                            Timer::after(Duration::from_millis(100)).await;
                            standby.set_low();
                            standby_enabled = true;
                        }
                    }
                }

                // Notify that drive command was executed
                event::send_event(Events::DriveCommandExecuted(action)).await;
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
        }
    }
}
