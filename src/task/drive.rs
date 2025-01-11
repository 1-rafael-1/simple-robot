//! Motor control with encoder-based feedback
//!
//! Implements robot movement control using TB6612FNG dual motor driver and
//! DFRobot FIT0450 DC motors with quadrature encoders.

use core::convert::Infallible;

use defmt::info;
use embassy_rp::{
    gpio::{self},
    pwm::{self, Config, Pwm, PwmError},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Timer};
use tb6612fng::{DriveCommand, MotorError};

use crate::{
    system::{
        event::{self, Events},
        resources::MotorDriverResources,
    },
    task::encoder_read::EncoderMeasurement,
};

/// Dispatches drive commands to the motor control task
static DRIVE_CONTROL: Signal<CriticalSectionRawMutex, Command> = Signal::new();

/// Motor control commands with speed parameters (0-100%)
#[derive(Debug, Clone)]
pub enum Command {
    /// Movement and control commands
    Drive(DriveAction),
    /// Encoder feedback for speed adjustment
    EncoderFeedback(EncoderMeasurement),
}

#[derive(Debug, Clone)]
pub enum DriveAction {
    Left(u8),
    Right(u8),
    Forward(u8),
    Backward(u8),
    Brake,
    Coast,
    Standby,
}

/// Queues a drive command for execution
pub fn send_command(command: Command) {
    DRIVE_CONTROL.signal(command);
}

/// Blocks until next motor command is available
async fn wait_command() -> Command {
    DRIVE_CONTROL.wait().await
}

// Motor shaft encoder characteristics
const PULSES_PER_REV: u32 = 8; // Pulses per full shaft rotation

// The mutexes are not used to share the resources, but rather to enable better code structuring

/// Left motor controller protected by mutex.
pub static LEFT_MOTOR: Mutex<CriticalSectionRawMutex, Option<Motor>> = Mutex::new(None);

/// Right motor controller protected by mutex
pub static RIGHT_MOTOR: Mutex<CriticalSectionRawMutex, Option<Motor>> = Mutex::new(None);

/// Single motor control interface
pub struct Motor {
    motor: tb6612fng::Motor<gpio::Output<'static>, gpio::Output<'static>, Pwm<'static>>,
    forward: bool,
    current_speed: i8,
}

impl Motor {
    /// Converts encoder pulses to motor shaft RPM
    /// RPM is calculated for the motor shaft before the 120:1 gear reduction
    fn calculate_rpm(&self, pulses: u16, elapsed_ms: u32) -> f32 {
        let hz = (pulses as f32 * 1000.0) / elapsed_ms as f32;
        let motor_rpm = (hz / PULSES_PER_REV as f32) * 60.0;
        if self.forward {
            motor_rpm
        } else {
            -motor_rpm
        }
    }

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

    /// Initializes both motors with configured PWM (10kHz) and GPIO pins
    fn setup_motors(
        d: MotorDriverResources,
    ) -> Result<(Self, Self, gpio::Output<'static>), MotorError<Infallible, Infallible, PwmError>> {
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
        let left_motor = Self::new(left_fwd, left_bckw, left_pwm)?;

        // Initialize right motor with its pins
        let right_fwd = gpio::Output::new(d.right_forward_pin, gpio::Level::Low);
        let right_bckw = gpio::Output::new(d.right_backward_pin, gpio::Level::Low);
        let right_pwm = pwm::Pwm::new_output_b(d.right_slice, d.right_pwm_pin, pwm_config);
        let right_motor = Self::new(right_fwd, right_bckw, right_pwm)?;

        // Initialize standby control pin
        let standby = gpio::Output::new(d.standby_pin, gpio::Level::Low);

        Ok((left_motor, right_motor, standby))
    }

    /// Sets motor speed and direction (-100 to +100)
    fn set_speed(&mut self, speed: i8) -> Result<(), MotorError<Infallible, Infallible, PwmError>> {
        self.current_speed = speed;
        self.forward = speed >= 0;

        match speed {
            s if s > 0 => self.motor.drive(DriveCommand::Forward(s as u8))?,
            s if s < 0 => self.motor.drive(DriveCommand::Backward(-s as u8))?,
            _ => self.motor.drive(DriveCommand::Stop)?,
        }
        Ok(())
    }

    /// Actively stops motor using electrical braking
    fn brake(&mut self) -> Result<(), MotorError<Infallible, Infallible, PwmError>> {
        self.current_speed = 0;
        self.motor.drive(DriveCommand::Brake)
    }

    /// Stops motor by letting it spin freely
    fn coast(&mut self) -> Result<(), MotorError<Infallible, Infallible, PwmError>> {
        self.current_speed = 0;
        self.motor.drive(DriveCommand::Stop)
    }

    /// Returns current motor speed setting (-100 to +100)
    fn current_speed(&self) -> i8 {
        self.current_speed
    }
}

/// Detects if robot is performing a stationary rotation
fn is_turning_in_place(left_speed: i8, right_speed: i8) -> bool {
    left_speed == -right_speed && left_speed != 0
}

/// Primary motor control task that processes movement commands and encoder feedback
#[embassy_executor::task]
pub async fn drive(d: MotorDriverResources) {
    // Initialize motors
    let (left_motor, right_motor, stby) = Motor::setup_motors(d).unwrap();

    // Initialize mutexes
    critical_section::with(|_| {
        *LEFT_MOTOR.try_lock().unwrap() = Some(left_motor);
        *RIGHT_MOTOR.try_lock().unwrap() = Some(right_motor);
    });

    // Motor driver standby control
    let mut standby = stby;
    let mut standby_enabled = true;

    // Lock motors for the entire system runtime, since we are the only task that can change motor states
    let mut left = LEFT_MOTOR.lock().await;
    let mut right = RIGHT_MOTOR.lock().await;
    let left = left.as_mut().unwrap();
    let right = right.as_mut().unwrap();

    loop {
        // Process any pending commands
        let command = wait_command().await;

        match command {
            Command::Drive(action) => {
                let left_speed_cmd = left.current_speed();
                let right_speed_cmd = right.current_speed();

                // Wake from standby if movement requested
                if standby_enabled {
                    match action {
                        DriveAction::Forward(_)
                        | DriveAction::Backward(_)
                        | DriveAction::Left(_)
                        | DriveAction::Right(_) => {
                            standby.set_high();
                            standby_enabled = false;
                            Timer::after(Duration::from_millis(100)).await;
                        }
                        _ => {}
                    }
                }

                // Execute drive action
                match action {
                    DriveAction::Forward(speed) => {
                        // Stop first if currently moving backward
                        if left_speed_cmd < 0 || right_speed_cmd < 0 {
                            info!("in conflicting motion, stopping");
                            left.coast().unwrap();
                            right.coast().unwrap();
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
                            left.coast().unwrap();
                            right.coast().unwrap();
                        } else {
                            let new_speed = (-left_speed_cmd + speed as i8).clamp(0, 100);
                            let neg_speed = -new_speed;
                            left.set_speed(neg_speed).unwrap();
                            right.set_speed(neg_speed).unwrap();
                        }
                    }
                    DriveAction::Left(speed) => {
                        if left_speed_cmd > right_speed_cmd {
                            // We're in a right turn, need to counter it
                            if is_turning_in_place(left_speed_cmd, right_speed_cmd) {
                                // If turning in place, stop completely
                                info!("stopping turn-in-place maneuver");
                                left.coast().unwrap();
                                right.coast().unwrap();
                            } else {
                                // If turning while moving, restore original motion
                                let original_speed = (left_speed_cmd + right_speed_cmd) / 2;
                                left.set_speed(original_speed).unwrap();
                                right.set_speed(original_speed).unwrap();
                            }
                        } else {
                            // Initiate or continue left turn
                            let new_left_speed = (left_speed_cmd - speed as i8).clamp(-100, 100);
                            let target_right_speed = (right_speed_cmd + speed as i8).clamp(-100, 100);
                            left.set_speed(new_left_speed).unwrap();
                            right.set_speed(target_right_speed).unwrap();
                        }
                    }
                    DriveAction::Right(speed) => {
                        if right_speed_cmd > left_speed_cmd {
                            // We're in a left turn, need to counter it
                            if is_turning_in_place(left_speed_cmd, right_speed_cmd) {
                                // If turning in place, stop completely
                                info!("stopping turn-in-place maneuver");
                                left.coast().unwrap();
                                right.coast().unwrap();
                            } else {
                                // If turning while moving, restore original motion
                                let original_speed = (left_speed_cmd + right_speed_cmd) / 2;
                                left.set_speed(original_speed).unwrap();
                                right.set_speed(original_speed).unwrap();
                            }
                        } else {
                            // Initiate or continue right turn
                            let new_left_speed = (left_speed_cmd + speed as i8).clamp(-100, 100);
                            let target_right_speed = (right_speed_cmd - speed as i8).clamp(-100, 100);
                            left.set_speed(new_left_speed).unwrap();
                            right.set_speed(target_right_speed).unwrap();
                        }
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
                event::send(Events::DriveCommandExecuted).await;
            }

            Command::EncoderFeedback(measurement) => {
                // Calculate motor RPMs
                let left_rpm = left.calculate_rpm(measurement.left.pulse_count, measurement.left.elapsed_ms);
                let right_rpm = right.calculate_rpm(measurement.right.pulse_count, measurement.right.elapsed_ms);

                // Apply speed adjustments if motors are running
                let left_speed = left.current_speed();
                let right_speed = right.current_speed();

                if left_speed != 0 || right_speed != 0 {
                    // Compare raw motor RPMs since encoders are on motor shaft
                    let rpm_ratio = if left_rpm != 0.0 { right_rpm / left_rpm } else { 1.0 };

                    // Calculate how far we are from perfect ratio
                    // let ratio_error = libm::fabsf(1.0 - rpm_ratio);
                    let ratio_error = (1.0 - rpm_ratio).abs();

                    // Only adjust if error is above threshold
                    if ratio_error > 0.05 {
                        // 5% tolerance
                        // Variable correction factor based on error magnitude
                        let correction_factor = if ratio_error > 0.2 {
                            0.8 // Aggressive correction when far off
                        } else {
                            0.4 // Fine adjustment when closer
                        };

                        let adjustment = 1.0 + ((1.0 - rpm_ratio) * correction_factor);
                        let adjusted_speed = (right_speed as f32 * adjustment).clamp(-100.0, 100.0) as i8;
                        right.set_speed(adjusted_speed).unwrap();

                        // Signal that we're still adjusting
                        event::send(Events::DriveCommandExecuted).await;
                    }
                }
            }
        }
    }
}
