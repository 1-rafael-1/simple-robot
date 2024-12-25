//! Motor control implementation with encoder feedback
//!
//! Controls the robot's movement through a TB6612FNG dual motor driver with
//! DFRobot FIT0450 DC motors that include quadrature encoders.

use crate::system::resources::MotorDriverResources;
use crate::task::encoder::{EncoderMeasurement, ENCODER_SIGNAL};
use core::convert::Infallible;
use defmt::info;
use embassy_rp::pwm::PwmError;
use embassy_rp::{
    gpio::{self},
    pwm::{self, Config, Pwm},
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use tb6612fng::{DriveCommand, MotorError};

/// Drive control signal for sending commands to the motor task
static DRIVE_CONTROL: Signal<CriticalSectionRawMutex, Command> = Signal::new();

/// Left motor control protected by mutex
pub static LEFT_MOTOR: Mutex<CriticalSectionRawMutex, Option<Motor>> = Mutex::new(None);

/// Right motor control protected by mutex
pub static RIGHT_MOTOR: Mutex<CriticalSectionRawMutex, Option<Motor>> = Mutex::new(None);

/// Last received encoder measurement
static LAST_MEASUREMENT: Mutex<CriticalSectionRawMutex, Option<EncoderMeasurement>> =
    Mutex::new(None);

/// Motor control commands with speed parameters in range 0-100
#[derive(Debug, Clone)]
pub enum Command {
    Left(u8),
    Right(u8),
    Forward(u8),
    Backward(u8),
    Brake,
    Coast,
    Standby,
}

/// Sends a drive command to the motor control task
pub fn send_command(command: Command) {
    DRIVE_CONTROL.signal(command);
}

/// Waits for next motor command
async fn wait_command() -> Command {
    DRIVE_CONTROL.wait().await
}

/// Motor control implementation
pub struct Motor {
    motor: tb6612fng::Motor<gpio::Output<'static>, gpio::Output<'static>, Pwm<'static>>,
    forward: bool,
    current_speed: i8,
}

impl Motor {
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

    /// Creates new motor instances with the given resources using the specified PWM configuration
    fn setup_motors(
        d: MotorDriverResources,
    ) -> Result<(Self, Self, gpio::Output<'static>), MotorError<Infallible, Infallible, PwmError>>
    {
        // PWM config for motor control (10kHz)
        let desired_freq_hz = 10_000;
        let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
        let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
        let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

        let mut pwm_config = Config::default();
        pwm_config.divider = divider.into();
        pwm_config.top = period;

        // Left motor
        let left_fwd = gpio::Output::new(d.left_forward_pin, gpio::Level::Low);
        let left_bckw = gpio::Output::new(d.left_backward_pin, gpio::Level::Low);
        let left_pwm = pwm::Pwm::new_output_a(d.left_slice, d.left_pwm_pin, pwm_config.clone());
        let left_motor = Self::new(left_fwd, left_bckw, left_pwm)?;

        // Right motor
        let right_fwd = gpio::Output::new(d.right_forward_pin, gpio::Level::Low);
        let right_bckw = gpio::Output::new(d.right_backward_pin, gpio::Level::Low);
        let right_pwm = pwm::Pwm::new_output_b(d.right_slice, d.right_pwm_pin, pwm_config);
        let right_motor = Self::new(right_fwd, right_bckw, right_pwm)?;

        // Initialize standby pin in disabled state
        let standby = gpio::Output::new(d.standby_pin, gpio::Level::Low);

        Ok((left_motor, right_motor, standby))
    }

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

    fn brake(&mut self) -> Result<(), MotorError<Infallible, Infallible, PwmError>> {
        self.current_speed = 0;
        self.motor.drive(DriveCommand::Brake)
    }

    fn coast(&mut self) -> Result<(), MotorError<Infallible, Infallible, PwmError>> {
        self.current_speed = 0;
        self.motor.drive(DriveCommand::Stop)
    }

    fn current_speed(&self) -> i8 {
        self.current_speed
    }

    pub fn is_forward(&self) -> bool {
        self.forward
    }
}

/// Checks if robot is executing a pure rotation (turning in place)
fn is_turning_in_place(left_speed: i8, right_speed: i8) -> bool {
    left_speed == -right_speed && left_speed != 0
}

/// Main motor control task that processes movement commands
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

    loop {
        // Process any pending commands
        let drive_command = wait_command().await;

        // Get latest encoder measurement
        let measurement = ENCODER_SIGNAL.wait().await;
        critical_section::with(|_| {
            *LAST_MEASUREMENT.try_lock().unwrap() = Some(measurement);
        });

        // Get current state
        let is_standby = standby_enabled;
        let left_speed_cmd;
        let right_speed_cmd;
        {
            let left = LEFT_MOTOR.lock().await;
            let right = RIGHT_MOTOR.lock().await;
            left_speed_cmd = left.as_ref().unwrap().current_speed();
            right_speed_cmd = right.as_ref().unwrap().current_speed();
        }

        // Wake from standby if movement requested
        if is_standby {
            match drive_command {
                Command::Forward(_)
                | Command::Backward(_)
                | Command::Left(_)
                | Command::Right(_) => {
                    standby.set_high();
                    standby_enabled = false;
                    Timer::after(Duration::from_millis(100)).await;
                }
                _ => {}
            }
        }

        match drive_command {
            Command::Forward(speed) => {
                // Stop first if currently moving backward
                if left_speed_cmd < 0 || right_speed_cmd < 0 {
                    info!("in conflicting motion, stopping");
                    let mut left = LEFT_MOTOR.lock().await;
                    let mut right = RIGHT_MOTOR.lock().await;
                    left.as_mut().unwrap().coast().unwrap();
                    right.as_mut().unwrap().coast().unwrap();
                } else {
                    // Use left motor as reference
                    let new_speed = (left_speed_cmd + speed as i8).clamp(0, 100);

                    // Set speeds with encoder feedback
                    let rpm_ratio =
                        if let Some(measurement) = LAST_MEASUREMENT.lock().await.as_ref() {
                            if measurement.left.rpm != 0.0 {
                                measurement.right.rpm / measurement.left.rpm
                            } else {
                                1.0
                            }
                        } else {
                            1.0
                        };
                    let adjustment = 1.0 + ((1.0 - rpm_ratio) * 0.5); // 50% correction factor
                    let adjusted_speed = (new_speed as f32 * adjustment).clamp(0.0, 100.0) as i8;

                    let mut left = LEFT_MOTOR.lock().await;
                    let mut right = RIGHT_MOTOR.lock().await;
                    left.as_mut().unwrap().set_speed(new_speed).unwrap();
                    right.as_mut().unwrap().set_speed(adjusted_speed).unwrap();
                }
            }
            Command::Backward(speed) => {
                // Stop first if currently moving forward
                if left_speed_cmd > 0 || right_speed_cmd > 0 {
                    info!("in conflicting motion, stopping");
                    let mut left = LEFT_MOTOR.lock().await;
                    let mut right = RIGHT_MOTOR.lock().await;
                    left.as_mut().unwrap().coast().unwrap();
                    right.as_mut().unwrap().coast().unwrap();
                } else {
                    // Use left motor as reference
                    let new_speed = (-left_speed_cmd + speed as i8).clamp(0, 100);
                    let neg_speed = -new_speed;

                    // Set speeds with encoder feedback
                    let rpm_ratio =
                        if let Some(measurement) = LAST_MEASUREMENT.lock().await.as_ref() {
                            if measurement.left.rpm != 0.0 {
                                measurement.right.rpm / measurement.left.rpm
                            } else {
                                1.0
                            }
                        } else {
                            1.0
                        };
                    let adjustment = 1.0 + ((1.0 - rpm_ratio) * 0.5); // 50% correction factor
                    let adjusted_speed = -(new_speed as f32 * adjustment).clamp(0.0, 100.0) as i8;

                    let mut left = LEFT_MOTOR.lock().await;
                    let mut right = RIGHT_MOTOR.lock().await;
                    left.as_mut().unwrap().set_speed(neg_speed).unwrap();
                    right.as_mut().unwrap().set_speed(adjusted_speed).unwrap();
                }
            }
            Command::Left(speed) => {
                if left_speed_cmd > right_speed_cmd {
                    // We're in a right turn, need to counter it
                    if is_turning_in_place(left_speed_cmd, right_speed_cmd) {
                        // If turning in place, stop completely
                        info!("stopping turn-in-place maneuver");
                        let mut left = LEFT_MOTOR.lock().await;
                        let mut right = RIGHT_MOTOR.lock().await;
                        left.as_mut().unwrap().coast().unwrap();
                        right.as_mut().unwrap().coast().unwrap();
                    } else {
                        // If turning while moving, restore original motion
                        let original_speed = (left_speed_cmd + right_speed_cmd) / 2;

                        let rpm_ratio =
                            if let Some(measurement) = LAST_MEASUREMENT.lock().await.as_ref() {
                                if measurement.left.rpm != 0.0 {
                                    measurement.right.rpm / measurement.left.rpm
                                } else {
                                    1.0
                                }
                            } else {
                                1.0
                            };
                        let adjustment = 1.0 + ((1.0 - rpm_ratio) * 0.5); // 50% correction factor

                        let adjusted_speed =
                            (original_speed as f32 * adjustment).clamp(-100.0, 100.0) as i8;

                        let mut left = LEFT_MOTOR.lock().await;
                        let mut right = RIGHT_MOTOR.lock().await;
                        left.as_mut().unwrap().set_speed(original_speed).unwrap();
                        right.as_mut().unwrap().set_speed(adjusted_speed).unwrap();
                    }
                } else {
                    // Initiate or continue left turn
                    let new_left_speed = (left_speed_cmd - speed as i8).clamp(-100, 100);
                    let target_right_speed = (right_speed_cmd + speed as i8).clamp(-100, 100);

                    let rpm_ratio =
                        if let Some(measurement) = LAST_MEASUREMENT.lock().await.as_ref() {
                            if measurement.left.rpm != 0.0 {
                                measurement.right.rpm / measurement.left.rpm
                            } else {
                                1.0
                            }
                        } else {
                            1.0
                        };

                    let target_rpm_ratio = if new_left_speed != 0 {
                        target_right_speed as f32 / new_left_speed as f32
                    } else {
                        1.0
                    };

                    let speed_adjustment = (target_rpm_ratio - rpm_ratio) * 0.5;
                    let adjusted_speed = ((target_right_speed.abs() as f32)
                        * (1.0 + speed_adjustment))
                        .clamp(-100.0, 100.0) as i8;

                    let mut left = LEFT_MOTOR.lock().await;
                    let mut right = RIGHT_MOTOR.lock().await;
                    left.as_mut().unwrap().set_speed(new_left_speed).unwrap();
                    right.as_mut().unwrap().set_speed(adjusted_speed).unwrap();
                }
            }
            Command::Right(speed) => {
                if right_speed_cmd > left_speed_cmd {
                    // We're in a left turn, need to counter it
                    if is_turning_in_place(left_speed_cmd, right_speed_cmd) {
                        // If turning in place, stop completely
                        info!("stopping turn-in-place maneuver");
                        let mut left = LEFT_MOTOR.lock().await;
                        let mut right = RIGHT_MOTOR.lock().await;
                        left.as_mut().unwrap().coast().unwrap();
                        right.as_mut().unwrap().coast().unwrap();
                    } else {
                        // If turning while moving, restore original motion
                        let original_speed = (left_speed_cmd + right_speed_cmd) / 2;

                        let rpm_ratio =
                            if let Some(measurement) = LAST_MEASUREMENT.lock().await.as_ref() {
                                if measurement.left.rpm != 0.0 {
                                    measurement.right.rpm / measurement.left.rpm
                                } else {
                                    1.0
                                }
                            } else {
                                1.0
                            };
                        let adjustment = 1.0 + ((1.0 - rpm_ratio) * 0.5); // 50% correction factor

                        let adjusted_speed =
                            (original_speed as f32 * adjustment).clamp(-100.0, 100.0) as i8;

                        let mut left = LEFT_MOTOR.lock().await;
                        let mut right = RIGHT_MOTOR.lock().await;
                        left.as_mut().unwrap().set_speed(original_speed).unwrap();
                        right.as_mut().unwrap().set_speed(adjusted_speed).unwrap();
                    }
                } else {
                    // Initiate or continue right turn
                    let new_left_speed = (left_speed_cmd + speed as i8).clamp(-100, 100);
                    let target_right_speed = (right_speed_cmd - speed as i8).clamp(-100, 100);

                    let rpm_ratio =
                        if let Some(measurement) = LAST_MEASUREMENT.lock().await.as_ref() {
                            if measurement.left.rpm != 0.0 {
                                measurement.right.rpm / measurement.left.rpm
                            } else {
                                1.0
                            }
                        } else {
                            1.0
                        };

                    let target_rpm_ratio = if new_left_speed != 0 {
                        target_right_speed as f32 / new_left_speed as f32
                    } else {
                        1.0
                    };

                    let speed_adjustment = (target_rpm_ratio - rpm_ratio) * 0.5;
                    let adjusted_speed = ((target_right_speed.abs() as f32)
                        * (1.0 + speed_adjustment))
                        .clamp(-100.0, 100.0) as i8;

                    let mut left = LEFT_MOTOR.lock().await;
                    let mut right = RIGHT_MOTOR.lock().await;
                    left.as_mut().unwrap().set_speed(new_left_speed).unwrap();
                    right.as_mut().unwrap().set_speed(adjusted_speed).unwrap();
                }
            }
            Command::Coast => {
                info!("coast");
                let mut left = LEFT_MOTOR.lock().await;
                let mut right = RIGHT_MOTOR.lock().await;
                left.as_mut().unwrap().coast().unwrap();
                right.as_mut().unwrap().coast().unwrap();
            }
            Command::Brake => {
                info!("brake");
                let mut left = LEFT_MOTOR.lock().await;
                let mut right = RIGHT_MOTOR.lock().await;
                left.as_mut().unwrap().brake().unwrap();
                right.as_mut().unwrap().brake().unwrap();
            }
            Command::Standby => {
                if !is_standby {
                    let mut left = LEFT_MOTOR.lock().await;
                    let mut right = RIGHT_MOTOR.lock().await;
                    left.as_mut().unwrap().brake().unwrap();
                    right.as_mut().unwrap().brake().unwrap();
                    Timer::after(Duration::from_millis(100)).await;
                    left.as_mut().unwrap().coast().unwrap();
                    right.as_mut().unwrap().coast().unwrap();
                    Timer::after(Duration::from_millis(100)).await;
                    standby.set_low();
                    standby_enabled = true;
                }
            }
        }
    }
}
