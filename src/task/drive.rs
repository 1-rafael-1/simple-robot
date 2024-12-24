//! Motor control implementation with encoder feedback
//!
//! Controls the robot's movement through a TB6612FNG dual motor driver with
//! DFRobot FIT0450 DC motors that include quadrature encoders.

use crate::system::resources::{MotorDriverResources, MotorEncoderResources};
use core::convert::Infallible;
use defmt::info;
use embassy_rp::pwm::PwmError;
use embassy_rp::{
    gpio::{self, Pull},
    pwm::{self, Config, InputMode, Pwm},
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
use tb6612fng::{DriveCommand, MotorError};

// DFRobot FIT0450 motor specifications
const PULSES_PER_REV: u32 = 8; // Encoder pulses per motor revolution
const GEAR_RATIO: u32 = 120; // 120:1 gear reduction

/// Drive control signal for sending commands to the motor task
static DRIVE_CONTROL: Signal<CriticalSectionRawMutex, Command> = Signal::new();

/// Left motor control protected by mutex
static LEFT_MOTOR: Mutex<CriticalSectionRawMutex, Option<Motor>> = Mutex::new(None);

/// Right motor control protected by mutex
static RIGHT_MOTOR: Mutex<CriticalSectionRawMutex, Option<Motor>> = Mutex::new(None);

/// Motor encoders protected by mutex
static MOTOR_ENCODERS: Mutex<CriticalSectionRawMutex, Option<MotorEncoders<'static>>> =
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

    fn is_forward(&self) -> bool {
        self.forward
    }
}

/// Motor encoders implementation
pub struct MotorEncoders<'d> {
    left: Pwm<'d>,
    right: Pwm<'d>,
    left_rpm: f32,
    right_rpm: f32,
}

impl<'d> MotorEncoders<'d> {
    fn new(left: Pwm<'d>, right: Pwm<'d>) -> Self {
        Self {
            left,
            right,
            left_rpm: 0.0,
            right_rpm: 0.0,
        }
    }

    fn update(&mut self, left_forward: bool, right_forward: bool) {
        // Read and reset counters
        let left_pulses = self.left.counter();
        self.left.set_counter(0);
        let right_pulses = self.right.counter();
        self.right.set_counter(0);

        // Convert pulses to RPM
        let left_hz = left_pulses as f32 * 10.0;
        let right_hz = right_pulses as f32 * 10.0;
        let left_motor_rpm = (left_hz / PULSES_PER_REV as f32) * 60.0;
        let right_motor_rpm = (right_hz / PULSES_PER_REV as f32) * 60.0;
        let left_wheel_rpm = left_motor_rpm / GEAR_RATIO as f32;
        let right_wheel_rpm = right_motor_rpm / GEAR_RATIO as f32;

        // Apply direction
        self.left_rpm = if left_forward {
            left_wheel_rpm
        } else {
            -left_wheel_rpm
        };
        self.right_rpm = if right_forward {
            right_wheel_rpm
        } else {
            -right_wheel_rpm
        };

        // Log encoder data
        info!(
            "Left Encoder: pulses={} freq={}Hz motor_rpm={} wheel_rpm={}",
            left_pulses, left_hz, left_motor_rpm, left_wheel_rpm
        );
        info!(
            "Right Encoder: pulses={} freq={}Hz motor_rpm={} wheel_rpm={}",
            right_pulses, right_hz, right_motor_rpm, right_wheel_rpm
        );
        info!(
            "Speed measurements - Left: {} RPM, Right: {} RPM",
            self.left_rpm, self.right_rpm
        );
    }

    fn calculate_speed_adjustment(&self) -> f32 {
        let rpm_ratio = if self.left_rpm != 0.0 {
            self.right_rpm / self.left_rpm
        } else {
            1.0
        };
        1.0 + ((1.0 - rpm_ratio) * 0.5) // 50% correction factor
    }

    fn left_rpm(&self) -> f32 {
        self.left_rpm
    }

    fn right_rpm(&self) -> f32 {
        self.right_rpm
    }
}

/// Checks if robot is executing a pure rotation (turning in place)
fn is_turning_in_place(left_speed: i8, right_speed: i8) -> bool {
    left_speed == -right_speed && left_speed != 0
}

/// Main motor control task that processes movement commands
#[embassy_executor::task]
pub async fn drive(d: MotorDriverResources, e: MotorEncoderResources) {
    // Configure PWM input for encoder pulse counting
    let config = Config::default();
    let left_encoder = Pwm::new_input(
        e.left_encoder_slice,
        e.left_encoder_pin,
        Pull::None,
        InputMode::RisingEdge,
        config.clone(),
    );
    let right_encoder = Pwm::new_input(
        e.right_encoder_slice,
        e.right_encoder_pin,
        Pull::None,
        InputMode::RisingEdge,
        config,
    );

    // PWM config for motor control (10kHz)
    let desired_freq_hz = 10_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    let mut pwm_config = Config::default();
    pwm_config.divider = divider.into();
    pwm_config.top = period;

    // Initialize motor driver pins
    let stby = gpio::Output::new(d.standby_pin, gpio::Level::Low);

    // Left motor
    let left_fwd = gpio::Output::new(d.left_forward_pin, gpio::Level::Low);
    let left_bckw = gpio::Output::new(d.left_backward_pin, gpio::Level::Low);
    let left_pwm = pwm::Pwm::new_output_a(d.left_slice, d.left_pwm_pin, pwm_config.clone());
    let left_motor = Motor::new(left_fwd, left_bckw, left_pwm).unwrap();

    // Right motor
    let right_fwd = gpio::Output::new(d.right_forward_pin, gpio::Level::Low);
    let right_bckw = gpio::Output::new(d.right_backward_pin, gpio::Level::Low);
    let right_pwm = pwm::Pwm::new_output_b(d.right_slice, d.right_pwm_pin, pwm_config.clone());
    let right_motor = Motor::new(right_fwd, right_bckw, right_pwm).unwrap();

    // Initialize encoders
    let encoders = MotorEncoders::new(left_encoder, right_encoder);

    // Initialize mutexes
    critical_section::with(|_| {
        *LEFT_MOTOR.try_lock().unwrap() = Some(left_motor);
        *RIGHT_MOTOR.try_lock().unwrap() = Some(right_motor);
        *MOTOR_ENCODERS.try_lock().unwrap() = Some(encoders);
    });

    // Motor driver standby control
    let mut standby = stby;
    let mut standby_enabled = true;

    // Create ticker for speed measurement (100ms interval)
    let mut ticker = Ticker::every(Duration::from_millis(100));

    loop {
        // Wait for next 100ms interval
        ticker.next().await;

        // Update encoder measurements
        {
            let mut encoders = MOTOR_ENCODERS.lock().await;
            let encoders = encoders.as_mut().unwrap();
            let left = LEFT_MOTOR.lock().await;
            let right = RIGHT_MOTOR.lock().await;
            encoders.update(
                left.as_ref().unwrap().is_forward(),
                right.as_ref().unwrap().is_forward(),
            );
        }

        // Process any pending commands
        let drive_command = wait_command().await;

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
                    let adjustment;
                    {
                        let encoders = MOTOR_ENCODERS.lock().await;
                        adjustment = encoders.as_ref().unwrap().calculate_speed_adjustment();
                    }

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
                    let adjustment;
                    {
                        let encoders = MOTOR_ENCODERS.lock().await;
                        adjustment = encoders.as_ref().unwrap().calculate_speed_adjustment();
                    }

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

                        let adjustment;
                        {
                            let encoders = MOTOR_ENCODERS.lock().await;
                            adjustment = encoders.as_ref().unwrap().calculate_speed_adjustment();
                        }

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

                    let rpm_ratio;
                    {
                        let encoders = MOTOR_ENCODERS.lock().await;
                        let e = encoders.as_ref().unwrap();
                        rpm_ratio = if e.left_rpm() != 0.0 {
                            e.right_rpm() / e.left_rpm()
                        } else {
                            1.0
                        };
                    }

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

                        let adjustment;
                        {
                            let encoders = MOTOR_ENCODERS.lock().await;
                            adjustment = encoders.as_ref().unwrap().calculate_speed_adjustment();
                        }

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

                    let rpm_ratio;
                    {
                        let encoders = MOTOR_ENCODERS.lock().await;
                        let e = encoders.as_ref().unwrap();
                        rpm_ratio = if e.left_rpm() != 0.0 {
                            e.right_rpm() / e.left_rpm()
                        } else {
                            1.0
                        };
                    }

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
