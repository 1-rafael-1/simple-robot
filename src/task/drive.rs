//! Motor control implementation
//!
//! - Controls TB6612FNG dual motor driver
//! - Manages motor speeds and directions
//! - Handles movement commands via signals

use crate::system::resources::MotorResources;
use defmt::info;
use embassy_rp::gpio;
use embassy_rp::pwm;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use tb6612fng::{DriveCommand, Motor, Tb6612fng};

/// Drive control signal
static DRIVE_CONTROL: Signal<CriticalSectionRawMutex, Command> = Signal::new();

/// Motor control commands
#[derive(Debug, Clone)]
pub enum Command {
    Left(u8),     // 0-100
    Right(u8),    // 0-100
    Forward(u8),  // 0-100
    Backward(u8), // 0-100
    Brake,        // Actively brake motors
    Coast,        // Stop motors
    Standby,      // Power saving
}

/// Sends drive command
pub fn send_command(command: Command) {
    DRIVE_CONTROL.signal(command);
}

/// Waits for next command
async fn wait_command() -> Command {
    DRIVE_CONTROL.wait().await
}

/// Checks if robot is turning in place
fn is_turning_in_place(left_speed: i8, right_speed: i8) -> bool {
    left_speed == -right_speed && left_speed != 0
}

#[embassy_executor::task]
pub async fn drive(r: MotorResources) {
    // PWM config (10kHz for DC motors)
    let desired_freq_hz = 10_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();

    // Calculate divider for 16-bit limit
    let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    let mut pwm_config = pwm::Config::default();
    pwm_config.divider = divider.into();
    pwm_config.top = period;

    // Initialize motor driver pins
    let stby = gpio::Output::new(r.standby_pin, gpio::Level::Low);

    // Left motor
    let left_fwd = gpio::Output::new(r.left_forward_pin, gpio::Level::Low);
    let left_bckw = gpio::Output::new(r.left_backward_pin, gpio::Level::Low);
    let left_pwm = pwm::Pwm::new_output_a(r.left_slice, r.left_pwm_pin, pwm_config.clone());
    let left_motor = Motor::new(left_fwd, left_bckw, left_pwm).unwrap();

    // Right motor
    let right_fwd = gpio::Output::new(r.right_forward_pin, gpio::Level::Low);
    let right_bckw = gpio::Output::new(r.right_backward_pin, gpio::Level::Low);
    let right_pwm = pwm::Pwm::new_output_b(r.right_slice, r.right_pwm_pin, pwm_config.clone());
    let right_motor = Motor::new(right_fwd, right_bckw, right_pwm).unwrap();

    // Motor driver
    let mut control = Tb6612fng::new(left_motor, right_motor, stby).unwrap();

    loop {
        let drive_command = wait_command().await;

        // Get current state
        let is_standby = control.current_standby().unwrap();
        let left_speed = control.motor_a.current_speed();
        let right_speed = control.motor_b.current_speed();

        // Wake from standby if movement requested
        if is_standby {
            match drive_command {
                Command::Forward(_)
                | Command::Backward(_)
                | Command::Left(_)
                | Command::Right(_) => {
                    control.disable_standby().unwrap();
                    Timer::after(Duration::from_millis(100)).await;
                }
                _ => {}
            }
        }

        match drive_command {
            Command::Forward(speed) => {
                // Stop first if currently moving backward
                if left_speed < 0 || right_speed < 0 {
                    info!("in conflicting motion, stopping");
                    control.motor_a.drive(DriveCommand::Stop).unwrap();
                    control.motor_b.drive(DriveCommand::Stop).unwrap();
                } else {
                    // Increment forward speed, clamped to valid range
                    let new_speed = (left_speed + speed as i8).clamp(0, 100) as u8;
                    info!("drive forward {}", new_speed);
                    control
                        .motor_a
                        .drive(DriveCommand::Forward(new_speed))
                        .unwrap();
                    control
                        .motor_b
                        .drive(DriveCommand::Forward(new_speed))
                        .unwrap();
                }
            }
            Command::Backward(speed) => {
                // Stop first if currently moving forward
                if left_speed > 0 || right_speed > 0 {
                    info!("in conflicting motion, stopping");
                    control.motor_a.drive(DriveCommand::Stop).unwrap();
                    control.motor_b.drive(DriveCommand::Stop).unwrap();
                } else {
                    // Increment backward speed, clamped to valid range
                    let new_speed = (-left_speed + speed as i8).clamp(0, 100) as u8;
                    info!("drive backward {}", new_speed);
                    control
                        .motor_a
                        .drive(DriveCommand::Backward(new_speed))
                        .unwrap();
                    control
                        .motor_b
                        .drive(DriveCommand::Backward(new_speed))
                        .unwrap();
                }
            }
            Command::Left(speed) => {
                if left_speed > right_speed {
                    // We're in a right turn, need to counter it
                    if is_turning_in_place(left_speed, right_speed) {
                        // If turning in place, stop completely
                        info!("stopping turn-in-place maneuver");
                        control.motor_a.drive(DriveCommand::Stop).unwrap();
                        control.motor_b.drive(DriveCommand::Stop).unwrap();
                    } else {
                        // If turning while moving, restore original motion speed
                        let original_speed = (left_speed + right_speed) / 2;
                        info!("restoring original motion speed {}", original_speed);

                        if original_speed > 0 {
                            control
                                .motor_a
                                .drive(DriveCommand::Forward(original_speed as u8))
                                .unwrap();
                            control
                                .motor_b
                                .drive(DriveCommand::Forward(original_speed as u8))
                                .unwrap();
                        } else if original_speed < 0 {
                            control
                                .motor_a
                                .drive(DriveCommand::Backward(-original_speed as u8))
                                .unwrap();
                            control
                                .motor_b
                                .drive(DriveCommand::Backward(-original_speed as u8))
                                .unwrap();
                        }
                    }
                } else {
                    // Initiate or continue left turn
                    let new_left_speed = (left_speed - speed as i8).clamp(-100, 100);
                    let new_right_speed = (right_speed + speed as i8).clamp(-100, 100);
                    info!("turn left L:{} R:{}", new_left_speed, new_right_speed);

                    // Set left motor
                    if new_left_speed > 0 {
                        control
                            .motor_a
                            .drive(DriveCommand::Forward(new_left_speed as u8))
                            .unwrap();
                    } else if new_left_speed < 0 {
                        control
                            .motor_a
                            .drive(DriveCommand::Backward(-new_left_speed as u8))
                            .unwrap();
                    } else {
                        control.motor_a.drive(DriveCommand::Stop).unwrap();
                    }

                    // Set right motor
                    if new_right_speed > 0 {
                        control
                            .motor_b
                            .drive(DriveCommand::Forward(new_right_speed as u8))
                            .unwrap();
                    } else if new_right_speed < 0 {
                        control
                            .motor_b
                            .drive(DriveCommand::Backward(-new_right_speed as u8))
                            .unwrap();
                    } else {
                        control.motor_b.drive(DriveCommand::Stop).unwrap();
                    }
                }
            }
            Command::Right(speed) => {
                if right_speed > left_speed {
                    // We're in a left turn, need to counter it
                    if is_turning_in_place(left_speed, right_speed) {
                        // If turning in place, stop completely
                        info!("stopping turn-in-place maneuver");
                        control.motor_a.drive(DriveCommand::Stop).unwrap();
                        control.motor_b.drive(DriveCommand::Stop).unwrap();
                    } else {
                        // If turning while moving, restore original motion speed
                        let original_speed = (left_speed + right_speed) / 2;
                        info!("restoring original motion speed {}", original_speed);

                        if original_speed > 0 {
                            control
                                .motor_a
                                .drive(DriveCommand::Forward(original_speed as u8))
                                .unwrap();
                            control
                                .motor_b
                                .drive(DriveCommand::Forward(original_speed as u8))
                                .unwrap();
                        } else if original_speed < 0 {
                            control
                                .motor_a
                                .drive(DriveCommand::Backward(-original_speed as u8))
                                .unwrap();
                            control
                                .motor_b
                                .drive(DriveCommand::Backward(-original_speed as u8))
                                .unwrap();
                        }
                    }
                } else {
                    // Initiate or continue right turn
                    let new_left_speed = (left_speed + speed as i8).clamp(-100, 100);
                    let new_right_speed = (right_speed - speed as i8).clamp(-100, 100);
                    info!("turn right L:{} R:{}", new_left_speed, new_right_speed);

                    // Set left motor
                    if new_left_speed > 0 {
                        control
                            .motor_a
                            .drive(DriveCommand::Forward(new_left_speed as u8))
                            .unwrap();
                    } else if new_left_speed < 0 {
                        control
                            .motor_a
                            .drive(DriveCommand::Backward(-new_left_speed as u8))
                            .unwrap();
                    } else {
                        control.motor_a.drive(DriveCommand::Stop).unwrap();
                    }

                    // Set right motor
                    if new_right_speed > 0 {
                        control
                            .motor_b
                            .drive(DriveCommand::Forward(new_right_speed as u8))
                            .unwrap();
                    } else if new_right_speed < 0 {
                        control
                            .motor_b
                            .drive(DriveCommand::Backward(-new_right_speed as u8))
                            .unwrap();
                    } else {
                        control.motor_b.drive(DriveCommand::Stop).unwrap();
                    }
                }
            }
            Command::Coast => {
                info!("coast");
                control.motor_a.drive(DriveCommand::Stop).unwrap();
                control.motor_b.drive(DriveCommand::Stop).unwrap();
            }
            Command::Brake => {
                info!("brake");
                control.motor_a.drive(DriveCommand::Brake).unwrap();
                control.motor_b.drive(DriveCommand::Brake).unwrap();
            }
            Command::Standby => {
                if !is_standby {
                    control.motor_a.drive(DriveCommand::Brake).unwrap();
                    control.motor_b.drive(DriveCommand::Brake).unwrap();
                    Timer::after(Duration::from_millis(100)).await;
                    control.motor_a.drive(DriveCommand::Stop).unwrap();
                    control.motor_b.drive(DriveCommand::Stop).unwrap();
                    Timer::after(Duration::from_millis(100)).await;
                    control.enable_standby().unwrap();
                }
            }
        }
    }
}
