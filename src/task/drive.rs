//! Drive Task Module
//!
//! This module implements the drive control task that manages motor operations.
//! It handles forward/backward motion, turning, and different stop modes using
//! a TB6612FNG motor driver.

use crate::system::drive_command;
use crate::system::resources::MotorResources;
use defmt::info;
use embassy_rp::gpio;
use embassy_rp::pwm;
use embassy_time::{Duration, Timer};
use tb6612fng::{DriveCommand, Motor, Tb6612fng};

/// Determines if we're turning in place by checking if motors are running
/// at equal speeds in opposite directions
fn is_turning_in_place(left_speed: i8, right_speed: i8) -> bool {
    left_speed == -right_speed && left_speed != 0
}

#[embassy_executor::task]
pub async fn drive(r: MotorResources) {
    // Configure PWM for motor control
    // We use 10kHz frequency as cheaper DC motors often work better at lower frequencies
    let desired_freq_hz = 10_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq(); // 150MHz

    // Calculate minimum divider needed to keep period under 16-bit limit (65535)
    let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    // Configure PWM
    let mut pwm_config = pwm::Config::default();
    pwm_config.divider = divider.into();
    pwm_config.top = period;

    // Initialize TB6612FNG motor driver pins
    let stby = gpio::Output::new(r.standby_pin, gpio::Level::Low);

    // motor A, here defined to be the left motor
    let left_fwd = gpio::Output::new(r.left_forward_pin, gpio::Level::Low);
    let left_bckw = gpio::Output::new(r.left_backward_pin, gpio::Level::Low);
    let left_pwm = pwm::Pwm::new_output_a(r.left_slice, r.left_pwm_pin, pwm_config.clone());
    let left_motor = Motor::new(left_fwd, left_bckw, left_pwm).unwrap();

    // motor B, here defined to be the right motor
    let right_fwd = gpio::Output::new(r.right_forward_pin, gpio::Level::Low);
    let right_bckw = gpio::Output::new(r.right_backward_pin, gpio::Level::Low);
    let right_pwm = pwm::Pwm::new_output_b(r.right_slice, r.right_pwm_pin, pwm_config.clone());
    let right_motor = Motor::new(right_fwd, right_bckw, right_pwm).unwrap();

    // Create motor driver controller instance
    let mut control = Tb6612fng::new(left_motor, right_motor, stby).unwrap();

    loop {
        let drive_command = drive_command::wait().await;

        // Get current state
        let is_standby = control.current_standby().unwrap();
        let left_speed = control.motor_a.current_speed();
        let right_speed = control.motor_b.current_speed();

        // Wake up from standby if movement is requested
        if is_standby {
            match drive_command {
                drive_command::Command::Forward(_)
                | drive_command::Command::Backward(_)
                | drive_command::Command::Left(_)
                | drive_command::Command::Right(_) => {
                    control.disable_standby().unwrap();
                    Timer::after(Duration::from_millis(100)).await;
                }
                _ => {}
            }
        }

        match drive_command {
            drive_command::Command::Forward(speed) => {
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
            drive_command::Command::Backward(speed) => {
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
            drive_command::Command::Left(speed) => {
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
            drive_command::Command::Right(speed) => {
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
            drive_command::Command::Coast => {
                info!("coast");
                control.motor_a.drive(DriveCommand::Stop).unwrap();
                control.motor_b.drive(DriveCommand::Stop).unwrap();
            }
            drive_command::Command::Brake => {
                info!("brake");
                control.motor_a.drive(DriveCommand::Brake).unwrap();
                control.motor_b.drive(DriveCommand::Brake).unwrap();
            }
            drive_command::Command::Standby => {
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
