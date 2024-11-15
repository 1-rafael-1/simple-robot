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
    // Standby pin controls power state of the driver
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
                // Check if we're currently in a right turn (left motor faster than right)
                if left_speed > right_speed {
                    // Cancel the turn by setting both motors to the lower speed
                    let balanced_speed = right_speed;
                    info!(
                        "cancelling right turn, balancing to speed {}",
                        balanced_speed
                    );

                    if balanced_speed > 0 {
                        control
                            .motor_a
                            .drive(DriveCommand::Forward(balanced_speed as u8))
                            .unwrap();
                        control
                            .motor_b
                            .drive(DriveCommand::Forward(balanced_speed as u8))
                            .unwrap();
                    } else if balanced_speed < 0 {
                        control
                            .motor_a
                            .drive(DriveCommand::Backward(-balanced_speed as u8))
                            .unwrap();
                        control
                            .motor_b
                            .drive(DriveCommand::Backward(-balanced_speed as u8))
                            .unwrap();
                    } else {
                        control.motor_a.drive(DriveCommand::Stop).unwrap();
                        control.motor_b.drive(DriveCommand::Stop).unwrap();
                    }
                } else {
                    // Normal left turn: reduce left motor speed, increase right motor speed
                    let new_left_speed = (left_speed - speed as i8).clamp(-100, 100);
                    let new_right_speed = (right_speed + speed as i8).clamp(-100, 100);
                    info!("turn left L:{} R:{}", new_left_speed, new_right_speed);

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
                // Check if we're currently in a left turn (right motor faster than left)
                if right_speed > left_speed {
                    // Cancel the turn by setting both motors to the lower speed
                    let balanced_speed = left_speed;
                    info!(
                        "cancelling left turn, balancing to speed {}",
                        balanced_speed
                    );

                    if balanced_speed > 0 {
                        control
                            .motor_a
                            .drive(DriveCommand::Forward(balanced_speed as u8))
                            .unwrap();
                        control
                            .motor_b
                            .drive(DriveCommand::Forward(balanced_speed as u8))
                            .unwrap();
                    } else if balanced_speed < 0 {
                        control
                            .motor_a
                            .drive(DriveCommand::Backward(-balanced_speed as u8))
                            .unwrap();
                        control
                            .motor_b
                            .drive(DriveCommand::Backward(-balanced_speed as u8))
                            .unwrap();
                    } else {
                        control.motor_a.drive(DriveCommand::Stop).unwrap();
                        control.motor_b.drive(DriveCommand::Stop).unwrap();
                    }
                } else {
                    // Normal right turn: increase left motor speed, reduce right motor speed
                    let new_left_speed = (left_speed + speed as i8).clamp(-100, 100);
                    let new_right_speed = (right_speed - speed as i8).clamp(-100, 100);
                    info!("turn right L:{} R:{}", new_left_speed, new_right_speed);

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
                // Let motors spin freely
                info!("coast");
                control.motor_a.drive(DriveCommand::Stop).unwrap();
                control.motor_b.drive(DriveCommand::Stop).unwrap();
            }
            drive_command::Command::Brake => {
                // Actively stop motors
                info!("brake");
                control.motor_a.drive(DriveCommand::Brake).unwrap();
                control.motor_b.drive(DriveCommand::Brake).unwrap();
            }
            drive_command::Command::Standby => {
                // Enter low-power standby mode
                if !is_standby {
                    // Brake first, then coast, then standby for clean shutdown
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
