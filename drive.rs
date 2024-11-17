//! Motor control implementation
//!
//! Controls TB6612FNG dual motor driver for movement.

use crate::system::drive_command;
use crate::system::resources::MotorResources;
use defmt::info;
use embassy_rp::gpio;
use embassy_rp::pwm;
use embassy_time::{Duration, Timer};
use tb6612fng::{DriveCommand, Motor, Tb6612fng};

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

    let mut control = Tb6612fng::new(left_motor, right_motor, stby).unwrap();

    loop {
        let drive_command = drive_command::wait().await;

        // Get current state
        let is_standby = control.current_standby().unwrap();
        let left_speed = control.motor_a.current_speed();
        let right_speed = control.motor_b.current_speed();

        // Wake from standby if movement requested
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

        // ... existing code ... (command handling remains unchanged)
    }
}