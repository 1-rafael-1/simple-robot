use crate::system::resources::MotorResources;
use embassy_rp::gpio;
use embassy_rp::pwm;
use tb6612fng::{DriveCommand, Motor, Tb6612fng};

/// Maximum PWM value (fully on)
const PWM_MAX: u16 = 65535;

/// Minimum PWM value (fully off)
const PWM_MIN: u16 = 0;

#[embassy_executor::task]
pub async fn test_motors(r: MotorResources) {
    let left_fwd = gpio::Output::new(r.left_forward_pin, gpio::Level::Low);
    let left_bckw = gpio::Output::new(r.left_backward_pin, gpio::Level::Low);
    let mut left_speed = pwm::Config::default();
    left_speed.top = PWM_MAX;
    left_speed.compare_a = PWM_MIN;
    let left_pwm = pwm::Pwm::new_output_a(r.left_slice, r.left_pwm_pin, left_speed);
    // let left_motor = Motor::new(left_fwd, left_bckw, &mut left_pwm).unwrap();
}
