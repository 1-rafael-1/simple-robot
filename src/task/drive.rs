use crate::system::command;
use crate::system::resources::MotorResources;
use defmt::info;
use embassy_rp::gpio;
use embassy_rp::pwm;
use embassy_time::{Duration, Timer};
use tb6612fng::{DriveCommand, Motor, Tb6612fng};

#[embassy_executor::task]
pub async fn drive(r: MotorResources) {
    // we want a PWM frequency of 1KHz, especially cheaper motors do not respond well to higher frequencies
    let desired_freq_hz = 10_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();

    // Calculate the clock divider
    // let divider = (clock_freq / 65536 / desired_freq).max(1) as u8;
    let divider = 16u8;

    // Calculate the period
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    // Configure PWM
    let mut pwm_config = pwm::Config::default();
    pwm_config.divider = divider.into();
    pwm_config.top = period;

    // standby pin
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

    // construct the motor driver
    let mut control = Tb6612fng::new(left_motor, right_motor, stby).unwrap();

    loop {
        let drive_command = command::wait().await;
        let is_standby = control.current_standby().unwrap();

        match drive_command {
            command::Command::Forward(speed) => {
                if is_standby {
                    control.disable_standby().unwrap();
                    Timer::after(Duration::from_millis(100)).await;
                }
                info!("drive straight");
                control.motor_a.drive(DriveCommand::Forward(speed)).unwrap();
                control.motor_b.drive(DriveCommand::Forward(speed)).unwrap();
            }
            command::Command::Backward(speed) => {
                info!("drive backward");
                control
                    .motor_a
                    .drive(DriveCommand::Backward(speed))
                    .unwrap();
                control
                    .motor_b
                    .drive(DriveCommand::Backward(speed))
                    .unwrap();
            }
            command::Command::Left(speed) => {
                info!("turn left");
                control
                    .motor_a
                    .drive(DriveCommand::Backward(speed))
                    .unwrap();
                control.motor_b.drive(DriveCommand::Forward(speed)).unwrap();
            }
            command::Command::Right(speed) => {
                info!("turn right");
                control.motor_a.drive(DriveCommand::Forward(speed)).unwrap();
                control
                    .motor_b
                    .drive(DriveCommand::Backward(speed))
                    .unwrap();
            }
            command::Command::Coast => {
                info!("coast");
                control.motor_a.drive(DriveCommand::Stop).unwrap();
                control.motor_b.drive(DriveCommand::Stop).unwrap();
            }
            command::Command::Brake => {
                info!("brake");
                control.motor_a.drive(DriveCommand::Brake).unwrap();
                control.motor_b.drive(DriveCommand::Brake).unwrap();
            }
            command::Command::Standby => {
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
