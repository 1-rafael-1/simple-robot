//! Ultrasonic Sensor Sweep
//!
//! Collects data of objects found in a <180° circle in front of the robot by sweeping a HC-SR04 sensor on a servo.
//! Data collection is TODO
//! At this point in development the Pi Pico2 was out of PWM Pins/Slices, thankfully in the great Embassy rp32 examples
//! there is a cool example on how to use PIO to substitute PWM.

use crate::system::resources::{Irqs, SweepServoResources, UltrasonicDistanceSensorResources};
use core::time::Duration;
use defmt::info;
use embassy_rp::pio::{Instance, Pio};
use embassy_rp::pio_programs::pwm::{PioPwm, PioPwmProgram};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

const DEFAULT_MIN_PULSE_WIDTH: u64 = 1000; // uncalibrated default, the shortest duty cycle sent to a servo
const DEFAULT_MAX_PULSE_WIDTH: u64 = 2000; // uncalibrated default, the longest duty cycle sent to a servo
const DEFAULT_MAX_DEGREE_ROTATION: u64 = 160; // 160 degrees is typical
const REFRESH_INTERVAL: u64 = 20000; // The period of each cycle

pub struct ServoBuilder<'d, T: Instance, const SM: usize> {
    pwm: PioPwm<'d, T, SM>,
    period: Duration,
    min_pulse_width: Duration,
    max_pulse_width: Duration,
    max_degree_rotation: u64,
}

impl<'d, T: Instance, const SM: usize> ServoBuilder<'d, T, SM> {
    pub fn new(pwm: PioPwm<'d, T, SM>) -> Self {
        Self {
            pwm,
            period: Duration::from_micros(REFRESH_INTERVAL),
            min_pulse_width: Duration::from_micros(DEFAULT_MIN_PULSE_WIDTH),
            max_pulse_width: Duration::from_micros(DEFAULT_MAX_PULSE_WIDTH),
            max_degree_rotation: DEFAULT_MAX_DEGREE_ROTATION,
        }
    }

    // pub fn set_period(mut self, duration: Duration) -> Self {
    //     self.period = duration;
    //     self
    // }

    pub fn set_min_pulse_width(mut self, duration: Duration) -> Self {
        self.min_pulse_width = duration;
        self
    }

    pub fn set_max_pulse_width(mut self, duration: Duration) -> Self {
        self.max_pulse_width = duration;
        self
    }

    pub fn set_max_degree_rotation(mut self, degree: u64) -> Self {
        self.max_degree_rotation = degree;
        self
    }

    pub fn build(mut self) -> Servo<'d, T, SM> {
        self.pwm.set_period(self.period);
        Servo {
            pwm: self.pwm,
            min_pulse_width: self.min_pulse_width,
            max_pulse_width: self.max_pulse_width,
            max_degree_rotation: self.max_degree_rotation,
        }
    }
}

pub struct Servo<'d, T: Instance, const SM: usize> {
    pwm: PioPwm<'d, T, SM>,
    min_pulse_width: Duration,
    max_pulse_width: Duration,
    max_degree_rotation: u64,
}

impl<'d, T: Instance, const SM: usize> Servo<'d, T, SM> {
    pub fn start(&mut self) {
        self.pwm.start();
    }

    // pub fn stop(&mut self) {
    //     self.pwm.stop();
    // }

    // pub fn write_time(&mut self, duration: Duration) {
    //     self.pwm.write(duration);
    // }

    // pub fn rotate(&mut self, degree: u64) {
    //     let degree_per_nano_second = (self.max_pulse_width.as_nanos() as u64
    //         - self.min_pulse_width.as_nanos() as u64)
    //         / self.max_degree_rotation;
    //     let mut duration = Duration::from_nanos(
    //         degree * degree_per_nano_second + self.min_pulse_width.as_nanos() as u64,
    //     );
    //     if self.max_pulse_width < duration {
    //         duration = self.max_pulse_width;
    //     }
    //     info!("duration {}", &duration);

    //     self.pwm.write(duration);
    // }

    pub fn rotate_float(&mut self, degree: f32) {
        let degree = degree.clamp(0.0, self.max_degree_rotation as f32) as f64;
        let degree_per_nano_second = (self.max_pulse_width.as_nanos() as f64
            - self.min_pulse_width.as_nanos() as f64)
            / self.max_degree_rotation as f64;
        let mut duration = Duration::from_nanos(
            (degree * degree_per_nano_second + self.min_pulse_width.as_nanos() as f64) as u64,
        );
        if self.max_pulse_width < duration {
            duration = self.max_pulse_width;
        }
        info!("degree {} duration {}", degree, &duration);

        self.pwm.write(duration);
    }
}

#[embassy_executor::task]
pub async fn ultrasonic_sweep(s: SweepServoResources, u: UltrasonicDistanceSensorResources) {
    let Pio {
        mut common, sm0, ..
    } = Pio::new(s.pio, Irqs);

    let prg = PioPwmProgram::new(&mut common);
    let pwm_pio = PioPwm::new(&mut common, sm0, s.pin, &prg);
    let mut servo = ServoBuilder::new(pwm_pio)
        .set_max_degree_rotation(180) // Example of adjusting values for MG996R servo
        .set_min_pulse_width(Duration::from_micros(500)) // This value was detemined by a rough experiment.
        .set_max_pulse_width(Duration::from_micros(2400)) // Along with this value.
        .build();

    servo.start();

    loop {
        // Sweep from 0 to 180 degrees
        let mut angle: f32 = 0.0;
        while angle <= 180.0 {
            servo.rotate_float(angle);
            angle += 0.25; // Increment by half a degree
            Timer::after_millis(10).await;
        }

        // Sweep back from 180 to 0 degrees
        while angle >= 0.0 {
            servo.rotate_float(angle);
            angle -= 0.25; // Decrement by half a degree
            Timer::after_millis(10).await;
        }
    }
}
