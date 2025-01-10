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
use hcsr04_async::{Config, DistanceUnit, Hcsr04, TemperatureUnit};
use moving_median::MovingMedian;
use {defmt_rtt as _, panic_probe as _};

// Servo Configuration constants

/// uncalibrated default, the shortest duty cycle sent to a servo
const SERVO_DEFAULT_MIN_PULSE_WIDTH: u64 = 1000;

/// uncalibrated default, the longest duty cycle sent to a servo
const SERVO_DEFAULT_MAX_PULSE_WIDTH: u64 = 2000;

/// uncalibrated default, the degree of rotation that corresponds to a full cycle
const SERVO_DEFAULT_MAX_DEGREE_ROTATION: u64 = 160;

/// The period of each cycle
const SERVO_REFRESH_INTERVAL: u64 = 20000;

// HCSR04 Configuration constants

/// Size of median filter window (3 samples balances noise reduction vs. latency)
const ULTRASONIC_MEDIAN_WINDOW_SIZE: usize = 3;

/// Fixed ambient temperature for distance calculations
/// Slight inaccuracy acceptable as we care more about consistent readings
const ULTRASONIC_TEMPERATURE: f64 = 21.5;

// /// Distance at which obstacles are detected (22cm gives good reaction time)
// const ULTRASONIC_MINIMUM_DISTANCE: f64 = 22.0;

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
            period: Duration::from_micros(SERVO_REFRESH_INTERVAL),
            min_pulse_width: Duration::from_micros(SERVO_DEFAULT_MIN_PULSE_WIDTH),
            max_pulse_width: Duration::from_micros(SERVO_DEFAULT_MAX_PULSE_WIDTH),
            max_degree_rotation: SERVO_DEFAULT_MAX_DEGREE_ROTATION,
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
    // Initialize the ultrasonic sensor
    let hcsr04_config: Config = Config {
        distance_unit: DistanceUnit::Centimeters,
        temperature_unit: TemperatureUnit::Celsius,
    };
    let trigger = Output::new(u.trigger_pin, Level::Low);
    let echo = Input::new(u.echo_pin, Pull::None);
    let mut sensor = Hcsr04::new(trigger, echo, hcsr04_config);

    let mut median_filter = MovingMedian::<f64, ULTRASONIC_MEDIAN_WINDOW_SIZE>::new();

    // Initialize the servo
    let Pio {
        mut common, sm0, ..
    } = Pio::new(s.pio, Irqs);

    let prg = PioPwmProgram::new(&mut common);
    let pwm_pio = PioPwm::new(&mut common, sm0, s.pin, &prg);
    let mut servo = ServoBuilder::new(pwm_pio)
        .set_max_degree_rotation(180) // TODO: adjust to what the servo actually moves.
        .set_min_pulse_width(Duration::from_micros(500)) // TODO: adjust to what the servo actually supports.
        .set_max_pulse_width(Duration::from_micros(2400)) // TODO: adjust to what the servo actually supports.
        .build();

    servo.start();

    let mut angle: f32 = 0.0;
    let mut direction: f32 = 0.25; // Positive for increasing angle, negative for decreasing

    loop {
        // Update servo position
        servo.rotate_float(angle);

        // Take 3 measurements at current angle
        for _ in 0..3 {
            let filtered_distance = match sensor.measure(ULTRASONIC_TEMPERATURE).await {
                Ok(distance_cm) => {
                    median_filter.add_value(distance_cm);
                    median_filter.median()
                }
                Err(_) => 200.0, // Return safe distance on error to prevent false positives
            };

            // TODO: Send measurement and angle to the orchestator or process them in any way
            info!("Angle: {}, Distance: {}", angle, filtered_distance);

            Timer::after_millis(5).await; // Small delay between measurements
        }

        // Update angle and check for direction change
        angle += direction;
        if angle >= 180.0 {
            angle = 180.0;
            direction = -0.25; // Start moving back
        } else if angle <= 0.0 {
            angle = 0.0;
            direction = 0.25; // Start moving forward
        }

        Timer::after_millis(10).await; // Delay for servo movement
    }
}
