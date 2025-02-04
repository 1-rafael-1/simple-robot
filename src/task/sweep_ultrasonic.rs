//! Ultrasonic Sensor Sweep Control
//!
//! Controls a servo-mounted HC-SR04 ultrasonic sensor to scan for objects in a 160° arc in front of the robot.
//! Uses PIO (Programmable I/O) for servo control due to PWM pin limitations on the Pi Pico.
//! Implements median filtering to reduce noise in distance measurements.

use core::time::Duration;

use defmt::{error, info};
use defmt_rtt as _;
use embassy_futures::select::{select, Either};
use embassy_rp::{
    gpio::{Input, Level, Output, Pull},
    pio::{Instance, Pio},
    pio_programs::pwm::{PioPwm, PioPwmProgram},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Delay, Instant, Timer};
use hcsr04_async::{Config, DistanceUnit, Hcsr04, Now, TemperatureUnit};
use moving_median::MovingMedian;
use panic_probe as _;

use crate::system::{
    event::{send_event, Events},
    resources::{Irqs, SweepServoResources, UltrasonicDistanceSensorResources},
};

/// Commands for ultrasonic sweep control
enum UltrasonicSweepCommand {
    /// Start ultrasonic sweep
    Start,
    /// Stop ultrasonic sweep
    Stop,
}

/// Control signal to trigger encoder measurements after specified duration
static US_SWEEP_CONTROL: Signal<CriticalSectionRawMutex, UltrasonicSweepCommand> = Signal::new();

/// Start continuous encoder readings
pub fn start_ultrasonic_sweep() {
    US_SWEEP_CONTROL.signal(UltrasonicSweepCommand::Start);
}

/// Stop encoder readings
pub fn stop_ultrasonic_sweep() {
    US_SWEEP_CONTROL.signal(UltrasonicSweepCommand::Stop);
}

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

/// Builder for configuring and creating a servo instance
///
/// Provides a fluent interface for setting servo parameters like pulse widths
/// and rotation limits before constructing the final servo instance.
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

/// Controls a servo motor using PIO-based PWM
///
/// Handles conversion between desired angle and PWM pulse width,
/// accounting for servo-specific timing requirements.
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

    /// Rotates the servo to the specified angle in degrees
    ///
    /// # Arguments
    /// * `degree` - Target angle between 0° and max_degree_rotation°
    ///
    /// Automatically clamps input to valid range and converts
    /// angle to appropriate PWM pulse width for the servo.
    pub fn rotate_float(&mut self, degree: f32) {
        let degree = degree.clamp(0.0, self.max_degree_rotation as f32) as f64;
        let degree_per_nano_second = (self.max_pulse_width.as_nanos() as f64 - self.min_pulse_width.as_nanos() as f64)
            / self.max_degree_rotation as f64;
        let mut duration =
            Duration::from_nanos((degree * degree_per_nano_second + self.min_pulse_width.as_nanos() as f64) as u64);
        if self.max_pulse_width < duration {
            duration = self.max_pulse_width;
        }
        self.pwm.write(duration);
    }
}

/// Provides system clock implementation for the HCSR04 ultrasonic sensor driver
/// by wrapping std::time::Instant
struct Clock;

impl Now for Clock {
    /// Returns current time in microseconds since system start
    fn now_micros(&self) -> u64 {
        Instant::now().as_micros()
    }
}

/// Embassy task that handles ultrasonic distance measurements while sweeping a servo
///
/// This task combines a servo motor sweep with ultrasonic distance measurements
/// to create a scanning range finder effect. Measurements are filtered through
/// a moving median filter to reduce noise.
#[embassy_executor::task]
pub async fn ultrasonic_sweep(s: SweepServoResources, u: UltrasonicDistanceSensorResources) {
    // Configure ultrasonic sensor with metric units
    let hcsr04_config: Config = Config {
        distance_unit: DistanceUnit::Centimeters,
        temperature_unit: TemperatureUnit::Celsius,
    };

    // Initialize GPIO pins for the HCSR04 sensor
    let trigger = Output::new(u.trigger_pin, Level::Low);
    let echo = Input::new(u.echo_pin, Pull::None);
    let mut sensor = Hcsr04::new(trigger, echo, hcsr04_config, Clock, Delay);

    // Create median filter to smooth out distance measurements
    let mut median_filter = MovingMedian::<f64, ULTRASONIC_MEDIAN_WINDOW_SIZE>::new();

    // Initialize PIO state machine for servo PWM control
    let Pio { mut common, sm0, .. } = Pio::new(s.pio, Irqs);

    let prg = PioPwmProgram::new(&mut common);
    let pwm_pio = PioPwm::new(&mut common, sm0, s.pin, &prg);
    let mut servo = ServoBuilder::new(pwm_pio)
        .set_max_degree_rotation(160) // SG90 servo has 160° range of motion
        .set_min_pulse_width(Duration::from_micros(500)) // SG90 minimum pulse width
        .set_max_pulse_width(Duration::from_micros(2400)) // SG90 maximum pulse width
        .build();

    servo.start();

    let mut angle: f32 = 0.0;
    let mut angle_increment: f32 = 2.5;
    let mut filtered_distance: f64;

    // 80 degrees is middle, 0 is right, 160 is left
    servo.rotate_float(80.0);
    Timer::after_millis(500).await;

    'command: loop {
        info!("Waiting for ultrasonic sweep command");
        // Wait for a command, consuming it
        match US_SWEEP_CONTROL.wait().await {
            UltrasonicSweepCommand::Start => {
                info!("Starting ultrasonic sweep");
            }
            UltrasonicSweepCommand::Stop => {
                info!("Stopping ultrasonic sweep");
                continue 'command;
            }
        };

        loop {
            // Update servo position
            servo.rotate_float(angle);

            // Give servo time to reach position, also see if we must stop the sweep
            match select(Timer::after_millis(25), US_SWEEP_CONTROL.wait()).await {
                Either::First(_) => {}
                Either::Second(_) => {
                    info!("Stopping ultrasonic sweep");
                    continue 'command;
                }
            }

            // Take multiple measurements based on ULTRASONIC_MEDIAN_WINDOW_SIZE
            for _ in 0..ULTRASONIC_MEDIAN_WINDOW_SIZE {
                Timer::after_millis(90).await;
                median_filter.add_value(match sensor.measure(ULTRASONIC_TEMPERATURE).await {
                    Ok(distance_cm) => {
                        if distance_cm > 400.0 {
                            400.0
                        } else {
                            distance_cm
                        }
                    }
                    Err(e) => {
                        error!("{}", e);
                        400.0 // Return safe distance on error to prevent false positives
                    }
                });
            }

            // Calculate median distance from the filtered measurements
            filtered_distance = median_filter.median();

            // Send reading event to orchestration task
            send_event(Events::UltrasonicSweepReadingTaken(filtered_distance, angle)).await;

            // Update angle and check for direction change
            angle += angle_increment;
            if angle >= servo.max_degree_rotation as f32 {
                angle = servo.max_degree_rotation as f32;
                angle_increment = angle_increment * -1.0; // Start moving back
            } else if angle <= 0.0 {
                angle = 0.0;
                angle_increment = angle_increment * -1.0; // Start moving forward
            }
        }
    }
}
