//! Ultrasonic Sensor Sweep Control
//!
//! Controls a servo-mounted HC-SR04 ultrasonic sensor to scan for objects in a 160° arc in front of the robot.
//! Uses PIO (Programmable I/O) for servo control due to PWM pin limitations on the Pi Pico.
//! Implements median filtering to reduce noise in distance measurements.

use core::time::Duration;

use defmt::{error, info};
use defmt_rtt as _;
use embassy_futures::select::{Either, select};
use embassy_rp::{
    gpio::{Input, Output},
    pio::Instance,
    pio_programs::pwm::PioPwm,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Delay, Instant, Timer};
use hcsr04_async::{Config, DistanceUnit, Hcsr04, Now, TemperatureUnit};
use moving_median::MovingMedian;
use panic_probe as _;

use crate::system::event::{Events, raise_event};

/// Commands for ultrasonic sweep control
enum UltrasonicSweepCommand {
    /// Start ultrasonic sweep mode
    StartSweep,
    /// Start ultrasonic fixed-angle mode
    StartFixed {
        /// Fixed angle in degrees.
        angle_deg: f32,
    },
    /// Stop ultrasonic measurements
    Stop,
}

/// Control signal to trigger encoder measurements after specified duration
static US_SWEEP_CONTROL: Signal<CriticalSectionRawMutex, UltrasonicSweepCommand> = Signal::new();

/// Start continuous ultrasonic sweep readings
pub fn start_ultrasonic_sweep() {
    US_SWEEP_CONTROL.signal(UltrasonicSweepCommand::StartSweep);
}

/// Start fixed-angle ultrasonic readings (no servo sweep)
pub fn start_ultrasonic_fixed(angle_deg: f32) {
    US_SWEEP_CONTROL.signal(UltrasonicSweepCommand::StartFixed { angle_deg });
}

/// Stop ultrasonic readings
pub fn stop_ultrasonic_sweep() {
    US_SWEEP_CONTROL.signal(UltrasonicSweepCommand::Stop);
}

// Servo Configuration constants

/// uncalibrated default, the shortest duty cycle sent to a servo
const SERVO_DEFAULT_MIN_PULSE_WIDTH: u64 = 1000;

/// uncalibrated default, the longest duty cycle sent to a servo
const SERVO_DEFAULT_MAX_PULSE_WIDTH: u64 = 2000;

/// uncalibrated default, the degree of rotation that corresponds to a full cycle
const SERVO_DEFAULT_MAX_DEGREE_ROTATION: f32 = 160.0;

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
    /// PIO PWM driver for the servo output.
    pwm: PioPwm<'d, T, SM>,
    /// PWM period for the servo refresh interval.
    period: Duration,
    /// Minimum pulse width for the servo.
    min_pulse_width: Duration,
    /// Maximum pulse width for the servo.
    max_pulse_width: Duration,
    /// Maximum rotation range in degrees.
    max_degree_rotation: f32,
}

impl<'d, T: Instance, const SM: usize> ServoBuilder<'d, T, SM> {
    /// Create a new servo builder with default timing values.
    pub const fn new(pwm: PioPwm<'d, T, SM>) -> Self {
        Self {
            pwm,
            period: Duration::from_micros(SERVO_REFRESH_INTERVAL),
            min_pulse_width: Duration::from_micros(SERVO_DEFAULT_MIN_PULSE_WIDTH),
            max_pulse_width: Duration::from_micros(SERVO_DEFAULT_MAX_PULSE_WIDTH),
            max_degree_rotation: SERVO_DEFAULT_MAX_DEGREE_ROTATION,
        }
    }

    /// Set the minimum pulse width for the servo.
    pub const fn set_min_pulse_width(mut self, duration: Duration) -> Self {
        self.min_pulse_width = duration;
        self
    }

    /// Set the maximum pulse width for the servo.
    pub const fn set_max_pulse_width(mut self, duration: Duration) -> Self {
        self.max_pulse_width = duration;
        self
    }

    /// Set the maximum rotation range in degrees.
    pub const fn set_max_degree_rotation(mut self, degree: f32) -> Self {
        self.max_degree_rotation = degree;
        self
    }

    /// Build the configured servo instance.
    #[allow(clippy::missing_const_for_fn)]
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
    /// PIO PWM driver for the servo output.
    pwm: PioPwm<'d, T, SM>,
    /// Minimum pulse width for the servo.
    min_pulse_width: Duration,
    /// Maximum pulse width for the servo.
    max_pulse_width: Duration,
    /// Maximum rotation range in degrees.
    max_degree_rotation: f32,
}

impl<T: Instance, const SM: usize> Servo<'_, T, SM> {
    /// Start PWM output for the servo.
    pub fn start(&mut self) {
        self.pwm.start();
    }

    /// Rotates the servo to the specified angle in degrees
    ///
    /// # Arguments
    /// * `degree` - Target angle between 0° and `max_degree_rotation`°
    ///
    /// Automatically clamps input to valid range and converts
    /// angle to appropriate PWM pulse width for the servo.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_precision_loss,
        clippy::cast_sign_loss
    )]
    pub fn rotate_float(&mut self, degree: f32) {
        let max_degree_rotation = self.max_degree_rotation;
        let degree = f64::from(degree.clamp(0.0, max_degree_rotation));
        let max_pulse_nanos = u64::try_from(self.max_pulse_width.as_nanos()).unwrap_or(u64::MAX);
        let min_pulse_nanos = u64::try_from(self.min_pulse_width.as_nanos()).unwrap_or(0);
        let max_pulse_nanos = max_pulse_nanos as f64;
        let min_pulse_nanos = min_pulse_nanos as f64;
        let degree_per_nano_second = (max_pulse_nanos - min_pulse_nanos) / f64::from(max_degree_rotation);
        let mut duration = Duration::from_nanos((degree * degree_per_nano_second + min_pulse_nanos + 0.5) as u64);
        if self.max_pulse_width < duration {
            duration = self.max_pulse_width;
        }
        self.pwm.write(duration);
    }
}

/// Provides system clock implementation for the HCSR04 ultrasonic sensor driver
/// by wrapping `embassy_time::Instant`
pub struct Clock;

impl Now for Clock {
    /// Returns current time in microseconds since system start
    fn now_micros(&self) -> u64 {
        Instant::now().as_micros()
    }
}

/// Initializes the ultrasonic sensor with trigger and echo pins
///
/// This helper is constructed inside the sweep task when it starts.
/// Returns a configured Hcsr04 instance ready for measurements.
#[allow(clippy::missing_const_for_fn)]
pub fn setup_ultrasonic_sensor(
    trigger_pin: Output<'static>,
    echo_pin: Input<'static>,
) -> Hcsr04<Output<'static>, Input<'static>, Clock, Delay> {
    let hcsr04_config: Config = Config {
        distance_unit: DistanceUnit::Centimeters,
        temperature_unit: TemperatureUnit::Celsius,
    };
    Hcsr04::new(trigger_pin, echo_pin, hcsr04_config, Clock, Delay)
}

/// Initializes the servo for ultrasonic sweep control
///
/// This helper is constructed inside the sweep task when it starts.
/// Returns a configured Servo instance ready for angle control.
pub fn setup_servo(
    pwm_pio: PioPwm<'static, embassy_rp::peripherals::PIO0, 0>,
) -> Servo<'static, embassy_rp::peripherals::PIO0, 0> {
    let mut servo = ServoBuilder::new(pwm_pio)
        .set_max_degree_rotation(160.0) // SG90 servo has 160° range of motion
        .set_min_pulse_width(Duration::from_micros(500)) // SG90 minimum pulse width
        .set_max_pulse_width(Duration::from_micros(2400)) // SG90 maximum pulse width
        .build();

    servo.start();
    servo
}

/// Embassy task that handles ultrasonic distance measurements while sweeping a servo
///
/// This task combines a servo motor sweep with ultrasonic distance measurements
/// to create a scanning range finder effect. Measurements are filtered through
/// a moving median filter to reduce noise.
#[embassy_executor::task]
pub async fn ultrasonic_sweep(
    pwm_pio: PioPwm<'static, embassy_rp::peripherals::PIO0, 0>,
    trigger_pin: Output<'static>,
    echo_pin: Input<'static>,
) {
    let mut sensor = setup_ultrasonic_sensor(trigger_pin, echo_pin);
    let mut servo = setup_servo(pwm_pio);

    // Create median filter to smooth out distance measurements
    let mut median_filter = MovingMedian::<f64, ULTRASONIC_MEDIAN_WINDOW_SIZE>::new();

    let mut angle: f32 = 0.0;
    let mut angle_increment: f32 = 2.5;
    let mut filtered_distance: f64;
    let mut sweeping = true;
    let mut fixed_angle: f32 = 80.0;

    // 80 degrees is middle, 0 is right, 160 is left
    servo.rotate_float(80.0);
    Timer::after_millis(500).await;

    'command: loop {
        info!("Waiting for ultrasonic command");
        // Wait for a command, consuming it
        match US_SWEEP_CONTROL.wait().await {
            UltrasonicSweepCommand::StartSweep => {
                info!("Starting ultrasonic sweep");
                sweeping = true;
            }
            UltrasonicSweepCommand::StartFixed { angle_deg } => {
                info!("Starting ultrasonic fixed-angle mode");
                sweeping = false;
                fixed_angle = angle_deg.clamp(0.0, servo.max_degree_rotation);
            }
            UltrasonicSweepCommand::Stop => {
                info!("Stopping ultrasonic measurements");
                servo.rotate_float(80.0);
                continue 'command;
            }
        }

        loop {
            // Update servo position
            let current_angle = if sweeping { angle } else { fixed_angle };
            servo.rotate_float(current_angle);

            // Give servo time to reach position, also see if we must stop or switch modes
            match select(Timer::after_millis(25), US_SWEEP_CONTROL.wait()).await {
                Either::First(()) => {}
                Either::Second(command) => match command {
                    UltrasonicSweepCommand::Stop => {
                        info!("Stopping ultrasonic measurements");
                        servo.rotate_float(80.0);
                        continue 'command;
                    }
                    UltrasonicSweepCommand::StartSweep => {
                        info!("Switching to ultrasonic sweep");
                        sweeping = true;
                    }
                    UltrasonicSweepCommand::StartFixed { angle_deg } => {
                        info!("Switching to ultrasonic fixed-angle mode");
                        sweeping = false;
                        fixed_angle = angle_deg.clamp(0.0, servo.max_degree_rotation);
                    }
                },
            }

            // Take multiple measurements based on ULTRASONIC_MEDIAN_WINDOW_SIZE
            for _ in 0..ULTRASONIC_MEDIAN_WINDOW_SIZE {
                Timer::after_millis(90).await;
                let sample = match sensor.measure(ULTRASONIC_TEMPERATURE).await {
                    Ok(distance_cm) => {
                        if distance_cm > 200.0 {
                            200.0
                        } else {
                            distance_cm
                        }
                    }
                    Err(e) => {
                        error!("{}", e);
                        Timer::after_millis(20).await;
                        200.0 // Return safe distance on error to prevent false positives
                    }
                };
                median_filter.add_value(sample);
            }

            // Calculate median distance from the filtered measurements
            filtered_distance = median_filter.median();

            // Send reading event to orchestration task
            raise_event(Events::UltrasonicSweepReadingTaken(filtered_distance, current_angle)).await;

            // Update angle and check for direction change
            if sweeping {
                angle += angle_increment;
                let max_degree_rotation = servo.max_degree_rotation;
                if angle >= max_degree_rotation {
                    angle = max_degree_rotation;
                    angle_increment = -angle_increment; // Start moving back
                } else if angle <= 0.0 {
                    angle = 0.0;
                    angle_increment = -angle_increment; // Start moving forward
                }
            }
        }
    }
}
