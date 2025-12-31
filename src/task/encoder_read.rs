//! Quadrature encoder feedback for motor speed control
//!
//! Provides wheel speed measurements using shaft-mounted quadrature encoders.
//! Readings are taken continuously while motors are running to enable closed-loop
//! speed control and motion correction.
//!
//! The encoder measurements flow from this task to the motion monitor,
//! which combines them with IMU data to detect and correct motion errors.
//!
//! # Operation
//! - Uses PWM input capture to count encoder pulses in fixed time windows
//! - Applies median filtering to smooth speed measurements
//! - Converts raw pulse counts to output shaft RPS/RPM
//! - Handles noise via deadband when motors are stopped
//! - Can be started/stopped on demand via control signals
//!
//! # Configuration
//! - 60ms measurement windows for reliable readings at low speeds
//! - 8 pulses per motor revolution with 120:1 gear ratio
//! - 5-sample median filter window
//! - 0.1 RPS (6 RPM) deadband threshold

use defmt::{Format, info};
use embassy_rp::pwm::{Config, Pwm};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use moving_median::MovingMedian;

use crate::system::event::{Events, send_event};

/// Sampling configuration
/// /// Measurement timing is calculated for reliable readings at low speeds:
/// - At 20% speed: ~1400 RPM = 23.33 RPS
/// - In 100ms window: 2.33 rotations
/// - With 8 pulses/rev: ~18 pulses per measurement
/// - Minimum window for 10 pulses (considered still reliable): 54ms
/// - Using 60ms for some headroom, at higher speeds we will be fine anyway
const SAMPLE_INTERVAL: Duration = Duration::from_millis(60);

/// Number of samples to use for median filtering
const MEDIAN_WINDOW_SIZE: usize = 5;

/// Encoder pulses per revolution
const PULSES_PER_MOTOR_REV: u16 = 8;
/// Motor gear ratio
const GEAR_RATIO: u16 = 120;
/// Encoder pulses per motor revolution
const PULSES_PER_OUTPUT_REV: u16 = PULSES_PER_MOTOR_REV * GEAR_RATIO; // 960

/// Combined encoder measurement data
#[derive(Clone, Copy, Debug, Format)]
pub struct EncoderMeasurement {
    pub left: WheelSpeed,
    pub right: WheelSpeed,
    pub timestamp: Instant,
}

/// Wheel speed measurement
#[derive(Clone, Copy, Debug, Format)]
pub struct WheelSpeed {
    /// Speed in revolutions per second at the output shaft
    pub rps: f32,
    /// Speed in RPM at the output shaft (for human readability)
    pub rpm: f32,
    /// Raw data for debugging
    pub raw: EncoderDataRaw,
}

/// Raw encoder data
#[derive(Clone, Copy, Debug, Format)]
pub struct EncoderDataRaw {
    /// Raw pulse count in measurement window
    pub pulse_count: u16,
    /// Measurement window duration in milliseconds
    pub elapsed_ms: u32,
}

/// Median filter for speed measurements
pub struct SpeedFilter {
    filter: MovingMedian<f32, MEDIAN_WINDOW_SIZE>,
}

impl SpeedFilter {
    /// Create a new speed filter
    pub fn new() -> Self {
        Self {
            filter: MovingMedian::new(),
        }
    }

    /// Add a new speed value to the filter
    pub fn update(&mut self, speed: f32) -> f32 {
        self.filter.add_value(speed);
        self.filter.median()
    }

    /// Reset the filter
    pub fn reset(&mut self) {
        self.filter.clear();
    }
}

impl WheelSpeed {
    pub fn from_encoder_data(data: EncoderDataRaw, filter: &mut SpeedFilter) -> Self {
        // Convert pulse count to output shaft revolutions
        let revolutions = data.pulse_count as f32 / PULSES_PER_OUTPUT_REV as f32;
        let seconds = data.elapsed_ms as f32 / 1000.0;

        // Calculate raw RPS
        let raw_rps = revolutions / seconds;

        // Apply median filter
        let filtered_rps = filter.update(raw_rps);

        Self {
            rps: filtered_rps,
            rpm: filtered_rps * 60.0,
            raw: data,
        }
    }

    /// Apply a deadband to handle noise when motor is stopped
    /// Based on the motor's lowest speed (60 RPM @ 3V = 1 RPS)
    /// Using 0.1 RPS as threshold (6 RPM)
    pub fn with_deadband(self) -> Self {
        const DEADBAND_THRESHOLD: f32 = 0.1;

        if self.rps.abs() < DEADBAND_THRESHOLD {
            Self {
                rps: 0.0,
                rpm: 0.0,
                raw: self.raw,
            }
        } else {
            self
        }
    }
}

/// Commands for encoder reading control
enum EncoderCommand {
    /// Start encoder readings
    Start,
    /// Stop encoder readings
    Stop,
}

/// Control signal to trigger encoder measurements after specified duration
static ENCODER_CONTROL: Signal<CriticalSectionRawMutex, EncoderCommand> = Signal::new();

/// Start continuous encoder readings
pub fn start_encoder_readings() {
    ENCODER_CONTROL.signal(EncoderCommand::Start);
}

/// Stop encoder readings
pub fn stop_encoder_readings() {
    ENCODER_CONTROL.signal(EncoderCommand::Stop);
}

/// Configures PWM inputs for encoder pulse counting
///
/// This function should be called from main.rs to set up PWM input configuration.
/// Returns a PWM Config object for encoder pulse counting on rising edges.
pub fn configure_encoder_pwm() -> Config {
    let mut config = Config::default();
    config.divider = 1.into();
    config.phase_correct = false;
    config
}

/// Primary encoder measurement task
///
/// Takes periodic measurements using PWM input capture on rising edges.
#[embassy_executor::task]
pub async fn encoder_read(left_encoder: Pwm<'static>, right_encoder: Pwm<'static>) {
    let mut left_filter = SpeedFilter::new();
    let mut right_filter = SpeedFilter::new();

    'command: loop {
        // Wait for next command, consuming it
        match ENCODER_CONTROL.wait().await {
            EncoderCommand::Start => {
                info!("Starting encoder measurement");
                'read: loop {
                    // Check if we should read the next command, because we possibly have a new command
                    if ENCODER_CONTROL.signaled() {
                        break 'read;
                    }

                    // Start measurement window
                    let start = Instant::now();
                    left_encoder.set_counter(0);
                    right_encoder.set_counter(0);

                    // Read more frequently to debug
                    for i in 0..6 {
                        Timer::after(Duration::from_millis(10)).await;
                        let interim_count = left_encoder.counter();
                        info!("Interim count {}: {}", i, interim_count);
                    }

                    // Read encoder measurements, starting with a delay for motor stabilization
                    Timer::after(SAMPLE_INTERVAL).await;

                    // Read encoder measurements
                    let end = Instant::now();
                    let left_pulses = left_encoder.counter();
                    let right_pulses = right_encoder.counter();
                    let elapsed = end - start;
                    let elapsed_ms = elapsed.as_millis() as u32;
                    info!("Encoder: L={} R={} in {}ms", left_pulses, right_pulses, elapsed_ms);

                    // Package measurement data
                    let measurement = EncoderMeasurement {
                        left: WheelSpeed::from_encoder_data(
                            EncoderDataRaw {
                                pulse_count: left_pulses,
                                elapsed_ms,
                            },
                            &mut left_filter,
                        )
                        .with_deadband(),
                        right: WheelSpeed::from_encoder_data(
                            EncoderDataRaw {
                                pulse_count: right_pulses,
                                elapsed_ms,
                            },
                            &mut right_filter,
                        )
                        .with_deadband(),
                        timestamp: Instant::now(),
                    };

                    // Signal measurement completion
                    send_event(Events::EncoderMeasurementTaken(measurement)).await;
                }
            }
            EncoderCommand::Stop => {
                info!("Stopping encoder measurement");

                // reset filters
                left_filter.reset();
                right_filter.reset();

                // Return to waiting for next command
                continue 'command;
            }
        }
    }
}
