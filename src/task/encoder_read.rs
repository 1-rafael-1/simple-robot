//! Quadrature encoder feedback for motor speed control
//!
//! Provides motor speed measurements using shaft-mounted encoders.
//! Readings are taken on-demand after drive commands to enable
//! closed-loop speed control of the motors.

use defmt::info;
use embassy_rp::{
    gpio::Pull,
    pwm::{Config, InputMode, Pwm},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};

use crate::system::{
    event::{self, Events},
    resources::MotorEncoderResources,
};

/// Control signal to trigger encoder measurements after specified duration
pub static ENCODER_CONTROL: Signal<CriticalSectionRawMutex, Duration> = Signal::new();

/// Requests a new encoder measurement after waiting for the specified duration
pub fn request_measurement(duration: Duration) {
    ENCODER_CONTROL.signal(duration);
}

/// Blocks until next measurement request, returns the requested delay
async fn wait() -> Duration {
    ENCODER_CONTROL.wait().await
}

/// Single encoder measurement data
#[derive(Clone, Copy, Debug)]
pub struct EncoderData {
    /// Raw pulse count in measurement window
    pub pulse_count: u16,
    /// Measurement window duration in milliseconds
    pub elapsed_ms: u32,
}

/// Combined measurements from both motor encoders
#[derive(Clone, Copy, Debug)]
pub struct EncoderMeasurement {
    pub left: EncoderData,
    pub right: EncoderData,
}

/// Primary encoder measurement task
///
/// Takes periodic measurements using PWM input capture on rising edges.
/// Measurement timing is calculated for reliable readings at low speeds:
/// - At 20% speed: ~1400 RPM = 23.33 RPS
/// - In 100ms window: 2.33 rotations
/// - With 8 pulses/rev: ~18 pulses per measurement
/// - Minimum window for 10 pulses (considered still reliable): 54ms
/// - Using 75ms for some headroom, at higher speeds we will be fine anyway
#[embassy_executor::task]
pub async fn read_encoder(resources: MotorEncoderResources) {
    // Configure PWM inputs for pulse counting on rising edges
    let config = Config::default();
    let left_encoder = Pwm::new_input(
        resources.left_encoder_slice,
        resources.left_encoder_pin,
        Pull::None,
        InputMode::RisingEdge,
        config.clone(),
    );
    let right_encoder = Pwm::new_input(
        resources.right_encoder_slice,
        resources.right_encoder_pin,
        Pull::None,
        InputMode::RisingEdge,
        config,
    );

    loop {
        // Wait for next measurement request with delay for motor stabilization
        let delay = wait().await;

        // Allow motors to reach target speed
        Timer::after(delay).await;

        // Start measurement window
        let start = Instant::now();
        left_encoder.set_counter(0);
        right_encoder.set_counter(0);

        // Fixed measurement window for consistent readings
        Timer::after(Duration::from_millis(75)).await;
        let end = Instant::now();
        let left_pulses = left_encoder.counter();
        let right_pulses = right_encoder.counter();
        let elapsed = end - start;
        let elapsed_ms = elapsed.as_millis() as u32;

        // Package measurement data
        let measurement = EncoderMeasurement {
            left: EncoderData {
                pulse_count: left_pulses,
                elapsed_ms,
            },
            right: EncoderData {
                pulse_count: right_pulses,
                elapsed_ms,
            },
        };

        // Log raw measurement data
        info!(
            "Encoder measurement - Left: {} pulses in {}ms, Right: {} pulses in {}ms",
            measurement.left.pulse_count,
            measurement.left.elapsed_ms,
            measurement.right.pulse_count,
            measurement.right.elapsed_ms
        );

        // Signal measurement completion
        event::send(Events::EncoderMeasurementTaken(measurement)).await;
    }
}
