//! Quadrature encoder task for motor speed feedback
//!
//! Provides independent encoder readings that are published through a signal channel.
//! The drive system subscribes to these measurements to make speed adjustments.

use crate::system::{
    event::{self, Events},
    resources::MotorEncoderResources,
};
use defmt::info;
use embassy_rp::{
    gpio::Pull,
    pwm::{Config, InputMode, Pwm},
};
use embassy_time::{Duration, Instant, Ticker};

/// Encoder measurement data
#[derive(Clone, Copy, Debug)]
pub struct EncoderData {
    /// Raw pulse count since last reading
    pub pulse_count: u16,
    /// Time elapsed since last measurement in milliseconds
    pub elapsed_ms: u32,
}

/// Encoder measurements for both motors
#[derive(Clone, Copy, Debug)]
pub struct EncoderMeasurement {
    pub left: EncoderData,
    pub right: EncoderData,
}

/// Main encoder reading task that publishes measurements periodically
#[embassy_executor::task]
pub async fn encoder(resources: MotorEncoderResources) {
    // Configure PWM input for encoder pulse counting
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

    // Create ticker for measurements (50ms interval for more frequent updates)
    let mut ticker = Ticker::every(Duration::from_millis(50));
    let mut last_update = Instant::now();

    loop {
        // Wait for next interval
        ticker.next().await;

        // Calculate elapsed time
        let now = Instant::now();
        let elapsed = now - last_update;
        let elapsed_ms = elapsed.as_millis() as u32;
        last_update = now;

        // Read and reset counters
        let left_pulses = left_encoder.counter();
        left_encoder.set_counter(0);
        let right_pulses = right_encoder.counter();
        right_encoder.set_counter(0);

        // Create measurement data with raw pulse counts and elapsed time
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

        // Log data
        info!(
            "Encoder measurement - Left: {} pulses in {}ms, Right: {} pulses in {}ms",
            measurement.left.pulse_count,
            measurement.left.elapsed_ms,
            measurement.right.pulse_count,
            measurement.right.elapsed_ms
        );

        // Notify system that measurement is ready
        event::send(Events::EncoderMeasurementTaken(measurement)).await;
    }
}
