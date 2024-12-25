//! Quadrature encoder task for motor speed feedback
//!
//! Provides independent encoder readings. Readings are taken on demand (triggered by the orchestrator after a drive command) and are
//! sent to the orchestrator for further processing.
//!
//! We usually want to use the encoder data to determine the motor speed in order to make motor speed corrections.

use crate::system::{
    event::{self, Events},
    resources::MotorEncoderResources,
};
use defmt::info;
use embassy_rp::{
    gpio::Pull,
    pwm::{Config, InputMode, Pwm},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};

// Encoder control signal to trigger encoder readings after Duration
pub static ENCODER_CONTROL: Signal<CriticalSectionRawMutex, Duration> = Signal::new();

pub fn request_measurement(duration: Duration) {
    ENCODER_CONTROL.signal(duration);
}

async fn wait() -> Duration {
    ENCODER_CONTROL.wait().await
}

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
pub async fn read_encoder(resources: MotorEncoderResources) {
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

    loop {
        // Wait for the next measurement request. The Duration received is used to give the motors time to reach their target speed before the
        // measurements are taken.
        let delay = wait().await;

        // Wait for the specified duration before taking measurements
        Timer::after(delay).await;

        // Read encoder values
        // First reset the counter before reading and get the current Instant
        let start = Instant::now();
        left_encoder.set_counter(0);
        right_encoder.set_counter(0);
        // Then wait a defined duration, get the current Instant again and make measurements. We must make sure that we take measurements
        // of at least 10 pulses.
        // The encoder is fixed to the motor shaft, so we do not need to concern ourselves with transmission. So:
        // At 20% speed the motor should have 1.400RPM, 1400/60s = 23.33 rotations per second.
        // 23,33rps * (100ms/1000ms) = 2.33 rotation in 100ms.
        // 8 pulses/rev * 2.33rev = 18.64 pulses in 100ms. -> 100ms is plenty. 54ms are bare minimum. So we settle on 70ms to have some margin.
        // Most of the time the robot will operate at >20% speed, so we will be fine.
        Timer::after(Duration::from_millis(100)).await;
        let end = Instant::now();
        let left_pulses = left_encoder.counter();
        let right_pulses = right_encoder.counter();
        let elapsed = end - start;
        let elapsed_ms = elapsed.as_millis() as u32;

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
