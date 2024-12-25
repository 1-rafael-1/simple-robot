//! Quadrature encoder task for motor speed feedback
//!
//! Provides independent encoder readings that are published through a signal channel.
//! The drive system subscribes to these measurements to make speed adjustments.

use defmt::info;
use embassy_rp::{
    gpio::Pull,
    pwm::{Config, InputMode, Pwm},
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker};

use crate::system::resources::MotorEncoderResources;

// DFRobot FIT0450 motor specifications
const PULSES_PER_REV: u32 = 8; // Encoder pulses per motor revolution
const GEAR_RATIO: u32 = 120; // 120:1 gear reduction

/// Encoder measurement data
#[derive(Clone, Copy, Debug)]
pub struct EncoderData {
    /// Wheel speed in RPM (negative for reverse rotation)
    pub rpm: f32,
    /// Raw pulse count since last reading
    pub pulse_count: u16,
}

/// Encoder measurements for both motors
#[derive(Clone, Copy, Debug)]
pub struct EncoderMeasurement {
    pub left: EncoderData,
    pub right: EncoderData,
}

/// Signal for publishing encoder measurements
pub static ENCODER_SIGNAL: Signal<CriticalSectionRawMutex, EncoderMeasurement> = Signal::new();

/// Converts raw pulse count to wheel RPM
fn calculate_rpm(pulses: u16, elapsed_ms: u32, forward: bool) -> f32 {
    let hz = (pulses as f32 * 1000.0) / elapsed_ms as f32;
    let motor_rpm = (hz / PULSES_PER_REV as f32) * 60.0;
    let wheel_rpm = motor_rpm / GEAR_RATIO as f32;
    if forward {
        wheel_rpm
    } else {
        -wheel_rpm
    }
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

        // Get motor directions from drive task
        let (left_forward, right_forward) = {
            let left = crate::task::drive::LEFT_MOTOR.lock().await;
            let right = crate::task::drive::RIGHT_MOTOR.lock().await;
            (
                left.as_ref().unwrap().is_forward(),
                right.as_ref().unwrap().is_forward(),
            )
        };

        // Create measurement data
        let measurement = EncoderMeasurement {
            left: EncoderData {
                rpm: calculate_rpm(left_pulses, elapsed_ms, left_forward),
                pulse_count: left_pulses,
            },
            right: EncoderData {
                rpm: calculate_rpm(right_pulses, elapsed_ms, right_forward),
                pulse_count: right_pulses,
            },
        };

        // Log data
        info!(
            "Encoder measurement - Left: {} RPM ({} pulses), Right: {} RPM ({} pulses)",
            measurement.left.rpm,
            measurement.left.pulse_count,
            measurement.right.rpm,
            measurement.right.pulse_count
        );

        // Publish measurement
        ENCODER_SIGNAL.signal(measurement);
    }
}
