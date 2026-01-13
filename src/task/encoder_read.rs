//! Encoder reading task for motor speed measurement
//!
//! This module provides a dedicated task for reading encoder pulse counts from all four motors.
//! It separates sensing (encoder reading) from actuation (motor control), following the same
//! architecture pattern as the IMU sensor task.
//!
//! # Architecture
//!
//! The encoder task operates independently from motor control:
//! - **Sensing**: Reads encoder pulse counts at configurable intervals
//! - **Events**: Publishes measurements via the event system
//! - **Commands**: Accepts control commands (start/stop/reset)
//! - **Power Management**: Can be stopped when not needed
//!
//! # Usage Patterns
//!
//! ## Basic Sampling
//! ```rust
//! // Start reading at 20Hz (50ms interval)
//! encoder_read::send_command(EncoderCommand::Start { interval_ms: 50 }).await;
//!
//! // Stop when done
//! encoder_read::send_command(EncoderCommand::Stop).await;
//! ```
//!
//! ## Delta Tracking (Measuring Motion)
//! ```rust
//! // Reset counters before a maneuver
//! encoder_read::send_command(EncoderCommand::Reset).await;
//!
//! // Perform the maneuver
//! motor_driver::send_command(SetSpeed { speed: 50 }).await;
//! Timer::after(Duration::from_secs(2)).await;
//!
//! // Read the measurement - absolute counts ARE the delta since reset
//! let measurement = wait_for_encoder_event().await;
//! let pulses = measurement.left_front;  // This is the delta!
//! ```
//!
//! ## Auto-Reset for Stop-and-Go
//! ```rust
//! // Enable auto-reset when motors stop
//! encoder_read::send_command(EncoderCommand::AutoResetOnMotorStop(true)).await;
//!
//! // Now encoders automatically reset each time motors stop
//! // Useful for measuring individual drive segments
//! ```
//!
//! # Encoder Hardware
//!
//! - **Type**: Quadrature encoders (using single channel for simplicity)
//! - **Resolution**: 8 pulses per motor revolution
//! - **Gear Ratio**: 120:1 (960 pulses per output shaft revolution)
//! - **Reading Method**: PWM input mode counting rising edges
//! - **Counter Size**: 16-bit (0-65535, wraps around)
//!
//! # Sampling Rate Guidelines
//!
//! - **20-50 Hz** (20-50ms): Good for most control loops
//! - **10 Hz** (100ms): Power-saving mode, suitable for monitoring
//! - **100 Hz** (10ms): High-frequency feedback for precise control
//!
//! Note: Faster sampling increases CPU load but provides better time resolution

use defmt::info;
use embassy_rp::pwm::Pwm;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};

use crate::system::event::{Events, send_event};

/// Commands for encoder reading control
#[derive(Debug, Clone, Copy)]
pub enum EncoderCommand {
    /// Start continuous encoder sampling
    ///
    /// # Parameters
    /// - `interval_ms`: Time between samples in milliseconds (e.g., 50 = 20Hz)
    Start { interval_ms: u64 },

    /// Stop encoder sampling (power saving)
    Stop,

    /// Reset all encoder counters to zero
    ///
    /// Useful for measuring deltas during maneuvers. After reset, the next
    /// measurement will show how many pulses occurred since the reset.
    Reset,

    /// Enable/disable automatic reset when motors stop
    ///
    /// When enabled, encoders automatically reset to zero whenever all motors
    /// are stopped (speed = 0). This is convenient for stop-and-go driving
    /// where you want to measure each segment independently.
    ///
    /// # Parameters
    /// - `true`: Enable auto-reset
    /// - `false`: Disable auto-reset (default)
    AutoResetOnMotorStop(bool),
}

/// Command channel for encoder control
const COMMAND_QUEUE_SIZE: usize = 5;
static ENCODER_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, EncoderCommand, COMMAND_QUEUE_SIZE> = Channel::new();

/// Send a command to the encoder task
pub async fn send_command(command: EncoderCommand) {
    ENCODER_COMMAND_CHANNEL.sender().send(command).await;
}

/// Receive a command (internal use by encoder task)
async fn receive_command() -> EncoderCommand {
    ENCODER_COMMAND_CHANNEL.receiver().receive().await
}

/// Try to receive a command without blocking (internal use by encoder task)
fn try_receive_command() -> Option<EncoderCommand> {
    ENCODER_COMMAND_CHANNEL.receiver().try_receive().ok()
}

/// Encoder measurement data
///
/// Contains pulse counts from all four motors and a timestamp.
/// Counts are 16-bit values (0-65535) that wrap around on overflow.
#[derive(Debug, Clone, Copy)]
pub struct EncoderMeasurement {
    /// Left front motor encoder count
    pub left_front: u16,
    /// Left rear motor encoder count
    pub left_rear: u16,
    /// Right front motor encoder count
    pub right_front: u16,
    /// Right rear motor encoder count
    pub right_rear: u16,
    /// Timestamp of measurement in milliseconds since boot
    pub timestamp_ms: u32,
}

/// Encoder channels for all 4 motors
struct EncoderChannels {
    left_front: Pwm<'static>,  // GPIO 7, PWM Slice 3B
    left_rear: Pwm<'static>,   // GPIO 21, PWM Slice 2B
    right_front: Pwm<'static>, // GPIO 9, PWM Slice 4B
    right_rear: Pwm<'static>,  // GPIO 27, PWM Slice 5B
}

impl EncoderChannels {
    /// Reset all encoder counters to zero
    fn reset_all(&mut self) {
        self.left_front.set_counter(0);
        self.left_rear.set_counter(0);
        self.right_front.set_counter(0);
        self.right_rear.set_counter(0);
    }

    /// Read all encoder counts at once
    fn read_all(&self) -> EncoderMeasurement {
        let timestamp_ms = Instant::now().as_millis() as u32;
        EncoderMeasurement {
            left_front: self.left_front.counter(),
            left_rear: self.left_rear.counter(),
            right_front: self.right_front.counter(),
            right_rear: self.right_rear.counter(),
            timestamp_ms,
        }
    }
}

/// Encoder reading task
///
/// Manages encoder sampling and publishes measurements via events.
/// Supports start/stop, manual reset, and auto-reset on motor stop.
///
/// # Parameters
/// - `encoder_left_front`: PWM channel configured as encoder input for left front motor
/// - `encoder_left_rear`: PWM channel configured as encoder input for left rear motor
/// - `encoder_right_front`: PWM channel configured as encoder input for right front motor
/// - `encoder_right_rear`: PWM channel configured as encoder input for right rear motor
#[embassy_executor::task]
pub async fn encoder_read(
    encoder_left_front: Pwm<'static>,
    encoder_left_rear: Pwm<'static>,
    encoder_right_front: Pwm<'static>,
    encoder_right_rear: Pwm<'static>,
) {
    info!("Encoder read task started");

    let mut encoders = EncoderChannels {
        left_front: encoder_left_front,
        left_rear: encoder_left_rear,
        right_front: encoder_right_front,
        right_rear: encoder_right_rear,
    };

    // Task state
    let mut sampling = false;
    let mut interval_ms = 50u64; // Default 20Hz
    let mut auto_reset_on_stop = false;
    let mut last_measurement = encoders.read_all();

    loop {
        if sampling {
            // Wait for either timeout or command
            let timeout = Timer::after(Duration::from_millis(interval_ms));

            // Check for commands without blocking
            if let Some(command) = try_receive_command() {
                match command {
                    EncoderCommand::Start {
                        interval_ms: new_interval,
                    } => {
                        info!(
                            "Encoder sampling already active, updating interval to {}ms",
                            new_interval
                        );
                        interval_ms = new_interval;
                    }
                    EncoderCommand::Stop => {
                        info!("Stopping encoder sampling");
                        sampling = false;
                    }
                    EncoderCommand::Reset => {
                        info!("Resetting encoder counters");
                        encoders.reset_all();
                    }
                    EncoderCommand::AutoResetOnMotorStop(enabled) => {
                        info!("Auto-reset on motor stop: {}", enabled);
                        auto_reset_on_stop = enabled;
                    }
                }
            } else {
                // No command, wait for sampling interval
                timeout.await;

                // Read encoders
                let measurement = encoders.read_all();

                // Check for auto-reset condition
                if auto_reset_on_stop && motors_stopped(&measurement, &last_measurement) {
                    info!("Motors stopped, auto-resetting encoders");
                    encoders.reset_all();
                    // Read again after reset
                    last_measurement = encoders.read_all();
                } else {
                    last_measurement = measurement;

                    // Send measurement event
                    send_event(Events::EncoderMeasurementTaken(measurement)).await;
                }
            }
        } else {
            // Not sampling, wait for start command
            match receive_command().await {
                EncoderCommand::Start {
                    interval_ms: new_interval,
                } => {
                    info!("Starting encoder sampling at {}ms interval", new_interval);
                    interval_ms = new_interval;
                    sampling = true;
                    // Take initial reading
                    last_measurement = encoders.read_all();
                }
                EncoderCommand::Stop => {
                    // Already stopped, ignore
                }
                EncoderCommand::Reset => {
                    info!("Resetting encoder counters (while stopped)");
                    encoders.reset_all();
                }
                EncoderCommand::AutoResetOnMotorStop(enabled) => {
                    info!(
                        "Auto-reset on motor stop: {} (will take effect when sampling starts)",
                        enabled
                    );
                    auto_reset_on_stop = enabled;
                }
            }
        }
    }
}

/// Check if motors have stopped by comparing consecutive measurements
///
/// Motors are considered stopped if all encoder counts are unchanged
/// between two consecutive readings.
fn motors_stopped(current: &EncoderMeasurement, previous: &EncoderMeasurement) -> bool {
    current.left_front == previous.left_front
        && current.left_rear == previous.left_rear
        && current.right_front == previous.right_front
        && current.right_rear == previous.right_rear
}
