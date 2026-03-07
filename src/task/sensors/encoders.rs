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

use crate::system::event::{Events, raise_event};

/// Commands for encoder reading control
#[derive(Debug, Clone, Copy)]
pub enum EncoderCommand {
    /// Start continuous encoder sampling
    ///
    /// # Parameters
    /// - `interval_ms`: Time between samples in milliseconds (e.g., 50 = 20Hz)
    Start {
        /// Sampling interval in milliseconds
        interval_ms: u64,
    },

    /// Stop encoder sampling (power saving)
    Stop,

    /// Reset all encoder counters to zero
    ///
    /// Useful for measuring deltas during maneuvers. After reset, the next
    /// measurement will show how many pulses occurred since the reset.
    Reset,
}

/// Size of the command queue for encoder control
const COMMAND_QUEUE_SIZE: usize = 16;
/// Command channel for encoder control
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
#[derive(Debug, Clone, Copy, defmt::Format)]
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
    pub timestamp_ms: u64,
}

/// Encoder channels for all 4 motors
struct EncoderChannels {
    /// Encoder input for left front motor
    left_front: Pwm<'static>,
    /// Encoder input for left rear motor
    left_rear: Pwm<'static>,
    /// Encoder input for right front motor
    right_front: Pwm<'static>,
    /// Encoder input for right rear motor
    right_rear: Pwm<'static>,
}

impl EncoderChannels {
    /// Reset all encoder counters to zero
    fn reset_all(&self) {
        self.left_front.set_counter(0);
        self.left_rear.set_counter(0);
        self.right_front.set_counter(0);
        self.right_rear.set_counter(0);
    }

    /// Read all encoder counts at once
    fn read_all(&self) -> EncoderMeasurement {
        let timestamp_ms = Instant::now().as_millis();
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

    let encoders = EncoderChannels {
        left_front: encoder_left_front,
        left_rear: encoder_left_rear,
        right_front: encoder_right_front,
        right_rear: encoder_right_rear,
    };

    // Task state
    let mut sampling = false;
    let mut interval_ms = 50u64; // Default 20Hz

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
                }
            } else {
                // No command, wait for sampling interval
                timeout.await;
                let measurement = encoders.read_all();
                raise_event(Events::EncoderMeasurementTaken(measurement)).await;
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
                }
                EncoderCommand::Stop => {
                    // Already stopped, ignore
                }
                EncoderCommand::Reset => {
                    info!("Resetting encoder counters (while stopped)");
                    encoders.reset_all();
                }
            }
        }
    }
}
