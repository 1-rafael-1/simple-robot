//! Event System
//!
//! Provides a centralized event handling system for inter-task communication.
//! Uses an async channel to coordinate events between different parts of the system.
//!
//! # Event Flow
//! 1. Tasks generate events (e.g., sensor readings, button presses)
//! 2. Events are sent through the channel
//! 3. The orchestrator task processes events and updates system state
//! 4. State changes trigger corresponding actions in other tasks
//!
//! # Channel Design
//! - Multi-producer: Any task can send events
//! - Single-consumer: Orchestrator task processes all events
//! - Bounded capacity: 10 events maximum to prevent memory exhaustion
//! - Async operation: Non-blocking event handling
//!
//! # Usage Example
//! ```rust
//! // Sending an event
//! event::send(Events::ButtonPressed(ButtonId::A)).await;
//!
//! // Receiving an event (in orchestrator)
//! let event = event::wait().await;
//! ```

use crate::system::state::OperationMode;
use defmt::Format;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Multi-producer, single-consumer event channel
///
/// Capacity of 10 events provides good balance between:
/// - Memory usage
/// - Event processing latency
/// - System responsiveness
pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, Events, 10> = Channel::new();

/// Sends an event to the system channel
///
/// Events are queued if channel is full. If multiple events
/// occur simultaneously, they are processed in order of arrival.
pub async fn send(event: Events) {
    EVENT_CHANNEL.sender().send(event).await;
}

/// Receives the next event from the system channel
///
/// Called by the orchestrator task to process events sequentially.
/// Waits asynchronously if no events are available.
pub async fn wait() -> Events {
    EVENT_CHANNEL.receiver().receive().await
}

/// System-wide events that can occur during robot operation
#[derive(Debug, Clone)]
pub enum Events {
    /// Operation mode change requested
    /// - Triggered by button holds or system conditions
    /// - Carries target operation mode
    OperationModeSet(OperationMode),

    /// Obstacle detection status changed
    /// - true: Obstacle detected within threshold
    /// - false: Path is clear
    ObstacleDetected(bool),

    /// Obstacle avoidance maneuver completed
    /// - Triggered after attempting to navigate around obstacle
    /// - Used to coordinate next movement decision
    ObstacleAvoidanceAttempted,

    /// New battery level reading
    /// - Value range: 0-100 percent
    /// - Triggers LED color updates
    /// - May affect operation decisions
    BatteryLevelMeasured(u8),

    /// Button press detected
    /// - Short press (< 1 second)
    /// - Maps to immediate actions
    ButtonPressed(ButtonId),

    /// Button hold initiated
    /// - Long press started
    /// - Used for mode changes
    ButtonHoldStart(ButtonId),

    /// Button hold released
    /// - Long press ended
    /// - Completes hold actions
    ButtonHoldEnd(ButtonId),

    /// System inactivity timeout
    /// - No user input for extended period
    /// - Triggers power saving measures
    InactivityTimeout,

    /// Drive command was executed
    /// - Signals that motor speeds have been set
    /// - Triggers encoder measurement
    DriveCommandExecuted,

    /// Encoder measurement completed
    /// - Contains latest pulse counts and timing
    /// - Used for speed adjustments
    EncoderMeasurementTaken(crate::task::encoder_read::EncoderMeasurement),

    /// Ultrasonic sensor reading received
    /// - Contains distance measurements and servo angle
    /// - used for display and obstacle detection
    /// - TODO Used for autonomous navigation
    UltrasonicSweepReadingTaken(f64, f32),
}

/// Remote control button identifiers
#[derive(Debug, Clone, Copy, Format, PartialEq)]
pub enum ButtonId {
    /// Forward/Mode toggle button
    A,
    /// Right turn button
    B,
    /// Left turn button
    C,
    /// Backward button
    D,
}
