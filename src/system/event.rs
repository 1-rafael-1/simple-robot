//! System Events
//!
//! Defines events and channels for inter-task communication.

use crate::system::state::OperationMode;
use defmt::Format;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Multi-producer, single-consumer event channel with capacity of 10
pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, Events, 10> = Channel::new();

/// Sends an event to the system channel
pub async fn send(event: Events) {
    EVENT_CHANNEL.sender().send(event).await;
}

/// Receives the next event from the system channel
pub async fn wait() -> Events {
    EVENT_CHANNEL.receiver().receive().await
}

/// System-wide events
#[derive(Debug, Clone)]
pub enum Events {
    /// Operation mode changed
    OperationModeSet(OperationMode),
    /// Obstacle detected
    ObstacleDetected(bool),
    /// An obstacle avoidance sequence has run
    ObstacleAvoidanceAttempted,
    /// Battery level measured
    BatteryLevelMeasured(u8),
    /// Button pressed
    ButtonPressed(ButtonId),
    /// Button hold started
    ButtonHoldStart(ButtonId),
    /// Button hold ended
    ButtonHoldEnd(ButtonId),
    /// Reached Inactivity Timeout
    InactivityTimeout,
}

/// Button identifiers
#[derive(Debug, Clone, Copy, Format, PartialEq)]
pub enum ButtonId {
    A,
    B,
    C,
    D,
}
