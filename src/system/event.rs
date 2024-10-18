//! System Events Module
//!
//! This module defines the event system used for inter-task communication
//! in the robot. It includes event types and a channel for event transmission,
//! and utility functions for sending and receiving events.

use crate::system::state::OperationMode;
use defmt::Format;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Channel for system-wide events
///
/// This channel allows multiple producers and a single consumer of events.
/// It has a capacity of 10 events.
pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, Events, 10> = Channel::new();

/// Sends an event to the system event channel
pub async fn send(event: Events) {
    EVENT_CHANNEL.sender().send(event).await;
}

/// Waits for and receives the next event from the system event channel
pub async fn wait() -> Events {
    EVENT_CHANNEL.receiver().receive().await
}

/// Enum representing system-wide events
#[derive(Debug, Clone)]
pub enum Events {
    /// Operation mode changed
    OperationModeSet(OperationMode),
    /// Obstacle detected
    ObstacleDetected(bool),
    /// Battery level measured
    BatteryLevelMeasured(u8),
    /// Button pressed
    ButtonPressed(ButtonId),
    /// Button hold started
    ButtonHoldStart(ButtonId),
    /// Button hold ended
    ButtonHoldEnd(ButtonId),
}

/// Button ID for button events.
#[derive(Debug, Clone, Copy, Format, PartialEq)]
pub enum ButtonId {
    A,
    B,
    C,
    D,
}

/// Enum representing drive commands
#[derive(Debug, Clone)]
pub enum DriveCommand {
    Left(u32),
    Right(u32),
    Forward(u8),
    Backward(u8),
    Brake,
    Coast,
    Standby,
}
