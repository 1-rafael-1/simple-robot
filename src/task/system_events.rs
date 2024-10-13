//! System Events Module
//!
//! This module defines the event system used for inter-task communication
//! in the robot. It includes event types, channels/signals for event transmission,
//! and utility functions for sending and receiving events.

use crate::task::system_state::OperationMode;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;

/// Channel for system-wide events
///
/// This channel allows multiple producers and a single consumer of events.
/// It has a capacity of 10 events.
pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, Events, 10> = Channel::new();

/// Sends an event to the system event channel
pub async fn send_event(event: Events) {
    EVENT_CHANNEL.sender().send(event).await;
}

/// Waits for and receives the next event from the system event channel
pub async fn wait_for_event() -> Events {
    EVENT_CHANNEL.receiver().receive().await
}

/// Signal for system indicator changes
pub static SYSTEM_INDICATOR_CHANGED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// Signals a change in the system indicator
pub async fn send_system_indicator_changed(value: bool) {
    SYSTEM_INDICATOR_CHANGED.signal(value);
}

/// Waits for a change in the system indicator
pub async fn wait_for_system_indicator_changed() -> bool {
    SYSTEM_INDICATOR_CHANGED.wait().await
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
