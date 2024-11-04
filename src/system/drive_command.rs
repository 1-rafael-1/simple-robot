//! Drive Command Module
//!
//! This module provides functionality for managing and signaling drive commands
//! in the robot system. It uses an embassy-sync Signal for thread-safe
//! communication across different parts of the system.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Signal for drive commands
///
/// This static variable represents a thread-safe signal that can be used
/// to notify different parts of the system about new drive commands.
pub static DRIVE: Signal<CriticalSectionRawMutex, Command> = Signal::new();

/// Sends a new drive command
///
/// This function is used to issue a new drive command to the system.
/// It's a synchronous operation that doesn't require awaiting.
pub fn update(command: Command) {
    DRIVE.signal(command);
}

/// Waits for a new drive command
///
/// This asynchronous function blocks until a new drive command
/// is signaled. It then returns the new command.
pub async fn wait() -> Command {
    DRIVE.wait().await
}

/// Enum representing drive commands
#[derive(Debug, Clone)]
pub enum Command {
    /// Turn left with specified intensity (0-100)
    Left(u8),
    /// Turn right with specified intensity (0-100)
    Right(u8),
    /// Move forward at specified speed (0-255)
    Forward(u8),
    /// Move backward at specified speed (0-255)
    Backward(u8),
    /// Apply brakes to stop immediately
    Brake,
    /// Stop applying power, allowing the robot to coast to a stop
    Coast,
    /// Set the driver into standby
    Standby,
}
