//! Drive commands
//!
//! Manages motor control commands via thread-safe signals.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Drive command signal
pub static DRIVE: Signal<CriticalSectionRawMutex, Command> = Signal::new();

/// Sends drive command
pub fn update(command: Command) {
    DRIVE.signal(command);
}

/// Waits for next drive command
pub async fn wait() -> Command {
    DRIVE.wait().await
}

/// Motor control commands
#[derive(Debug, Clone)]
pub enum Command {
    /// Left turn (0-100)
    Left(u8),
    /// Right turn (0-100)
    Right(u8),
    /// Forward speed (0-255)
    Forward(u8),
    /// Backward speed (0-255)
    Backward(u8),
    /// Immediate stop
    Brake,
    /// Gradual stop
    Coast,
    /// Power saving mode
    Standby,
}
