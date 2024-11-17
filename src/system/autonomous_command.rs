//! Autonomous control commands
//!
//! Channel for autonomous drive task control.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Signal for autonomous drive control
pub static AUTONOMOUS_CONTROL: Signal<CriticalSectionRawMutex, Command> = Signal::new();

/// Autonomous operation commands
#[derive(Debug, Clone)]
pub enum Command {
    /// Start autonomous mode
    Start,
    /// Stop autonomous mode
    Stop,
    /// Handle detected obstacle
    AvoidObstacle,
}

/// Sends autonomous control command
pub fn signal(command: Command) {
    AUTONOMOUS_CONTROL.signal(command);
}

/// Waits for next control command
pub async fn wait() -> Command {
    AUTONOMOUS_CONTROL.wait().await
}
