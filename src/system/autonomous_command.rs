//! Autonomous Command Module
//!
//! This module provides a communication channel for autonomous operation commands.
//! It uses embassy-sync's Signal for thread-safe command passing between the
//! orchestrator and the autonomous drive task.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Signal for autonomous drive control
pub static AUTONOMOUS_CONTROL: Signal<CriticalSectionRawMutex, Command> = Signal::new();

/// Commands that control autonomous operation behavior
///
/// These commands represent the core state transitions possible
/// during autonomous operation
#[derive(Debug, Clone)]
pub enum Command {
    /// Start autonomous operation
    Start,
    /// Stop autonomous operation
    Stop,
    /// Obstacle has been detected
    AvoidObstacle,
}

/// Sends a new autonomous control command to the autonomous drive task
///
/// This is a non-blocking operation used by the orchestrator to
/// signal state changes to the autonomous system
///
/// # Arguments
/// * `command` - The command to send
pub fn signal(command: Command) {
    AUTONOMOUS_CONTROL.signal(command);
}

/// Waits for the next autonomous control command
///
/// This is a blocking operation used by the autonomous drive task
/// to receive its next command. It will await until a command is available.
///
/// # Returns
/// * `Command` - The next command to process
pub async fn wait() -> Command {
    AUTONOMOUS_CONTROL.wait().await
}
