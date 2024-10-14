//! System Indicator Module
//!
//! This module provides functionality for managing and signaling changes
//! in the system indicator. It uses an embassy-sync Signal for thread-safe
//! communication across different parts of the system.
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Signal for system indicator changes
///
/// This static variable represents a thread-safe signal that can be used
/// to notify different parts of the system about changes in the system indicator.
pub static SYSTEM_INDICATOR_CHANGED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// Signals a change in the system indicator
///
/// This function is used to notify the system that the indicator has changed.
/// It's a synchronous operation that doesn't require awaiting.
pub fn send(value: bool) {
    SYSTEM_INDICATOR_CHANGED.signal(value);
}

/// Waits for a change in the system indicator
///
/// This asynchronous function blocks until a change in the system indicator
/// is signaled. It then returns the new value of the indicator.
pub async fn wait() -> bool {
    SYSTEM_INDICATOR_CHANGED.wait().await
}
