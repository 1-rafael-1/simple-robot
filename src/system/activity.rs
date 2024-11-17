//! Activity signaling for standby control
//!
//! Provides a channel to track user interactions for standby mode management.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Signal for inactivity timer resets
static ACTIVITY_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signals user activity occurrence
pub fn signal_activity() {
    ACTIVITY_SIGNAL.signal(());
}

/// Waits for next activity signal
pub async fn wait() -> () {
    ACTIVITY_SIGNAL.wait().await
}
