//! System indicator state
//!
//! Manages system indicator changes via signals.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// System indicator state signal
pub static SYSTEM_INDICATOR_CHANGED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// Updates indicator state
pub fn update(affirm: bool) {
    SYSTEM_INDICATOR_CHANGED.signal(affirm);
}

/// Waits for next indicator change
pub async fn wait() -> bool {
    SYSTEM_INDICATOR_CHANGED.wait().await
}
