use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Signal for system indicator changes
pub static SYSTEM_INDICATOR_CHANGED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// Signals a change in the system indicator
pub fn send(value: bool) {
    SYSTEM_INDICATOR_CHANGED.signal(value);
}

/// Waits for a change in the system indicator
pub async fn wait() -> bool {
    SYSTEM_INDICATOR_CHANGED.wait().await
}
