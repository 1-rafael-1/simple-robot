//! Inactivity tracking
//!
//! Monitors user interaction and triggers standby after timeout.

use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};

use crate::system::event;

/// Signal for inactivity timer resets
static ACTIVITY_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Standby timeout (s)
const INACTIVITY_TIMEOUT: Duration = Duration::from_secs(180); // 3 minutes

/// Signals user activity occurrence
pub fn signal_activity() {
    ACTIVITY_SIGNAL.signal(());
}

/// Waits for next activity signal
pub async fn wait() -> () {
    ACTIVITY_SIGNAL.wait().await
}

/// Inactivity monitoring task
#[embassy_executor::task]
pub async fn track_inactivity() {
    loop {
        match select(Timer::after(INACTIVITY_TIMEOUT), wait()).await {
            Either::First(_) => {
                event::send_event(event::Events::InactivityTimeout).await;
            }
            Either::Second(_) => {
                continue;
            }
        }
    }
}
