//! Inactivity Tracker Module
//!
//! This module monitors user interaction and triggers standby mode
//! after a period of inactivity.

use crate::system::event;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};

/// Duration of inactivity before entering standby mode
const INACTIVITY_TIMEOUT: Duration = Duration::from_secs(180); // 3 minutes

/// Signal for resetting the inactivity timer
static ACTIVITY_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signals that user activity has occurred
pub fn signal_activity() {
    ACTIVITY_SIGNAL.signal(());
}

/// Task that monitors inactivity and sends events when timeout occurs
#[embassy_executor::task]
pub async fn track_inactivity() {
    loop {
        match select(Timer::after(INACTIVITY_TIMEOUT), ACTIVITY_SIGNAL.wait()).await {
            Either::First(_) => {
                // Send inactivity timeout event to orchestrator
                event::send(event::Events::InactivityTimeout).await;
            }
            Either::Second(_) => {
                // Activity detected, continue monitoring
                continue;
            }
        }
    }
}
