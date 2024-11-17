//! Inactivity tracking
//!
//! Monitors user interaction and triggers standby after timeout.

use crate::system::{activity, event};
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Timer};

/// Standby timeout (s)
const INACTIVITY_TIMEOUT: Duration = Duration::from_secs(180); // 3 minutes

/// Inactivity monitoring task
#[embassy_executor::task]
pub async fn track_inactivity() {
    loop {
        match select(Timer::after(INACTIVITY_TIMEOUT), activity::wait()).await {
            Either::First(_) => {
                event::send(event::Events::InactivityTimeout).await;
            }
            Either::Second(_) => {
                continue;
            }
        }
    }
}
