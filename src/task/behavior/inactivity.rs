//! Inactivity-related behavior handlers.

use defmt::info;

/// Handle inactivity timeout.
#[allow(clippy::unused_async)]
pub async fn handle_inactivity_timeout() {
    info!("Inactivity timeout");
    // TODO: Implement power saving
    // - Switch to manual mode
    // - Enter standby
}
