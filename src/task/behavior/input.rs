//! Input-related behavior handlers.

use defmt::info;

use crate::system::event::RCButtonId;

/// Handle button press events.
#[allow(clippy::unused_async)]
pub async fn handle_button_pressed(_button_id: RCButtonId) {
    info!("Button pressed");
    // TODO: Implement button actions
    // - Map to drive commands
    // - Signal activity tracker
}

/// Handle button hold start events.
#[allow(clippy::unused_async)]
pub async fn handle_button_hold_start(_button_id: RCButtonId) {
    info!("Button hold started");
    // TODO: Implement hold start actions
    // - Prepare for mode change
    // - Signal activity tracker
}

/// Handle button hold end events.
#[allow(clippy::unused_async)]
pub async fn handle_button_hold_end(_button_id: RCButtonId) {
    info!("Button hold ended");
    // TODO: Implement hold end actions
    // - Complete mode change
    // - Signal activity tracker
}
