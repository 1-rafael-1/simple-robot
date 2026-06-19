//! Battery-related behavior handlers.

use defmt::debug;

use crate::{system::state::power, task::indicators::rgb_led_indicate};

/// Handle battery measurement (level and voltage).
pub async fn handle_battery_measured(level: u8, voltage: f32) {
    debug!("Battery level measured");

    power::set_battery(level, voltage).await;

    rgb_led_indicate::update_indicator(false);
}
