//! Battery-related behavior handlers.

use defmt::info;

use crate::{system::state, task::indicators::rgb_led_indicate};

/// Handle battery measurement (level and voltage).
pub async fn handle_battery_measured(level: u8, voltage: f32) {
    info!("Battery level measured");

    {
        let mut state = state::SYSTEM_STATE.lock().await;
        state.battery_voltage = Some(voltage);
        state.battery_level = Some(level);
    }

    rgb_led_indicate::update_indicator(false);
}
