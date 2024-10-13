use crate::task::resources::RGBLedResources;
use crate::task::system_events::wait_for_system_indicator_changed;
use crate::task::system_state::SYSTEM_STATE;
use embassy_rp::pwm::{Config, Pwm};

const PWM_MAX: u16 = 65535;
const PWM_MIN: u16 = 0;

#[embassy_executor::task]
pub async fn rgb_led_indicator(r: RGBLedResources) {
    // red
    let mut config_red = Config::default();
    config_red.top = PWM_MAX;
    config_red.compare_a = PWM_MIN;
    let mut pwm_red = Pwm::new_output_a(r.pwm_red, r.red_pin, config_red.clone());

    // green
    let mut config_green = Config::default();
    config_green.top = PWM_MAX;
    config_green.compare_a = PWM_MAX;
    let mut pwm_green = Pwm::new_output_a(r.pwm_green, r.green_pin, config_green.clone());

    loop {
        wait_for_system_indicator_changed().await;

        let battery_level = {
            let state = SYSTEM_STATE.lock().await;
            state.battery_level
        };

        // Update PWM configurations
        config_red.compare_a = (f32::from(battery_level / 100) * f32::from(PWM_MAX)) as u16;
        config_green.compare_a = (f32::from(battery_level / 100) * f32::from(PWM_MAX)) as u16;

        // Apply new configurations
        pwm_red.set_config(&config_red);
        pwm_green.set_config(&config_green);
    }
}
