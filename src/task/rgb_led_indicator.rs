//! RGB LED Indicator Module
//!
//! This module controls the RGB LED indicator for the robot, providing visual feedback
//! about the system state, including battery level and operation mode.
use crate::system::indicator;
use crate::system::resources::RGBLedResources;
use crate::system::state::{OperationMode, SYSTEM_STATE};
use defmt::info;
use embassy_futures::select::select;
use embassy_futures::select::Either;
use embassy_rp::pwm::{Config, Pwm};
use embassy_time::{Duration, Timer};

/// Maximum PWM value (fully on)
const PWM_MAX: u16 = 65535;

/// Minimum PWM value (fully off)
const PWM_MIN: u16 = 0;

/// Interval for LED blinking in autonomous mode
const BLINK_INTERVAL: Duration = Duration::from_millis(700);

/// Controls the RGB LED indicator based on system state.
///
/// This task manages the RGB LED, adjusting its color and behavior to reflect:
/// - Battery level: Green (full) to Red (empty)
/// - Operation mode: Solid (Manual) or Blinking (Autonomous)
#[embassy_executor::task]
pub async fn rgb_led_indicator(r: RGBLedResources) {
    // Configure red LED PWM
    let mut config_red = Config::default();
    config_red.top = PWM_MAX;
    config_red.compare_a = PWM_MIN;
    let mut pwm_red = Pwm::new_output_a(r.pwm_red, r.red_pin, config_red.clone());

    // Configure green LED PWM
    let mut config_green = Config::default();
    config_green.top = PWM_MAX;
    config_green.compare_a = PWM_MAX;
    let mut pwm_green = Pwm::new_output_a(r.pwm_green, r.green_pin, config_green.clone());

    let mut led_on = true;

    // set initial color to off
    config_red.compare_a = PWM_MIN;
    config_green.compare_a = PWM_MIN;
    pwm_red.set_config(&config_red);
    pwm_green.set_config(&config_green);

    loop {
        // Wait for a change in system state
        indicator::wait().await;

        // Retrieve current battery level and operation mode
        let (battery_level, operation_mode) = {
            let state = SYSTEM_STATE.lock().await;
            (state.battery_level, state.operation_mode)
        };

        info!(
            "Battery level: {}%, operation mode: {:?}",
            battery_level, operation_mode
        );

        // Calculate PWM values based on battery level
        let green_pwm = (f32::from(battery_level) / 100.0 * f32::from(PWM_MAX)) as u16;
        let red_pwm = PWM_MAX - green_pwm;

        match operation_mode {
            OperationMode::Manual => {
                // In manual mode, keep LEDs continuously on
                config_red.compare_a = red_pwm;
                config_green.compare_a = green_pwm;
                pwm_red.set_config(&config_red);
                pwm_green.set_config(&config_green);
            }
            OperationMode::Autonomous => {
                // In autonomous mode, blink LEDs
                'blink: loop {
                    if led_on {
                        config_red.compare_a = red_pwm;
                        config_green.compare_a = green_pwm;
                    } else {
                        config_red.compare_a = PWM_MIN;
                        config_green.compare_a = PWM_MIN;
                    }
                    pwm_red.set_config(&config_red);
                    pwm_green.set_config(&config_green);

                    led_on = !led_on;

                    // Wait for either the blink interval to pass or a system state change
                    if let Either::Second(_) =
                        select(Timer::after(BLINK_INTERVAL), indicator::wait()).await
                    {
                        // If system state changed, propagate the change and break the blink loop
                        indicator::send(true);
                        break 'blink;
                    }
                }
            }
        }
    }
}
