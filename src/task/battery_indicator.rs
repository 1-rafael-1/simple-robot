//! #battery_indicator
//!
//! Contains a task that updates the battery indicator LED according to the battery level, going from green to red color as the battery drains.

use crate::task::resources::{BatteryIndicatorResources, Irqs};
use defmt::info;
use embassy_rp::adc::{Adc, Channel, Config as AdcConfig};
use embassy_rp::gpio::Pull;
use embassy_rp::pwm::{Config, Pwm};
use embassy_time::{Duration, Timer};

const MEASUREMENT_INTERVAL: Duration = Duration::from_secs(60);
const PWM_MAX: u16 = 65535;
const PWM_MIN: u16 = 0;
const BATTERY_VOLTAGE_LOWER: f32 = 2.5;
const BATTERY_VOLTAGE_UPPER: f32 = 4.2;

/// Battery Indicator task that uses ADC to measure battery voltage and then controls the LED color accordingly using PWM,
/// changing from green for a full battery to red for an empty one.
#[embassy_executor::task]
pub async fn battery_indicator(r: BatteryIndicatorResources) {
    info!("Battery Indicator task started");

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

    // adc
    let mut adc = Adc::new(r.adc, Irqs, AdcConfig::default());
    let vsys_in = r.vsys_pin;
    let mut channel = Channel::new_pin(vsys_in, Pull::None);

    loop {
        // read the adc value
        let adc_value = adc.read(&mut channel).await.unwrap();
        // reference voltage is 3.3V, and the vsys pin voltage divider ratio is 3. The ADC is 12-bit, so 2^12 = 4096
        let voltage = (adc_value as f32) * 3.3 * 3.0 / 4096.0;

        let battery_level = if voltage >= BATTERY_VOLTAGE_UPPER {
            1.0
        } else if voltage <= BATTERY_VOLTAGE_LOWER {
            0.0
        } else {
            (voltage - BATTERY_VOLTAGE_LOWER) / (BATTERY_VOLTAGE_UPPER - BATTERY_VOLTAGE_LOWER)
        };

        // Update PWM configurations
        config_red.compare_a = ((1.0 - battery_level) * PWM_MAX as f32) as u16;
        config_green.compare_a = (battery_level * PWM_MAX as f32) as u16;

        // Apply new configurations
        pwm_red.set_config(&config_red);
        pwm_green.set_config(&config_green);

        info!(
            "Battery level: {}%, Red: {}, Green: {}",
            battery_level * 100.0,
            config_red.compare_a,
            config_green.compare_a,
        );

        Timer::after(MEASUREMENT_INTERVAL).await;
    }
}
