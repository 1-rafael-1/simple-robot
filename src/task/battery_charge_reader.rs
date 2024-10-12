use crate::task::resources::{BatteryChargeResources, Irqs};
use crate::task::system_events::{send_event, Events};
use embassy_rp::adc::{Adc, Channel, Config as AdcConfig};
use embassy_rp::gpio::Pull;
use embassy_time::{Duration, Timer};

const MEASUREMENT_INTERVAL: Duration = Duration::from_secs(60);
const BATTERY_VOLTAGE_LOWER: f32 = 2.5;
const BATTERY_VOLTAGE_UPPER: f32 = 4.2;
const REF_VOLTAGE: f32 = 3.3; // V
const V_DIVIDER_RATIO: f32 = 3.0; // V/V
const ADC_RANGE: f32 = 4096.0; // ADC resolution is 12-bit

#[embassy_executor::task]
pub async fn battery_charge_reader(r: BatteryChargeResources) {
    // adc
    let mut adc = Adc::new(r.adc, Irqs, AdcConfig::default());
    let vsys_in = r.vsys_pin;
    let mut channel = Channel::new_pin(vsys_in, Pull::None);

    loop {
        let voltage =
            (adc.read(&mut channel).await.unwrap_or(0) as f32) * REF_VOLTAGE * V_DIVIDER_RATIO
                / ADC_RANGE;

        let battery_level = if voltage >= BATTERY_VOLTAGE_UPPER {
            1.0
        } else if voltage <= BATTERY_VOLTAGE_LOWER {
            0.0
        } else {
            (voltage - BATTERY_VOLTAGE_LOWER) / (BATTERY_VOLTAGE_UPPER - BATTERY_VOLTAGE_LOWER)
        };

        send_event(Events::BatteryLevelMeasured((battery_level * 100.0) as u8)).await;

        Timer::after(MEASUREMENT_INTERVAL).await;
    }
}
