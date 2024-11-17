//! Battery charge monitoring
//!
//! Reads battery voltage and calculates charge level.

use crate::system::event;
use crate::system::resources::{BatteryChargeResources, Irqs};
use embassy_rp::adc::{Adc, Channel, Config as AdcConfig};
use embassy_rp::gpio::Pull;
use embassy_time::{Duration, Timer};
use moving_median::MovingMedian;

/// Measurement interval (s)
const MEASUREMENT_INTERVAL: Duration = Duration::from_secs(20);

/// Battery min voltage (V)
const BATTERY_VOLTAGE_LOWER: f32 = 2.5;

/// Battery max voltage (V)
const BATTERY_VOLTAGE_UPPER: f32 = 4.2;

/// ADC reference voltage (V)
const REF_VOLTAGE: f32 = 3.3;

/// Voltage divider ratio
const V_DIVIDER_RATIO: f32 = 3.0;

/// ADC resolution (12-bit)
const ADC_RANGE: f32 = 4096.0;

/// Median filter window size
const MEDIAN_WINDOW_SIZE: usize = 9;

/// Battery charge monitoring task
#[embassy_executor::task]
pub async fn battery_charge_read(r: BatteryChargeResources) {
    // Initialize ADC
    let mut adc = Adc::new(r.adc, Irqs, AdcConfig::default());
    let vsys_in = r.vsys_pin;
    let mut channel = Channel::new_pin(vsys_in, Pull::None);

    // Setup median filter
    let mut median_filter = MovingMedian::<f32, MEDIAN_WINDOW_SIZE>::new();

    // Initial delay for stable readings
    Timer::after(Duration::from_millis(500)).await;

    loop {
        // Read ADC and convert to voltage
        let mut voltage =
            f32::from(adc.read(&mut channel).await.unwrap_or(0)) * REF_VOLTAGE * V_DIVIDER_RATIO
                / ADC_RANGE;

        median_filter.add_value(voltage);
        voltage = median_filter.median();

        // Calculate charge percentage
        let battery_level = if voltage >= BATTERY_VOLTAGE_UPPER {
            0.99
        } else if voltage <= BATTERY_VOLTAGE_LOWER {
            0.0
        } else {
            (voltage - BATTERY_VOLTAGE_LOWER) / (BATTERY_VOLTAGE_UPPER - BATTERY_VOLTAGE_LOWER)
        };

        event::send(event::Events::BatteryLevelMeasured(
            (battery_level * 100.0) as u8,
        ))
        .await;

        Timer::after(MEASUREMENT_INTERVAL).await;
    }
}
