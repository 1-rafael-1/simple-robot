//! Battery Charge Reader Module
//!
//! This module is responsible for periodically reading the battery voltage
//! and calculating the battery charge level.

use crate::system::event;
use crate::system::resources::{BatteryChargeResources, Irqs};
use embassy_rp::adc::{Adc, Channel, Config as AdcConfig};
use embassy_rp::gpio::Pull;
use embassy_time::{Duration, Timer};
use moving_median::MovingMedian;

/// Interval between battery measurements
const MEASUREMENT_INTERVAL: Duration = Duration::from_secs(20);

/// Lower voltage threshold for battery (0% charge)
const BATTERY_VOLTAGE_LOWER: f32 = 2.5;

/// Upper voltage threshold for battery (100% charge)
const BATTERY_VOLTAGE_UPPER: f32 = 4.2;

/// Reference voltage for ADC
const REF_VOLTAGE: f32 = 3.3;

/// Voltage divider ratio used in the circuit
const V_DIVIDER_RATIO: f32 = 3.0;

/// ADC resolution (12-bit)
const ADC_RANGE: f32 = 4096.0;

/// Size of the moving median window for filtering measurements
const MEDIAN_WINDOW_SIZE: usize = 9;

/// Task for reading battery charge
///
/// This task periodically reads the battery voltage using an ADC,
/// calculates the battery charge level, and sends an event with the result.
#[embassy_executor::task]
pub async fn battery_charge_read(r: BatteryChargeResources) {
    // Initialize ADC
    let mut adc = Adc::new(r.adc, Irqs, AdcConfig::default());
    let vsys_in = r.vsys_pin;
    let mut channel = Channel::new_pin(vsys_in, Pull::None);

    // Initialize moving median filter for smoothing out the results
    let mut median_filter = MovingMedian::<f32, MEDIAN_WINDOW_SIZE>::new();

    // delay for a while before starting, measurements on start up are not reliable
    Timer::after(Duration::from_millis(500)).await;

    loop {
        // Read ADC and convert to voltage
        let mut voltage =
            f32::from(adc.read(&mut channel).await.unwrap_or(0)) * REF_VOLTAGE * V_DIVIDER_RATIO
                / ADC_RANGE;

        median_filter.add_value(voltage);
        voltage = median_filter.median();

        // Calculate battery level as a percentage
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
