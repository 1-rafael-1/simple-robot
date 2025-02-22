//! Battery charge monitoring
//!
//! Monitors Li-Ion battery voltage and calculates charge level percentage.
//!
//! # Measurement Strategy
//! - Reads voltage through ADC every 20 seconds
//! - Uses voltage divider (3:1 ratio) to scale battery voltage to ADC range
//! - Applies median filtering over 9 samples to reduce noise
//! - Initial 500ms delay ensures system stabilization
//!
//! # Voltage Calculations
//! ```text
//! Battery Voltage = (ADC Value * 3.3V * 3.0) / 4096
//! Where:
//! - 3.3V is ADC reference voltage
//! - 3.0 is voltage divider ratio
//! - 4096 is ADC resolution (12-bit)
//! ```
//!
//! # Charge Level Calculation
//! - Maps voltage range 2.5V-4.2V to 0%-99%
//! - Linear interpolation between min/max voltages
//! - Caps at 99% to indicate charging might still occur
//! - Reports 0% at or below 2.5V (warning level)
//!
//! # ADC Management
//! - Uses mutex-protected global ADC instance
//! - Lock acquired only during reading
//! - Quick release ensures other tasks can access ADC

use embassy_rp::{adc::Channel, gpio::Pull};
use embassy_time::{Duration, Timer};
use moving_median::MovingMedian;

use crate::system::{
    event,
    resources::{BatteryChargeResources, get_adc},
};

/// Time between voltage measurements (20s provides good balance of
/// responsiveness and power efficiency)
const MEASUREMENT_INTERVAL: Duration = Duration::from_secs(20);

/// Minimum battery voltage (2.5V indicates battery needs charging)
const BATTERY_VOLTAGE_LOWER: f32 = 2.5;

/// Maximum battery voltage (4.2V is typical Li-Ion full charge)
const BATTERY_VOLTAGE_UPPER: f32 = 4.2;

/// ADC reference voltage (3.3V is RP2040's reference)
const REF_VOLTAGE: f32 = 3.3;

/// Hardware voltage divider ratio (3:1 scales battery voltage to ADC range)
const V_DIVIDER_RATIO: f32 = 3.0;

/// ADC resolution (12-bit = 4096 steps)
const ADC_RANGE: f32 = 4096.0;

/// Median filter window (9 samples provides good noise reduction while
/// maintaining reasonable memory usage)
const MEDIAN_WINDOW_SIZE: usize = 9;

/// Battery monitoring task that continuously measures voltage and
/// reports charge level as a percentage
#[embassy_executor::task]
pub async fn battery_charge_read(r: BatteryChargeResources) {
    let vsys_in = r.vsys_pin;
    let mut channel = Channel::new_pin(vsys_in, Pull::None);

    // Setup median filter for smoothing voltage readings
    let mut median_filter = MovingMedian::<f32, MEDIAN_WINDOW_SIZE>::new();

    // Initial delay to ensure system stabilization before first reading
    Timer::after(Duration::from_millis(500)).await;

    loop {
        // Read voltage with ADC lock management:
        // 1. Acquire ADC lock (automatically released when scope ends)
        // 2. Perform ADC reading
        // 3. Convert raw ADC value to actual voltage
        let voltage = {
            // SAFETY: ADC lock is automatically released when this scope ends,
            // ensuring other tasks can access the ADC when needed
            let mut adc_guard = get_adc().lock().await;
            let adc = adc_guard.as_mut().unwrap();

            // Read ADC value and convert to voltage
            // Formula: (adc_value * reference_voltage * voltage_divider_ratio) / adc_resolution
            f32::from(adc.read(&mut channel).await.unwrap_or(0)) * REF_VOLTAGE * V_DIVIDER_RATIO / ADC_RANGE
            // Lock is automatically dropped here when scope ends
        };

        // Apply median filtering to reduce noise in readings
        median_filter.add_value(voltage);
        let filtered_voltage = median_filter.median();

        // Calculate battery charge percentage:
        // - 99% if voltage >= max voltage (avoiding 100% to indicate charging might still occur)
        // - 0% if voltage <= min voltage
        // - Linear interpolation between min and max otherwise
        let battery_level = if filtered_voltage >= BATTERY_VOLTAGE_UPPER {
            0.99
        } else if filtered_voltage <= BATTERY_VOLTAGE_LOWER {
            0.0
        } else {
            (filtered_voltage - BATTERY_VOLTAGE_LOWER) / (BATTERY_VOLTAGE_UPPER - BATTERY_VOLTAGE_LOWER)
        };

        // Send battery level event (as percentage)
        event::send_event(event::Events::BatteryLevelMeasured((battery_level * 100.0) as u8)).await;

        // Wait for next measurement interval
        Timer::after(MEASUREMENT_INTERVAL).await;
    }
}
