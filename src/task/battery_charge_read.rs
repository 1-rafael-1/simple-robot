//! Battery charge monitoring
//!
//! Monitors 2S Li-Ion battery pack voltage (two 18650 cells in series).
//!
//! # Battery Specifications
//! - Configuration: 2S (two cells in series)
//! - Voltage range: 8.4V (fully charged) to 6.0V (cutoff)
//! - Cell chemistry: Li-Ion 18650
//!
//! # Measurement Strategy
//! - Reads voltage through ADC every 5 seconds (faster for motor control responsiveness)
//! - Uses voltage divider to scale battery voltage to ADC range
//! - Initial 500ms delay ensures system stabilization
//!
//! # Hardware Configuration
//! ```text
//! Battery+ ----[R1=20kΩ]----+----[R2=10kΩ]---- GND
//!                           |
//!                     GPIO26 (ADC0)
//!
//! Voltage Divider Ratio: R2/(R1+R2) = 10k/(20k+10k) = 0.333
//! Source Impedance: R1||R2 = ~6.7kΩ (suitable for ADC)
//! At 8.4V battery: ADC sees 2.8V (within safe range)
//! At 6.0V battery: ADC sees 2.0V
//! Current draw: ~280µA at 8.4V
//! ```
//!
//! # Pin Connection
//! - ADC Pin: GPIO26 (ADC0)
//! - Voltage Source: Battery pack before voltage regulator (6V-8.4V range)
//! - The voltage divider scales battery voltage to ADC input range (< 3.3V)
//!
//! # Voltage Calculations
//! ```text
//! Battery Voltage = (ADC Value * 3.3V) / (4096 * 0.333)
//! Where:
//! - 3.3V is ADC reference voltage
//! - 0.333 is voltage divider ratio (R2/(R1+R2))
//! - 4096 is ADC resolution (12-bit)
//! ```
//!
//! # Charge Level Calculation
//! - Maps voltage range 6.0V-8.4V to 0%-99%
//! - Linear interpolation between min/max voltages
//! - Caps at 99% to indicate charging might still occur
//! - Reports 0% at or below 6.0V (warning level)
//!
//! # Motor Driver Integration
//! - Battery voltage is shared with motor driver for voltage compensation
//! - Motor driver uses this to maintain consistent 6V output as battery drains

use embassy_rp::adc::{Adc, Async as AdcAsync, Channel};
use embassy_time::{Duration, Timer};

use crate::system::event;

/// Time between voltage measurements (5s provides fast response for motor control)
const MEASUREMENT_INTERVAL: Duration = Duration::from_secs(5);

/// Minimum battery voltage for 2S Li-Ion pack (6.0V is safe cutoff)
const BATTERY_VOLTAGE_LOWER: f32 = 6.0;

/// Maximum battery voltage for 2S Li-Ion pack (8.4V is full charge, 4.2V per cell)
const BATTERY_VOLTAGE_UPPER: f32 = 8.4;

/// ADC reference voltage (3.3V is RP2040's reference)
const REF_VOLTAGE: f32 = 3.3;

/// Hardware voltage divider ratio
/// R1 = 20kΩ (high side), R2 = 10kΩ (low side to ADC)
/// Ratio = R2/(R1+R2) = 10/(20+10) = 0.333
/// Source impedance: ~6.7kΩ (suitable for RP2350 ADC)
const V_DIVIDER_RATIO: f32 = 0.333;

/// ADC resolution (12-bit = 4096 steps)
const ADC_RANGE: f32 = 4096.0;

/// Battery monitoring task that continuously measures voltage and
/// reports charge level as a percentage
#[embassy_executor::task]
pub async fn battery_charge_read(mut adc: Adc<'static, AdcAsync>, mut channel: Channel<'static>) {
    // Initial delay to ensure system stabilization before first reading
    Timer::after(Duration::from_millis(500)).await;

    loop {
        // Read ADC value and convert to battery voltage
        // Formula: (adc_value * reference_voltage) / (adc_resolution * voltage_divider_ratio)
        // The voltage divider brings battery voltage down, so we divide by the ratio to get actual voltage
        let adc_raw = adc.read(&mut channel).await.unwrap_or(0);
        let voltage = f32::from(adc_raw) * REF_VOLTAGE / (ADC_RANGE * V_DIVIDER_RATIO);

        // Calculate battery charge percentage:
        // - 99% if voltage >= max voltage (avoiding 100% to indicate charging might still occur)
        // - 0% if voltage <= min voltage
        // - Linear interpolation between min and max otherwise
        let battery_level = if voltage >= BATTERY_VOLTAGE_UPPER {
            0.99
        } else if voltage <= BATTERY_VOLTAGE_LOWER {
            0.0
        } else {
            (voltage - BATTERY_VOLTAGE_LOWER) / (BATTERY_VOLTAGE_UPPER - BATTERY_VOLTAGE_LOWER)
        };

        defmt::debug!(
            "Battery: ADC raw={}, voltage={}V, level={}%",
            adc_raw,
            voltage,
            (battery_level * 100.0) as u8
        );

        // Send consolidated battery measurement event (single event instead of two)
        // Battery monitoring is critical for safety - must not drop events
        event::send_event(event::Events::BatteryMeasured {
            level: (battery_level * 100.0) as u8,
            voltage,
        })
        .await;

        // Wait for next measurement interval
        Timer::after(MEASUREMENT_INTERVAL).await;
    }
}
