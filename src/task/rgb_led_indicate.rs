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
use embassy_rp::pwm;
use embassy_rp::pwm::SetDutyCycle;
use embassy_time::{Duration, Timer};

/// Interval for LED blinking in autonomous mode
const MODE_BLINK_INTERVAL: Duration = Duration::from_millis(700);

/// Interval for LED blinking when affirming state change
const AFFIRM_BLINK_INTERVAL: Duration = Duration::from_millis(30);

/// Controls the RGB LED indicator based on system state.
///
/// This task manages the RGB LED, adjusting its color and behavior to reflect:
/// - Battery level: Green (full) to Red (empty)
/// - Operation mode: Solid (Manual) or Blinking (Autonomous)
#[embassy_executor::task]
pub async fn rgb_led_indicate(r: RGBLedResources) {
    // configure pwm for rgb led, 100Hz should suffice
    // configure pwm for rgb led, 100Hz
    let desired_freq_hz = 100;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq(); // 150MHz

    // Calculate minimum divider needed to keep period under 16-bit limit (65535)s
    let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    // Configure red LED PWM
    let mut config_red = pwm::Config::default();
    config_red.divider = divider.into();
    config_red.top = period;
    let mut pwm_red = pwm::Pwm::new_output_a(r.pwm_red, r.red_pin, config_red.clone());

    // Configure green LED PWM
    let mut config_green = pwm::Config::default();
    config_green.divider = divider.into();
    config_green.top = period;
    let mut pwm_green = pwm::Pwm::new_output_a(r.pwm_green, r.green_pin, config_green.clone());

    // set initial color to off
    let mut led_on = false;
    let _ = pwm_red.set_duty_cycle_fully_off();
    let _ = pwm_green.set_duty_cycle_fully_off();

    loop {
        // Wait for a change in system state
        let affirm = indicator::wait().await;

        // affirm a change in the indicator by blinking the LED
        if affirm {
            for _ in 0..5 {
                if led_on {
                    let _ = pwm_red.set_duty_cycle_fully_off();
                    let _ = pwm_green.set_duty_cycle_fully_on();
                } else {
                    let _ = pwm_red.set_duty_cycle_fully_on();
                    let _ = pwm_green.set_duty_cycle_fully_off();
                }
                led_on = !led_on;
                Timer::after(AFFIRM_BLINK_INTERVAL).await;
            }
            led_on = false;
        }

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
        let green_pwm = battery_level.clamp(0, 100);
        let red_pwm = (100u8 - battery_level).clamp(0, 100);

        match operation_mode {
            OperationMode::Manual => {
                // In manual mode, keep LEDs continuously on
                let _ = pwm_red.set_duty_cycle_percent(red_pwm);
                let _ = pwm_green.set_duty_cycle_percent(green_pwm);
            }
            OperationMode::Autonomous => {
                // In autonomous mode, blink LEDs
                'blink: loop {
                    if led_on {
                        let _ = pwm_red.set_duty_cycle_percent(red_pwm);
                        let _ = pwm_green.set_duty_cycle_percent(green_pwm);
                    } else {
                        let _ = pwm_red.set_duty_cycle_fully_off();
                        let _ = pwm_green.set_duty_cycle_fully_off();
                    }

                    led_on = !led_on;

                    // Wait for either the blink interval to pass or a system state change
                    if let Either::Second(_) =
                        select(Timer::after(MODE_BLINK_INTERVAL), indicator::wait()).await
                    {
                        // If system state changed, propagate the change and break the blink loop
                        indicator::update(true);
                        break 'blink;
                    }
                }
            }
        }
    }
}
