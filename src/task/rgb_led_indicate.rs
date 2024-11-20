//! RGB LED indication for system status
//!
//! - Manual mode: Solid color (green to red) based on battery level
//! - Autonomous mode: Blinking color based on battery level
//! - State changes: Quick alternating blink sequence

use crate::system::resources::RGBLedResources;
use crate::system::state::{OperationMode, SYSTEM_STATE};
use embassy_futures::select::select;
use embassy_futures::select::Either;
use embassy_rp::pwm;
use embassy_rp::pwm::SetDutyCycle;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};

/// Indicator state signal for LED control
pub static INDICATOR_CHANGED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// Triggers an LED indicator state update
pub fn update_indicator(affirm: bool) {
    INDICATOR_CHANGED.signal(affirm);
}

/// Waits for next indicator state change
async fn wait_indicator() -> bool {
    INDICATOR_CHANGED.wait().await
}

/// Blink interval for autonomous mode (ms)
const MODE_BLINK_INTERVAL: Duration = Duration::from_millis(700);

/// Blink interval for state changes (ms)
const AFFIRM_BLINK_INTERVAL: Duration = Duration::from_millis(30);

/// Controls RGB LED based on system state and operation mode
#[embassy_executor::task]
pub async fn rgb_led_indicate(r: RGBLedResources) {
    // PWM config (100Hz)
    let desired_freq_hz = 100;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();

    // Calculate divider for 16-bit limit
    let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    // Red LED PWM
    let mut config_red = pwm::Config::default();
    config_red.divider = divider.into();
    config_red.top = period;
    let mut pwm_red = pwm::Pwm::new_output_a(r.pwm_red, r.red_pin, config_red.clone());

    // Green LED PWM
    let mut config_green = pwm::Config::default();
    config_green.divider = divider.into();
    config_green.top = period;
    let mut pwm_green = pwm::Pwm::new_output_a(r.pwm_green, r.green_pin, config_green.clone());

    // Initial state
    let mut led_on = false;
    let _ = pwm_red.set_duty_cycle_fully_off();
    let _ = pwm_green.set_duty_cycle_fully_off();

    loop {
        let affirm = wait_indicator().await;

        // Quick blink to confirm state change
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

        // Get current state
        let (battery_level, operation_mode) = {
            let state = SYSTEM_STATE.lock().await;
            (state.battery_level, state.operation_mode)
        };

        // Set LED colors based on battery
        let green_pwm = battery_level.clamp(0, 100);
        let red_pwm = (100u8 - battery_level).clamp(0, 100);

        match operation_mode {
            OperationMode::Manual => {
                // Solid color in manual mode
                let _ = pwm_red.set_duty_cycle_percent(red_pwm);
                let _ = pwm_green.set_duty_cycle_percent(green_pwm);
            }
            OperationMode::Autonomous => {
                // Continuous blink in autonomous mode
                'autonomous_blink: loop {
                    if led_on {
                        let _ = pwm_red.set_duty_cycle_percent(red_pwm);
                        let _ = pwm_green.set_duty_cycle_percent(green_pwm);
                    } else {
                        let _ = pwm_red.set_duty_cycle_fully_off();
                        let _ = pwm_green.set_duty_cycle_fully_off();
                    }

                    led_on = !led_on;

                    // In case enother indicator update happens, break
                    if let Either::Second(_) =
                        select(Timer::after(MODE_BLINK_INTERVAL), wait_indicator()).await
                    {
                        update_indicator(true);
                        break 'autonomous_blink;
                    }
                }
            }
        }
    }
}
