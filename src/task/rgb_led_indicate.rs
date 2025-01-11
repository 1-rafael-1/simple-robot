//! RGB LED indication for system status
//!
//! Provides visual feedback about system state through an RGB LED:
//!
//! # Operation Modes
//! - Manual Mode: Solid color indicating battery level
//!   - Green: Full battery (100%)
//!   - Red: Low battery (0%)
//!   - Smooth transition between colors as battery drains
//!
//! - Autonomous Mode: Blinking color indicating battery level
//!   - Same color scheme as manual mode
//!   - 700ms blink interval for good visibility
//!
//! # State Change Indication
//! - Quick alternating red/green blink sequence (5 blinks)
//! - 30ms interval for noticeable but brief feedback
//!
//! # PWM Control
//! - 100Hz PWM frequency for flicker-free operation
//! - Independent control of red and green channels
//! - Duty cycle proportional to battery level
//! - Smooth transitions between states

use embassy_futures::select::{select, Either};
use embassy_rp::{pwm, pwm::SetDutyCycle};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};

use crate::system::{
    resources::RGBLedResources,
    state::{OperationMode, SYSTEM_STATE},
};

/// Signal for triggering LED state updates
pub static INDICATOR_CHANGED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// Triggers an LED indicator state update
///
/// - affirm: true to show state change confirmation blink sequence
pub fn update_indicator(affirm: bool) {
    INDICATOR_CHANGED.signal(affirm);
}

/// Waits for next indicator state change signal
async fn wait_indicator() -> bool {
    INDICATOR_CHANGED.wait().await
}

/// Interval for autonomous mode blinking (700ms provides good visibility
/// while not being too distracting)
const MODE_BLINK_INTERVAL: Duration = Duration::from_millis(700);

/// Interval for state change confirmation blinks (30ms is quick but noticeable)
const AFFIRM_BLINK_INTERVAL: Duration = Duration::from_millis(30);

/// Main LED control task that manages visual feedback based on system state
///
/// Uses PWM to control LED brightness and color:
/// - Red channel indicates low battery
/// - Green channel indicates high battery
/// - Mixing creates intermediate colors
/// - Blinking patterns indicate operation mode
#[embassy_executor::task]
pub async fn rgb_led_indicate(r: RGBLedResources) {
    // Configure PWM for 100Hz operation (good balance of resolution and response)
    let desired_freq_hz = 100;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();

    // Calculate divider for 16-bit PWM resolution
    let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    // Initialize red LED PWM
    let mut config_red = pwm::Config::default();
    config_red.divider = divider.into();
    config_red.top = period;
    let mut pwm_red = pwm::Pwm::new_output_a(r.pwm_red, r.red_pin, config_red.clone());

    // Initialize green LED PWM
    let mut config_green = pwm::Config::default();
    config_green.divider = divider.into();
    config_green.top = period;
    let mut pwm_green = pwm::Pwm::new_output_a(r.pwm_green, r.green_pin, config_green.clone());

    // Start with LED off
    let mut led_on = false;
    let _ = pwm_red.set_duty_cycle_fully_off();
    let _ = pwm_green.set_duty_cycle_fully_off();

    loop {
        let affirm = wait_indicator().await;

        // Show state change confirmation sequence if requested
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

        // Get current system state
        let (battery_level, operation_mode) = {
            let state = SYSTEM_STATE.lock().await;
            (state.battery_level, state.operation_mode)
        };

        // Calculate PWM duty cycles based on battery level
        // Green is typically brighter, so we reduce its intensity to .4
        let (green_pwm, red_pwm) = if battery_level >= 50 {
            // Upper half: Green fades to yellow
            let blend_factor = (battery_level - 50) * 2; // Scale 50-100 to 0-100
            let green = ((100 as f32) * 0.4) as u8; // Reduce green intensity
            let red = ((100 - blend_factor) as f32) as u8; // Keep red at full
            (green, red)
        } else {
            // Lower half: Yellow fades to red
            let blend_factor = battery_level * 2; // Scale 0-50 to 0-100
            let green = ((blend_factor as f32) * 0.4) as u8; // Reduce green intensity
            let red = 100; // Keep red at full
            (green, red)
        };

        match operation_mode {
            OperationMode::Manual => {
                // Solid color indicating battery level
                let _ = pwm_green.set_duty_cycle_percent(green_pwm);
                let _ = pwm_red.set_duty_cycle_percent(red_pwm);
            }
            OperationMode::Autonomous => {
                // Blink pattern indicating autonomous operation
                'autonomous_blink: loop {
                    if led_on {
                        let _ = pwm_green.set_duty_cycle_percent(green_pwm);
                        let _ = pwm_red.set_duty_cycle_percent(red_pwm);
                    } else {
                        let _ = pwm_green.set_duty_cycle_fully_off();
                        let _ = pwm_red.set_duty_cycle_fully_off();
                    }

                    led_on = !led_on;

                    // Break blink loop if new indicator update received
                    if let Either::Second(_) = select(Timer::after(MODE_BLINK_INTERVAL), wait_indicator()).await {
                        update_indicator(true);
                        break 'autonomous_blink;
                    }
                }
            }
        }
    }
}
