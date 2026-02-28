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
//! - Independent control of red, green, and blue channels via PIO PWM
//! - Duty cycle proportional to battery level
//! - Smooth transitions between states

use core::time::Duration as CoreDuration;

use embassy_futures::select::{Either, select};
use embassy_rp::{peripherals::PIO1, pio_programs::pwm::PioPwm};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};

use crate::system::state::{OperationMode, SYSTEM_STATE};

/// Signal for triggering LED state updates
pub static INDICATOR_CHANGED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// Triggers an LED indicator state update
///
/// - affirm: true to show state change confirmation blink sequence
pub fn update_indicator(affirm: bool) {
    INDICATOR_CHANGED.signal(affirm);
}

/// Waits for next indicator state change signal
async fn wait() -> bool {
    INDICATOR_CHANGED.wait().await
}

/// Interval for autonomous mode blinking (700ms provides good visibility
/// while not being too distracting)
const MODE_BLINK_INTERVAL: Duration = Duration::from_millis(700);

/// Interval for state change confirmation blinks (30ms is quick but noticeable)
const AFFIRM_BLINK_INTERVAL: Duration = Duration::from_millis(30);

/// PWM period for the PIO-driven RGB LED (100 Hz).
const PWM_PERIOD: CoreDuration = CoreDuration::from_micros(10_000);

/// Sets a PIO PWM channel to a duty cycle percentage.
#[allow(clippy::cast_possible_truncation)]
fn set_pwm_percent<const SM: usize>(pwm: &mut PioPwm<'static, PIO1, SM>, percent: u8) {
    let clamped = u128::from(percent.min(100));
    let high_us = PWM_PERIOD.as_micros() * clamped / 100;
    let high = CoreDuration::from_micros(high_us as u64);
    pwm.write(high);
}

/// Sets all RGB channels in one call.
fn set_rgb(
    pwm_red: &mut PioPwm<'static, PIO1, 0>,
    pwm_green: &mut PioPwm<'static, PIO1, 1>,
    pwm_blue: &mut PioPwm<'static, PIO1, 2>,
    red: u8,
    green: u8,
    blue: u8,
) {
    set_pwm_percent(pwm_red, red);
    set_pwm_percent(pwm_green, green);
    set_pwm_percent(pwm_blue, blue);
}

/// Main LED control task that manages visual feedback based on system state
///
/// Uses PIO PWM to control LED brightness and color:
/// - Red channel indicates low battery
/// - Green channel indicates high battery
/// - Blue channel reserved for future status modes
/// - Mixing creates intermediate colors
/// - Blinking patterns indicate operation mode
#[allow(clippy::cast_lossless, clippy::cast_sign_loss, clippy::cast_possible_truncation)]
#[embassy_executor::task]
pub async fn rgb_led_indicate(
    mut pwm_red: PioPwm<'static, PIO1, 0>,
    mut pwm_green: PioPwm<'static, PIO1, 1>,
    mut pwm_blue: PioPwm<'static, PIO1, 2>,
) {
    pwm_red.set_period(PWM_PERIOD);
    pwm_green.set_period(PWM_PERIOD);
    pwm_blue.set_period(PWM_PERIOD);

    pwm_red.start();
    pwm_green.start();
    pwm_blue.start();

    // Start with LED off
    let mut led_on = false;
    set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 0, 0, 0);

    loop {
        let affirm = wait().await;

        // Show state change confirmation sequence if requested
        if affirm {
            for _ in 0..5 {
                if led_on {
                    set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 0, 100, 0);
                } else {
                    set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 100, 0, 0);
                }
                led_on = !led_on;
                Timer::after(AFFIRM_BLINK_INTERVAL).await;
            }
            led_on = false;
            set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 0, 0, 0);
        }

        // Get current system state
        let (battery_level, operation_mode) = {
            let state = SYSTEM_STATE.lock().await;
            (state.battery_level, state.operation_mode)
        };

        // Calculate PWM duty cycles based on battery level
        if battery_level.is_none() {
            // No battery level info: solid blue
            set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 0, 0, 100);
            continue;
        }

        let batt_lvl = battery_level.unwrap_or_default();

        let (green_pwm, red_pwm) = if batt_lvl >= 50 {
            // Upper half: Green fades to yellow
            let blend_factor = (batt_lvl - 50) * 2; // Scale 50-100 to 0-100
            let green = 100;
            let red = ((100 - blend_factor) as f32) as u8; // Keep red at full
            (green, red)
        } else {
            // Lower half: Yellow fades to red
            let blend_factor = batt_lvl * 2; // Scale 0-50 to 0-100
            let green = blend_factor; // Green fades out
            let red = 100; // Keep red at full
            (green, red)
        };

        match operation_mode {
            OperationMode::Manual => {
                // Solid color indicating battery level
                set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, red_pwm, green_pwm, 0);
            }
            OperationMode::Autonomous => {
                // Blink pattern indicating autonomous operation
                'autonomous_blink: loop {
                    if led_on {
                        set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, red_pwm, green_pwm, 0);
                    } else {
                        set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 0, 0, 0);
                    }

                    led_on = !led_on;

                    // Break blink loop if new indicator update received
                    if let Either::Second(_) = select(Timer::after(MODE_BLINK_INTERVAL), wait()).await {
                        update_indicator(true);
                        break 'autonomous_blink;
                    }
                }
            }
        }
    }
}
