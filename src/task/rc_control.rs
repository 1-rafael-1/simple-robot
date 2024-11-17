//! RC button handling
//!
//! Processes RC controller button inputs and generates events.

use crate::system::event;
use crate::system::resources::{RCResourcesA, RCResourcesB, RCResourcesC, RCResourcesD};
use embassy_futures::select::{select, Either};
use embassy_rp::gpio::{Input, Level, Pull};
use embassy_time::{Duration, Timer};

/// Button hold threshold (ms)
const HOLD_DURATION: Duration = Duration::from_millis(700);

/// Button debounce delay (ms)
const DEBOUNCE_DURATION: Duration = Duration::from_millis(30);

/// Button A handler
#[embassy_executor::task]
pub async fn rc_button_a_handle(r: RCResourcesA) {
    let mut btn = Input::new(r.btn_a, Pull::Down);
    handle_button(&mut btn, event::ButtonId::A).await;
}

/// Button B handler
#[embassy_executor::task]
pub async fn rc_button_b_handle(r: RCResourcesB) {
    let mut btn = Input::new(r.btn_b, Pull::Down);
    handle_button(&mut btn, event::ButtonId::B).await;
}

/// Button C handler
#[embassy_executor::task]
pub async fn rc_button_c_handle(r: RCResourcesC) {
    let mut btn = Input::new(r.btn_c, Pull::Down);
    handle_button(&mut btn, event::ButtonId::C).await;
}

/// Button D handler
#[embassy_executor::task]
pub async fn rc_button_d_handle(r: RCResourcesD) {
    let mut btn = Input::new(r.btn_d, Pull::Down);
    handle_button(&mut btn, event::ButtonId::D).await;
}

/// Processes button input and generates events
///
/// Generates:
/// - ButtonPressed for short press
/// - ButtonHoldStart/End for long press
async fn handle_button(button: &mut Input<'static>, id: event::ButtonId) {
    loop {
        let init_level = debounce(button).await;

        if init_level != Level::High {
            continue;
        };

        match select(Timer::after(HOLD_DURATION), debounce(button)).await {
            Either::First(()) => {
                event::send(event::Events::ButtonHoldStart(id)).await;
                button.wait_for_low().await;
                event::send(event::Events::ButtonHoldEnd(id)).await;
            }
            Either::Second(_) => {
                event::send(event::Events::ButtonPressed(id)).await;
            }
        };
    }
}

/// Ensures stable button state
async fn debounce(button: &mut Input<'static>) -> Level {
    loop {
        let st_level = button.get_level();
        button.wait_for_any_edge().await;
        Timer::after(DEBOUNCE_DURATION).await;
        let end_level = button.get_level();
        if st_level != end_level {
            break end_level;
        }
    }
}
