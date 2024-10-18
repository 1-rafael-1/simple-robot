//! RC Controller module for handling button inputs and generating events.
//!
//! This module defines tasks for each button on the RC controller and
//! provides functionality to handle button presses and holds.
use crate::system::event;
use crate::system::resources::{RCResourcesA, RCResourcesB, RCResourcesC, RCResourcesD};
use defmt::info;
use embassy_futures::select::{select, Either};
use embassy_rp::gpio::{Input, Level, Pull};
use embassy_time::{Duration, Timer};

/// The duration threshold for distinguishing between a button press and a button hold.
const HOLD_DURATION: Duration = Duration::from_millis(700);

/// The debounce duration used to debounce buttons.
const DEBOUNCE_DURATION: Duration = Duration::from_millis(30);

/// Task for handling button A on the RC controller.
///
/// This task initializes the GPIO for button A and continuously monitors its state.
#[embassy_executor::task]
pub async fn rc_button_a_handler(r: RCResourcesA) {
    let mut btn = Input::new(r.btn_a, Pull::Down);
    handle_button(&mut btn, event::ButtonId::A).await;
}

/// Task for handling button B on the RC controller.
///
/// This task initializes the GPIO for button B and continuously monitors its state.
#[embassy_executor::task]
pub async fn rc_button_b_handler(r: RCResourcesB) {
    let mut btn = Input::new(r.btn_b, Pull::Down);
    handle_button(&mut btn, event::ButtonId::B).await;
}

/// Task for handling button C on the RC controller.
///
/// This task initializes the GPIO for button C and continuously monitors its state.
#[embassy_executor::task]
pub async fn rc_button_c_handler(r: RCResourcesC) {
    let mut btn = Input::new(r.btn_c, Pull::Down);
    handle_button(&mut btn, event::ButtonId::C).await;
}

/// Task for handling button D on the RC controller.
///
/// This task initializes the GPIO for button D and continuously monitors its state.
#[embassy_executor::task]
pub async fn rc_button_d_handler(r: RCResourcesD) {
    let mut btn = Input::new(r.btn_d, Pull::Down);
    handle_button(&mut btn, event::ButtonId::D).await;
}

/// Handles button input and generates appropriate events.
///
/// This function runs in an infinite loop, continuously monitoring the button state.
/// It distinguishes between short presses and long holds, generating different events for each.
/// The function uses debouncing to ensure reliable button detection.
///
/// # Arguments
///
/// * `button` - A mutable reference to the Input representing the button
/// * `id` - The `ButtonId` associated with this button
///
/// # Events
///
/// * `ButtonPressed` - Sent when a short press is detected
/// * `ButtonHoldStart` - Sent when a long press is initiated (after `HOLD_THRESHOLD`)
/// * `ButtonHoldEnd` - Sent when a long press ends
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

/// Debounces the button input to prevent false triggers due to noise.
///
/// This function waits for a stable button state by checking the button level
/// before and after a short delay. It returns only when a stable state is detected.
///
/// # Arguments
///
/// * `button` - A mutable reference to the Input representing the button
///
/// # Returns
///
/// The stable `Level` of the button after debouncing
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
