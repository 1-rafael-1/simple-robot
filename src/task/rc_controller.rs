//! RC Controller module for handling button inputs and generating events.
//!
//! This module defines tasks for each button on the RC controller and
//! provides functionality to handle button presses and holds.
use crate::system::event;
use crate::system::resources::{RCResourcesA, RCResourcesB, RCResourcesC, RCResourcesD};
use embassy_futures::select::{select, Either};
use embassy_rp::gpio::{Input, Pull};
use embassy_time::{Duration, Timer};

/// The duration threshold for distinguishing between a button press and a button hold.
const HOLD_THRESHOLD: Duration = Duration::from_millis(200);

/// Task for handling button A on the RC controller.
#[embassy_executor::task]
pub async fn rc_button_a_handler(r: RCResourcesA) {
    let mut btn = Input::new(r.btn_a, Pull::Down);
    handle_button(&mut btn, event::ButtonId::A).await;
}

/// Task for handling button B on the RC controller.
#[embassy_executor::task]
pub async fn rc_button_b_handler(r: RCResourcesB) {
    let mut btn = Input::new(r.btn_b, Pull::Down);
    handle_button(&mut btn, event::ButtonId::B).await;
}

/// Task for handling button C on the RC controller.
#[embassy_executor::task]
pub async fn rc_button_c_handler(r: RCResourcesC) {
    let mut btn = Input::new(r.btn_c, Pull::Down);
    handle_button(&mut btn, event::ButtonId::C).await;
}

/// Task for handling button D on the RC controller.
#[embassy_executor::task]
pub async fn rc_button_d_handler(r: RCResourcesD) {
    let mut btn = Input::new(r.btn_d, Pull::Down);
    handle_button(&mut btn, event::ButtonId::D).await;
}

/// Handles button input and generates appropriate events.
///
/// This function runs in an infinite loop, continuously monitoring the button state.
/// It distinguishes between short presses and long holds, generating different events for each.
///
/// # Arguments
///
/// * `button` - A mutable reference to the Input representing the button
/// * `id` - The `ButtonId` associated with this button
///
/// # Events
///
/// * `ButtonPressed` - Sent when a short press is detected
/// * `ButtonHoldStart` - Sent when a long press is initiated
/// * `ButtonHoldEnd` - Sent when a long press ends
pub async fn handle_button(button: &mut Input<'static>, id: event::ButtonId) {
    loop {
        button.wait_for_high().await;

        match select(Timer::after(HOLD_THRESHOLD), button.wait_for_low()).await {
            Either::First(()) => {
                event::send(event::Events::ButtonPressed(id)).await;
            }
            Either::Second(()) => {
                event::send(event::Events::ButtonHoldStart(id)).await;
                button.wait_for_low().await;
                event::send(event::Events::ButtonHoldEnd(id)).await;
            }
        };
    }
}
