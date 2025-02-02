//! RC button handling
//!
//! Processes RC controller button inputs and generates events.

use defmt::info;
use embassy_futures::select::{select, Either};
use embassy_rp::gpio::{AnyPin, Input, Level, Pull};
use embassy_time::{Duration, Timer};

use crate::system::event::{send_event, ButtonId, Events};

/// Button hold threshold (ms)
const HOLD_DURATION: Duration = Duration::from_millis(700);

/// Button debounce delay (ms)
const DEBOUNCE_DURATION: Duration = Duration::from_millis(30);

/// Button handler task
#[embassy_executor::task(pool_size = 4)]
pub async fn rc_button_handle(pin: AnyPin, id: ButtonId) {
    let mut btn = Input::new(pin, Pull::Down);
    handle_button(&mut btn, id).await;
}

/// Processes button input and generates events
///
/// Generates:
/// - ButtonPressed for short press
/// - ButtonHoldStart/End for long press
async fn handle_button(button: &mut Input<'static>, id: ButtonId) {
    loop {
        let init_level = debounce(button).await;

        info!("btn {}", id);

        if init_level != Level::High {
            continue;
        };

        match select(Timer::after(HOLD_DURATION), debounce(button)).await {
            Either::First(()) => {
                send_event(Events::ButtonHoldStart(id)).await;
                button.wait_for_low().await;
                send_event(Events::ButtonHoldEnd(id)).await;
            }
            Either::Second(_) => {
                send_event(Events::ButtonPressed(id)).await;
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
