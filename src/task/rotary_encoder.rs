//! EC11 rotary encoder handling (PIO quadrature + button input)
//!
//! Produces events for:
//! - Rotary turn increments/decrements
//! - Button press / hold start / hold end
//!
//! Pin plan (per docs):
//! - GPIO22: Encoder A
//! - GPIO23: Encoder B
//! - GPIO24: Encoder button (active-low with pull-up)

use embassy_futures::select::{Either, select};
use embassy_rp::{
    gpio::{Input, Level},
    pio_programs::rotary_encoder::{Direction as PioDirection, PioEncoder},
};
use embassy_time::{Duration, Timer};

use crate::system::event::{Events, RotaryDirection, raise_event};

/// Button hold threshold (ms)
const HOLD_DURATION: Duration = Duration::from_millis(700);

/// Button debounce delay (ms)
const DEBOUNCE_DURATION: Duration = Duration::from_millis(30);

/// Task that reads quadrature turns and emits increment/decrement events.
#[embassy_executor::task]
pub async fn rotary_encoder_turns(mut encoder: PioEncoder<'static, embassy_rp::peripherals::PIO1, 3>) {
    loop {
        let dir = encoder.read().await;
        let event = match dir {
            PioDirection::Clockwise => Events::RotaryTurned(RotaryDirection::Clockwise),
            PioDirection::CounterClockwise => Events::RotaryTurned(RotaryDirection::CounterClockwise),
        };
        raise_event(event).await;
    }
}

/// Task that handles the EC11 button (active-low, pull-up enabled).
#[embassy_executor::task]
pub async fn rotary_encoder_button(mut button: Input<'static>) {
    loop {
        let level = debounce(&mut button).await;

        // Active-low: pressed when LOW.
        if level != Level::Low {
            continue;
        }

        match select(Timer::after(HOLD_DURATION), debounce(&mut button)).await {
            Either::First(()) => {
                raise_event(Events::RotaryButtonHoldStart).await;
                button.wait_for_high().await;
                raise_event(Events::RotaryButtonHoldEnd).await;
            }
            Either::Second(_) => {
                raise_event(Events::RotaryButtonPressed).await;
            }
        }
    }
}

/// Ensures stable button state after any edge.
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
