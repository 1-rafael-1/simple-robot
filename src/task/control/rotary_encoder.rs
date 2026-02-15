//! EC11 rotary encoder handling (PIO quadrature + button input)
//!
//! Produces events for:
//! - Rotary turn increments/decrements
//! - Button press / hold start / hold end
//!
//! Pin plan (per docs):
//! - GPIO22: Encoder A
//! - GPIO23: Encoder B
//!
//! The button is connected to the port expander, so it is read via a signal triggered by the expander's interrupt when the button state changes.

use embassy_futures::select::{Either, select};
use embassy_rp::pio_programs::rotary_encoder::{Direction as PioDirection, PioEncoder};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Timer};

use crate::system::event::{Events, RotaryDirection, raise_event};

/// Button hold threshold (ms)
const HOLD_DURATION: Duration = Duration::from_millis(700);

/// Button debounce delay (ms)
const DEBOUNCE_DURATION: Duration = Duration::from_millis(30);

/// Button signal.
/// Since we are using the port expander for the button, we must use a signal instead of a GPIO input directly.
/// The signal is used to trigger the debounce logic when the button is pressed or released.
pub static BTN_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// Triggers the button signal with the current state of the button (pressed or released).
/// Updates the button state in the mutex and then signals any waiting tasks to wake up and process the new state.
pub async fn trigger_button_signal(pressed: bool) {
    // Update the button state
    {
        let mut state = BUTTON_STATE.lock().await;
        state.pressed = pressed;
    }
    // Trigger the signal to wake up any waiting tasks
    BTN_SIGNAL.signal(pressed);
}

/// Waits for the next button signal.
async fn wait_for_button_signal() {
    BTN_SIGNAL.wait().await;
}

/// Button state protected by a mutex.
pub static BUTTON_STATE: Mutex<CriticalSectionRawMutex, ButtonState> = Mutex::new(ButtonState { pressed: false });

/// Represents the current state of the rotary encoder button (pressed or released).
pub struct ButtonState {
    /// True if the button is currently pressed, false if released.
    pub pressed: bool,
}

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
pub async fn rotary_encoder_button() {
    loop {
        let pressed = debounce().await;

        // Active-low: pressed when LOW.
        if !pressed {
            continue;
        }

        match select(Timer::after(HOLD_DURATION), debounce()).await {
            Either::First(()) => {
                raise_event(Events::RotaryButtonHoldStart).await;
                wait_for_released().await;
                raise_event(Events::RotaryButtonHoldEnd).await;
            }
            Either::Second(_) => {
                raise_event(Events::RotaryButtonPressed).await;
            }
        }
    }
}

/// Ensures stable button state after any edge.
async fn debounce() -> bool {
    loop {
        // Wait for the next edge from the expander, then verify stable state.
        wait_for_button_signal().await;
        Timer::after(DEBOUNCE_DURATION).await;

        let state_after_debounce = {
            let btn_state = BUTTON_STATE.lock().await;
            btn_state.pressed
        };

        // If another edge happened during debounce, keep waiting.
        let edge_happened_during_debounce = BTN_SIGNAL.signaled();
        if !edge_happened_during_debounce {
            break state_after_debounce;
        }
    }
}

/// Waits until the button is released (active-low means waiting for HIGH).
async fn wait_for_released() {
    loop {
        let pressed = {
            let btn_state = BUTTON_STATE.lock().await;
            btn_state.pressed
        };
        if !pressed {
            break;
        }
        wait_for_button_signal().await;
    }
}
