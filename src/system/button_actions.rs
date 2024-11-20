//! Button action handling
//!
//! Maps remote control button inputs to robot actions.

use crate::system::event;
use crate::system::state;
use crate::task::drive;
use crate::task::rgb_led_indicate;

/// Button action types
#[derive(Debug, Clone, Copy)]
pub enum ButtonActionType {
    /// Single press
    Press,
    /// Hold initiated
    HoldStart,
    /// Hold released
    HoldEnd,
}

/// Processes button actions
pub async fn handle_button_action(button_id: event::ButtonId, action_type: ButtonActionType) {
    match action_type {
        ButtonActionType::Press => {
            // If we're in autonomous mode, any button press switches to manual mode
            let state = state::SYSTEM_STATE.lock().await;
            if state.operation_mode == state::OperationMode::Autonomous {
                event::send(event::Events::OperationModeSet(
                    state::OperationMode::Manual,
                ))
                .await;
                return;
            }
        }
        _ => (), // No action for other combinations
    }

    match (button_id, action_type) {
        (event::ButtonId::A, ButtonActionType::HoldEnd) => {
            let state = state::SYSTEM_STATE.lock().await;
            event::send(event::Events::OperationModeSet(
                if state.operation_mode == state::OperationMode::Manual {
                    state::OperationMode::Autonomous
                } else {
                    state::OperationMode::Manual
                },
            ))
            .await;
        }
        (event::ButtonId::A, ButtonActionType::Press) => {
            rgb_led_indicate::update_indicator(true);
            drive::send_command(drive::Command::Forward(20));
        }
        (event::ButtonId::B, ButtonActionType::Press) => {
            rgb_led_indicate::update_indicator(true);
            drive::send_command(drive::Command::Right(20));
        }
        (event::ButtonId::C, ButtonActionType::Press) => {
            rgb_led_indicate::update_indicator(true);
            drive::send_command(drive::Command::Left(20));
        }
        (event::ButtonId::D, ButtonActionType::Press) => {
            rgb_led_indicate::update_indicator(true);
            drive::send_command(drive::Command::Backward(20));
        }
        _ => (), // No action for other combinations
    }
}
