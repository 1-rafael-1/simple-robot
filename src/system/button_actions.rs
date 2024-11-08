//! Button Actions Module
//!
//! This module handles the actions triggered by button inputs on the robot's remote control.
//! It defines the types of button actions and provides a central function for handling
//! different button actions based on the button ID and action type.
use crate::system::drive_command;
use crate::system::event;
use crate::system::indicator;
use crate::system::state;

/// Enum representing the types of button actions
///
/// This enum defines the different types of actions that can be performed with a button,
/// including a single press, the start of a hold, and the end of a hold.
#[derive(Debug, Clone, Copy)]
pub enum ButtonActionType {
    /// Represents a short, single press of a button
    Press,
    /// Represents the moment a button is held down
    HoldStart,
    /// Represents the moment a held button is released
    HoldEnd,
}

/// Handles button actions based on the button ID and action type
///
/// This function is the central point for processing all button actions. It matches
/// the combination of button ID and action type to determine and execute the
/// appropriate action for the robot.
///
/// # Arguments
///
/// * `button_id` - The identifier of the button that was acted upon
/// * `action_type` - The type of action performed on the button
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
        // just testing! -> for now, just wakes the indicator
        (event::ButtonId::A, ButtonActionType::Press) => {
            indicator::update(true);
            drive_command::update(drive_command::Command::Forward(20));
        }
        (event::ButtonId::B, ButtonActionType::Press) => {
            indicator::update(true);
            drive_command::update(drive_command::Command::Right(20));
        }
        (event::ButtonId::C, ButtonActionType::Press) => {
            indicator::update(true);
            drive_command::update(drive_command::Command::Left(20));
        }
        (event::ButtonId::D, ButtonActionType::Press) => {
            indicator::update(true);
            drive_command::update(drive_command::Command::Backward(20));
        }
        // Add other button actions here
        _ => (), // No action for other combinations
    }
}
