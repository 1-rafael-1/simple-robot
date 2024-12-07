//! Button Action Handler
//!
//! Maps remote control button inputs to robot actions and manages button
//! interaction patterns. Supports both immediate actions (press) and
//! mode changes (hold).
//!
//! # Button Mapping
//! - Button A:
//!   - Press: Drive forward
//!   - Hold: Toggle between Manual/Autonomous modes
//! - Button B:
//!   - Press: Turn right
//!   - Hold: No action
//! - Button C:
//!   - Press: Turn left
//!   - Hold: No action
//! - Button D:
//!   - Press: Drive backward
//!   - Hold: No action
//!
//! # Mode Switching
//! Any button press in autonomous mode switches back to manual mode,
//! providing a quick way to regain control. Mode toggling through
//! button A hold works in any mode.
//!
//! # Movement Commands
//! All movement commands use 20% power to provide:
//! - Safe initial speed
//! - Predictable response
//! - Good maneuverability
//!
//! # Visual Feedback
//! LED indicator updates confirm button actions:
//! - Blink sequence for mode changes
//! - Quick flash for movement commands

use crate::system::event;
use crate::system::state;
use crate::task::drive;
use crate::task::rgb_led_indicate;

/// Button interaction types defining how a button event is interpreted
#[derive(Debug, Clone, Copy)]
pub enum ButtonActionType {
    /// Single press - triggers immediate action
    Press,
    /// Hold initiated - starts long-press action
    HoldStart,
    /// Hold released - completes long-press action
    HoldEnd,
}

/// Processes button actions and triggers corresponding system responses
///
/// Handles both immediate actions (movement) and mode changes based on
/// button identity and action type. Provides visual feedback through
/// LED indicators for all actions.
pub async fn handle_button_action(button_id: event::ButtonId, action_type: ButtonActionType) {
    match action_type {
        ButtonActionType::Press => {
            // Any button press in autonomous mode switches to manual
            let state = state::SYSTEM_STATE.lock().await;
            if state.operation_mode == state::OperationMode::Autonomous {
                event::send(event::Events::OperationModeSet(
                    state::OperationMode::Manual,
                ))
                .await;
                return;
            }
        }
        _ => (), // Other action types handled in specific button logic
    }

    match (button_id, action_type) {
        // Button A - Forward/Mode control
        (event::ButtonId::A, ButtonActionType::HoldEnd) => {
            // Toggle operation mode
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
            // Drive forward at 20% power
            rgb_led_indicate::update_indicator(true);
            drive::send_command(drive::Command::Forward(20));
        }

        // Button B - Right turn
        (event::ButtonId::B, ButtonActionType::Press) => {
            // Turn right at 20% differential
            rgb_led_indicate::update_indicator(true);
            drive::send_command(drive::Command::Right(20));
        }

        // Button C - Left turn
        (event::ButtonId::C, ButtonActionType::Press) => {
            // Turn left at 20% differential
            rgb_led_indicate::update_indicator(true);
            drive::send_command(drive::Command::Left(20));
        }

        // Button D - Reverse
        (event::ButtonId::D, ButtonActionType::Press) => {
            // Drive backward at 20% power
            rgb_led_indicate::update_indicator(true);
            drive::send_command(drive::Command::Backward(20));
        }

        // All other button/action combinations have no effect
        _ => (),
    }
}
