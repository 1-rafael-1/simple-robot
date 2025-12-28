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
//!   - Press: Turn right (torque bias)
//!   - Hold: Precise 90-degree clockwise rotation
//! - Button C:
//!   - Press: Turn left (torque bias)
//!   - Hold: Precise 90-degree counter-clockwise rotation
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
//! - Manual movements use 20% power for:
//!   - Safe initial speed
//!   - Predictable response
//!   - Good maneuverability
//! - Precise rotations maintain current motion if active
//!
//! # Visual Feedback
//! LED indicator updates confirm button actions:
//! - Blink sequence for mode changes
//! - Quick flash for movement commands

use crate::{
    system::{event, state},
    task::{
        drive::{self, DriveAction, DriveCommand, MotorSide, RotationDirection, RotationMotion},
        rgb_led_indicate,
    },
};

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
    if let ButtonActionType::Press = action_type {
        // Any button press in autonomous mode switches to manual
        let state = state::SYSTEM_STATE.lock().await;
        if state.operation_mode == state::OperationMode::Autonomous {
            event::send_event(event::Events::OperationModeSet(state::OperationMode::Manual)).await;
            return;
        }
    }

    match (button_id, action_type) {
        // Button A - Forward/Mode control
        (event::ButtonId::A, ButtonActionType::HoldEnd) => {
            // Toggle operation mode
            let state = state::SYSTEM_STATE.lock().await;
            event::send_event(event::Events::OperationModeSet(
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
            drive::send_drive_command(DriveCommand::Drive(DriveAction::Forward(20)));
        }

        // Button B - Right turn
        (event::ButtonId::B, ButtonActionType::Press) => {
            // Turn right using torque bias at 20% differential
            rgb_led_indicate::update_indicator(true);
            drive::send_drive_command(DriveCommand::Drive(DriveAction::TorqueBias {
                reduce_side: MotorSide::Right,
                bias_amount: 20,
            }));
        }
        (event::ButtonId::B, ButtonActionType::HoldEnd) => {
            // Precise 90-degree clockwise rotation
            // If we're moving, maintain that motion during rotation
            let left_motor = drive::LEFT_MOTOR.lock().await;
            let current_speed = left_motor.as_ref().unwrap().current_speed();

            let motion = if current_speed > 0 {
                RotationMotion::WhileForward(current_speed as u8)
            } else if current_speed < 0 {
                RotationMotion::WhileBackward(-current_speed as u8)
            } else {
                RotationMotion::Stationary
            };

            drive::send_drive_command(DriveCommand::Drive(DriveAction::RotateExact {
                degrees: 90.0,
                direction: RotationDirection::Clockwise,
                motion,
            }));
        }

        // Button C - Left turn
        (event::ButtonId::C, ButtonActionType::Press) => {
            // Turn left using torque bias at 20% differential
            rgb_led_indicate::update_indicator(true);
            drive::send_drive_command(DriveCommand::Drive(DriveAction::TorqueBias {
                reduce_side: MotorSide::Left,
                bias_amount: 20,
            }));
        }
        (event::ButtonId::C, ButtonActionType::HoldEnd) => {
            // Precise 90-degree counter-clockwise rotation
            // If we're moving, maintain that motion during rotation
            let left_motor = drive::LEFT_MOTOR.lock().await;
            let current_speed = left_motor.as_ref().unwrap().current_speed();

            let motion = if current_speed > 0 {
                RotationMotion::WhileForward(current_speed as u8)
            } else if current_speed < 0 {
                RotationMotion::WhileBackward(-current_speed as u8)
            } else {
                RotationMotion::Stationary
            };

            drive::send_drive_command(DriveCommand::Drive(DriveAction::RotateExact {
                degrees: 90.0,
                direction: RotationDirection::CounterClockwise,
                motion,
            }));
        }

        // Button D - Reverse
        (event::ButtonId::D, ButtonActionType::Press) => {
            // Drive backward at 20% power
            rgb_led_indicate::update_indicator(true);
            drive::send_drive_command(DriveCommand::Drive(DriveAction::Backward(20)));
        }

        // All other button/action combinations have no effect
        _ => (),
    }
}
