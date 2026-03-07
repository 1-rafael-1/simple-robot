//! Button Action Handler
//!
//! Maps remote control button inputs to robot actions and manages button
//! interaction patterns. Supports immediate actions (press) and
//! extended actions (hold).
//!
//! # Button Mapping
//! - Button A:
//!   - Press: Drive forward
//!   - Hold: No action
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
//! - None (buttons no longer toggle operation mode)
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
//! - Quick flash for movement commands

use crate::{
    system::event,
    task::{
        drive::{self, DriveAction, DriveCommand, MotorSide, RotationDirection, RotationMotion},
        indicators::rgb_led_indicate,
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
/// Handles immediate actions (movement) based on button identity and
/// action type. Provides visual feedback through LED indicators for all actions.
pub async fn handle_button_action(button_id: event::ButtonId, action_type: ButtonActionType) {
    match (button_id, action_type) {
        // Button A - Forward control
        (event::ButtonId::A, ButtonActionType::Press) => {
            // Drive forward at 20% power
            rgb_led_indicate::update_indicator(true);
            drive::send_drive_command(DriveCommand::Drive(DriveAction::Forward(20))).await;
        }

        // Button B - Right turn
        (event::ButtonId::B, ButtonActionType::Press) => {
            // Turn right using torque bias at 20% differential
            rgb_led_indicate::update_indicator(true);
            drive::send_drive_command(DriveCommand::Drive(DriveAction::TorqueBias {
                reduce_side: MotorSide::Right,
                bias_amount: 20,
            }))
            .await;
        }
        (event::ButtonId::B, ButtonActionType::HoldEnd) => {
            // Precise 90-degree clockwise rotation
            // TODO: Track motor state to enable rotation while moving
            // For now, always use stationary rotation
            let motion = RotationMotion::Stationary;

            drive::send_drive_command(DriveCommand::Drive(DriveAction::RotateExact {
                degrees: 90.0,
                direction: RotationDirection::Clockwise,
                motion,
            }))
            .await;
        }

        // Button C - Left turn
        (event::ButtonId::C, ButtonActionType::Press) => {
            // Turn left using torque bias at 20% differential
            rgb_led_indicate::update_indicator(true);
            drive::send_drive_command(DriveCommand::Drive(DriveAction::TorqueBias {
                reduce_side: MotorSide::Left,
                bias_amount: 20,
            }))
            .await;
        }
        (event::ButtonId::C, ButtonActionType::HoldEnd) => {
            // Precise 90-degree counter-clockwise rotation
            // TODO: Track motor state to enable rotation while moving
            // For now, always use stationary rotation
            let motion = RotationMotion::Stationary;

            drive::send_drive_command(DriveCommand::Drive(DriveAction::RotateExact {
                degrees: 90.0,
                direction: RotationDirection::CounterClockwise,
                motion,
            }))
            .await;
        }

        // Button D - Reverse
        (event::ButtonId::D, ButtonActionType::Press) => {
            // Drive backward at 20% power
            rgb_led_indicate::update_indicator(true);
            drive::send_drive_command(DriveCommand::Drive(DriveAction::Backward(20))).await;
        }

        // All other button/action combinations have no effect
        _ => (),
    }
}
