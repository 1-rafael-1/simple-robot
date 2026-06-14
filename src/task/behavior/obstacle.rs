//! Obstacle-related behavior handlers.

use defmt::info;

use crate::{
    system::{event::ObstacleSource, state::perception},
    task::{
        autonomous_mode::coast_obstacle_avoid,
        drive::{InterruptKind, send_drive_interrupt},
        indicators::rgb_led_indicate::update_obstacle_indicator,
        ui::{self, state::UiMode},
    },
};

/// Reset obstacle detection state and clear all perception data.
pub async fn reset_obstacle_state() {
    perception::reset_all().await;
    update_obstacle_indicator(false);

    let ui_mode = {
        let ui_state = crate::task::ui::state::UI_STATE.lock().await;
        ui_state.mode
    };
    if matches!(ui_mode, UiMode::RunningAutonomous { .. }) {
        ui::refresh().await;
    }
}

/// Handle obstacle detection status changes.
///
/// When an obstacle is detected while coast-and-avoid autonomous mode is active,
/// an `EmergencyBrake` interrupt is sent to the drive task so that the active
/// `DriveDistance` command resolves immediately as `Cancelled`.  The
/// coast-and-avoid loop will then run its avoidance maneuver and resume.
pub async fn handle_obstacle_detected(source: ObstacleSource, detected: bool) {
    info!(
        "Obstacle detection status changed: source={:?} detected={}",
        source, detected
    );

    let change = match source {
        ObstacleSource::Ir => perception::set_ir_obstacle(detected),
        ObstacleSource::Ultrasonic => perception::set_ultrasonic_obstacle(detected),
    };

    let combined = perception::is_obstacle_detected();
    if combined && coast_obstacle_avoid::is_active() && coast_obstacle_avoid::is_forward_phase() {
        info!("coast-avoid forward phase — issuing EmergencyBrake");
        send_drive_interrupt(InterruptKind::EmergencyBrake);
    }

    if change != perception::ChangeDetected::NoChange {
        update_obstacle_indicator(combined);

        let ui_mode = {
            let ui_state = crate::task::ui::state::UI_STATE.lock().await;
            ui_state.mode
        };
        if matches!(ui_mode, UiMode::RunningAutonomous { .. }) {
            ui::refresh().await;
        }
    }
}

/// Handle obstacle avoidance completion.
#[allow(clippy::unused_async)]
pub async fn handle_obstacle_avoidance_attempted() {
    info!("Obstacle avoidance attempted");
}
