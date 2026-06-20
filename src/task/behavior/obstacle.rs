//! Obstacle-related behavior handlers.

use defmt::info;

use crate::{
    system::{event::ObstacleSource, state::perception},
    task::{
        autonomous_mode::coast_obstacle_avoid,
        drive::{InterruptKind, send_drive_interrupt},
        indicators::rgb_led_indicate::update_obstacle_indicator,
    },
};

/// Reset obstacle detection state and clear all perception data.
pub async fn reset_obstacle_state() {
    perception::reset_all().await;
    update_obstacle_indicator(false);
}

/// Handle obstacle detection status changes.
///
/// When an obstacle is detected while coast-and-avoid autonomous mode is active,
/// an `EmergencyBrake` interrupt is sent to the drive task so that the active
/// `DriveDistance` command resolves immediately as `Cancelled`.  The
/// coast-and-avoid loop will then run its avoidance maneuver and resume.
pub fn handle_obstacle_detected(source: ObstacleSource, detected: bool) {
    info!(
        "Obstacle detection status changed: source={:?} detected={}",
        source, detected
    );

    match source {
        ObstacleSource::Ir => {
            perception::set_ir_obstacle(detected);
        }
        ObstacleSource::Ultrasonic => {
            perception::set_ultrasonic_obstacle(detected);
        }
    }

    let combined = perception::is_obstacle_detected();
    if combined && coast_obstacle_avoid::is_active() && coast_obstacle_avoid::is_forward_phase() {
        info!("coast-avoid forward phase — issuing EmergencyBrake");
        send_drive_interrupt(InterruptKind::EmergencyBrake);
    }

    // Always update the indicator — for Ultrasonic the atomics may already
    // have been set by set_ultrasonic_reading(), causing set_ultrasonic_obstacle()
    // to return NoChange even though the obstacle state genuinely changed.
    update_obstacle_indicator(combined);
}

/// Handle obstacle avoidance completion.
#[allow(clippy::unused_async)]
pub async fn handle_obstacle_avoidance_attempted() {
    info!("Obstacle avoidance attempted");
}
