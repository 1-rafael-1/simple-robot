//! Obstacle-related behavior handlers.

use defmt::info;

use crate::{
    system::{event::ObstacleSource, state::perception},
    task::{
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
/// Updates perception atomics and unconditionally sends an `EmergencyBrake`
/// interrupt to the drive task. This is a system-wide safety invariant — IR
/// sensors are armed in all modes (RC, autonomous, testing). The interrupt
/// brakes motors, bumps the command epoch, and drains queued commands
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
    update_obstacle_indicator(combined);
    if combined {
        send_drive_interrupt(InterruptKind::EmergencyBrake);
    }
}

/// Handle obstacle avoidance completion.
#[allow(clippy::unused_async)]
pub async fn handle_obstacle_avoidance_attempted() {
    info!("Obstacle avoidance attempted");
}
