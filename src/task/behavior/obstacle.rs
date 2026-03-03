//! Obstacle-related behavior handlers.

use defmt::info;

use crate::{
    system::state::SYSTEM_STATE,
    task::{
        autonomous_mode::coast_obstacle_avoid,
        drive::{InterruptKind, send_drive_interrupt},
        indicators::rgb_led_indicate::update_indicator,
    },
};

/// Handle obstacle detection status changes.
///
/// When an obstacle is detected while coast-and-avoid autonomous mode is active,
/// an `EmergencyBrake` interrupt is sent to the drive task so that the active
/// `DriveDistance` command resolves immediately as `Cancelled`.  The
/// coast-and-avoid loop will then run its avoidance maneuver and resume.
#[allow(clippy::unused_async)]
pub async fn handle_obstacle_detected(detected: bool) {
    info!("Obstacle detection status changed: {}", detected);

    {
        let mut state = SYSTEM_STATE.lock().await;
        state.obstacle_detected = detected;
    }

    if detected && coast_obstacle_avoid::is_active() {
        info!("coast-avoid active — issuing EmergencyBrake");
        send_drive_interrupt(InterruptKind::EmergencyBrake);
    }

    if detected {
        update_indicator(true);
    }
}

/// Handle obstacle avoidance completion.
#[allow(clippy::unused_async)]
pub async fn handle_obstacle_avoidance_attempted() {
    info!("Obstacle avoidance attempted");
}
