//! Obstacle-related behavior handlers.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;

use crate::{
    system::{event::ObstacleSource, state::SYSTEM_STATE},
    task::{
        autonomous_mode::coast_obstacle_avoid,
        drive::{InterruptKind, send_drive_interrupt},
        indicators::rgb_led_indicate::update_obstacle_indicator,
    },
};

/// Tracks obstacle detection by IR sensors.
static IR_OBSTACLE_DETECTED: AtomicBool = AtomicBool::new(false);
/// Tracks obstacle detection by ultrasonic sensor.
static ULTRASONIC_OBSTACLE_DETECTED: AtomicBool = AtomicBool::new(false);

/// Handle obstacle detection status changes.
///
/// When an obstacle is detected while coast-and-avoid autonomous mode is active,
/// an `EmergencyBrake` interrupt is sent to the drive task so that the active
/// `DriveDistance` command resolves immediately as `Cancelled`.  The
/// coast-and-avoid loop will then run its avoidance maneuver and resume.
#[allow(clippy::unused_async)]
pub async fn handle_obstacle_detected(source: ObstacleSource, detected: bool) {
    info!(
        "Obstacle detection status changed: source={:?} detected={}",
        source, detected
    );

    match source {
        ObstacleSource::Ir => {
            IR_OBSTACLE_DETECTED.store(detected, Ordering::Relaxed);
        }
        ObstacleSource::Ultrasonic => {
            ULTRASONIC_OBSTACLE_DETECTED.store(detected, Ordering::Relaxed);
        }
    }

    let combined = IR_OBSTACLE_DETECTED.load(Ordering::Relaxed) || ULTRASONIC_OBSTACLE_DETECTED.load(Ordering::Relaxed);

    let changed = {
        let mut state = SYSTEM_STATE.lock().await;
        let changed = state.obstacle_detected != combined;
        if changed {
            state.obstacle_detected = combined;
        }
        changed
    };

    if changed {
        if combined && coast_obstacle_avoid::is_active() {
            info!("coast-avoid active — issuing EmergencyBrake");
            send_drive_interrupt(InterruptKind::EmergencyBrake);
        }

        update_obstacle_indicator(combined);
    }
}

/// Handle obstacle avoidance completion.
#[allow(clippy::unused_async)]
pub async fn handle_obstacle_avoidance_attempted() {
    info!("Obstacle avoidance attempted");
}
