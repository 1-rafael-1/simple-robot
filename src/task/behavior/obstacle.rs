//! Obstacle-related behavior handlers.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;

use crate::{
    system::{
        event::ObstacleSource,
        state::{SYSTEM_STATE, UiMode},
    },
    task::{
        autonomous_mode::coast_obstacle_avoid,
        drive::{InterruptKind, send_drive_interrupt},
        indicators::rgb_led_indicate::update_obstacle_indicator,
        ui,
    },
};

/// Tracks obstacle detection by IR sensors.
static IR_OBSTACLE_DETECTED: AtomicBool = AtomicBool::new(false);
/// Tracks obstacle detection by ultrasonic sensor.
static ULTRASONIC_OBSTACLE_DETECTED: AtomicBool = AtomicBool::new(false);

/// Reset obstacle detection state (clears IR/US flags and combined state).
pub async fn reset_obstacle_state() {
    IR_OBSTACLE_DETECTED.store(false, Ordering::Relaxed);
    ULTRASONIC_OBSTACLE_DETECTED.store(false, Ordering::Relaxed);

    {
        let mut state = SYSTEM_STATE.lock().await;
        state.ir_obstacle_detected = false;
        state.ultrasonic_obstacle_detected = false;
        state.obstacle_detected = false;
    }

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

    let ir_detected = IR_OBSTACLE_DETECTED.load(Ordering::Relaxed);
    let ultrasonic_detected = ULTRASONIC_OBSTACLE_DETECTED.load(Ordering::Relaxed);
    let combined = ir_detected || ultrasonic_detected;

    let changed = {
        let mut state = SYSTEM_STATE.lock().await;
        state.ir_obstacle_detected = ir_detected;
        state.ultrasonic_obstacle_detected = ultrasonic_detected;
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
