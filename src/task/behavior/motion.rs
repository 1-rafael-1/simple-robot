//! Motion-related behavior handlers.

use defmt::info;

use crate::task::sensors::imu;

/// Handle rotation completion.
#[allow(clippy::unused_async)]
pub async fn handle_rotation_completed() {
    info!("Rotation completed");
    // TODO: Implement rotation completion logic
    // - Update display
    // - Signal next movement phase
}

/// Handle motion data collection control.
pub fn handle_start_stop_motion_data(start: bool) {
    info!("Motion data collection control");

    // For now, "motion data collection" means IMU orientation streaming for control loops.
    // Encoders are started/stopped by the drive task depending on control mode (e.g., drift compensation),
    // so we only manage the IMU here.
    if start {
        info!("Starting IMU readings");
        imu::start_imu_readings();
    } else {
        info!("Stopping IMU readings");
        imu::stop_imu_readings();
    }
}

/// Handle ultrasonic sweep control.
#[allow(clippy::unused_async)]
pub async fn handle_start_stop_ultrasonic_sweep(_start: bool) {
    info!("Ultrasonic sweep control");
    // TODO: Implement ultrasonic sweep control
    // - Start/stop sweep task
}
