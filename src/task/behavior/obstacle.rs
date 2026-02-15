//! Obstacle-related behavior handlers.

use defmt::info;

/// Handle obstacle detection status changes.
#[allow(clippy::unused_async)]
pub async fn handle_obstacle_detected(_detected: bool) {
    info!("Obstacle detection status changed");
    // TODO: Implement obstacle response
    // - Trigger avoidance in autonomous mode
    // - Update LED indicators
}

/// Handle obstacle avoidance completion.
#[allow(clippy::unused_async)]
pub async fn handle_obstacle_avoidance_attempted() {
    info!("Obstacle avoidance attempted");
    // TODO: Implement post-avoidance logic
    // - Resume normal operation or retry
}
