//! IR Obstacle Detection
//!
//! Detects obstacles using the VMA330 IR obstacle avoidance sensor.
//!
//! # Sensor Operation
//! - Uses digital input with edge detection
//! - Responds to both rising and falling edges
//! - Includes debounce delay to filter noise
//!
//! # Operation
//! - IR sensor outputs low (0) when obstacle detected
//! - High (1) when no obstacle present
//! - Edge detection ensures immediate response to changes

use embassy_rp::gpio::{Input, Pull};
use embassy_time::{Duration, Timer};

use crate::system::{
    event::{send, Events},
    resources::IRSensorResources,
};

/// Debounce delay to filter out noise
const DEBOUNCE_DELAY: Duration = Duration::from_millis(100);

/// Main obstacle detection task that monitors for obstacles using IR sensor
///
/// Uses edge detection to respond to changes in sensor state, with debouncing
/// to filter out noise. The sensor outputs low (0) when an obstacle is detected.
#[embassy_executor::task]
pub async fn ir_obstacle_detect(r: IRSensorResources) {
    // Initialize IR sensor pin as digital input with pull-up
    let mut ir_pin = Input::new(r.ir_pin, Pull::Up);

    // perform initial measure to ensure initial state is caught
    Timer::after(DEBOUNCE_DELAY).await;
    let current_state = ir_pin.is_low();
    let mut last_state = current_state;
    // and send initial event
    send(Events::ObstacleDetected(current_state)).await;

    loop {
        // Wait for any edge (rising or falling)
        ir_pin.wait_for_any_edge().await;

        // Debounce delay
        Timer::after(DEBOUNCE_DELAY).await;

        // Read current state after debounce
        let current_state = ir_pin.is_low();

        // Only send event if state has changed
        if current_state != last_state {
            send(Events::ObstacleDetected(current_state)).await;
            last_state = current_state;
        }
    }
}
