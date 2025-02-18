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
//! - But we have inverted the sensor output in hardware
//! - Edge detection ensures immediate response to changes

use embassy_rp::gpio::{Input, Pull};
use embassy_time::{Duration, Timer};

use crate::system::{
    event::{send_event, Events},
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
    // Initialize IR sensor pin as digital input, with pull-down resistor. No actual floating condition is expected, as long as
    // the sensor(s) are connected properly, since they will always be either high or low.
    let mut ir_right = Input::new(r.ir_right_pin, Pull::Down);

    // perform initial measure to ensure initial state is caught
    Timer::after(DEBOUNCE_DELAY).await;

    // Read initial state. We have inverted the sensor output in hardware, so high is obstacle detected here.
    let mut obstacle_detected = ir_right.is_high();
    let mut last_obstacle_detected = obstacle_detected;
    // and send initial event
    send_event(Events::ObstacleDetected(obstacle_detected)).await;

    loop {
        ir_right.wait_for_any_edge().await;
        Timer::after(DEBOUNCE_DELAY).await;

        // Read current state after debounce
        obstacle_detected = ir_right.is_high();

        // Only send event if state has changed
        if obstacle_detected != last_obstacle_detected {
            send_event(Events::ObstacleDetected(obstacle_detected)).await;
            last_obstacle_detected = obstacle_detected;
        }
    }
}
