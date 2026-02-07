//! IR Obstacle Detection
//!
//! Detects obstacles using the VMA330 IR obstacle avoidance sensor.
//!
//! # Sensor Operation
//! - Port expander monitors the IR input and signals changes
//! - This task waits for signals and emits system events
//! - Includes debounce delay to filter noise
//!
//! # Operation
//! - IR sensor outputs low (0) when obstacle detected
//! - High (1) when no obstacle present
//! - But we have inverted the sensor output in hardware
//! - The port expander signals the updated state

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};

use crate::system::event::{Events, raise_event};

/// Debounce delay to filter out noise
const DEBOUNCE_DELAY: Duration = Duration::from_millis(100);

/// Channel used by the port expander to signal IR state changes
static IR_SIGNAL_CHANNEL: Channel<CriticalSectionRawMutex, bool, 4> = Channel::new();

/// Signal IR obstacle state changes from the port expander task
pub async fn signal_ir_obstacle(state: bool) {
    IR_SIGNAL_CHANNEL.sender().send(state).await;
}

/// Main obstacle detection task that waits for port expander signals
///
/// The port expander detects input changes and signals the new state. This task
/// debounces and raises the system event when the state changes.
#[embassy_executor::task]
pub async fn ir_obstacle_detect() {
    let mut last_obstacle_detected: Option<bool> = None;

    loop {
        let obstacle_detected = IR_SIGNAL_CHANNEL.receiver().receive().await;
        Timer::after(DEBOUNCE_DELAY).await;

        // Only send event if state has changed
        if last_obstacle_detected != Some(obstacle_detected) {
            raise_event(Events::ObstacleDetected(obstacle_detected)).await;
            last_obstacle_detected = Some(obstacle_detected);
        }
    }
}
