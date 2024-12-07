//! Distance sensor handling
//!
//! Measures distances and detects obstacles using the HC-SR04 ultrasonic sensor.
//!
//! # Sensor Operation
//! - Uses async HC-SR04 driver for non-blocking measurements
//! - Measurements taken every 100ms
//! - Distance reported in centimeters
//! - Assumes fixed ambient temperature of 21.5°C
//!
//! # Signal Processing
//! - Uses a moving median filter to reduce noise
//! - Window size of 3 measurements provides good balance of:
//!   - Noise reduction
//!   - Quick response to real changes
//!   - Memory efficiency
//!
//! # Error Handling
//! - Failed measurements return a safe distance (200cm)
//! - This ensures the robot continues operation even with occasional sensor errors
//!
//! # Obstacle Detection
//! - Obstacles detected when filtered distance ≤ 22cm
//! - Detection events sent to system orchestrator
//! - Conservative threshold allows time for robot to react

use crate::system::event::{send, Events};
use crate::system::resources::DistanceSensorResources;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Duration, Timer};
use hcsr04_async::{Config, DistanceUnit, Hcsr04, TemperatureUnit};
use moving_median::MovingMedian;

/// Time between measurements (100ms provides good balance of responsiveness and stability)
const MEASUREMENT_INTERVAL: Duration = Duration::from_millis(100);

/// Size of median filter window (3 samples balances noise reduction vs. latency)
const MEDIAN_WINDOW_SIZE: usize = 3;

/// Fixed ambient temperature for distance calculations
/// Slight inaccuracy acceptable as we care more about consistent readings
const TEMPERATURE: f64 = 21.5;

/// Distance at which obstacles are detected (22cm gives good reaction time)
const MINIMUM_DISTANCE: f64 = 22.0;

/// Main distance measurement task that continuously monitors for obstacles
///
/// Uses median filtering to smooth measurements and sends obstacle detection
/// events to the system orchestrator when filtered distance is below threshold.
#[embassy_executor::task]
pub async fn distance_measure(r: DistanceSensorResources) {
    // Configure sensor for centimeter measurements
    let config: Config = Config {
        distance_unit: DistanceUnit::Centimeters,
        temperature_unit: TemperatureUnit::Celsius,
    };

    // Initialize sensor with trigger and echo pins
    let trigger = Output::new(r.trigger_pin, Level::Low);
    let echo = Input::new(r.echo_pin, Pull::None);
    let mut sensor = Hcsr04::new(trigger, echo, config);

    // Initialize median filter for noise reduction
    let mut median_filter = MovingMedian::<f64, MEDIAN_WINDOW_SIZE>::new();

    loop {
        // Measure distance and apply median filtering
        let filtered_distance = match sensor.measure(TEMPERATURE).await {
            Ok(distance_cm) => {
                median_filter.add_value(distance_cm);
                median_filter.median()
            }
            Err(_) => 200.0, // Return safe distance on error to prevent false positives
        };

        // Send obstacle detection event based on filtered distance
        send(Events::ObstacleDetected(
            filtered_distance <= MINIMUM_DISTANCE,
        ))
        .await;

        // Wait before next measurement
        Timer::after(MEASUREMENT_INTERVAL).await;
    }
}
