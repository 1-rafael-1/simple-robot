//! Distance Measurement Module
//!
//! This module is responsible for periodically measuring the distance to obstacles
//! using an HC-SR04 ultrasonic sensor and detecting if an obstacle is within a minimum distance.

use crate::task::resources::DistanceSensorResources;
use crate::task::system_events::{send_event, Events};
use defmt::info;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Duration, Timer};
use hcsr04_async::{Config, DistanceUnit, Hcsr04, TemperatureUnit};
use moving_median::MovingMedian;

/// Interval between distance measurements
const MEASUREMENT_INTERVAL: Duration = Duration::from_millis(500);

/// Size of the moving median window for filtering measurements
const MEDIAN_WINDOW_SIZE: usize = 5;

/// Assumed ambient temperature for sound speed calculation
const TEMPERATURE: f64 = 21.5;

/// Minimum distance threshold for obstacle detection (in cm)
const MINIMUM_DISTANCE: f64 = 20.0;

/// Task for measuring distance and detecting obstacles
///
/// This task periodically measures the distance using an HC-SR04 ultrasonic sensor,
/// applies a median filter to the measurements, and sends an event if an obstacle
/// is detected within the minimum distance threshold.
#[embassy_executor::task]
pub async fn distance_measure(r: DistanceSensorResources) {
    // Configure the HC-SR04 sensor
    let config: Config = Config {
        distance_unit: DistanceUnit::Centimeters,
        temperature_unit: TemperatureUnit::Celsius,
    };

    // Initialize GPIO pins for the sensor
    let trigger = Output::new(r.trigger_pin, Level::Low);
    let echo = Input::new(r.echo_pin, Pull::None);

    // Create the sensor instance
    let mut sensor = Hcsr04::new(trigger, echo, config);

    // Initialize the median filter
    let mut median_filter = MovingMedian::<MEDIAN_WINDOW_SIZE>::new();

    loop {
        // Measure distance and apply median filter
        let filtered_distance = match sensor.measure(TEMPERATURE).await {
            Ok(distance_cm) => {
                median_filter.add_value(distance_cm);
                median_filter.median()
            }
            Err(_) => 200.0, // Default to a large distance on error
        };

        info!("Distance: {:?}", filtered_distance);

        send_event(Events::ObstacleDetected(
            filtered_distance <= MINIMUM_DISTANCE,
        ))
        .await;
        Timer::after(MEASUREMENT_INTERVAL).await;
    }
}
