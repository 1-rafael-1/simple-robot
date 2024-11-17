//! Distance sensor handling
//!
//! Measures distances and detects obstacles using HC-SR04.

use crate::system::event::{send, Events};
use crate::system::resources::DistanceSensorResources;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Duration, Timer};
use hcsr04_async::{Config, DistanceUnit, Hcsr04, TemperatureUnit};
use moving_median::MovingMedian;

/// Measurement interval (ms)
const MEASUREMENT_INTERVAL: Duration = Duration::from_millis(100);

/// Median filter window size
const MEDIAN_WINDOW_SIZE: usize = 3;

/// Ambient temperature (°C)
const TEMPERATURE: f64 = 21.5;

/// Obstacle detection threshold (cm)
const MINIMUM_DISTANCE: f64 = 22.0;

/// Distance measurement task
#[embassy_executor::task]
pub async fn distance_measure(r: DistanceSensorResources) {
    // Configure sensor
    let config: Config = Config {
        distance_unit: DistanceUnit::Centimeters,
        temperature_unit: TemperatureUnit::Celsius,
    };

    // Setup sensor
    let trigger = Output::new(r.trigger_pin, Level::Low);
    let echo = Input::new(r.echo_pin, Pull::None);
    let mut sensor = Hcsr04::new(trigger, echo, config);

    let mut median_filter = MovingMedian::<f64, MEDIAN_WINDOW_SIZE>::new();

    loop {
        // Measure and filter distance
        let filtered_distance = match sensor.measure(TEMPERATURE).await {
            Ok(distance_cm) => {
                median_filter.add_value(distance_cm);
                median_filter.median()
            }
            Err(_) => 200.0, // Safe distance on error
        };

        send(Events::ObstacleDetected(
            filtered_distance <= MINIMUM_DISTANCE,
        ))
        .await;

        Timer::after(MEASUREMENT_INTERVAL).await;
    }
}
