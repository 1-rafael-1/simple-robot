use crate::task::resources::DistanceSensorResources;
use crate::task::system_events::{send_event, Events};
use defmt::info;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Duration, Timer};
use hcsr04_async::{Config, DistanceUnit, Hcsr04, TemperatureUnit};
use moving_median::MovingMedian;

const MEASUREMENT_INTERVAL: Duration = Duration::from_millis(500);
const MEDIAN_WINDOW_SIZE: usize = 5;
const TEMPERATURE: f64 = 21.5;
const MINIMUM_DISTANCE: f64 = 20.0;

#[embassy_executor::task]
pub async fn distance_measurement(r: DistanceSensorResources) {
    info!("Distance measurement started");
    let config: Config = Config {
        distance_unit: DistanceUnit::Centimeters,
        temperature_unit: TemperatureUnit::Celsius,
    };
    let trigger = Output::new(r.trigger_pin, Level::Low);
    let echo = Input::new(r.echo_pin, Pull::None);
    let mut sensor = Hcsr04::new(trigger, echo, config);
    let mut median_filter = MovingMedian::<MEDIAN_WINDOW_SIZE>::new();

    loop {
        let filtered_distance = match sensor.measure(TEMPERATURE).await {
            Ok(distance_cm) => {
                median_filter.add_value(distance_cm);
                let filtered_distance = median_filter.median();
                filtered_distance
            }
            Err(_) => 200.0,
        };

        info!("Distance: {:?}", filtered_distance);

        // signal if we have an obstacle within our minimum range
        send_event(Events::ObstacleDetected(
            filtered_distance <= MINIMUM_DISTANCE,
        ))
        .await;

        // Wait for the next measurement interval
        Timer::after(MEASUREMENT_INTERVAL).await;
    }
}
