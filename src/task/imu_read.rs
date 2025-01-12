//! IMU (Inertial Measurement Unit) reading functionality using the MPU6050 sensor.
//!
//! This module provides an asynchronous task that handles reading accelerometer and gyroscope
//! data from an MPU6050 IMU sensor over I2C. The readings are triggered on-demand using a
//! signal mechanism.
//!
//! # Architecture
//!
//! The module operates as follows:
//! 1. The main task `inertial_measurement_handle` initializes the MPU6050 sensor with DMP
//!    (Digital Motion Processor) support
//! 2. It then waits for measurement requests via the `IMU_CONTROL` signal
//! 3. When triggered, it reads both accelerometer and gyroscope data
//! 4. The measurements are combined and sent as an event (`Events::InertialMeasurementTaken`)
//!
//! # Usage
//!
//! ```rust
//! // Request a new IMU measurement
//! imu_read::request_measurement();
//!
//! // The measurement will be available via the Events::InertialMeasurementTaken event
//! ```
//!
//! # Data Format
//!
//! - Accelerometer readings are in milli-g (thousandths of gravitational acceleration)
//! - Gyroscope readings are in degrees per second
//! - Both types of readings provide data for all three axes (x, y, z)

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Delay;
use mpu6050_dmp::{address::Address, sensor_async::Mpu6050};

use crate::system::{
    event::{send, Events},
    resources::I2c0BusShared,
};

/// Control signal used to trigger new IMU readings.
pub static IMU_CONTROL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Requests a new IMU measurement to be taken.
pub fn request_measurement() {
    IMU_CONTROL.signal(());
}

/// Blocks until the next measurement request is received.
async fn wait() -> () {
    IMU_CONTROL.wait().await
}

/// Combined IMU measurement containing both accelerometer and gyroscope data.
///
/// This structure represents a complete inertial measurement from the MPU6500,
/// combining both acceleration and rotational velocity readings.
#[derive(Clone, Copy, Debug)]
pub struct ImuMeasurement {
    /// Accelerometer readings in milli-g
    pub accel: ImuData,
    /// Gyroscope readings in degrees per second
    pub gyro: ImuData,
}

/// Single IMU measurement data containing readings for all three axes.
///
/// This structure is used for both accelerometer (in milli-g) and
/// gyroscope (in degrees per second) measurements.
#[derive(Clone, Copy, Debug)]
pub struct ImuData {
    /// X-axis reading
    pub x: i16,
    /// Y-axis reading
    pub y: i16,
    /// Z-axis reading
    pub z: i16,
}

/// Embassy task that handles IMU measurements.
///
/// This task:
/// 1. Initializes the MPU6050 sensor with DMP support
/// 2. Waits for measurement requests via `IMU_CONTROL`
/// 3. Takes accelerometer and gyroscope readings when triggered
/// 4. Sends the measurements via the event system
///
/// # Arguments
/// * `i2c_bus` - Shared I2C bus for communicating with the MPU6050 sensor
#[embassy_executor::task]
pub async fn inertial_measurement_handle(i2c_bus: &'static I2c0BusShared) {
    let i2c = I2cDevice::new(i2c_bus);

    // Initialize MPU6500
    let mut sensor = Mpu6050::new(i2c, Address::default()).await.unwrap();
    let mut delay = Delay;
    sensor.initialize_dmp(&mut delay).await.unwrap();

    loop {
        wait().await;

        let accel_data = sensor.accel().await.unwrap();
        let imu_data_accel = ImuData {
            x: accel_data.x(),
            y: accel_data.y(),
            z: accel_data.z(),
        };

        let gyro_data = sensor.gyro().await.unwrap();
        let imu_data_gyro = ImuData {
            x: gyro_data.x(),
            y: gyro_data.y(),
            z: gyro_data.z(),
        };

        let imu_measurement = ImuMeasurement {
            accel: imu_data_accel,
            gyro: imu_data_gyro,
        };

        send(Events::InertialMeasurementTaken(imu_measurement)).await;
    }
}
