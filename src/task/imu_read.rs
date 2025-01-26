//! IMU (Inertial Measurement Unit) reading functionality using the MPU9250 sensor.
//!
//! This module provides an asynchronous task that handles reading IMU data
//! from an MPU9250 sensor over I2C for motion control.
//!
//! # Architecture
//!
//! The module operates in two modes:
//! 1. Standby - not reading
//! 2. Active - continuous IMU readings at 100Hz
//!
//! # Usage
//!
//! ```rust
//! // Start IMU readings
//! imu_read::start_readings();
//!
//! // Stop readings when done
//! imu_read::stop_readings();
//! ```

use core::f32::consts::PI;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Delay, Duration, Instant, Timer};
use mpu6050_dmp::{
    accel::AccelFullScale,
    address::Address,
    calibration::{CalibrationParameters, ReferenceGravity},
    gyro::GyroFullScale,
    quaternion::Quaternion,
    sensor_async::Mpu6050,
    yaw_pitch_roll::YawPitchRoll,
};

use crate::system::{
    event::{send_event, Events},
    resources::I2c0BusShared,
};

// Sampling configuration
const SAMPLE_INTERVAL: Duration = Duration::from_millis(10); // 100Hz sampling

/// Complete IMU measurement data
#[derive(Debug, Clone, Copy)]
pub struct ImuMeasurement {
    /// Orientation in 3D space
    pub orientation: Orientation,
    /// Timestamp of measurement in milliseconds
    pub timestamp_ms: u32,
}

/// 3D orientation using Euler angles (derived from quaternion)
#[derive(Debug, Clone, Copy)]
pub struct Orientation {
    /// Rotation around vertical axis (turning left/right) in degrees
    pub yaw: f32,
    /// Forward/backward tilt in degrees
    pub pitch: f32,
    /// Left/right tilt in degrees
    pub roll: f32,
}

/// Commands for IMU reading control
enum ImuCommand {
    /// Start IMU readings
    Start,
    /// Stop IMU readings
    Stop,
}

/// Control signal for IMU reading state
static IMU_CONTROL: Signal<CriticalSectionRawMutex, ImuCommand> = Signal::new();

/// Start continuous IMU readings
pub fn start_imu_readings() {
    IMU_CONTROL.signal(ImuCommand::Start);
}

/// Stop IMU readings
pub fn stop_imu_readings() {
    IMU_CONTROL.signal(ImuCommand::Stop);
}

/// Embassy task that handles IMU measurements
#[embassy_executor::task]
pub async fn inertial_measurement_handle(i2c_bus: &'static I2c0BusShared) {
    let i2c = I2cDevice::new(i2c_bus);

    // Initialize MPU9250
    let mut sensor = Mpu6050::new(i2c, Address::default()).await.unwrap();
    let mut delay = Delay;
    sensor.initialize_dmp(&mut delay).await.unwrap();

    let calibration_params =
        CalibrationParameters::new(AccelFullScale::G2, GyroFullScale::Deg500, ReferenceGravity::YP);

    sensor.set_sample_rate_divider(9).await.unwrap(); // 100Hz
    sensor.enable_fifo().await.unwrap();
    sensor.calibrate(&mut delay, &calibration_params).await.unwrap();

    let mut buffer = [0u8; 28];

    // Configure for appropriate ranges and filtering
    // TODO: Add specific MPU9250 configuration/calibration here

    'command: loop {
        // Wait for a command, consuming it
        match IMU_CONTROL.wait().await {
            ImuCommand::Start => {
                // Enter reading loop
                'read: loop {
                    // Check if we should read the IMU_CONTROL signal again because we possibly have a new command
                    if IMU_CONTROL.signaled() {
                        break 'read;
                    }

                    // we need a complete DMP packet (28 bytes)
                    let fifo_count = sensor.get_fifo_count().await.unwrap();
                    if fifo_count < 28 {
                        continue 'read;
                    }

                    if let Ok(fifo_data) = sensor.read_fifo(&mut buffer).await {
                        let timestamp_ms = Instant::now().as_millis() as u32;
                        let quat = Quaternion::from_bytes(&fifo_data[..16]).unwrap().normalize();
                        let ypr = YawPitchRoll::from(quat);

                        // Convert to degrees
                        let yaw_deg = ypr.yaw * 180.0 / PI;
                        let pitch_deg = ypr.pitch * 180.0 / PI;
                        let roll_deg = ypr.roll * 180.0 / PI;

                        // Send event with IMU measurement
                        let orientation = Orientation {
                            yaw: yaw_deg,
                            pitch: pitch_deg,
                            roll: roll_deg,
                        };
                        let imu_measurement = ImuMeasurement {
                            orientation,
                            timestamp_ms,
                        };
                        send_event(Events::ImuMeasurementTaken(imu_measurement)).await;
                    }
                    Timer::after(SAMPLE_INTERVAL).await;
                }
            }
            ImuCommand::Stop => {
                // Return to waiting for next command
                continue 'command;
            }
        }
    }
}
