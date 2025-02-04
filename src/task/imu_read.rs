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

use defmt::{info, Debug2Format};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Delay, Duration, Instant, Timer};
use mpu6050_dmp::{
    accel::{Accel, AccelFullScale},
    address::Address,
    config::DigitalLowPassFilter,
    gyro::{Gyro, GyroFullScale},
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

pub struct OrientationReference {
    initial_quaternion: Option<Quaternion>,
}

impl OrientationReference {
    pub fn new() -> Self {
        Self {
            initial_quaternion: None,
        }
    }

    pub fn initialize(&mut self, quat: Quaternion) {
        self.initial_quaternion = Some(quat);
    }

    pub fn get_relative_orientation(&self, current: Quaternion) -> YawPitchRoll {
        match self.initial_quaternion {
            None => YawPitchRoll::from(current),
            Some(initial) => {
                // Calculate relative quaternion
                // Manual quaternion multiplication with negated initial components
                let relative = Quaternion {
                    w: current.w * initial.w + current.x * initial.x + current.y * initial.y + current.z * initial.z,
                    x: current.w * (-initial.x) + current.x * initial.w + current.y * (-initial.z)
                        - current.z * (-initial.y),
                    y: current.w * (-initial.y) - current.x * (-initial.z)
                        + current.y * initial.w
                        + current.z * (-initial.x),
                    z: current.w * (-initial.z) + current.x * (-initial.y) - current.y * (-initial.x)
                        + current.z * initial.w,
                };
                YawPitchRoll::from(relative)
            }
        }
    }
}

/// Embassy task that handles IMU measurements
#[embassy_executor::task]
pub async fn inertial_measurement_read(i2c_bus: &'static I2c0BusShared) {
    let i2c = I2cDevice::new(i2c_bus);

    // Initialize MPU9250
    let mut sensor = Mpu6050::new(i2c, Address::default()).await.unwrap();
    let mut delay = Delay;
    sensor.initialize_dmp(&mut delay).await.unwrap();

    let filter = DigitalLowPassFilter::Filter2;
    sensor.set_digital_lowpass_filter(filter).await.unwrap();

    sensor.set_accel_full_scale(AccelFullScale::G2).await.unwrap();
    let accel_offsets = Accel::new(396, 324, 16616);
    sensor.set_accel_calibration(&accel_offsets).await.unwrap();

    sensor.set_gyro_full_scale(GyroFullScale::Deg2000).await.unwrap();
    let gyro_offsets = Gyro::new(3, 29, -8);
    sensor.set_gyro_calibration(&gyro_offsets).await.unwrap();

    sensor.set_sample_rate_divider(9).await.unwrap(); // 100Hz
    sensor.enable_fifo().await.unwrap();

    let mut buffer = [0u8; 28];

    // let mut yaw_filter = MovingMedian::<f32, 5>::new();
    // let mut pitch_filter = MovingMedian::<f32, 5>::new();
    // let mut roll_filter = MovingMedian::<f32, 5>::new();

    'command: loop {
        // Wait for a command, consuming it
        match IMU_CONTROL.wait().await {
            ImuCommand::Start => {
                // Enter reading loop
                info!("Starting IMU reading");

                let mut orientation_ref = OrientationReference::new();

                loop {
                    match select(IMU_CONTROL.wait(), sensor.get_fifo_count()).await {
                        Either::First(ImuCommand::Stop) => {
                            info!("IMU stopped. Waiting for next command.");
                            continue 'command;
                        }
                        Either::First(_) => continue,
                        Either::Second(fifo_count) => {
                            // we need a complete DMP packet (28 bytes)
                            if fifo_count.unwrap() < 28 {
                                continue;
                            }

                            if let Ok(fifo_data) = sensor.read_fifo(&mut buffer).await {
                                let timestamp_ms = Instant::now().as_millis() as u32;
                                let quat = Quaternion::from_bytes(&fifo_data[..16]).unwrap().normalize();

                                // Initialize reference on first reading
                                if orientation_ref.initial_quaternion.is_none() {
                                    orientation_ref.initialize(quat);
                                    info!("Initial orientation stored");
                                    continue;
                                }

                                // let ypr = YawPitchRoll::from(quat);
                                let ypr = orientation_ref.get_relative_orientation(quat);

                                info!("Quaternion: {}", Debug2Format(&quat));

                                // Convert to degrees
                                let yaw_deg = ypr.yaw * 180.0 / PI;
                                let pitch_deg = ypr.pitch * 180.0 / PI;
                                let roll_deg = ypr.roll * 180.0 / PI;
                                // yaw_filter.add_value(yaw_deg);
                                // pitch_filter.add_value(pitch_deg);
                                // roll_filter.add_value(roll_deg);

                                // Send event with IMU measurement
                                let orientation = Orientation {
                                    // yaw: yaw_filter.median(),
                                    // pitch: pitch_filter.median(),
                                    // roll: roll_filter.median(),
                                    yaw: yaw_deg,
                                    pitch: pitch_deg,
                                    roll: roll_deg,
                                };
                                info!("{}", Debug2Format(&orientation));
                                let imu_measurement = ImuMeasurement {
                                    orientation,
                                    timestamp_ms,
                                };
                                send_event(Events::ImuMeasurementTaken(imu_measurement)).await;
                            }
                        }
                    }
                    Timer::after(SAMPLE_INTERVAL).await;
                }
            }
            ImuCommand::Stop => {
                info!("IMU stopped. Waiting for next command.");
                // Return to waiting for next command
                continue 'command;
            }
        }
    }
}
