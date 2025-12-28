//! IMU (Inertial Measurement Unit) reading functionality using the ICM20948 sensor.
//!
//! This module provides an asynchronous task that handles reading IMU data
//! from an ICM20948 9-axis sensor over I2C for motion control.
//!
//! # Architecture
//!
//! The module operates in two modes:
//! 1. Standby - not reading
//! 2. Active - continuous IMU readings at 100Hz with AHRS fusion
//!
//! # AHRS Filter Configuration
//!
//! The Madgwick filter is configured for an indoor tracked robot with:
//! - Beta = 0.15: Balanced filter for motor vibration rejection and responsiveness
//! - 100 Hz update rate: Fast enough for motion tracking, low enough to avoid noise amplification
//! - 9-axis fusion: Uses magnetometer for absolute heading (no yaw drift)
//!
//! For a small tracked robot (~30cm) with brushed motors:
//! - Vibration filtering is critical (DLPF settings)
//! - Moderate beta value balances convergence speed and noise rejection
//! - Magnetometer provides heading reference but may need calibration in magnetic environments
//!
//! # Usage
//!
//! ```rust
//! // Start IMU readings
//! imu_read::start_imu_readings();
//!
//! // Stop readings when done
//! imu_read::stop_imu_readings();
//! ```

use core::f32::consts::PI;

use ahrs::{Ahrs, Madgwick};
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_futures::select::{Either, select};
use embassy_rp::{
    Peri,
    peripherals::{PIN_3, PIN_8},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Delay, Duration, Instant, Timer};
use icm20948::{
    AccelConfig, AccelDlpf, AccelFullScale, GyroConfig, GyroDlpf, GyroFullScale, I2cInterface, Icm20948Driver,
    MagConfig, MagMode,
};
use micromath::F32Ext;
use nalgebra::{UnitQuaternion, Vector3};

use crate::{
    I2cBusShared,
    system::event::{Events, send_event},
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

/// 3D orientation using Euler angles (derived from quaternion via AHRS)
#[derive(Debug, Clone, Copy)]
pub struct Orientation {
    /// Rotation around vertical axis (turning left/right) in degrees
    /// Range: -180 to +180, or normalized to 0-360
    pub yaw: f32,
    /// Forward/backward tilt in degrees
    /// Positive = nose up, Negative = nose down
    pub pitch: f32,
    /// Left/right tilt in degrees
    /// Positive = right side down, Negative = left side down
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

/// Convert nalgebra UnitQuaternion to Euler angles (in degrees)
///
/// Uses the aerospace sequence (ZYX - yaw, pitch, roll) which is standard for vehicle orientation:
/// - Yaw (Z-axis): heading/compass direction
/// - Pitch (Y-axis): nose up/down
/// - Roll (X-axis): banking left/right
fn quaternion_to_euler(q: &UnitQuaternion<f32>) -> Orientation {
    // Extract quaternion components (w, x, y, z)
    let quat = q.quaternion();
    let w = quat.w;
    let x = quat.i;
    let y = quat.j;
    let z = quat.k;

    // Roll (x-axis rotation) - banking left/right
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = libm::atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation) - nose up/down
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if libm::fabsf(sinp) >= 1.0 {
        libm::copysignf(PI / 2.0, sinp) // Use 90 degrees if out of range
    } else {
        libm::asinf(sinp)
    };

    // Yaw (z-axis rotation) - heading/compass direction
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = libm::atan2f(siny_cosp, cosy_cosp);

    // Convert to degrees and round to 0.1 degree precision
    Orientation {
        roll: (roll * 180.0 / PI * 10.0).round() / 10.0,
        pitch: (pitch * 180.0 / PI * 10.0).round() / 10.0,
        yaw: (yaw * 180.0 / PI * 10.0).round() / 10.0,
    }
}

/// Embassy task that handles IMU measurements with 9-axis AHRS fusion
#[embassy_executor::task]
pub async fn inertial_measurement_read(
    i2c_bus: &'static I2cBusShared,
    _imu_int: Peri<'static, PIN_8>,
    _imu_add: Peri<'static, PIN_3>,
) {
    let i2c = I2cDevice::new(i2c_bus);

    // Initialize ICM20948
    info!("Initializing ICM20948...");
    let i2c_interface = I2cInterface::default(i2c);
    let mut sensor = match Icm20948Driver::new(i2c_interface).await {
        Ok(imu) => {
            info!("ICM20948 detected!");
            imu
        }
        Err(e) => {
            info!("Failed to detect ICM20948: {:?}", e);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    let mut delay = Delay;
    if let Err(e) = sensor.init(&mut delay).await {
        info!("Failed to initialize ICM20948: {:?}", e);
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }

    Timer::after(Duration::from_millis(100)).await;
    info!("ICM20948 initialized successfully!");

    // Configure accelerometer for tracked robot:
    // ±4g range: Handles bumps and shocks from tracked drive
    // 111 Hz DLPF: Good vibration filtering for brushed motors
    // ~100 Hz sample rate: Fast enough for motion control
    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G4,
        dlpf: AccelDlpf::Hz111,
        dlpf_enable: true,
        sample_rate_div: 10, // (1125 Hz / (1 + 10)) = ~102 Hz
    };

    if let Err(e) = sensor.configure_accelerometer(accel_config).await {
        info!("Failed to configure accelerometer: {:?}", e);
    } else {
        info!("Accelerometer: ±4g, 111Hz DLPF, ~102Hz sample rate");
    }

    // Configure gyroscope for tracked robot:
    // ±500°/s range: Captures fast rotations (tracks can spin quickly)
    // 51 Hz DLPF: Good vibration filtering for brushed motors
    // ~100 Hz sample rate: Matches accelerometer
    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps500,
        dlpf: GyroDlpf::Hz51,
        dlpf_enable: true,
        sample_rate_div: 10, // (1125 Hz / (1 + 10)) = ~102 Hz
    };

    if let Err(e) = sensor.configure_gyroscope(gyro_config).await {
        info!("Failed to configure gyroscope: {:?}", e);
    } else {
        info!("Gyroscope: ±500°/s, 51Hz DLPF, ~102Hz sample rate");
    }

    // Configure magnetometer for absolute heading:
    // Continuous 100Hz mode for smooth compass data
    // Provides absolute yaw reference (no drift)
    let mag_config = MagConfig {
        mode: MagMode::Continuous100Hz,
    };

    if let Err(e) = sensor.init_magnetometer(mag_config, &mut delay).await {
        info!("Failed to initialize magnetometer: {:?}", e);
    } else {
        info!("Magnetometer: Continuous 100Hz mode");
    }

    Timer::after(Duration::from_millis(100)).await;
    info!("All sensors configured!");

    // Initialize Madgwick filter for AHRS sensor fusion
    //
    // For a small indoor tracked robot with brushed motors:
    // - Beta = 0.15: Balanced setting
    //   * Higher than 0.1 = more responsive to changes
    //   * Lower than 0.3 = good vibration rejection
    //   * Good for tracked robots that make sudden direction changes
    // - 100 Hz sample rate: Standard for motion control
    //
    // The Madgwick filter fuses:
    // - Gyroscope: Fast response, accurate short-term, but drifts
    // - Accelerometer: Stable long-term gravity reference, but noisy
    // - Magnetometer: Absolute heading reference, prevents yaw drift
    const SAMPLE_RATE_HZ: f32 = 100.0;
    const BETA: f32 = 0.15; // Balanced for indoor tracked robot with motor vibrations
    let mut madgwick = Madgwick::new(1.0 / SAMPLE_RATE_HZ, BETA);

    info!("Madgwick AHRS filter initialized:");
    info!("  Sample rate: {} Hz", SAMPLE_RATE_HZ);
    info!("  Beta: {} (balanced for tracked robot)", BETA);
    info!("  Mode: 9-axis fusion (gyro + accel + mag)");

    // Gyro bias calibration
    info!("Calibrating gyroscope... Keep device stationary for 3 seconds!");
    Timer::after(Duration::from_secs(3)).await;

    let mut gyro_bias_x = 0.0f32;
    let mut gyro_bias_y = 0.0f32;
    let mut gyro_bias_z = 0.0f32;
    const CALIBRATION_SAMPLES: u32 = 100;

    for _ in 0..CALIBRATION_SAMPLES {
        if let Ok(gyro_data) = sensor.read_gyroscope().await {
            gyro_bias_x += gyro_data.x;
            gyro_bias_y += gyro_data.y;
            gyro_bias_z += gyro_data.z;
        }
        Timer::after(Duration::from_millis(10)).await;
    }

    gyro_bias_x /= CALIBRATION_SAMPLES as f32;
    gyro_bias_y /= CALIBRATION_SAMPLES as f32;
    gyro_bias_z /= CALIBRATION_SAMPLES as f32;

    info!("Gyro bias calibration complete:");
    info!("  X: {} °/s", gyro_bias_x);
    info!("  Y: {} °/s", gyro_bias_y);
    info!("  Z: {} °/s", gyro_bias_z);
    info!("");
    info!("Starting AHRS processing at {} Hz", SAMPLE_RATE_HZ);

    'command: loop {
        // Wait for a command, consuming it
        match IMU_CONTROL.wait().await {
            ImuCommand::Start => {
                // Enter reading loop
                info!("Starting IMU reading");

                loop {
                    match select(IMU_CONTROL.wait(), Timer::after(SAMPLE_INTERVAL)).await {
                        Either::First(ImuCommand::Stop) => {
                            info!("IMU stopped. Waiting for next command.");
                            continue 'command;
                        }
                        Either::First(_) => continue,
                        Either::Second(_) => {
                            // Read accelerometer (in g-force)
                            let accel_data = match sensor.read_accelerometer().await {
                                Ok(data) => Vector3::new(data.x, data.y, data.z),
                                Err(_) => continue,
                            };

                            // Read gyroscope (in degrees/second) and apply bias correction
                            let gyro_data = match sensor.read_gyroscope().await {
                                Ok(data) => data,
                                Err(_) => continue,
                            };

                            let gyro_x = gyro_data.x - gyro_bias_x;
                            let gyro_y = gyro_data.y - gyro_bias_y;
                            let gyro_z = gyro_data.z - gyro_bias_z;

                            // Convert gyroscope from degrees/s to radians/s for AHRS
                            let gyro_rad = Vector3::new(gyro_x * PI / 180.0, gyro_y * PI / 180.0, gyro_z * PI / 180.0);

                            // Read magnetometer (in μT)
                            let mag_data = match sensor.read_magnetometer().await {
                                Ok(data) => Vector3::new(data.x, data.y, data.z),
                                Err(_) => continue,
                            };

                            // Check if magnetometer reading is reasonable
                            // Typical Earth's magnetic field: 25-65 μT
                            // Allow wider range for poor calibration or magnetic interference
                            let mag_magnitude = mag_data.norm();
                            let use_magnetometer = mag_magnitude > 10.0 && mag_magnitude < 200.0;

                            // Update Madgwick filter
                            let result = if use_magnetometer {
                                // 9-axis fusion: gyro + accel + mag (absolute heading, no yaw drift)
                                madgwick.update(&gyro_rad, &accel_data, &mag_data)
                            } else {
                                // 6-axis IMU only: gyro + accel (yaw will drift over time)
                                madgwick.update_imu(&gyro_rad, &accel_data)
                            };

                            if let Ok(quat) = result {
                                // Convert quaternion to Euler angles
                                let orientation = quaternion_to_euler(quat);
                                let timestamp_ms = Instant::now().as_millis() as u32;

                                let imu_measurement = ImuMeasurement {
                                    orientation,
                                    timestamp_ms,
                                };

                                send_event(Events::ImuMeasurementTaken(imu_measurement)).await;
                            }
                        }
                    }
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
