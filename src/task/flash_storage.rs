//! Flash storage task for persistent calibration data
//!
//! This module provides a task that manages storing and retrieving calibration data
//! to/from flash memory using the `sequential-storage` crate. It handles both motor
//! calibration and IMU calibration data.
//!
//! The data is stored in a reserved section of flash memory defined in memory.x

use defmt::*;
use embassy_rp::flash::{Async, ERASE_SIZE, Flash};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use embedded_storage_async::nor_flash::NorFlash;
use sequential_storage::{
    cache::NoCache,
    map::{Key, SerializationError, Value, fetch_item, store_item},
};

use crate::task::motor_driver::{MotorCalibration, MotorCommand, send_motor_command};

/// Size of one flash sector (4KB on RP2350)
const FLASH_SECTOR_SIZE: usize = ERASE_SIZE;

/// Number of sectors to use for storage (2 sectors for wear leveling)
const STORAGE_SECTOR_COUNT: usize = 2;

/// Total storage size
const STORAGE_SIZE: usize = FLASH_SECTOR_SIZE * STORAGE_SECTOR_COUNT;

/// Flash storage offset from the end of flash (last 8KB = 2 sectors)
/// This should match the settings in memory.x
const STORAGE_OFFSET: u32 = 2048 * 1024 - STORAGE_SIZE as u32;

/// Size of the command queue
const COMMAND_QUEUE_SIZE: usize = 4;

/// Channel for sending flash storage commands
static FLASH_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, FlashCommand, COMMAND_QUEUE_SIZE> = Channel::new();

/// Static storage for calibration data (shared between tasks)
static CALIBRATION_DATA: embassy_sync::mutex::Mutex<CriticalSectionRawMutex, Option<CalibrationData>> =
    embassy_sync::mutex::Mutex::new(None);

/// Send a flash storage command
pub async fn send_flash_command(command: FlashCommand) {
    FLASH_COMMAND_CHANNEL.send(command).await;
}

/// Try to send a flash storage command without blocking
pub fn try_send_flash_command(command: FlashCommand) -> bool {
    FLASH_COMMAND_CHANNEL.try_send(command).is_ok()
}

/// Receive a flash storage command
async fn receive_flash_command() -> FlashCommand {
    FLASH_COMMAND_CHANNEL.receive().await
}

/// Commands that can be sent to the flash storage task
#[derive(Debug, Clone, Format)]
pub enum FlashCommand {
    /// Save motor calibration data to flash
    SaveMotor(MotorCalibration),

    /// Save IMU calibration data to flash
    SaveImu(ImuCalibration),

    /// Load all calibration data from flash and apply to motor driver
    Load,

    /// Erase all stored calibration data
    Erase,
}

/// IMU calibration data structure
#[derive(Debug, Clone, Copy, Format, Default)]
pub struct ImuCalibration {
    /// Gyroscope X-axis bias (rad/s)
    pub gyro_x_bias: f32,
    /// Gyroscope Y-axis bias (rad/s)
    pub gyro_y_bias: f32,
    /// Gyroscope Z-axis bias (rad/s)
    pub gyro_z_bias: f32,

    /// Accelerometer X-axis bias (m/s²)
    pub accel_x_bias: f32,
    /// Accelerometer Y-axis bias (m/s²)
    pub accel_y_bias: f32,
    /// Accelerometer Z-axis bias (m/s²)
    pub accel_z_bias: f32,

    /// Magnetometer X-axis bias
    pub mag_x_bias: f32,
    /// Magnetometer Y-axis bias
    pub mag_y_bias: f32,
    /// Magnetometer Z-axis bias
    pub mag_z_bias: f32,
}

/// Combined calibration data
#[derive(Debug, Clone, Copy, Format, Default)]
pub struct CalibrationData {
    pub motor: MotorCalibration,
    pub imu: ImuCalibration,
}

/// Storage keys for sequential-storage
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
enum StorageKey {
    MotorCalibration = 0,
    ImuCalibration = 1,
}

impl Key for StorageKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0] = *self as u8;
        Ok(1)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized,
    {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        match buffer[0] {
            0 => Ok((StorageKey::MotorCalibration, 1)),
            1 => Ok((StorageKey::ImuCalibration, 1)),
            _ => Err(SerializationError::InvalidFormat),
        }
    }
}

/// Serialize motor calibration to bytes
impl Value<'_> for MotorCalibration {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 16 {
            return Err(SerializationError::BufferTooSmall);
        }

        buffer[0..4].copy_from_slice(&self.left_front.to_le_bytes());
        buffer[4..8].copy_from_slice(&self.left_rear.to_le_bytes());
        buffer[8..12].copy_from_slice(&self.right_front.to_le_bytes());
        buffer[12..16].copy_from_slice(&self.right_rear.to_le_bytes());

        Ok(16)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<Self, SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < 16 {
            return Err(SerializationError::BufferTooSmall);
        }

        let left_front = f32::from_le_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]);
        let left_rear = f32::from_le_bytes([buffer[4], buffer[5], buffer[6], buffer[7]]);
        let right_front = f32::from_le_bytes([buffer[8], buffer[9], buffer[10], buffer[11]]);
        let right_rear = f32::from_le_bytes([buffer[12], buffer[13], buffer[14], buffer[15]]);

        Ok(MotorCalibration::new(left_front, left_rear, right_front, right_rear))
    }
}

/// Serialize IMU calibration to bytes
impl Value<'_> for ImuCalibration {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 36 {
            return Err(SerializationError::BufferTooSmall);
        }

        buffer[0..4].copy_from_slice(&self.gyro_x_bias.to_le_bytes());
        buffer[4..8].copy_from_slice(&self.gyro_y_bias.to_le_bytes());
        buffer[8..12].copy_from_slice(&self.gyro_z_bias.to_le_bytes());
        buffer[12..16].copy_from_slice(&self.accel_x_bias.to_le_bytes());
        buffer[16..20].copy_from_slice(&self.accel_y_bias.to_le_bytes());
        buffer[20..24].copy_from_slice(&self.accel_z_bias.to_le_bytes());
        buffer[24..28].copy_from_slice(&self.mag_x_bias.to_le_bytes());
        buffer[28..32].copy_from_slice(&self.mag_y_bias.to_le_bytes());
        buffer[32..36].copy_from_slice(&self.mag_z_bias.to_le_bytes());

        Ok(36)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<Self, SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < 36 {
            return Err(SerializationError::BufferTooSmall);
        }

        Ok(ImuCalibration {
            gyro_x_bias: f32::from_le_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]),
            gyro_y_bias: f32::from_le_bytes([buffer[4], buffer[5], buffer[6], buffer[7]]),
            gyro_z_bias: f32::from_le_bytes([buffer[8], buffer[9], buffer[10], buffer[11]]),
            accel_x_bias: f32::from_le_bytes([buffer[12], buffer[13], buffer[14], buffer[15]]),
            accel_y_bias: f32::from_le_bytes([buffer[16], buffer[17], buffer[18], buffer[19]]),
            accel_z_bias: f32::from_le_bytes([buffer[20], buffer[21], buffer[22], buffer[23]]),
            mag_x_bias: f32::from_le_bytes([buffer[24], buffer[25], buffer[26], buffer[27]]),
            mag_y_bias: f32::from_le_bytes([buffer[28], buffer[29], buffer[30], buffer[31]]),
            mag_z_bias: f32::from_le_bytes([buffer[32], buffer[33], buffer[34], buffer[35]]),
        })
    }
}

/// Get current calibration data
pub async fn get_calibration() -> CalibrationData {
    CALIBRATION_DATA.lock().await.unwrap_or_default()
}

/// Flash storage task
///
/// This task handles all flash read/write operations for calibration data.
/// It responds to commands sent via the command channel and uses sequential-storage
/// for wear leveling and data integrity.
#[embassy_executor::task]
pub async fn flash_storage(mut flash: Flash<'static, embassy_rp::peripherals::FLASH, Async, { 2048 * 1024 }>) {
    info!("Flash storage task started");

    // Define flash range for storage
    let flash_range = STORAGE_OFFSET..(STORAGE_OFFSET + STORAGE_SIZE as u32);

    // Try to load existing calibration on startup
    let mut cache = NoCache::new();

    match fetch_item::<StorageKey, MotorCalibration, _>(
        &mut flash,
        flash_range.clone(),
        &mut cache,
        &mut [],
        &StorageKey::MotorCalibration,
    )
    .await
    {
        Ok(Some(motor_cal)) => {
            info!("Loaded motor calibration from flash");
            info!(
                "  LF={}, LR={}, RF={}, RR={}",
                motor_cal.left_front, motor_cal.left_rear, motor_cal.right_front, motor_cal.right_rear
            );

            // Update shared state
            let mut data = CALIBRATION_DATA.lock().await;
            if let Some(ref mut cal) = *data {
                cal.motor = motor_cal;
            } else {
                *data = Some(CalibrationData {
                    motor: motor_cal,
                    imu: ImuCalibration::default(),
                });
            }

            // Apply to motor driver
            send_motor_command(MotorCommand::UpdateAllCalibration {
                left_front: motor_cal.left_front,
                left_rear: motor_cal.left_rear,
                right_front: motor_cal.right_front,
                right_rear: motor_cal.right_rear,
            })
            .await;
        }
        Ok(None) => {
            info!("No motor calibration found, using defaults");
        }
        Err(e) => {
            error!("Failed to load motor calibration: {}", defmt::Debug2Format(&e));
        }
    }

    match fetch_item::<StorageKey, ImuCalibration, _>(
        &mut flash,
        flash_range.clone(),
        &mut cache,
        &mut [],
        &StorageKey::ImuCalibration,
    )
    .await
    {
        Ok(Some(imu_cal)) => {
            info!("Loaded IMU calibration from flash");
            let mut data = CALIBRATION_DATA.lock().await;
            if let Some(ref mut cal) = *data {
                cal.imu = imu_cal;
            } else {
                *data = Some(CalibrationData {
                    motor: MotorCalibration::default(),
                    imu: imu_cal,
                });
            }
        }
        Ok(None) => {
            info!("No IMU calibration found, using defaults");
        }
        Err(e) => {
            error!("Failed to load IMU calibration: {}", defmt::Debug2Format(&e));
        }
    }

    // Main command processing loop
    loop {
        let command = receive_flash_command().await;
        debug!("Flash command received: {:?}", command);

        match command {
            FlashCommand::SaveMotor(motor_cal) => {
                info!("Saving motor calibration to flash...");

                // Update shared state first
                let mut data = CALIBRATION_DATA.lock().await;
                if let Some(ref mut cal) = *data {
                    cal.motor = motor_cal;
                } else {
                    *data = Some(CalibrationData {
                        motor: motor_cal,
                        imu: ImuCalibration::default(),
                    });
                }
                drop(data);

                match store_item(
                    &mut flash,
                    flash_range.clone(),
                    &mut cache,
                    &mut [],
                    &StorageKey::MotorCalibration,
                    &motor_cal,
                )
                .await
                {
                    Ok(_) => {
                        info!("Motor calibration saved successfully");
                    }
                    Err(e) => {
                        error!("Failed to save motor calibration: {}", defmt::Debug2Format(&e));
                    }
                }
            }

            FlashCommand::SaveImu(imu_cal) => {
                info!("Saving IMU calibration to flash...");

                // Update shared state first
                let mut data = CALIBRATION_DATA.lock().await;
                if let Some(ref mut cal) = *data {
                    cal.imu = imu_cal;
                } else {
                    *data = Some(CalibrationData {
                        motor: MotorCalibration::default(),
                        imu: imu_cal,
                    });
                }
                drop(data);

                match store_item(
                    &mut flash,
                    flash_range.clone(),
                    &mut cache,
                    &mut [],
                    &StorageKey::ImuCalibration,
                    &imu_cal,
                )
                .await
                {
                    Ok(_) => {
                        info!("IMU calibration saved successfully");
                    }
                    Err(e) => {
                        error!("Failed to save IMU calibration: {}", defmt::Debug2Format(&e));
                    }
                }
            }

            FlashCommand::Load => {
                info!("Loading calibration from flash...");

                match fetch_item::<StorageKey, MotorCalibration, _>(
                    &mut flash,
                    flash_range.clone(),
                    &mut cache,
                    &mut [],
                    &StorageKey::MotorCalibration,
                )
                .await
                {
                    Ok(Some(motor_cal)) => {
                        info!("Motor calibration reloaded");
                        let mut data = CALIBRATION_DATA.lock().await;
                        if let Some(ref mut cal) = *data {
                            cal.motor = motor_cal;
                        } else {
                            *data = Some(CalibrationData {
                                motor: motor_cal,
                                imu: ImuCalibration::default(),
                            });
                        }
                        drop(data);

                        // Apply to motor driver
                        send_motor_command(MotorCommand::UpdateAllCalibration {
                            left_front: motor_cal.left_front,
                            left_rear: motor_cal.left_rear,
                            right_front: motor_cal.right_front,
                            right_rear: motor_cal.right_rear,
                        })
                        .await;
                    }
                    Ok(None) => {
                        warn!("No motor calibration to reload");
                    }
                    Err(e) => {
                        error!("Failed to reload motor calibration: {}", defmt::Debug2Format(&e));
                    }
                }

                match fetch_item::<StorageKey, ImuCalibration, _>(
                    &mut flash,
                    flash_range.clone(),
                    &mut cache,
                    &mut [],
                    &StorageKey::ImuCalibration,
                )
                .await
                {
                    Ok(Some(imu_cal)) => {
                        info!("IMU calibration loaded");
                        let mut data = CALIBRATION_DATA.lock().await;
                        if let Some(ref mut cal) = *data {
                            cal.imu = imu_cal;
                        } else {
                            *data = Some(CalibrationData {
                                motor: MotorCalibration::default(),
                                imu: imu_cal,
                            });
                        }
                    }
                    Ok(None) => {
                        warn!("No IMU calibration to reload");
                    }
                    Err(e) => {
                        error!("Failed to reload IMU calibration: {}", defmt::Debug2Format(&e));
                    }
                }
            }

            FlashCommand::Erase => {
                info!("Erasing all calibration data...");

                // Erase the flash range
                let start_addr = flash_range.start;
                let end_addr = flash_range.end;

                match flash.erase(start_addr, end_addr).await {
                    Ok(_) => {
                        info!("Calibration erased successfully");

                        // Reset to defaults
                        let defaults = CalibrationData::default();
                        *CALIBRATION_DATA.lock().await = Some(defaults);
                    }
                    Err(e) => {
                        error!("Failed to erase calibration: {}", defmt::Debug2Format(&e));
                    }
                }
            }
        }

        // Small delay to prevent tight loop
        Timer::after(Duration::from_millis(10)).await;
    }
}

/// Get the flash storage offset (for debugging)
pub fn get_storage_offset() -> u32 {
    STORAGE_OFFSET
}

/// Get the storage size (for debugging)
pub fn get_storage_size() -> usize {
    STORAGE_SIZE
}
