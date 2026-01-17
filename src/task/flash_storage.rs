//! Flash storage task for persistent calibration data
//!
//! This module provides a task that manages storing and retrieving calibration data
//! to/from flash memory using the `sequential-storage` crate. It handles both motor
//! calibration and IMU calibration data.
//!
//! The data is stored in a reserved section of flash memory defined in memory.x

use defmt::*;
use embassy_rp::flash::{Async, ERASE_SIZE, Flash};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal};
use embassy_time::{Duration, Timer};
use embedded_storage_async::nor_flash::NorFlash;
use sequential_storage::{
    cache::NoCache,
    map::{Key, SerializationError, Value, fetch_item, store_item},
};

use crate::{
    system::event::{Events, send_event},
    task::motor_driver::MotorCalibration,
};

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
const COMMAND_QUEUE_SIZE: usize = 3;

/// Channel for sending flash storage commands
static FLASH_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, FlashCommand, COMMAND_QUEUE_SIZE> = Channel::new();

/// Signal for returning motor calibration data
static MOTOR_CALIBRATION_SIGNAL: Signal<CriticalSectionRawMutex, MotorCalibration> = Signal::new();

/// Signal for returning IMU calibration data
static IMU_CALIBRATION_SIGNAL: Signal<CriticalSectionRawMutex, ImuCalibration> = Signal::new();

/// Internal storage for calibration data (managed by flash_storage task only)
static CALIBRATION_DATA: embassy_sync::mutex::Mutex<CriticalSectionRawMutex, Option<CalibrationData>> =
    embassy_sync::mutex::Mutex::new(None);

/// Send a flash storage command
pub async fn send_flash_command(command: FlashCommand) {
    FLASH_COMMAND_CHANNEL.send(command).await;
}

/// Receive a flash storage command
async fn receive_flash_command() -> FlashCommand {
    FLASH_COMMAND_CHANNEL.receive().await
}

/// Type of calibration data
#[derive(Debug, Clone, Copy, Format, PartialEq, Eq)]
pub enum CalibrationKind {
    /// Motor calibration data
    Motor,
    /// IMU calibration data
    Imu,
}

/// Calibration data variants
#[derive(Debug, Clone, Copy, Format)]
pub enum CalibrationDataKind {
    Motor(MotorCalibration),
    Imu(ImuCalibration),
}

/// Commands that can be sent to the flash storage task
#[derive(Debug, Clone, Format)]
pub enum FlashCommand {
    /// Save calibration data to flash
    SaveData(CalibrationDataKind),

    /// Request calibration data (responds via signal)
    GetData(CalibrationKind),

    /// Load all calibration data from flash and apply to motor driver
    LoadAll,

    /// Erase all stored calibration data
    EraseAll,
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

/// Receive motor calibration signal response
///
/// This waits for the motor calibration signal from the flash storage task.
/// The signal is automatically cleared before waiting to ensure fresh data.
/// This is safe because the flash storage task is the only one that modifies the data.
///
/// # Example
/// ```rust
/// send_flash_command(FlashCommand::GetData(CalibrationKind::Motor)).await;
/// let motor_cal = receive_motor_calibration_signal().await;
/// ```
pub async fn receive_motor_calibration_signal() -> MotorCalibration {
    MOTOR_CALIBRATION_SIGNAL.wait().await
}

/// Receive IMU calibration signal response
///
/// This waits for the IMU calibration signal from the flash storage task.
/// The signal is automatically cleared before waiting to ensure fresh data.
/// This is safe because the flash storage task is the only one that modifies the data.
///
/// # Example
/// ```rust
/// send_flash_command(FlashCommand::GetData(CalibrationKind::Imu)).await;
/// let imu_cal = receive_imu_calibration_signal().await;
/// ```
pub async fn receive_imu_calibration_signal() -> ImuCalibration {
    IMU_CALIBRATION_SIGNAL.wait().await
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

    // Create cache for flash operations
    let mut cache = NoCache::new();

    // Create scratch buffer for serialization/deserialization
    // Motor calibration needs 16 bytes, IMU calibration needs 36 bytes
    // Using 128 bytes to be safe and align with common practice
    let mut data_buffer: [u8; 128] = [0; 128];

    // Main command processing loop - wait for orchestrator to request calibration loading
    loop {
        let command = receive_flash_command().await;
        debug!("Flash command received: {:?}", command);

        match command {
            FlashCommand::GetData(kind) => match kind {
                CalibrationKind::Motor => {
                    info!("Loading motor calibration from flash...");

                    match fetch_item::<StorageKey, MotorCalibration, _>(
                        &mut flash,
                        flash_range.clone(),
                        &mut cache,
                        &mut data_buffer,
                        &StorageKey::MotorCalibration,
                    )
                    .await
                    {
                        Ok(Some(motor_cal)) => {
                            info!(
                                "Motor calibration loaded: LF={}, LR={}, RF={}, RR={}",
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
                            drop(data);

                            // Send event with calibration data
                            send_event(Events::CalibrationDataLoaded(
                                CalibrationKind::Motor,
                                Some(CalibrationDataKind::Motor(motor_cal)),
                            ))
                            .await;
                        }
                        Ok(None) => {
                            info!("No motor calibration found in flash");
                            // Send event with None to indicate no data
                            send_event(Events::CalibrationDataLoaded(CalibrationKind::Motor, None)).await;
                        }
                        Err(e) => {
                            error!("Failed to load motor calibration: {}", defmt::Debug2Format(&e));
                            // Send event with None to indicate failure
                            send_event(Events::CalibrationDataLoaded(CalibrationKind::Motor, None)).await;
                        }
                    }
                }
                CalibrationKind::Imu => {
                    info!("Loading IMU calibration from flash...");

                    match fetch_item::<StorageKey, ImuCalibration, _>(
                        &mut flash,
                        flash_range.clone(),
                        &mut cache,
                        &mut data_buffer,
                        &StorageKey::ImuCalibration,
                    )
                    .await
                    {
                        Ok(Some(imu_cal)) => {
                            info!("IMU calibration loaded from flash");

                            // Update shared state
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

                            // Send event with calibration data
                            send_event(Events::CalibrationDataLoaded(
                                CalibrationKind::Imu,
                                Some(CalibrationDataKind::Imu(imu_cal)),
                            ))
                            .await;
                        }
                        Ok(None) => {
                            info!("No IMU calibration found in flash");
                            // Send event with None to indicate no data
                            send_event(Events::CalibrationDataLoaded(CalibrationKind::Imu, None)).await;
                        }
                        Err(e) => {
                            error!("Failed to load IMU calibration: {}", defmt::Debug2Format(&e));
                            // Send event with None to indicate failure
                            send_event(Events::CalibrationDataLoaded(CalibrationKind::Imu, None)).await;
                        }
                    }
                }
            },

            FlashCommand::SaveData(data_kind) => {
                match data_kind {
                    CalibrationDataKind::Motor(motor_cal) => {
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
                            &mut data_buffer,
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
                    CalibrationDataKind::Imu(imu_cal) => {
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
                            &mut data_buffer,
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
                }
            }

            FlashCommand::LoadAll => {
                info!("Loading calibration from flash...");

                match fetch_item::<StorageKey, MotorCalibration, _>(
                    &mut flash,
                    flash_range.clone(),
                    &mut cache,
                    &mut data_buffer,
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

                        // Send event to notify orchestrator
                        send_event(Events::CalibrationDataLoaded(
                            CalibrationKind::Motor,
                            Some(CalibrationDataKind::Motor(motor_cal)),
                        ))
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
                    &mut data_buffer,
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
                        drop(data);

                        // Send event to notify orchestrator
                        send_event(Events::CalibrationDataLoaded(
                            CalibrationKind::Imu,
                            Some(CalibrationDataKind::Imu(imu_cal)),
                        ))
                        .await;
                    }
                    Ok(None) => {
                        warn!("No IMU calibration to reload");
                    }
                    Err(e) => {
                        error!("Failed to reload IMU calibration: {}", defmt::Debug2Format(&e));
                    }
                }
            }

            FlashCommand::EraseAll => {
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
