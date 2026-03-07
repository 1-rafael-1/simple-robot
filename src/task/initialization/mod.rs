//! System initialization and calibration coordination.
//!
//! Orchestrates boot-time setup, calibration loading, and related UI updates.

use core::fmt::Write;

use defmt::info;
use heapless::String;

use crate::{
    system::state::{CalibrationStatus, calibration},
    task::{
        io::{display, flash_storage},
        motor_driver,
        sensors::imu,
        ui,
    },
};

/// Handle system initialization.
pub async fn handle_initialize() {
    info!("System initializing");

    // Display initialization message.
    display::display_update(display::DisplayAction::Clear).await;
    let mut txt: String<20> = String::new();
    let _ = write!(txt, "Initializing...");
    display::display_update(display::DisplayAction::ShowText(txt, 0)).await;

    // Request motor calibration from flash.
    info!("Requesting motor calibration from flash");
    flash_storage::send_flash_command(flash_storage::FlashCommand::GetData(
        flash_storage::CalibrationKind::Motor,
    ))
    .await;

    // Request IMU calibration from flash.
    info!("Requesting IMU calibration from flash");
    flash_storage::send_flash_command(flash_storage::FlashCommand::GetData(
        flash_storage::CalibrationKind::Imu,
    ))
    .await;

    // Request IMU calibration flags from flash.
    info!("Requesting IMU calibration flags from flash");
    flash_storage::send_flash_command(flash_storage::FlashCommand::GetImuFlags).await;

    // Note: initialization completes when calibration data arrives via events.
}

/// Handle calibration data loaded from flash storage.
pub async fn handle_calibration_data_loaded(
    kind: flash_storage::CalibrationKind,
    data: Option<flash_storage::CalibrationDataKind>,
) {
    use flash_storage::{CalibrationDataKind, CalibrationKind};

    match kind {
        CalibrationKind::Motor => {
            if let Some(CalibrationDataKind::Motor(motor_cal)) = data {
                info!(
                    "Motor calibration loaded: [{}, {}, {}, {}]",
                    motor_cal.left_front, motor_cal.left_rear, motor_cal.right_front, motor_cal.right_rear
                );

                {
                    let mut state = calibration::CALIBRATION_STATE.lock().await;
                    state.motor_calibration_status = CalibrationStatus::Loaded;
                }

                motor_driver::send_motor_command(motor_driver::MotorCommand::LoadCalibration(motor_cal)).await;

                let mut txt: String<20> = String::new();
                let _ = write!(txt, "Calibration loaded");
                display::display_update(display::DisplayAction::ShowText(txt, 1)).await;
            } else {
                info!("No motor calibration found - using defaults");

                {
                    let mut state = calibration::CALIBRATION_STATE.lock().await;
                    state.motor_calibration_status = CalibrationStatus::NotAvailable;
                }

                let mut txt: String<20> = String::new();
                let _ = write!(txt, "Need motor calib");
                display::display_update(display::DisplayAction::ShowText(txt, 1)).await;
            }
        }
        CalibrationKind::Imu => {
            if let Some(CalibrationDataKind::Imu(imu_cal)) = data {
                info!("IMU calibration loaded from flash");

                {
                    let mut state = calibration::CALIBRATION_STATE.lock().await;
                    state.imu_calibration_status = CalibrationStatus::Loaded;
                }

                imu::load_imu_calibration(imu_cal);

                let mut txt: String<20> = String::new();
                let _ = write!(txt, "IMU cal loaded");
                display::display_update(display::DisplayAction::ShowText(txt, 2)).await;
            } else {
                info!("No IMU calibration found - using defaults");

                {
                    let mut state = calibration::CALIBRATION_STATE.lock().await;
                    state.imu_calibration_status = CalibrationStatus::NotAvailable;
                    state.mag_calibration_status = CalibrationStatus::NotAvailable;
                    state.accel_calibration_status = CalibrationStatus::NotAvailable;
                    state.gyro_calibration_status = CalibrationStatus::NotAvailable;
                }

                let mut txt: String<20> = String::new();
                let _ = write!(txt, "Need IMU calib");
                display::display_update(display::DisplayAction::ShowText(txt, 2)).await;
            }
        }
    }

    check_initialization_complete().await;
}

/// Handle IMU calibration flags loaded from flash.
pub async fn handle_imu_calibration_flags_loaded(flags: Option<flash_storage::ImuCalibrationFlags>) {
    {
        let mut state = calibration::CALIBRATION_STATE.lock().await;
        if let Some(flags) = flags {
            state.gyro_calibration_status = if flags.gyro {
                CalibrationStatus::Loaded
            } else {
                CalibrationStatus::NotAvailable
            };
            state.accel_calibration_status = if flags.accel {
                CalibrationStatus::Loaded
            } else {
                CalibrationStatus::NotAvailable
            };
            state.mag_calibration_status = if flags.mag {
                CalibrationStatus::Loaded
            } else {
                CalibrationStatus::NotAvailable
            };
            state.imu_calibration_status = if flags.gyro && flags.accel && flags.mag {
                CalibrationStatus::Loaded
            } else {
                CalibrationStatus::NotAvailable
            };
        } else {
            state.imu_calibration_status = CalibrationStatus::NotAvailable;
            state.mag_calibration_status = CalibrationStatus::NotAvailable;
            state.accel_calibration_status = CalibrationStatus::NotAvailable;
            state.gyro_calibration_status = CalibrationStatus::NotAvailable;
        }
    }

    if ui::ui_initialized().await {
        ui::refresh().await;
    }

    check_initialization_complete().await;
}

/// Check if initialization is complete and update display if so.
async fn check_initialization_complete() {
    let should_show_menu = {
        let state = calibration::CALIBRATION_STATE.lock().await;

        if state.is_initialized() {
            info!("System initialization complete");
            info!("  Motor calibration: {:?}", state.motor_calibration_status);
            info!("  IMU calibration: {:?}", state.imu_calibration_status);
            true
        } else {
            false
        }
    };

    if should_show_menu && !ui::ui_is_calibrating().await {
        ui::show_main_menu().await;
    }
}

/// Handle calibration status updates.
pub async fn handle_calibration_status(
    header: Option<heapless::String<20>>,
    line1: Option<heapless::String<20>>,
    line2: Option<heapless::String<20>>,
    line3: Option<heapless::String<20>>,
) {
    if let Some(text) = header {
        display::display_update(display::DisplayAction::ShowText(text, 0)).await;
    }
    if let Some(text) = line1 {
        display::display_update(display::DisplayAction::ShowText(text, 1)).await;
    }
    if let Some(text) = line2 {
        display::display_update(display::DisplayAction::ShowText(text, 2)).await;
    }
    if let Some(text) = line3 {
        display::display_update(display::DisplayAction::ShowText(text, 3)).await;
    }
}
