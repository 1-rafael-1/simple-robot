//! UI screen selection and menu helper utilities.

use crate::system::{
    event::RotaryDirection,
    state::{CalibrationSelection, DriveMode, MenuSelection, TestSelection},
};

/// Map a main menu index to its logical selection.
pub const fn menu_selection_from_index(index: usize) -> MenuSelection {
    match index {
        0 => MenuSelection::SystemInfo,
        1 => MenuSelection::Calibrate,
        2 => MenuSelection::DriveMode,
        _ => MenuSelection::TestMode,
    }
}

/// Map a drive mode submenu index to its drive mode.
/// Returns `None` when the selection is the Back entry.
pub const fn drive_mode_from_index(index: usize) -> Option<DriveMode> {
    match index {
        0 => Some(DriveMode::CoastAndAvoid),
        _ => None,
    }
}

/// Map a calibration menu index to its selection.
/// Returns `None` when the selection is the Back entry.
pub const fn calibration_selection_from_index(index: usize) -> Option<CalibrationSelection> {
    match index {
        0 => Some(CalibrationSelection::Motor),
        1 => Some(CalibrationSelection::Mag),
        2 => Some(CalibrationSelection::Accel),
        3 => Some(CalibrationSelection::Gyro),
        _ => None,
    }
}

/// Map a test menu index to its selection.
/// Returns `None` when the selection is the Back entry.
pub const fn test_selection_from_index(index: usize) -> Option<TestSelection> {
    match index {
        0 => Some(TestSelection::Combined),
        1 => Some(TestSelection::Imu),
        2 => Some(TestSelection::IrUltrasonic),
        _ => None,
    }
}

/// Human-readable label for a calibration selection.
pub const fn calibration_label(kind: CalibrationSelection) -> &'static str {
    match kind {
        CalibrationSelection::Motor => "Motor",
        CalibrationSelection::Mag => "Mag",
        CalibrationSelection::Accel => "Accel",
        CalibrationSelection::Gyro => "Gyro",
    }
}

/// Compute the next menu index, with wrap-around.
pub const fn next_menu_index(current: usize, len: usize, direction: RotaryDirection) -> usize {
    if len == 0 {
        return 0;
    }

    match direction {
        RotaryDirection::Clockwise => {
            if current == 0 {
                len - 1
            } else {
                current - 1
            }
        }
        RotaryDirection::CounterClockwise => (current + 1) % len,
    }
}

/// Compute the maximum scroll offset for the system info screen.
pub fn max_system_info_scroll(info: &super::screens::SystemInfoData) -> usize {
    let line_count = super::screens::system_info_line_count(info);
    line_count.saturating_sub(super::screens::DISPLAY_LINES)
}
