//! UI menu helpers and selection mapping.

use crate::system::{
    event::RotaryDirection,
    state::{CalibrationSelection, MenuSelection},
};

/// Map a main menu index to its logical selection.
pub const fn menu_selection_from_index(index: usize) -> MenuSelection {
    match index {
        0 => MenuSelection::SystemInfo,
        1 => MenuSelection::Calibrate,
        _ => MenuSelection::TestMode,
    }
}

/// Map a calibration menu index to its selection.
pub const fn calibration_selection_from_index(index: usize) -> CalibrationSelection {
    match index {
        0 => CalibrationSelection::Motor,
        1 => CalibrationSelection::Mag,
        2 => CalibrationSelection::Accel,
        _ => CalibrationSelection::Gyro,
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
        RotaryDirection::Clockwise => (current + 1) % len,
        RotaryDirection::CounterClockwise => {
            if current == 0 {
                len - 1
            } else {
                current - 1
            }
        }
    }
}

/// Compute the maximum scroll offset for the system info screen.
pub fn max_system_info_scroll(info: &super::ui_menu::SystemInfoData) -> usize {
    let line_count = super::ui_menu::system_info_line_count(info);
    line_count.saturating_sub(super::ui_menu::DISPLAY_LINES)
}
