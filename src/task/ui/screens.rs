//! UI screen rendering helpers
//!
//! This module provides small, reusable helpers to render UI screens
//! and system info on the OLED via the display task.

use core::fmt::Write;

use heapless::{String, Vec};

use crate::{
    system::state::CalibrationStatus,
    task::io::display::{DisplayAction, TextStyle, display_update},
};

/// Maximum number of text lines supported by the OLED layout.
pub const DISPLAY_LINES: usize = 4;

/// Maximum line length supported by the display text contract.
pub const MAX_LINE_LEN: usize = 20;

/// Main menu entries.
pub const MAIN_MENU_ITEMS: [&str; 4] = ["System Info", "Calibrate", "Drive Mode", "Test Mode"];

/// Drive mode submenu entries.
pub const DRIVE_MODE_MENU_ITEMS: [&str; 2] = ["Coast & Avoid", "Back"];

/// Test mode submenu entries.
pub const TEST_MENU_ITEMS: [&str; 3] = ["Combined Test", "IMU Test", "Back"];

/// Calibration submenu entries.
pub const CALIBRATE_MENU_ITEMS: [&str; 5] = ["Motor", "Mag", "Accel", "Gyro", "Back"];

/// Snapshot of the system info values needed for display.
#[derive(Clone, Copy)]
pub struct SystemInfoData {
    /// Battery level percentage (0-100), if available
    pub battery_level: Option<u8>,
    /// Battery voltage in volts, if available
    pub battery_voltage: Option<f32>,
    /// Motor calibration status
    pub motor_calibration_status: CalibrationStatus,
    /// Magnetometer calibration status
    pub mag_calibration_status: CalibrationStatus,
    /// Accelerometer calibration status
    pub accel_calibration_status: CalibrationStatus,
    /// Gyroscope calibration status
    pub gyro_calibration_status: CalibrationStatus,
}

/// Render the main menu with the given selected index.
pub async fn render_main_menu(selected_index: usize) {
    render_menu("Main Menu", &MAIN_MENU_ITEMS, selected_index).await;
}

/// Render the calibration submenu with the given selected index.
pub async fn render_calibrate_menu(selected_index: usize) {
    render_menu("Calibrate", &CALIBRATE_MENU_ITEMS, selected_index).await;
}

/// Render the drive mode submenu with the given selected index.
pub async fn render_drive_mode_menu(selected_index: usize) {
    render_menu("Drive Mode", &DRIVE_MODE_MENU_ITEMS, selected_index).await;
}

/// Render the test mode submenu with the given selected index.
pub async fn render_test_menu(selected_index: usize) {
    render_menu("Test Mode", &TEST_MENU_ITEMS, selected_index).await;
}

/// Render system info lines with the provided scroll offset.
pub async fn render_system_info(scroll_offset: usize, info: &SystemInfoData) {
    let lines = build_system_info_lines(info);

    for line_idx in 0..DISPLAY_LINES {
        let display_line = u8::try_from(line_idx).unwrap_or(0);
        let content = lines.get(scroll_offset + line_idx).cloned().unwrap_or_else(blank_line);
        display_update(DisplayAction::ShowText(content, display_line)).await;
    }
}

/// Returns the total number of system info lines.
pub fn system_info_line_count(info: &SystemInfoData) -> usize {
    build_system_info_lines(info).len()
}

/// Render a generic menu with a header and list of items.
async fn render_menu(header: &str, items: &[&str], selected_index: usize) {
    let header_line = format_header(header);
    display_update(DisplayAction::ShowText(header_line, 0)).await;

    let visible_items = DISPLAY_LINES - 1;
    let total_items = items.len();
    let max_start = total_items.saturating_sub(visible_items);
    let mut start_index = if selected_index.saturating_add(1) > visible_items {
        selected_index + 1 - visible_items
    } else {
        0
    };
    if start_index > max_start {
        start_index = max_start;
    }

    for i in 0..visible_items {
        let item_index = start_index + i;
        let is_selected = item_index == selected_index;
        let line = items
            .get(item_index)
            .map_or_else(blank_line, |label| format_menu_item(label, is_selected));
        let display_line = u8::try_from(i + 1).unwrap_or(0);
        let style = if is_selected {
            TextStyle::Bold
        } else {
            TextStyle::Normal
        };
        display_update(DisplayAction::ShowTextStyled(line, display_line, style)).await;
    }
}

/// Build the list of system info lines (scrollable).
fn build_system_info_lines(info: &SystemInfoData) -> Vec<String<MAX_LINE_LEN>, 8> {
    let mut lines: Vec<String<MAX_LINE_LEN>, 8> = Vec::new();

    let _ = lines.push(format_battery_level_line(info.battery_level));
    let _ = lines.push(format_battery_voltage_line(info.battery_voltage));
    let _ = lines.push(format_cal_line("Motor", info.motor_calibration_status));
    let _ = lines.push(format_cal_line("Mag", info.mag_calibration_status));
    let _ = lines.push(format_cal_line("Accel", info.accel_calibration_status));
    let _ = lines.push(format_cal_line("Gyro", info.gyro_calibration_status));

    lines
}

/// Format the menu header.
fn format_header(label: &str) -> String<MAX_LINE_LEN> {
    let mut s: String<MAX_LINE_LEN> = String::new();
    let _ = write!(s, "{label}");
    s
}

/// Format a menu item with selection highlight.
///
/// Highlight style: leading `.` when selected, leading space otherwise.
fn format_menu_item(label: &str, selected: bool) -> String<MAX_LINE_LEN> {
    let mut s: String<MAX_LINE_LEN> = String::new();
    if selected {
        let _ = write!(s, "> {label}");
    } else {
        let _ = write!(s, "  {label}");
    }
    s
}

/// Format a calibration status line.
fn format_cal_line(prefix: &str, status: CalibrationStatus) -> String<MAX_LINE_LEN> {
    let mut s: String<MAX_LINE_LEN> = String::new();
    let status_text = calibration_status_label(status);
    let _ = write!(s, "{prefix}: {status_text}");
    s
}

/// Format the battery level line.
fn format_battery_level_line(level: Option<u8>) -> String<MAX_LINE_LEN> {
    let mut s: String<MAX_LINE_LEN> = String::new();
    match level {
        Some(lvl) => {
            let _ = write!(s, "Batt {lvl:>3}%");
        }
        None => {
            let _ = write!(s, "Batt --%");
        }
    }
    s
}

/// Format the battery voltage line.
fn format_battery_voltage_line(voltage: Option<f32>) -> String<MAX_LINE_LEN> {
    let mut s: String<MAX_LINE_LEN> = String::new();
    match voltage {
        Some(volts) => {
            let _ = write!(s, "Batt {volts:>4.1}V");
        }
        None => {
            let _ = write!(s, "Batt --.-V");
        }
    }
    s
}

/// Human-readable labels for calibration status.
const fn calibration_status_label(status: CalibrationStatus) -> &'static str {
    match status {
        CalibrationStatus::NotLoaded => "Unknown",
        CalibrationStatus::Loaded => "Loaded",
        CalibrationStatus::NotAvailable => "Missing",
    }
}

/// Returns a blank display line.
const fn blank_line() -> String<MAX_LINE_LEN> {
    String::new()
}
