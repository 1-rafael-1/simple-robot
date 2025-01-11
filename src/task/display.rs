//! Display
//!
//! Handles the SSD1306 OLED display output, primarily showing a radar-like visualization
//! of the ultrasonic sensor sweep. Displays detected objects in a 160° arc, with distance
//! measurements up to 100cm.
//! Data visualization includes a sweeping line and persistent points for detected objects.

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embedded_graphics::{
    geometry::Size,
    mono_font::{ascii::FONT_9X18_BOLD, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use heapless::Vec;
use micromath::F32Ext;
use ssd1306_async::{prelude::*, I2CDisplayInterface, Ssd1306};

use crate::system::resources::I2c0BusShared;

/// Display actions that can be requested by other tasks
pub enum DisplayAction {
    /// Show a sensor sweep pattern with distance (cm) and angle (degrees)
    ShowSweep(f64, f32),
    /// Show a text message on the display
    ShowText(&'static str, u8),
    /// Clear the entire display
    Clear,
}

// Control signal to trigger display updates
pub static DISPLAY_CONTROL: Signal<CriticalSectionRawMutex, DisplayAction> = Signal::new();

/// Requests a display update with the specified action
pub fn request_update(display_action: DisplayAction) {
    DISPLAY_CONTROL.signal(display_action);
}

/// Blocks until next update request, returns the requested display action
async fn wait() -> DisplayAction {
    DISPLAY_CONTROL.wait().await
}

/// Center X coordinate for the radar display (middle of 128px width)
const CENTER_X: i32 = 64;
/// Center Y coordinate for the radar display (middle of 64px height)
const CENTER_Y: i32 = 32;
/// Radius of the radar display in pixels
const RADIUS: i32 = 30;
/// Maximum distance to display (maps to RADIUS pixels)
const MAX_DISTANCE_CM: f32 = 100.0;
/// Maximum number of stored points for display, limited to define heapless Vec
const MAX_POINTS: usize = 128;

/// Main display task that manages the SSD1306 OLED screen
///
/// # Calculations
///
/// ## Angle Conversion
/// - Input angle is -80° to +80° from vertical
/// - Subtract 90° to align with standard coordinate system
/// - Convert to radians: angle_rad = (angle - 90°) * π/180
///
/// ## Sweep Line
/// - End point calculated using polar to cartesian conversion:
///   x = center_x + radius * cos(angle_rad)
///   y = center_y + radius * sin(angle_rad)
///
/// ## Distance Point Plotting
/// - Scale input distance (0-100cm) to display radius (0-30px):
///   scaled = (distance/MAX_DISTANCE_CM) * RADIUS
/// - Calculate point position using scaled distance:
///   point_x = center_x + scaled * cos(angle_rad)
///   point_y = center_y + scaled * sin(angle_rad)
///
/// ## Point Retention
/// - Calculate angle between stored point and sweep line:
///   point_angle = atan2(dy, dx)
/// - Remove points within ±0.1 radians of sweep line
/// - Keep fixed maximum number of points (MAX_POINTS)
#[embassy_executor::task]
pub async fn display(i2c_bus: &'static I2c0BusShared) {
    let display_i2c = I2cDevice::new(i2c_bus);
    let interface = I2CDisplayInterface::new(display_i2c);
    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();
    display.init().await.unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .build();

    let mut display_action: DisplayAction;
    let mut points: Vec<(i32, i32), MAX_POINTS> = Vec::new();

    request_update(DisplayAction::Clear);

    loop {
        // Wait for the next display update request and clear the display
        display_action = wait().await;
        display.clear();

        match display_action {
            DisplayAction::ShowSweep(distance, angle) => {
                // Clear sweep area with filled black circle
                Circle::new(Point::new(CENTER_X, CENTER_Y), RADIUS as u32)
                    .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
                    .draw(&mut display)
                    .unwrap();

                // Draw base half-circle
                Circle::new(Point::new(CENTER_X, CENTER_Y), RADIUS as u32)
                    .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                    .draw(&mut display)
                    .unwrap();

                // Convert angle to radians (adjust from -80° to +80° range)
                let rad_angle = (angle - 90.0) * core::f32::consts::PI / 180.0;

                // Calculate and draw sweep line
                let line_end_x = CENTER_X + (RADIUS as f32 * rad_angle.cos()) as i32;
                let line_end_y = CENTER_Y + (RADIUS as f32 * rad_angle.sin()) as i32;
                Line::new(Point::new(CENTER_X, CENTER_Y), Point::new(line_end_x, line_end_y))
                    .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                    .draw(&mut display)
                    .unwrap();

                // Process distance point
                if distance <= MAX_DISTANCE_CM as f64 {
                    let scaled_distance = (distance as f32 / MAX_DISTANCE_CM) * RADIUS as f32;
                    let point_x = CENTER_X + (scaled_distance * rad_angle.cos()) as i32;
                    let point_y = CENTER_Y + (scaled_distance * rad_angle.sin()) as i32;

                    // Clear points near sweep line
                    points.retain(|&(px, py)| {
                        let dx = px - CENTER_X;
                        let dy = py - CENTER_Y;
                        let point_angle = (dy as f32).atan2(dx as f32);
                        (point_angle - rad_angle).abs() > 0.1
                    });

                    // Store new point and remove oldest if necessary
                    if points.push((point_x, point_y)).is_err() {
                        points.remove(0);
                        let _ = points.push((point_x, point_y));
                    }
                }

                // Draw all stored points
                for &(px, py) in points.iter() {
                    display.set_pixel(px as u32, py as u32, true);
                }
            }
            DisplayAction::ShowText(text, line) => {
                // Display text at specified line
                let point: Point = match line {
                    0 => Point::new(0, 0),
                    1 => Point::new(0, 16),
                    2 => Point::new(0, 32),
                    3 => Point::new(0, 48),
                    _ => panic!("Invalid line number"),
                };

                // clear the existing text before drawing new text
                let text_bounds = Rectangle::new(point, Size::new(128, 16));
                text_bounds
                    .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
                    .draw(&mut display)
                    .unwrap();
                // draw the new text
                Text::with_baseline(text, point, text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();
            }
            DisplayAction::Clear => display.clear(),
        }

        // Write out the display data
        display.flush().await.unwrap();
    }
}
