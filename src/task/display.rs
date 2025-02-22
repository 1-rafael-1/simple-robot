//! Display
//!
/// Display task managing SSD1306 OLED screen (128x64 pixels)
/// Layout:
/// - Top 16px: Reserved for text/status
/// - Bottom 48px: Radar sweep visualization
/// - Origin (0,0): Top-left of display
/// - Y axis increases downward
///
/// Coordinate Systems:
/// - Display: (0,0) at top-left, y increases downward
/// - Servo: 0° left, 90° up, 180° right
/// - Radar: Center at (64,64), 0° left through 180° right
///
/// Sweep Visualization:
/// - Half-circle arc at bottom (160° span)
/// - Moving line showing current servo angle
/// - Length represents maximum range (400cm)
///
/// Distance Points:
/// - Created when object detected (distance < 400cm)
/// - Position: polar to cartesian conversion from sweep angle
/// - Cleared when sweep line approaches (10° zone)
/// - Maximum 1500 points stored
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embedded_graphics::{
    geometry::Size,
    mono_font::{MonoTextStyleBuilder, ascii::FONT_9X15_BOLD as Font},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Arc, Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use heapless::{String, Vec};
use micromath::F32Ext;
use ssd1306_async::{I2CDisplayInterface, Ssd1306, prelude::*};

use crate::system::resources::I2c0BusShared;

/// Display actions that can be requested by other tasks
pub enum DisplayAction {
    /// Show a sensor sweep pattern with distance (cm) and angle (degrees)
    ShowSweep(f64, f32),
    /// Show a text message on the display
    ShowText(String<20>, u8),
    /// Clear the entire display
    Clear,
}

/// Control channel to trigger display updates
pub static DISPLAY_CHANNEL: Channel<CriticalSectionRawMutex, DisplayAction, 4> = Channel::new();

/// Requests a display update with the specified action
pub async fn display_update(display_action: DisplayAction) {
    DISPLAY_CHANNEL.send(display_action).await;
}

/// Blocks until next update request, returns the requested display action
async fn wait() -> DisplayAction {
    DISPLAY_CHANNEL.receive().await
}

// Display dimensions

/// Display width in pixels
const DISPLAY_WIDTH: i32 = 128;
/// Display height in pixels
const DISPLAY_HEIGHT: i32 = 64;
/// Header height in pixels for text/status display
const DISPLAY_HEADER_HEIGHT: i32 = 16;

// Sweep visualization parameters

/// Center X coordinate for the radar display (middle of 128px width)
const CENTER_X: i32 = 64;
/// Center Y coordinate for the radar display (bottom of display)
const CENTER_Y: i32 = 64;
/// Radius of the sweep display in pixels
const RADIUS: i32 = 45;
/// Diameter of the sweep display in pixels
const DIAMETER: u32 = (RADIUS * 2) as u32;
/// Maximum distance to display (maps to RADIUS pixels)
const MAX_DISTANCE_CM: f32 = 400.0;
/// Maximum number of stored points for display, limited to define heapless Vec
const MAX_POINTS: usize = 1500;

/// Point storage for detected objects
#[derive(Clone, Copy)]
struct SweepPoint {
    /// Display X coordinate
    x: i32,
    /// Display Y coordinate
    y: i32,
    /// Servo angle at which point was detected
    angle: f32,
}

/// Clear zone angle in degrees
const CLEAR_ZONE: f32 = 10.0;

type PointsBuffer = heapless::Vec<SweepPoint, MAX_POINTS>;

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
    let i2c = I2cDevice::new(i2c_bus);
    let interface = I2CDisplayInterface::new(i2c);
    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();
    display.init().await.unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&Font)
        .text_color(BinaryColor::On)
        .build();

    let mut display_action: DisplayAction;
    let mut points: PointsBuffer = Vec::new();
    let mut last_angle: f32 = 0.0; // Move state into the task
    let mut moving_right: bool = true; // Move state into the task

    display_update(DisplayAction::Clear).await;

    display.clear();
    display.flush().await.unwrap();

    loop {
        // Wait for the next display update request and clear the display
        display_action = wait().await;
        // display.clear();

        match display_action {
            DisplayAction::ShowSweep(distance, angle) => {
                // Clear only sweep area (below header)
                Rectangle::new(
                    Point::new(0, DISPLAY_HEADER_HEIGHT),
                    Size::new(DISPLAY_WIDTH as u32, (DISPLAY_HEIGHT - DISPLAY_HEADER_HEIGHT) as u32),
                )
                .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
                .draw(&mut display)
                .unwrap();

                // Draw base half-circle
                Arc::new(
                    Point::new(CENTER_X - RADIUS, CENTER_Y - RADIUS), // Bounding box top-left
                    DIAMETER,
                    Angle::from_degrees(190.0), // Start at -10° from horizontal (left)
                    Angle::from_degrees(160.0), // Sweep 160° clockwise
                )
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                .draw(&mut display)
                .unwrap();

                // Convert servo angle (0-160°) to display coordinates
                // Servo: 0° = left, 160° = right
                // Offset angle by 10° to align with arc's starting position
                let display_angle = angle + 10.0; // Add 10° to align with arc start
                let rad_angle = display_angle.to_radians();

                // Calculate sweep line endpoint
                let line_end_x = CENTER_X + (RADIUS as f32 * rad_angle.cos()) as i32;
                let line_end_y = CENTER_Y - (RADIUS as f32 * rad_angle.sin()) as i32;
                Line::new(Point::new(CENTER_X, CENTER_Y), Point::new(line_end_x, line_end_y))
                    .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                    .draw(&mut display)
                    .unwrap();

                // Use same angle offset for point plotting to maintain consistency
                if distance <= MAX_DISTANCE_CM as f64 {
                    let scaled_distance = (distance as f32 / MAX_DISTANCE_CM) * RADIUS as f32;
                    let point = SweepPoint {
                        x: CENTER_X + (scaled_distance * rad_angle.cos()) as i32,
                        y: CENTER_Y - (scaled_distance * rad_angle.sin()) as i32,
                        angle: display_angle, // Store offset angle for point clearing
                    };
                    let _ = points.push(point);
                }

                // Track sweep direction for point clearing
                // Large angle changes (>90°) indicate direction reversal
                // Otherwise compare with last angle to determine direction
                let moving_right_new = if (angle - last_angle).abs() > 90.0 {
                    !moving_right // Reversed direction
                } else {
                    angle > last_angle // Moving right if angle increasing
                };
                last_angle = angle;
                moving_right = moving_right_new;

                // Clear points near sweep line based on direction
                // Points are removed when the sweep line approaches within CLEAR_ZONE degrees
                // Different retention logic based on sweep direction to properly clear points
                points.retain(|point| {
                    let angle_diff = (point.angle - angle).rem_euclid(180.0);
                    if moving_right {
                        angle_diff > CLEAR_ZONE && angle_diff < 180.0
                    } else {
                        angle_diff < (180.0 - CLEAR_ZONE) && angle_diff > 0.0
                    }
                });

                // Add new detection point if object within range
                if distance <= MAX_DISTANCE_CM as f64 {
                    // Scale distance from cm to display pixels
                    // Maps 0-200cm to 0-RADIUS pixels proportionally
                    let scaled_distance = (distance as f32 / MAX_DISTANCE_CM) * RADIUS as f32;

                    // Convert polar coordinates (distance, angle) to cartesian (x, y)
                    let rad_angle = angle.to_radians();
                    let point = SweepPoint {
                        x: CENTER_X + (scaled_distance * rad_angle.cos()) as i32,
                        y: CENTER_Y - (scaled_distance * rad_angle.sin()) as i32,
                        angle, // Store angle for later point clearing
                    };
                    let _ = points.push(point); // Add to fixed-size buffer
                }

                // Draw all points
                for point in points.iter() {
                    Pixel(Point::new(point.x, point.y), BinaryColor::On)
                        .draw(&mut display)
                        .unwrap();
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

                // Clear only text line area
                Rectangle::new(point, Size::new(DISPLAY_WIDTH as u32, 16))
                    .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
                    .draw(&mut display)
                    .unwrap();

                // Draw text
                Text::with_baseline(&text, point, text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();
            }
            DisplayAction::Clear => display.clear(),
        }

        // Write out the display data
        display.flush().await.unwrap();
    }
}
