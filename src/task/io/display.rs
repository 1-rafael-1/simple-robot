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
/// - Length represents maximum range (200cm)
///
/// Distance Points:
/// - Created when object detected (distance < 200cm)
/// - Position: polar to cartesian conversion from sweep angle
/// - Cleared when sweep line approaches (10° zone)
/// - Maximum 1500 points stored
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    geometry::Size,
    mono_font::{
        MonoTextStyle, MonoTextStyleBuilder,
        ascii::{FONT_7X14, FONT_7X14_BOLD},
    },
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Arc, Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use heapless::{String, Vec};
use micromath::F32Ext;
use ssd1306_async::{
    I2CDisplayInterface, Ssd1306, i2c_interface::I2CInterface, mode::BufferedGraphicsMode, prelude::*,
};

use crate::I2cBusShared;

/// Text style for display rendering
pub enum TextStyle {
    /// Normal weight text
    Normal,
    /// Bold text
    Bold,
}

/// Display actions that can be requested by other tasks
pub enum DisplayAction {
    /// Show a sensor sweep pattern with distance (cm) and angle (degrees)
    ShowSweep(f64, f32),
    /// Show a text message on the display (bold)
    ShowText(String<20>, u8),
    /// Show a text message with an explicit style
    ShowTextStyled(String<20>, u8, TextStyle),
    /// Clear the entire display
    Clear,
}

/// Display error types
#[derive(Debug)]
enum DisplayError {
    /// Error for invalid text line number
    InvalidLine,
    /// Points buffer reached its fixed capacity
    PointsBufferFull,
    /// Drawing operation failed (embedded-graphics draw error)
    DrawError,
}

/// SSD1306 display driver type used by the display task.
type DisplayDriver = Ssd1306<
    I2CInterface<
        I2cDevice<
            'static,
            CriticalSectionRawMutex,
            embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>,
        >,
    >,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>,
>;

/// Control channel to trigger display updates
pub static DISPLAY_CHANNEL: Channel<CriticalSectionRawMutex, DisplayAction, 16> = Channel::new();

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
const MAX_DISTANCE_CM: f64 = 200.0;
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

/// Fixed-size buffer for storing detected points
type PointsBuffer = heapless::Vec<SweepPoint, MAX_POINTS>;

/// Main display task that manages the SSD1306 OLED screen
///
/// # Calculations
///
/// ## Angle Conversion
/// - Input angle is -80° to +80° from vertical
/// - Subtract 90° to align with standard coordinate system
/// - Convert to radians: `angle_rad` = (angle - 90°) * π/180
///
/// ## Sweep Line
/// - End point calculated using polar to cartesian conversion:
///   x = `center_x` + radius * cos(`angle_rad`)
///   y = `center_y` + radius * sin(`angle_rad`)
///
/// ## Distance Point Plotting
/// - Scale input distance (0-100cm) to display radius (0-30px):
///   scaled = (distance/`MAX_DISTANCE_CM`) * RADIUS
/// - Calculate point position using scaled distance:
///   `point_x` = `center_x` + scaled * cos(`angle_rad`)
///   `point_y` = `center_y` + scaled * sin(`angle_rad`)
///
/// ## Point Retention
/// - Calculate angle between stored point and sweep line:
///   `point_angle` = atan2(dy, dx)
/// - Remove points within ±0.1 radians of sweep line
/// - Keep fixed maximum number of points (`MAX_POINTS`)
#[embassy_executor::task]
pub async fn display(i2c_bus: &'static I2cBusShared) {
    const INIT_RETRIES: u8 = 5;
    const INIT_RETRY_DELAY: Duration = Duration::from_millis(200);
    const REINIT_BACKOFF: Duration = Duration::from_secs(2);

    let i2c = I2cDevice::new(i2c_bus);
    let interface = I2CDisplayInterface::new(i2c);
    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();

    let text_style_bold = MonoTextStyleBuilder::new()
        .font(&FONT_7X14_BOLD)
        .text_color(BinaryColor::On)
        .build();

    let text_style_regular = MonoTextStyleBuilder::new()
        .font(&FONT_7X14)
        .text_color(BinaryColor::On)
        .build();

    let mut points: PointsBuffer = Vec::new();
    let mut last_angle: f32 = 0.0; // Move state into the task
    let mut moving_right: bool = true; // Move state into the task

    // Try to initialize the display a few times. If it still fails, continue boot
    // in "display-offline" mode: we keep draining the channel to avoid blocking
    // producers, and periodically retry initialization.
    let mut display_online = false;
    for attempt in 1..=INIT_RETRIES {
        if display.init().await.is_ok() {
            display_online = true;
            break;
        }
        defmt::warn!("display init failed (attempt {}/{})", attempt, INIT_RETRIES);
        Timer::after(INIT_RETRY_DELAY).await;
    }

    if display_online {
        display.clear();

        let mut txt: String<20> = String::new();
        let _ = txt.push_str("Display OK");
        let _ = handle_show_text(&mut display, text_style_bold, &txt, 0);

        if display.flush().await.is_err() {
            defmt::warn!("display flush failed right after init; going offline");
            display_online = false;
        }
    } else {
        defmt::warn!("display unavailable; continuing without display output");
    }

    loop {
        let display_action = wait().await;

        if !display_online {
            // Drain actions so senders don't block. Periodically retry init.
            for attempt in 1..=INIT_RETRIES {
                if display.init().await.is_ok() {
                    display_online = true;
                    points.clear();
                    last_angle = 0.0;
                    moving_right = true;

                    display.clear();
                    if display.flush().await.is_err() {
                        defmt::warn!("display flush failed after re-init; staying offline");
                        display_online = false;
                    } else {
                        defmt::warn!("display re-initialized successfully");
                    }
                    break;
                }

                if attempt == 1 {
                    defmt::warn!("retrying display init...");
                }
                Timer::after(INIT_RETRY_DELAY).await;
            }

            if !display_online {
                Timer::after(REINIT_BACKOFF).await;
            }

            continue;
        }

        if let Err(error) = handle_display_action(
            &mut display,
            text_style_bold,
            text_style_regular,
            &mut points,
            &mut last_angle,
            &mut moving_right,
            display_action,
        ) {
            defmt::warn!(
                "display action failed ({}); taking display offline",
                defmt::Debug2Format(&error)
            );
            display_online = false;
            continue;
        }

        if display.flush().await.is_err() {
            defmt::warn!("display flush failed; taking display offline");
            display_online = false;
        }
    }
}

/// Handles the specified display action
fn handle_display_action(
    display: &mut DisplayDriver,
    text_style_bold: MonoTextStyle<BinaryColor>,
    text_style_regular: MonoTextStyle<BinaryColor>,
    points: &mut PointsBuffer,
    last_angle: &mut f32,
    moving_right: &mut bool,
    display_action: DisplayAction,
) -> Result<(), DisplayError> {
    match display_action {
        DisplayAction::ShowSweep(distance, angle) => {
            handle_show_sweep(display, points, last_angle, moving_right, distance, angle)
        }
        DisplayAction::ShowText(text, line) => handle_show_text(display, text_style_bold, &text, line),
        DisplayAction::ShowTextStyled(text, line, style) => {
            let chosen_style = match style {
                TextStyle::Normal => text_style_regular,
                TextStyle::Bold => text_style_bold,
            };
            handle_show_text(display, chosen_style, &text, line)
        }
        DisplayAction::Clear => {
            display.clear();
            Ok(())
        }
    }
}

/// Handles the `ShowSweep` action by updating the radar sweep visualization
fn handle_show_sweep(
    display: &mut DisplayDriver,
    points: &mut PointsBuffer,
    last_angle: &mut f32,
    moving_right: &mut bool,
    distance: f64,
    angle: f32,
) -> Result<(), DisplayError> {
    // Clear only sweep area (below header)
    Rectangle::new(
        Point::new(0, DISPLAY_HEADER_HEIGHT),
        Size::new(DISPLAY_WIDTH as u32, (DISPLAY_HEIGHT - DISPLAY_HEADER_HEIGHT) as u32),
    )
    .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
    .draw(display)
    .map_err(|_| DisplayError::DrawError)?;

    // Draw base half-circle
    Arc::new(
        Point::new(CENTER_X - RADIUS, CENTER_Y - RADIUS), // Bounding box top-left
        DIAMETER,
        Angle::from_degrees(190.0), // Start at -10° from horizontal (left)
        Angle::from_degrees(160.0), // Sweep 160° clockwise
    )
    .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
    .draw(display)
    .map_err(|_| DisplayError::DrawError)?;

    // Convert servo angle (0-160°) to display coordinates
    // Servo: 0° = left, 160° = right
    // Offset angle by 10° to align with arc's starting position
    let display_angle = angle + 10.0; // Add 10° to align with arc start
    let rad_angle = display_angle.to_radians();

    // Calculate sweep line endpoint
    #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
    let line_end_x = CENTER_X + (RADIUS as f32 * rad_angle.cos()) as i32;
    #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
    let line_end_y = CENTER_Y - (RADIUS as f32 * rad_angle.sin()) as i32;
    Line::new(Point::new(CENTER_X, CENTER_Y), Point::new(line_end_x, line_end_y))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(display)
        .map_err(|_| DisplayError::DrawError)?;

    // Track sweep direction for point clearing
    // Large angle changes (>90°) indicate direction reversal
    // Otherwise compare with last angle to determine direction
    let moving_right_new = if (angle - *last_angle).abs() > 90.0 {
        !*moving_right // Reversed direction
    } else {
        angle > *last_angle // Moving right if angle increasing
    };
    *last_angle = angle;
    *moving_right = moving_right_new;

    // Clear points near sweep line based on direction
    // Points are removed when the sweep line approaches within CLEAR_ZONE degrees
    // Different retention logic based on sweep direction to properly clear points
    // Use display_angle for consistency with point storage (points store display_angle)
    points.retain(|point| {
        let angle_diff = (point.angle - display_angle).rem_euclid(180.0);
        if *moving_right {
            angle_diff > CLEAR_ZONE && angle_diff < 180.0
        } else {
            angle_diff < (180.0 - CLEAR_ZONE) && angle_diff > 0.0
        }
    });

    // Add new detection point if object within range
    if distance <= MAX_DISTANCE_CM {
        // Scale distance from cm to display pixels
        // Maps 0-200cm to 0-RADIUS pixels proportionally
        let scaled_distance = (distance / MAX_DISTANCE_CM) * f64::from(RADIUS);

        // Convert polar coordinates (distance, angle) to cartesian (x, y)
        // Use display_angle (with 10° offset) to match sweep line positioning
        let point_rad_angle = display_angle.to_radians();
        let point = SweepPoint {
            #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
            x: CENTER_X + (scaled_distance * f64::from(point_rad_angle.cos())) as i32,
            #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
            y: CENTER_Y - (scaled_distance * f64::from(point_rad_angle.sin())) as i32,
            angle: display_angle, // Store display_angle for consistent point clearing
        };
        // If buffer is full, drop the oldest point to make room (normal steady-state behavior)
        if points.push(point).is_err() {
            // Buffer is full, remove oldest point and try again
            points.remove(0);
            let _ = points.push(point); // This should succeed now, but ignore if it still fails
        }
    }

    // Draw all points
    for point in points.iter() {
        Pixel(Point::new(point.x, point.y), BinaryColor::On)
            .draw(display)
            .map_err(|_| DisplayError::DrawError)?;
    }

    Ok(())
}

/// Handles the `ShowText` action by displaying the specified text on the given line
fn handle_show_text(
    display: &mut DisplayDriver,
    text_style: MonoTextStyle<BinaryColor>,
    text: &String<20>,
    line: u8,
) -> Result<(), DisplayError> {
    // Display text at specified line
    let point: Point = match line {
        0 => Point::new(0, 0),
        1 => Point::new(0, 16),
        2 => Point::new(0, 32),
        3 => Point::new(0, 48),
        _ => return Err(DisplayError::InvalidLine),
    };

    // Clear only text line area
    Rectangle::new(point, Size::new(DISPLAY_WIDTH as u32, 16))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(display)
        .map_err(|_| DisplayError::DrawError)?;

    // Draw text
    Text::with_baseline(text, point, text_style, Baseline::Top)
        .draw(display)
        .map_err(|_| DisplayError::DrawError)?;

    Ok(())
}
