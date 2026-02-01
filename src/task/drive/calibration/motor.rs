//! Motor calibration procedure
//!
//! Coordinates encoder measurements with motor commands to calculate calibration
//! factors that match individual motor speeds within each track and between tracks.
//!
//! The calibration procedure includes an iterative refinement loop that continues
//! adjusting calibration factors until tracks match within 1% tolerance.

use defmt::info;
use embassy_time::{Duration, Timer};

use crate::{
    system::helper::string_helper::status_text,
    task::{
        drive::{
            feedback::{clear_encoder_measurement, wait_for_encoder_event_timeout},
            types::{
                CALIBRATION_COAST_DURATION_MS, CALIBRATION_SAMPLE_DURATION_MS, CALIBRATION_SPEED_INDIVIDUAL,
                CALIBRATION_SPEED_TRACK,
            },
        },
        motor_driver::{self, MotorCommand, Track},
    },
};

/// Maximum track matching tolerance (1% difference between left and right tracks)
const TRACK_MATCH_TOLERANCE: f32 = 0.01;

/// Maximum iterations for track matching refinement
const MAX_REFINEMENT_ITERATIONS: u8 = 5;

/// Run the motor calibration procedure
///
/// Coordinates a multi-step calibration process across three tasks:
/// - `encoder_read`: Provides pulse count measurements
/// - `motor_driver`: Applies speed commands and stores calibration factors
/// - `flash_storage`: Persists calibration data
pub(crate) async fn run_motor_calibration() {
    use heapless::String;

    use crate::{
        system::event,
        task::{encoder_read, flash_storage, motor_driver::MotorCalibration},
    };

    info!("=== Starting Motor Calibration ===");

    // Display calibration header
    event::raise_event(event::Events::CalibrationStatus {
        header: status_text("Motor Calibration"),
        line1: status_text("Enabling drivers"),
        line2: None,
        line3: None,
    })
    .await;

    // Enable both motor driver chips (take out of standby)
    info!("Enabling motor driver chips");
    motor_driver::send_motor_command(MotorCommand::SetDriverEnable {
        track: Track::Left,
        enabled: true,
    })
    .await;
    motor_driver::send_motor_command(MotorCommand::SetDriverEnable {
        track: Track::Right,
        enabled: true,
    })
    .await;
    Timer::after(Duration::from_millis(10)).await; // Brief delay for enable to take effect

    // Start encoder readings at 50Hz for calibration
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(100)).await; // Let encoder task start

    // Track calibration factors as we calculate them
    let mut calibration = MotorCalibration::default();
    info!("Starting calibration with default factors: {:?}", calibration);

    // Step 1: Test left track motors individually
    info!("Step 1: Testing left track motors individually");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Step 1/8"),
        line2: status_text("Test left motors"),
        line3: None,
    })
    .await;

    // Test left front motor alone
    info!("  Testing left front motor");
    info!("    -> Commanding LEFT FRONT motor to 100%");
    info!("    -> All other motors OFF");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: None,
        line2: None,
        line3: status_text("Left front..."),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(200)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for reset
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(200)).await; // Wait for first fresh sample

    // Use RAW command (not SetAllMotors) to avoid applying old calibration data
    motor_driver::send_motor_command(MotorCommand::SetAllMotorsRaw {
        left_front: CALIBRATION_SPEED_INDIVIDUAL,
        left_rear: 0,
        right_front: 0,
        right_rear: 0,
    })
    .await;

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let left_front_count = if let Some(measurement) = wait_for_encoder_event_timeout(500).await {
        info!("    ENCODER READINGS: {:?}", measurement);
        info!("    -> left_front encoder: {}", measurement.left_front);
        info!("    -> left_rear encoder: {}", measurement.left_rear);
        info!("    -> right_front encoder: {}", measurement.right_front);
        info!("    -> right_rear encoder: {}", measurement.right_rear);
        measurement.left_front
    } else {
        info!("    Warning: No encoder event received for left front");
        0
    };
    info!("    ✓ Left front encoder count: {}", left_front_count);

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // Test left rear motor alone
    info!("  Testing left rear motor");
    info!("    -> Commanding LEFT REAR motor to 100%");
    info!("    -> All other motors OFF");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: None,
        line2: None,
        line3: status_text("Left rear..."),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(200)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for reset
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(200)).await; // Wait for first fresh sample

    // Use RAW command (not SetAllMotors) to avoid applying old calibration data
    motor_driver::send_motor_command(MotorCommand::SetAllMotorsRaw {
        left_front: 0,
        left_rear: CALIBRATION_SPEED_INDIVIDUAL,
        right_front: 0,
        right_rear: 0,
    })
    .await;

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let left_rear_count = if let Some(measurement) = wait_for_encoder_event_timeout(500).await {
        info!("    ENCODER READINGS: {:?}", measurement);
        info!("    -> left_front encoder: {}", measurement.left_front);
        info!("    -> left_rear encoder: {}", measurement.left_rear);
        info!("    -> right_front encoder: {}", measurement.right_front);
        info!("    -> right_rear encoder: {}", measurement.right_rear);
        measurement.left_rear
    } else {
        info!("    Warning: No encoder event received for left rear");
        0
    };
    info!("    ✓ Left rear encoder count: {}", left_rear_count);

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // Step 2: Match left track motors
    info!("Step 2: Matching left track motors");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Step 2/8"),
        line2: status_text("Match left track"),
        line3: None,
    })
    .await;
    let _left_weaker_count = if left_front_count == 0 && left_rear_count == 0 {
        info!("  ERROR: Both left motors show zero counts - calibration cannot proceed");
        0
    } else if left_front_count < left_rear_count {
        info!("  Left front is weaker (reference)");
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: None,
            line2: None,
            line3: status_text("Adj left rear"),
        })
        .await;
        if left_rear_count > 0 && left_front_count > 0 {
            let factor = left_front_count as f32 / left_rear_count as f32;
            if factor > 0.0 && factor <= 1.0 {
                calibration.left_rear *= factor;
                motor_driver::send_motor_command(MotorCommand::UpdateCalibration {
                    track: Track::Left,
                    motor: motor_driver::Motor::Rear,
                    factor: calibration.left_rear,
                })
                .await;
                info!(
                    "  Adjustment factor: {} (cumulative: {})",
                    factor, calibration.left_rear
                );
            } else {
                info!("  ERROR: Invalid factor {} - skipping adjustment", factor);
            }
        } else {
            info!("  ERROR: Cannot calculate factor - zero count detected");
        }
        left_front_count
    } else {
        info!("  Left rear is weaker (reference)");
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: None,
            line2: None,
            line3: status_text("Adj left front"),
        })
        .await;
        if left_front_count > 0 && left_rear_count > 0 {
            let factor = left_rear_count as f32 / left_front_count as f32;
            if factor > 0.0 && factor <= 1.0 {
                calibration.left_front *= factor;
                motor_driver::send_motor_command(MotorCommand::UpdateCalibration {
                    track: Track::Left,
                    motor: motor_driver::Motor::Front,
                    factor: calibration.left_front,
                })
                .await;
                info!(
                    "  Adjustment factor: {} (cumulative: {})",
                    factor, calibration.left_front
                );
            } else {
                info!("  ERROR: Invalid factor {} - skipping adjustment", factor);
            }
        } else {
            info!("  ERROR: Cannot calculate factor - zero count detected");
        }
        left_rear_count
    };

    // Step 3: Verify left track match
    info!("Step 3: Verifying left track match");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Step 3/8"),
        line2: status_text("Verify left track"),
        line3: status_text("Testing..."),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(200)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for reset
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(200)).await; // Wait for first fresh sample

    // Use RAW command (not SetAllMotors) to avoid applying old calibration data
    motor_driver::send_motor_command(MotorCommand::SetAllMotorsRaw {
        left_front: CALIBRATION_SPEED_TRACK,
        left_rear: CALIBRATION_SPEED_TRACK,
        right_front: 0,
        right_rear: 0,
    })
    .await;

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let left_track_count = if let Some(measurement) = wait_for_encoder_event_timeout(500).await {
        info!("  ENCODER READINGS (LEFT TRACK @ 60%): {:?}", measurement);
        info!(
            "  Left front: {}, Left rear: {}",
            measurement.left_front, measurement.left_rear
        );
        // Use average of both motors for track performance
        let avg = (measurement.left_front + measurement.left_rear) / 2;
        info!("  ✓ Left track average: {}", avg);
        avg
    } else {
        info!("    Warning: No encoder event received for left track");
        0
    };

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // Step 4: Test right track motors individually (similar to left track)
    info!("Step 4: Testing right track motors individually");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Step 4/8"),
        line2: status_text("Test right motors"),
        line3: None,
    })
    .await;

    // Test right front motor alone
    info!("  Testing right front motor");
    info!("    -> Commanding RIGHT FRONT motor to 100%");
    info!("    -> All other motors OFF");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: None,
        line2: None,
        line3: status_text("Right front..."),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(200)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for reset
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(200)).await; // Wait for first fresh sample

    // Use RAW command (not SetAllMotors) to avoid applying old calibration data
    motor_driver::send_motor_command(MotorCommand::SetAllMotorsRaw {
        left_front: 0,
        left_rear: 0,
        right_front: CALIBRATION_SPEED_INDIVIDUAL,
        right_rear: 0,
    })
    .await;

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let right_front_count = if let Some(measurement) = wait_for_encoder_event_timeout(500).await {
        info!("    ENCODER READINGS: {:?}", measurement);
        info!("    -> left_front encoder: {}", measurement.left_front);
        info!("    -> left_rear encoder: {}", measurement.left_rear);
        info!("    -> right_front encoder: {}", measurement.right_front);
        info!("    -> right_rear encoder: {}", measurement.right_rear);
        measurement.right_front
    } else {
        info!("    Warning: No encoder event received for right front");
        0
    };
    info!("    ✓ Right front encoder count: {}", right_front_count);

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // Test right rear motor alone
    info!("  Testing right rear motor");
    info!("    -> Commanding RIGHT REAR motor to 100%");
    info!("    -> All other motors OFF");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: None,
        line2: None,
        line3: status_text("Right rear..."),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(200)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for reset
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(200)).await; // Wait for first fresh sample

    // Use RAW command (not SetAllMotors) to avoid applying old calibration data
    motor_driver::send_motor_command(MotorCommand::SetAllMotorsRaw {
        left_front: 0,
        left_rear: 0,
        right_front: 0,
        right_rear: CALIBRATION_SPEED_INDIVIDUAL,
    })
    .await;

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let right_rear_count = if let Some(measurement) = wait_for_encoder_event_timeout(500).await {
        info!("    ENCODER READINGS: {:?}", measurement);
        info!("    -> left_front encoder: {}", measurement.left_front);
        info!("    -> left_rear encoder: {}", measurement.left_rear);
        info!("    -> right_front encoder: {}", measurement.right_front);
        info!("    -> right_rear encoder: {}", measurement.right_rear);
        measurement.right_rear
    } else {
        info!("    Warning: No encoder event received for right rear");
        0
    };
    info!("    ✓ Right rear encoder count: {}", right_rear_count);

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // Step 5: Match right track motors
    info!("Step 5: Matching right track motors");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Step 5/8"),
        line2: status_text("Match right track"),
        line3: None,
    })
    .await;
    let _right_weaker_count = if right_front_count == 0 && right_rear_count == 0 {
        info!("  ERROR: Both right motors show zero counts - calibration cannot proceed");
        0
    } else if right_front_count < right_rear_count {
        info!("  Right front is weaker (reference)");
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: None,
            line2: None,
            line3: status_text("Adj right rear"),
        })
        .await;
        if right_rear_count > 0 && right_front_count > 0 {
            let factor = right_front_count as f32 / right_rear_count as f32;
            if factor > 0.0 && factor <= 1.0 {
                calibration.right_rear *= factor;
                motor_driver::send_motor_command(MotorCommand::UpdateCalibration {
                    track: Track::Right,
                    motor: motor_driver::Motor::Rear,
                    factor: calibration.right_rear,
                })
                .await;
                info!(
                    "  Adjustment factor: {} (cumulative: {})",
                    factor, calibration.right_rear
                );
            } else {
                info!("  ERROR: Invalid factor {} - skipping adjustment", factor);
            }
        } else {
            info!("  ERROR: Cannot calculate factor - zero count detected");
        }
        right_front_count
    } else {
        info!("  Right rear is weaker (reference)");
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: None,
            line2: None,
            line3: status_text("Adj right front"),
        })
        .await;
        if right_front_count > 0 && right_rear_count > 0 {
            let factor = right_rear_count as f32 / right_front_count as f32;
            if factor > 0.0 && factor <= 1.0 {
                calibration.right_front *= factor;
                motor_driver::send_motor_command(MotorCommand::UpdateCalibration {
                    track: Track::Right,
                    motor: motor_driver::Motor::Front,
                    factor: calibration.right_front,
                })
                .await;
                info!(
                    "  Adjustment factor: {} (cumulative: {})",
                    factor, calibration.right_front
                );
            } else {
                info!("  ERROR: Invalid factor {} - skipping adjustment", factor);
            }
        } else {
            info!("  ERROR: Cannot calculate factor - zero count detected");
        }
        right_rear_count
    };

    // Step 6: Verify right track match
    info!("Step 6: Verifying right track match");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Step 6/8"),
        line2: status_text("Verify right trk"),
        line3: status_text("Testing..."),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(200)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for reset
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(200)).await; // Wait for first fresh sample

    // Use RAW command (not SetAllMotors) to avoid applying old calibration data
    motor_driver::send_motor_command(MotorCommand::SetAllMotorsRaw {
        left_front: 0,
        left_rear: 0,
        right_front: CALIBRATION_SPEED_TRACK,
        right_rear: CALIBRATION_SPEED_TRACK,
    })
    .await;

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let right_track_count = if let Some(measurement) = wait_for_encoder_event_timeout(500).await {
        info!("  ENCODER READINGS (RIGHT TRACK @ 60%): {:?}", measurement);
        info!(
            "  Right front: {}, Right rear: {}",
            measurement.right_front, measurement.right_rear
        );
        // Use average of both motors for track performance
        let avg = (measurement.right_front + measurement.right_rear) / 2;
        info!("  ✓ Right track average: {}", avg);
        avg
    } else {
        info!("    Warning: No encoder event received for right track");
        0
    };

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // Step 7: Iterative track matching with 1% tolerance
    //
    // IMPORTANT: Steps 1-6 use SetAllMotorsRaw to measure the raw physical motor characteristics.
    // Step 7 uses SetAllMotors (calibrated) to iteratively test and refine the calibration.
    // This is correct because:
    // - Steps 1-6: Measure raw motor performance to compute initial calibration factors
    // - Step 7: Test the calibration by applying it, measure remaining error, adjust, repeat
    //
    // Each iteration applies the updated calibration and measures whether tracks now match.
    info!("Step 7: Iterative track matching (targeting 1% tolerance)");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Step 7/8"),
        line2: status_text("Match tracks"),
        line3: None,
    })
    .await;

    let mut iteration = 0;
    let mut tracks_matched = false;

    while iteration < MAX_REFINEMENT_ITERATIONS && !tracks_matched {
        iteration += 1;
        info!("  Iteration {}/{}", iteration, MAX_REFINEMENT_ITERATIONS);

        // Test both tracks at the same time - ensure clean encoder state
        encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
        Timer::after(Duration::from_millis(200)).await; // Longer wait for stop to complete
        encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
        Timer::after(Duration::from_millis(100)).await; // Wait for reset to complete
        clear_encoder_measurement().await;

        // Verify reset by reading encoders (should be near zero)
        encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
        Timer::after(Duration::from_millis(500)).await; // Wait longer for first stable reading

        if let Some(verification) = wait_for_encoder_event_timeout(500).await {
            info!(
                "  Pre-test encoder verification: LF={}, RF={}",
                verification.left_front, verification.right_front
            );
        }

        // Reset again and wait to ensure we start clean
        encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
        Timer::after(Duration::from_millis(100)).await;
        encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
        Timer::after(Duration::from_millis(100)).await;
        clear_encoder_measurement().await;
        encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
        Timer::after(Duration::from_millis(200)).await;

        // CRITICAL: Use SetAllMotors (calibrated) not SetAllMotorsRaw during iterations!
        // We need to test the cumulative effect of our calibration adjustments.
        // Each iteration applies the updated calibration factors to see if tracks match.
        motor_driver::send_motor_command(MotorCommand::SetAllMotors {
            left_front: CALIBRATION_SPEED_TRACK,
            left_rear: CALIBRATION_SPEED_TRACK,
            right_front: CALIBRATION_SPEED_TRACK,
            right_rear: CALIBRATION_SPEED_TRACK,
        })
        .await;

        Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

        if let Some(measurement) = wait_for_encoder_event_timeout(500).await {
            // Use only front encoders for track comparison (tracks mechanically link front/rear)
            let left_count = measurement.left_front;
            let right_count = measurement.right_front;

            info!(
                "  Track encoder counts: Left={}, Right={} (using front encoders only)",
                left_count, right_count
            );

            if left_count > 0 && right_count > 0 {
                // Calculate percentage difference
                let max_count = left_count.max(right_count) as f32;
                let diff = (left_count as i32 - right_count as i32).abs() as f32;
                let percent_diff = diff / max_count;

                info!("  Difference: {}%", percent_diff * 100.0);

                if percent_diff <= TRACK_MATCH_TOLERANCE {
                    info!("  ✓ Tracks matched within tolerance!");
                    tracks_matched = true;
                    event::raise_event(event::Events::CalibrationStatus {
                        header: None,
                        line1: None,
                        line2: None,
                        line3: status_text("Tracks matched!"),
                    })
                    .await;
                } else {
                    // Apply correction and send to motor driver for next iteration
                    if left_count > right_count {
                        let factor = right_count as f32 / left_count as f32;
                        info!(
                            "  Adjusting left track down by factor: {} (cumulative: {} -> {})",
                            factor,
                            calibration.left_front,
                            calibration.left_front * factor
                        );
                        event::raise_event(event::Events::CalibrationStatus {
                            header: None,
                            line1: None,
                            line2: None,
                            line3: status_text("Adj left track"),
                        })
                        .await;

                        calibration.left_front *= factor;
                        calibration.left_rear *= factor;
                        // Update motor driver immediately so next iteration tests with new calibration
                        motor_driver::send_motor_command(MotorCommand::UpdateCalibration {
                            track: Track::Left,
                            motor: motor_driver::Motor::Front,
                            factor: calibration.left_front,
                        })
                        .await;
                        motor_driver::send_motor_command(MotorCommand::UpdateCalibration {
                            track: Track::Left,
                            motor: motor_driver::Motor::Rear,
                            factor: calibration.left_rear,
                        })
                        .await;
                    } else {
                        let factor = left_count as f32 / right_count as f32;
                        info!(
                            "  Adjusting right track down by factor: {} (cumulative: {} -> {})",
                            factor,
                            calibration.right_front,
                            calibration.right_front * factor
                        );
                        event::raise_event(event::Events::CalibrationStatus {
                            header: None,
                            line1: None,
                            line2: None,
                            line3: status_text("Adj right track"),
                        })
                        .await;

                        calibration.right_front *= factor;
                        calibration.right_rear *= factor;
                        // Update motor driver immediately so next iteration tests with new calibration
                        motor_driver::send_motor_command(MotorCommand::UpdateCalibration {
                            track: Track::Right,
                            motor: motor_driver::Motor::Front,
                            factor: calibration.right_front,
                        })
                        .await;
                        motor_driver::send_motor_command(MotorCommand::UpdateCalibration {
                            track: Track::Right,
                            motor: motor_driver::Motor::Rear,
                            factor: calibration.right_rear,
                        })
                        .await;
                    }
                }
            } else {
                info!("  ERROR: Zero encoder counts detected - cannot continue refinement");
                break;
            }
        } else {
            info!("  ERROR: No encoder measurement received");
            break;
        }

        motor_driver::send_motor_command(MotorCommand::CoastAll).await;
        info!("  Coasting motors for 1 second...");
        Timer::after(Duration::from_millis(1000)).await; // Longer coast to let motors fully stop

        // Verify motors have stopped by checking encoder counts don't change
        encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
        Timer::after(Duration::from_millis(100)).await;
        encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
        Timer::after(Duration::from_millis(200)).await;

        if let Some(stopped_check) = wait_for_encoder_event_timeout(500).await {
            info!(
                "  Post-coast encoder check: LF={}, RF={}",
                stopped_check.left_front, stopped_check.right_front
            );
        }
    }

    if !tracks_matched {
        info!("  WARNING: Maximum iterations reached without achieving 1% tolerance");
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: None,
            line2: None,
            line3: status_text("Partial match"),
        })
        .await;
    }

    // Step 8: Final verification with all motors
    info!("Step 8: Final verification with all motors");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Step 8/8"),
        line2: status_text("Final verify"),
        line3: status_text("All motors..."),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(200)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for reset
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(200)).await; // Wait for first fresh sample

    // Final verification: Use calibrated SetAllMotors to verify the calibration works!
    // This is the ONLY place in calibration where we want calibration applied.
    motor_driver::send_motor_command(MotorCommand::SetAllMotors {
        left_front: CALIBRATION_SPEED_TRACK,
        left_rear: CALIBRATION_SPEED_TRACK,
        right_front: CALIBRATION_SPEED_TRACK,
        right_rear: CALIBRATION_SPEED_TRACK,
    })
    .await;

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    if let Some(measurement) = wait_for_encoder_event_timeout(500).await {
        info!("  FINAL VERIFICATION (ALL MOTORS @ 60%): {:?}", measurement);
        info!("  Final encoder counts:");
        info!("    Left front:  {}", measurement.left_front);
        info!("    Left rear:   {}", measurement.left_rear);
        info!("    Right front: {}", measurement.right_front);
        info!("    Right rear:  {}", measurement.right_rear);
        let left_avg = (measurement.left_front + measurement.left_rear) / 2;
        let right_avg = (measurement.right_front + measurement.right_rear) / 2;
        info!("  Track averages: Left={}, Right={}", left_avg, right_avg);
    }

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;

    // Step 9: Validate and save calibration to flash
    info!("Step 9: Validating and saving calibration to flash storage");
    info!("Calibration achieved track matching: {}", tracks_matched);
    info!("Final calibration factors: {:?}", calibration);

    // Validate that all factors are reasonable (non-zero and not too extreme)
    let all_valid = calibration.left_front > 0.0
        && calibration.left_front <= 1.0
        && calibration.left_rear > 0.0
        && calibration.left_rear <= 1.0
        && calibration.right_front > 0.0
        && calibration.right_front <= 1.0
        && calibration.right_rear > 0.0
        && calibration.right_rear <= 1.0;

    if all_valid {
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("Saving..."),
            line2: status_text("To flash storage"),
            line3: status_text("Please wait..."),
        })
        .await;

        flash_storage::send_flash_command(flash_storage::FlashCommand::SaveData(
            flash_storage::CalibrationDataKind::Motor(calibration),
        ))
        .await;

        info!("✓ Calibration saved successfully");
    } else {
        info!("✗ ERROR: Calibration factors invalid - NOT saving to flash");
        info!("  Check encoder wiring and ensure motors are running during calibration");
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("CALIB FAILED"),
            line2: status_text("Invalid factors"),
            line3: status_text("Check encoders"),
        })
        .await;
        Timer::after(Duration::from_millis(3000)).await; // Show error message
    }

    // Stop encoder readings
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;

    // Disable motor drivers (return to standby mode)
    info!("Disabling motor driver chips");
    motor_driver::send_motor_command(MotorCommand::SetDriverEnable {
        track: Track::Left,
        enabled: false,
    })
    .await;
    motor_driver::send_motor_command(MotorCommand::SetDriverEnable {
        track: Track::Right,
        enabled: false,
    })
    .await;

    info!("=== Calibration Complete ===");

    // Show final results
    // Format calibration factors for display (abbreviated to fit)
    let mut line2 = String::<20>::new();
    let _ = core::fmt::write(
        &mut line2,
        format_args!("L:{:.2} {:.2}", calibration.left_front, calibration.left_rear),
    );
    let mut line3 = String::<20>::new();
    let _ = core::fmt::write(
        &mut line3,
        format_args!("R:{:.2} {:.2}", calibration.right_front, calibration.right_rear),
    );
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Complete!"),
        line2: Some(line2),
        line3: Some(line3),
    })
    .await;
}
