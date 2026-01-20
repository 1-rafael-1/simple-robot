//! Motor calibration procedure
//!
//! Coordinates encoder measurements with motor commands to calculate calibration
//! factors that match individual motor speeds within each track and between tracks.

use defmt::info;
use embassy_time::{Duration, Timer};

use crate::task::{
    drive::{
        feedback::{clear_encoder_measurement, wait_for_encoder_event_timeout},
        types::{
            CALIBRATION_COAST_DURATION_MS, CALIBRATION_SAMPLE_DURATION_MS, CALIBRATION_SPEED_INDIVIDUAL,
            CALIBRATION_SPEED_TRACK,
        },
    },
    motor_driver::{self, MotorCommand, Track},
};

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
    event::send_event(event::Events::CalibrationStatus {
        header: Some(String::try_from("Motor Calibration").unwrap()),
        line1: Some(String::try_from("Enabling drivers").unwrap()),
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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 1/8").unwrap()),
        line2: Some(String::try_from("Test left motors").unwrap()),
        line3: None,
    })
    .await;

    // Test left front motor alone
    info!("  Testing left front motor");
    info!("    -> Commanding LEFT FRONT motor to 100%");
    info!("    -> All other motors OFF");
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: None,
        line2: None,
        line3: Some(String::try_from("Left front...").unwrap()),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(50)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for first fresh sample

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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: None,
        line2: None,
        line3: Some(String::try_from("Left rear...").unwrap()),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(50)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for first fresh sample

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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 2/8").unwrap()),
        line2: Some(String::try_from("Match left track").unwrap()),
        line3: None,
    })
    .await;
    let _left_weaker_count = if left_front_count == 0 && left_rear_count == 0 {
        info!("  ERROR: Both left motors show zero counts - calibration cannot proceed");
        0
    } else if left_front_count < left_rear_count {
        info!("  Left front is weaker (reference)");
        event::send_event(event::Events::CalibrationStatus {
            header: None,
            line1: None,
            line2: None,
            line3: Some(String::try_from("Adj left rear").unwrap()),
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
        event::send_event(event::Events::CalibrationStatus {
            header: None,
            line1: None,
            line2: None,
            line3: Some(String::try_from("Adj left front").unwrap()),
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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 3/8").unwrap()),
        line2: Some(String::try_from("Verify left track").unwrap()),
        line3: Some(String::try_from("Testing...").unwrap()),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(50)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for first fresh sample

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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 4/8").unwrap()),
        line2: Some(String::try_from("Test right motors").unwrap()),
        line3: None,
    })
    .await;

    // Test right front motor alone
    info!("  Testing right front motor");
    info!("    -> Commanding RIGHT FRONT motor to 100%");
    info!("    -> All other motors OFF");
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: None,
        line2: None,
        line3: Some(String::try_from("Right front...").unwrap()),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(50)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for first fresh sample

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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: None,
        line2: None,
        line3: Some(String::try_from("Right rear...").unwrap()),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(50)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for first fresh sample

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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 5/8").unwrap()),
        line2: Some(String::try_from("Match right track").unwrap()),
        line3: None,
    })
    .await;
    let _right_weaker_count = if right_front_count == 0 && right_rear_count == 0 {
        info!("  ERROR: Both right motors show zero counts - calibration cannot proceed");
        0
    } else if right_front_count < right_rear_count {
        info!("  Right front is weaker (reference)");
        event::send_event(event::Events::CalibrationStatus {
            header: None,
            line1: None,
            line2: None,
            line3: Some(String::try_from("Adj right rear").unwrap()),
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
        event::send_event(event::Events::CalibrationStatus {
            header: None,
            line1: None,
            line2: None,
            line3: Some(String::try_from("Adj right front").unwrap()),
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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 6/8").unwrap()),
        line2: Some(String::try_from("Verify right trk").unwrap()),
        line3: Some(String::try_from("Testing...").unwrap()),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(50)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for first fresh sample

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

    // Step 7: Match tracks (scale stronger track down to weaker track)
    info!("Step 7: Matching tracks (between-track calibration)");
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 7/8").unwrap()),
        line2: Some(String::try_from("Match tracks").unwrap()),
        line3: None,
    })
    .await;

    if left_track_count > 0 && right_track_count > 0 {
        if left_track_count > right_track_count {
            // Left track is stronger, scale it down to match right
            let factor = right_track_count as f32 / left_track_count as f32;
            if factor > 0.0 && factor <= 1.0 {
                info!("  Left track stronger, scaling down by factor: {}", factor);
                event::send_event(event::Events::CalibrationStatus {
                    header: None,
                    line1: None,
                    line2: None,
                    line3: Some(String::try_from("Scale left down").unwrap()),
                })
                .await;

                calibration.left_front *= factor;
                calibration.left_rear *= factor;
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
                info!(
                    "  ERROR: Invalid track factor {} - skipping between-track adjustment",
                    factor
                );
            }
        } else if right_track_count > left_track_count {
            // Right track is stronger, scale it down to match left
            let factor = left_track_count as f32 / right_track_count as f32;
            if factor > 0.0 && factor <= 1.0 {
                info!("  Right track stronger, scaling down by factor: {}", factor);
                event::send_event(event::Events::CalibrationStatus {
                    header: None,
                    line1: None,
                    line2: None,
                    line3: Some(String::try_from("Scale right down").unwrap()),
                })
                .await;

                calibration.right_front *= factor;
                calibration.right_rear *= factor;
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
            } else {
                info!(
                    "  ERROR: Invalid track factor {} - skipping between-track adjustment",
                    factor
                );
            }
        } else {
            info!("  Tracks already matched");
            event::send_event(event::Events::CalibrationStatus {
                header: None,
                line1: None,
                line2: None,
                line3: Some(String::try_from("Tracks matched!").unwrap()),
            })
            .await;
        }
    } else {
        info!("  ERROR: Cannot match tracks - one or both tracks have zero counts");
    }

    // Step 8: Final verification with all motors
    info!("Step 8: Final verification with all motors");
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 8/8").unwrap()),
        line2: Some(String::try_from("Final verify").unwrap()),
        line3: Some(String::try_from("All motors...").unwrap()),
    })
    .await;

    // Stop sampling, reset, clear, then restart for clean measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(50)).await; // Let stop take effect
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    clear_encoder_measurement().await; // Clear stale measurement
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(100)).await; // Wait for first fresh sample

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
        event::send_event(event::Events::CalibrationStatus {
            header: None,
            line1: Some(String::try_from("Saving...").unwrap()),
            line2: Some(String::try_from("To flash storage").unwrap()),
            line3: Some(String::try_from("Please wait...").unwrap()),
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
        event::send_event(event::Events::CalibrationStatus {
            header: None,
            line1: Some(String::try_from("CALIB FAILED").unwrap()),
            line2: Some(String::try_from("Invalid factors").unwrap()),
            line3: Some(String::try_from("Check encoders").unwrap()),
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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Complete!").unwrap()),
        line2: Some(line2),
        line3: Some(line3),
    })
    .await;
}
