//! IMU calibration procedure
//!
//! Calibrates gyroscope, accelerometer, and magnetometer, plus measures
//! motor interference effects on magnetometer for runtime compensation.

use core::fmt::Write;

use defmt::info;
use embassy_time::{Duration, Instant, Timer};

use crate::{
    system::helper::string_helper::status_text,
    task::{
        drive::{
            sensors::data::{
                clear_accel_measurement, clear_gyro_measurement, clear_mag_measurement, get_latest_accel_measurement,
                get_latest_gyro_measurement, measure_mag_average, subtract_mag, wait_for_mag_event_timeout,
            },
            types::ImuCalibrationKind,
        },
        motor_driver::{self, MotorCommand, Track},
    },
};

/// Run the IMU calibration procedure
///
/// Measures gyroscope, accelerometer, and magnetometer biases, plus motor interference.
///
/// # Why IMU Calibration Uses Calibrated Motor Commands
///
/// Unlike motor calibration (which must use raw commands), IMU calibration CORRECTLY
/// uses calibrated motor commands (`SetTrack`, `SetTracks`). Here's why:
///
/// **Goal**: Measure magnetic interference at *actual operational speeds*
/// - We want to know the interference when motors run at 50% and 100% *actual* speed
/// - During normal operation, motor calibration is applied
/// - So we need interference measurements with calibration applied
///
/// **Example**:
/// - If `left_front` has calibration factor 0.8
/// - Command "100%" becomes 80% actual (due to calibration)
/// - IMU needs to know interference at 80% actual, not 100% raw
/// - This way, interference compensation matches real-world operation
///
/// **Ordering requirement**:
/// - Motor calibration MUST run before IMU calibration
/// - This ensures IMU measures interference at correctly calibrated speeds
/// - If motor calibration changes later, IMU calibration should be re-run
#[allow(clippy::too_many_lines)]
#[allow(clippy::similar_names)]
#[allow(clippy::cast_precision_loss)]
pub async fn run_imu_calibration(kind: ImuCalibrationKind) {
    use heapless::String;

    use crate::{
        system::event,
        task::{io::flash_storage, sensors::imu as imu_read},
    };

    info!("=== Starting IMU Calibration ===");

    let run_gyro = matches!(kind, ImuCalibrationKind::Gyro | ImuCalibrationKind::Full);
    let run_accel = matches!(kind, ImuCalibrationKind::Accel | ImuCalibrationKind::Full);
    let run_mag = matches!(kind, ImuCalibrationKind::Mag | ImuCalibrationKind::Full);
    let is_full = matches!(kind, ImuCalibrationKind::Full);

    let cached_calibration = flash_storage::get_cached_imu_calibration().await.unwrap_or_default();
    let mut gyro_x_bias = cached_calibration.gyro_x_bias;
    let mut gyro_y_bias = cached_calibration.gyro_y_bias;
    let mut gyro_z_bias = cached_calibration.gyro_z_bias;
    let mut accel_x_bias = cached_calibration.accel_x_bias;
    let mut accel_y_bias = cached_calibration.accel_y_bias;
    let mut accel_z_bias = cached_calibration.accel_z_bias;
    let mut mag_x_bias = cached_calibration.mag_x_bias;
    let mut mag_y_bias = cached_calibration.mag_y_bias;
    let mut mag_z_bias = cached_calibration.mag_z_bias;
    let mut mag_x_interference_50 = cached_calibration.mag_x_interference_50;
    let mut mag_y_interference_50 = cached_calibration.mag_y_interference_50;
    let mut mag_z_interference_50 = cached_calibration.mag_z_interference_50;
    let mut mag_x_interference_100 = cached_calibration.mag_x_interference_100;
    let mut mag_y_interference_100 = cached_calibration.mag_y_interference_100;
    let mut mag_z_interference_100 = cached_calibration.mag_z_interference_100;
    let mut imu_flags = flash_storage::get_cached_imu_flags().await.unwrap_or_default();
    let mut mag_completed = false;

    // Display calibration header
    event::raise_event(event::Events::CalibrationStatus {
        header: status_text("IMU Calibration"),
        line1: status_text("Initializing"),
        line2: None,
        line3: None,
    })
    .await;

    if run_gyro || run_accel || run_mag {
        imu_read::start_imu_readings();
        Timer::after(Duration::from_millis(500)).await; // Wait for IMU to start
    }

    if run_gyro || run_accel {
        // Step 1: Gyroscope and/or Accelerometer Calibration (stationary)
        let step_label = if run_gyro && run_accel {
            "Gyro+Accel cal"
        } else if run_gyro {
            "Gyro calibration"
        } else {
            "Accel calibration"
        };
        info!("Step 1: {=str}", step_label);
        let (line1, line2, line3) = if is_full {
            (
                status_text("Step 1/9"),
                status_text(step_label),
                status_text("Keep still 10s"),
            )
        } else {
            (status_text(step_label), status_text("Keep still 10s"), None)
        };
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1,
            line2,
            line3,
        })
        .await;

        // Collect gyro and accel samples while stationary
        info!("  Collecting calibration samples (stationary)...");
        let mut gyro_x_sum = 0.0f32;
        let mut gyro_y_sum = 0.0f32;
        let mut gyro_z_sum = 0.0f32;
        let mut accel_x_sum = 0.0f32;
        let mut accel_y_sum = 0.0f32;
        let mut accel_z_sum = 0.0f32;
        let num_samples = 1000; // 1000 samples at 100Hz = 10 seconds

        if run_gyro {
            clear_gyro_measurement().await;
        }
        if run_accel {
            clear_accel_measurement().await;
        }
        Timer::after(Duration::from_millis(100)).await; // Wait for fresh data

        for i in 0..num_samples {
            if run_gyro {
                // Wait for fresh gyro data
                let start = embassy_time::Instant::now();
                while embassy_time::Instant::now().duration_since(start).as_millis() < 20 {
                    if let Some(gyro_data) = get_latest_gyro_measurement().await {
                        gyro_x_sum += gyro_data.x;
                        gyro_y_sum += gyro_data.y;
                        gyro_z_sum += gyro_data.z;
                        break;
                    }
                    Timer::after(Duration::from_millis(1)).await;
                }
            }

            if run_accel {
                // Wait for fresh accel data
                let start = embassy_time::Instant::now();
                while embassy_time::Instant::now().duration_since(start).as_millis() < 20 {
                    if let Some(accel_data) = get_latest_accel_measurement().await {
                        accel_x_sum += accel_data.x;
                        accel_y_sum += accel_data.y;
                        accel_z_sum += accel_data.z;
                        break;
                    }
                    Timer::after(Duration::from_millis(1)).await;
                }
            }

            // Update progress every 100 samples
            if i % 100 == 0 {
                let progress = (i * 100) / num_samples;
                let mut line = String::new();
                let _ = write!(line, "Progress: {progress}%");
                event::raise_event(event::Events::CalibrationStatus {
                    header: None,
                    line1: None,
                    line2: None,
                    line3: Some(line),
                })
                .await;
            }
        }

        let computed_gyro_x_bias = gyro_x_sum / num_samples as f32;
        let computed_gyro_y_bias = gyro_y_sum / num_samples as f32;
        let computed_gyro_z_bias = gyro_z_sum / num_samples as f32;
        let computed_accel_x_bias = accel_x_sum / num_samples as f32;
        let computed_accel_y_bias = accel_y_sum / num_samples as f32;
        let computed_accel_z_bias = (accel_z_sum / num_samples as f32) - 1.0; // Subtract 1g for gravity

        if run_gyro {
            gyro_x_bias = computed_gyro_x_bias;
            gyro_y_bias = computed_gyro_y_bias;
            gyro_z_bias = computed_gyro_z_bias;

            info!("  ✓ Gyro bias calculated:");
            info!("    X: {} deg/s", gyro_x_bias);
            info!("    Y: {} deg/s", gyro_y_bias);
            info!("    Z: {} deg/s", gyro_z_bias);
        }

        if run_accel {
            accel_x_bias = computed_accel_x_bias;
            accel_y_bias = computed_accel_y_bias;
            accel_z_bias = computed_accel_z_bias;

            info!("  ✓ Accel bias calculated:");
            info!("    X: {} g", accel_x_bias);
            info!("    Y: {} g", accel_y_bias);
            info!("    Z: {} g (gravity removed)", accel_z_bias);
        }
    }

    if run_mag {
        const MAG_TIMEOUT_LIMIT: u32 = 25; // ~5s at 200ms
        const MAG_MAX_SECONDS: u64 = 90;

        // Step 2: Magnetometer Calibration (moving through all axes)
        info!("Step 2: Magnetometer Calibration (CRITICAL - Manual Rotation Required)");
        let (line1, line2, line3) = if is_full {
            (
                status_text("Step 2/9"),
                status_text("MAG CALIBRATION"),
                status_text("Prepare to move"),
            )
        } else {
            (status_text("Mag calibration"), status_text("Prepare to move"), None)
        };
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1,
            line2,
            line3,
        })
        .await;
        Timer::after(Duration::from_secs(3)).await;

        let (line1, line2, line3) = if is_full {
            (
                status_text("Step 2/9"),
                status_text("ROTATE SLOWLY"),
                status_text("All axes 60s"),
            )
        } else {
            (
                status_text("Mag calibration"),
                status_text("Rotate slowly"),
                status_text("All axes 60s"),
            )
        };
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1,
            line2,
            line3,
        })
        .await;

        info!("╔═══════════════════════════════════════════════════╗");
        info!("║  CRITICAL: MAGNETOMETER CALIBRATION REQUIRED     ║");
        info!("╠═══════════════════════════════════════════════════╣");
        info!("║  Slowly rotate robot through ALL axes for 60s:   ║");
        info!("║  • PITCH: Tilt forward/backward                  ║");
        info!("║  • ROLL: Tilt left/right                         ║");
        info!("║  • YAW: Spin clockwise/counterclockwise          ║");
        info!("║                                                   ║");
        info!("║  Goal: Robot must experience full 3D rotation    ║");
        info!("║  to map Earth's magnetic field in all            ║");
        info!("║  orientations. Move slowly and smoothly.         ║");
        info!("╚═══════════════════════════════════════════════════╝");

        // Collect magnetometer samples to find min/max for hard iron calibration
        let mut mag_x_min = f32::MAX;
        let mut mag_x_max = f32::MIN;
        let mut mag_y_min = f32::MAX;
        let mut mag_y_max = f32::MIN;
        let mut mag_z_min = f32::MAX;
        let mut mag_z_max = f32::MIN;
        let mag_samples = 6000; // 6000 samples at 100Hz = 60 seconds
        let mut mag_samples_collected: usize = 0;
        let mut mag_timeout_streak: u32 = 0;
        let mut last_status_secs: u64 = 0;
        let start_time = Instant::now();
        let mut mag_failed = false;

        while mag_samples_collected < mag_samples {
            let elapsed_ms = Instant::now().duration_since(start_time).as_millis();
            let elapsed_secs = elapsed_ms / 1000;

            if elapsed_secs >= MAG_MAX_SECONDS {
                mag_failed = true;
                break;
            }

            if let Some(mag_data) = wait_for_mag_event_timeout(200).await {
                mag_samples_collected += 1;
                mag_timeout_streak = 0;

                // Track min/max for each axis
                if mag_data.x < mag_x_min {
                    mag_x_min = mag_data.x;
                }
                if mag_data.x > mag_x_max {
                    mag_x_max = mag_data.x;
                }
                if mag_data.y < mag_y_min {
                    mag_y_min = mag_data.y;
                }
                if mag_data.y > mag_y_max {
                    mag_y_max = mag_data.y;
                }
                if mag_data.z < mag_z_min {
                    mag_z_min = mag_data.z;
                }
                if mag_data.z > mag_z_max {
                    mag_z_max = mag_data.z;
                }
            } else {
                mag_timeout_streak += 1;
            }

            if elapsed_secs.is_multiple_of(5) && elapsed_secs != last_status_secs {
                last_status_secs = elapsed_secs;
                let mut line = String::new();
                let _ = write!(line, "S:{mag_samples_collected}/{mag_samples} T:{elapsed_secs}s");
                event::raise_event(event::Events::CalibrationStatus {
                    header: None,
                    line1: None,
                    line2: None,
                    line3: Some(line),
                })
                .await;
            }

            if mag_timeout_streak >= MAG_TIMEOUT_LIMIT {
                mag_failed = true;
                break;
            }
        }

        if mag_failed || mag_samples_collected == 0 {
            info!("Mag calibration timed out ({} samples)", mag_samples_collected);
            event::raise_event(event::Events::CalibrationStatus {
                header: None,
                line1: status_text("MAG TIMEOUT"),
                line2: status_text("Check IMU"),
                line3: status_text("No mag data"),
            })
            .await;
        } else {
            // Calculate hard iron bias (center of min/max sphere)
            let computed_mag_x_bias = f32::midpoint(mag_x_max, mag_x_min);
            let computed_mag_y_bias = f32::midpoint(mag_y_max, mag_y_min);
            let computed_mag_z_bias = f32::midpoint(mag_z_max, mag_z_min);

            mag_x_bias = computed_mag_x_bias;
            mag_y_bias = computed_mag_y_bias;
            mag_z_bias = computed_mag_z_bias;

            info!("  ✓ Magnetometer calibration complete!");
            info!("  Hard iron bias (offset to center):");
            info!("    X: min={} max={} → bias={}", mag_x_min, mag_x_max, mag_x_bias);
            info!("    Y: min={} max={} → bias={}", mag_y_min, mag_y_max, mag_y_bias);
            info!("    Z: min={} max={} → bias={}", mag_z_min, mag_z_max, mag_z_bias);

            // Enable motor drivers for interference testing
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
            Timer::after(Duration::from_millis(100)).await;

            // Step 3: Motor interference baseline (motors off)
            info!("Step 3: Measuring magnetic baseline (motors OFF)");
            event::raise_event(event::Events::CalibrationStatus {
                header: None,
                line1: if is_full {
                    status_text("Step 3/9")
                } else {
                    status_text("Mag interference")
                },
                line2: status_text("Baseline (OFF)"),
                line3: status_text("5 seconds..."),
            })
            .await;

            clear_mag_measurement().await;
            Timer::after(Duration::from_millis(500)).await; // Wait for fresh data
            let baseline_mag = measure_mag_average(500).await; // 500 samples at 100Hz = 5s
            info!(
                "  Baseline mag: x={} y={} z={}",
                baseline_mag.x, baseline_mag.y, baseline_mag.z
            );

            // Step 4: All motors at 50%
            info!("Step 4: Motor interference at 50% (all motors)");
            event::raise_event(event::Events::CalibrationStatus {
                header: None,
                line1: if is_full {
                    status_text("Step 4/9")
                } else {
                    status_text("Mag interference")
                },
                line2: status_text("All motors 50%"),
                line3: status_text("Measuring 6s..."),
            })
            .await;

            motor_driver::send_motor_command(MotorCommand::SetTrack {
                track: Track::Left,
                speed: 50,
            })
            .await;
            motor_driver::send_motor_command(MotorCommand::SetTrack {
                track: Track::Right,
                speed: 50,
            })
            .await;
            Timer::after(Duration::from_millis(1000)).await; // Let stabilize
            clear_mag_measurement().await;
            Timer::after(Duration::from_millis(500)).await; // Wait for fresh data
            let mag_all_50 = measure_mag_average(500).await;
            let interference_all_50 = subtract_mag(mag_all_50, baseline_mag);
            info!(
                "  All 50% interference: x={} y={} z={}",
                interference_all_50.x, interference_all_50.y, interference_all_50.z
            );

            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(500)).await;

            // Step 5: All motors at 100%
            info!("Step 5: Motor interference at 100% (all motors)");
            event::raise_event(event::Events::CalibrationStatus {
                header: None,
                line1: if is_full {
                    status_text("Step 5/9")
                } else {
                    status_text("Mag interference")
                },
                line2: status_text("All motors 100%"),
                line3: status_text("Measuring 6s..."),
            })
            .await;

            motor_driver::send_motor_command(MotorCommand::SetTrack {
                track: Track::Left,
                speed: 100,
            })
            .await;
            motor_driver::send_motor_command(MotorCommand::SetTrack {
                track: Track::Right,
                speed: 100,
            })
            .await;
            Timer::after(Duration::from_millis(1000)).await;
            clear_mag_measurement().await;
            Timer::after(Duration::from_millis(500)).await;
            let mag_all_100 = measure_mag_average(500).await;
            let interference_all_100 = subtract_mag(mag_all_100, baseline_mag);
            info!(
                "  All 100% interference: x={} y={} z={}",
                interference_all_100.x, interference_all_100.y, interference_all_100.z
            );

            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(500)).await;

            // Step 6: Left track at 50%
            info!("Step 6: Motor interference at 50% (left track only)");
            event::raise_event(event::Events::CalibrationStatus {
                header: None,
                line1: if is_full {
                    status_text("Step 6/9")
                } else {
                    status_text("Mag interference")
                },
                line2: status_text("Left track 50%"),
                line3: status_text("Measuring 6s..."),
            })
            .await;

            motor_driver::send_motor_command(MotorCommand::SetTrack {
                track: Track::Left,
                speed: 50,
            })
            .await;
            motor_driver::send_motor_command(MotorCommand::SetTrack {
                track: Track::Right,
                speed: 0,
            })
            .await;
            Timer::after(Duration::from_millis(1000)).await;
            clear_mag_measurement().await;
            Timer::after(Duration::from_millis(500)).await;
            let mag_left_50 = measure_mag_average(500).await;
            let interference_left_50 = subtract_mag(mag_left_50, baseline_mag);
            info!(
                "  Left 50% interference: x={} y={} z={}",
                interference_left_50.x, interference_left_50.y, interference_left_50.z
            );

            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(500)).await;

            // Step 7: Left track at 100%
            info!("Step 7: Motor interference at 100% (left track only)");
            event::raise_event(event::Events::CalibrationStatus {
                header: None,
                line1: if is_full {
                    status_text("Step 7/9")
                } else {
                    status_text("Mag interference")
                },
                line2: status_text("Left track 100%"),
                line3: status_text("Measuring 6s..."),
            })
            .await;

            motor_driver::send_motor_command(MotorCommand::SetTrack {
                track: Track::Left,
                speed: 100,
            })
            .await;
            motor_driver::send_motor_command(MotorCommand::SetTrack {
                track: Track::Right,
                speed: 0,
            })
            .await;
            Timer::after(Duration::from_millis(1000)).await;
            clear_mag_measurement().await;
            Timer::after(Duration::from_millis(500)).await;
            let mag_left_100 = measure_mag_average(500).await;
            let interference_left_100 = subtract_mag(mag_left_100, baseline_mag);
            info!(
                "  Left 100% interference: x={} y={} z={}",
                interference_left_100.x, interference_left_100.y, interference_left_100.z
            );

            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(500)).await;

            // Step 8: Right track at 50%
            info!("Step 8: Motor interference at 50% (right track only)");
            event::raise_event(event::Events::CalibrationStatus {
                header: None,
                line1: if is_full {
                    status_text("Step 8/9")
                } else {
                    status_text("Mag interference")
                },
                line2: status_text("Right track 50%"),
                line3: status_text("Measuring 6s..."),
            })
            .await;

            motor_driver::send_motor_command(MotorCommand::SetTrack {
                track: Track::Left,
                speed: 0,
            })
            .await;
            motor_driver::send_motor_command(MotorCommand::SetTrack {
                track: Track::Right,
                speed: 50,
            })
            .await;
            Timer::after(Duration::from_millis(1000)).await;
            clear_mag_measurement().await;
            Timer::after(Duration::from_millis(500)).await;
            let mag_right_50 = measure_mag_average(500).await;
            let interference_right_50 = subtract_mag(mag_right_50, baseline_mag);
            info!(
                "  Right 50% interference: x={} y={} z={}",
                interference_right_50.x, interference_right_50.y, interference_right_50.z
            );

            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(500)).await;

            // Step 9: Right track at 100%
            info!("Step 9: Motor interference at 100% (right track only)");
            event::raise_event(event::Events::CalibrationStatus {
                header: None,
                line1: if is_full {
                    status_text("Step 9/9")
                } else {
                    status_text("Mag interference")
                },
                line2: status_text("Right track 100%"),
                line3: status_text("Measuring 6s..."),
            })
            .await;

            motor_driver::send_motor_command(MotorCommand::SetTrack {
                track: Track::Left,
                speed: 0,
            })
            .await;
            motor_driver::send_motor_command(MotorCommand::SetTrack {
                track: Track::Right,
                speed: 100,
            })
            .await;
            Timer::after(Duration::from_millis(1000)).await;
            clear_mag_measurement().await;
            Timer::after(Duration::from_millis(500)).await;
            let mag_right_100 = measure_mag_average(500).await;
            let interference_right_100 = subtract_mag(mag_right_100, baseline_mag);
            info!(
                "  Right 100% interference: x={} y={} z={}",
                interference_right_100.x, interference_right_100.y, interference_right_100.z
            );

            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(500)).await;

            mag_x_interference_50 = [interference_all_50.x, interference_left_50.x, interference_right_50.x];
            mag_y_interference_50 = [interference_all_50.y, interference_left_50.y, interference_right_50.y];
            mag_z_interference_50 = [interference_all_50.z, interference_left_50.z, interference_right_50.z];
            mag_x_interference_100 = [
                interference_all_100.x,
                interference_left_100.x,
                interference_right_100.x,
            ];
            mag_y_interference_100 = [
                interference_all_100.y,
                interference_left_100.y,
                interference_right_100.y,
            ];
            mag_z_interference_100 = [
                interference_all_100.z,
                interference_left_100.z,
                interference_right_100.z,
            ];
            mag_completed = true;
        }
    }

    // Step 9: Build calibration struct
    info!("Step 9: Building complete calibration data structure");
    let (line1, line2) = if is_full {
        (status_text("Step 9/9"), status_text("Finalizing"))
    } else {
        (status_text("Finalizing"), None)
    };
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1,
        line2,
        line3: None,
    })
    .await;
    let imu_calibration = flash_storage::ImuCalibration {
        gyro_x_bias,
        gyro_y_bias,
        gyro_z_bias,
        accel_x_bias,
        accel_y_bias,
        accel_z_bias,
        mag_x_bias,
        mag_y_bias,
        mag_z_bias,
        mag_x_interference_50,
        mag_y_interference_50,
        mag_z_interference_50,
        mag_x_interference_100,
        mag_y_interference_100,
        mag_z_interference_100,
    };

    // Step 11: Save to flash
    info!("Step 11: Saving complete calibration to flash");
    info!("╔═══════════════════════════════════════════════════╗");
    info!("║        FINAL CALIBRATION SUMMARY                 ║");
    info!("╠═══════════════════════════════════════════════════╣");
    info!("║  Gyro bias (deg/s):                              ║");
    info!("║    X: {} Y: {} Z: {}", gyro_x_bias, gyro_y_bias, gyro_z_bias);
    info!("║  Accel bias (g):                                 ║");
    info!("║    X: {} Y: {} Z: {}", accel_x_bias, accel_y_bias, accel_z_bias);
    info!("║  Magnetometer hard iron bias (μT):               ║");
    info!("║    X: {} Y: {} Z: {}", mag_x_bias, mag_y_bias, mag_z_bias);
    info!("║  Motor interference patterns captured ✓          ║");
    info!("╚═══════════════════════════════════════════════════╝");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Complete!"),
        line2: status_text("Saving to flash"),
        line3: status_text("Please wait"),
    })
    .await;

    if run_gyro {
        imu_flags.gyro = true;
    }
    if run_accel {
        imu_flags.accel = true;
    }
    if mag_completed {
        imu_flags.mag = true;
    }

    flash_storage::send_flash_command(flash_storage::FlashCommand::SaveData(
        flash_storage::CalibrationDataKind::Imu(imu_calibration),
    ))
    .await;
    flash_storage::send_flash_command(flash_storage::FlashCommand::SaveImuFlags(imu_flags)).await;

    // Brief delay to allow flash operation to complete
    Timer::after(Duration::from_millis(500)).await;

    // Step 12: Apply to IMU task immediately
    info!("Applying calibration to IMU task");
    imu_read::load_imu_calibration(imu_calibration);

    // Display completion
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Complete!"),
        line2: status_text("Calibration saved"),
        line3: None,
    })
    .await;

    event::raise_event(event::Events::CalibrationCompleted).await;

    Timer::after(Duration::from_secs(2)).await;

    info!("=== IMU Calibration Complete ===");
}
