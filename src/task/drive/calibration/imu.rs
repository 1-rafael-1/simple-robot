//! IMU calibration procedure  
//!
//! Calibrates gyroscope, accelerometer, and magnetometer, plus measures
//! motor interference effects on magnetometer for runtime compensation.

use defmt::info;
use embassy_time::{Duration, Timer};

use crate::{
    system::event,
    task::{
        drive::feedback::{
            clear_accel_measurement, clear_gyro_measurement, clear_mag_measurement,
            get_latest_accel_measurement, get_latest_gyro_measurement, measure_mag_average,
            subtract_mag, wait_for_mag_event_timeout,
        },
        flash_storage,
        imu_read,
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
/// - If left_front has calibration factor 0.8
/// - Command "100%" becomes 80% actual (due to calibration)
/// - IMU needs to know interference at 80% actual, not 100% raw
/// - This way, interference compensation matches real-world operation
///
/// **Ordering requirement**:
/// - Motor calibration MUST run before IMU calibration
/// - This ensures IMU measures interference at correctly calibrated speeds
/// - If motor calibration changes later, IMU calibration should be re-run
pub(crate) async fn run_imu_calibration() {
    use heapless::String;

    use crate::{
        system::event,
        task::{flash_storage, imu_read},
    };

    info!("=== Starting IMU Calibration ===");

    // Display calibration header
    event::send_event(event::Events::CalibrationStatus {
        header: Some(String::try_from("IMU Calibration").unwrap()),
        line1: Some(String::try_from("Initializing").unwrap()),
        line2: None,
        line3: None,
    })
    .await;

    // Step 1: Gyroscope and Accelerometer Calibration (stationary)
    info!("Step 1: Gyroscope and Accelerometer Calibration");
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 1/9").unwrap()),
        line2: Some(String::try_from("Gyro/Accel cal").unwrap()),
        line3: Some(String::try_from("Keep still 10s").unwrap()),
    })
    .await;

    // Start IMU readings for calibration
    use core::fmt::Write;
    imu_read::start_imu_readings();
    Timer::after(Duration::from_millis(500)).await; // Wait for IMU to start

    // Collect gyro and accel samples while stationary
    info!("  Collecting gyro/accel samples (stationary)...");
    let mut gyro_x_sum = 0.0f32;
    let mut gyro_y_sum = 0.0f32;
    let mut gyro_z_sum = 0.0f32;
    let mut accel_x_sum = 0.0f32;
    let mut accel_y_sum = 0.0f32;
    let mut accel_z_sum = 0.0f32;
    let num_samples = 1000; // 1000 samples at 100Hz = 10 seconds

    clear_gyro_measurement().await;
    clear_accel_measurement().await;
    Timer::after(Duration::from_millis(100)).await; // Wait for fresh data

    for i in 0..num_samples {
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

        // Update progress every 100 samples
        if i % 100 == 0 {
            let progress = (i * 100) / num_samples;
            let mut line = String::new();
            let _ = write!(line, "Progress: {}%", progress);
            event::send_event(event::Events::CalibrationStatus {
                header: None,
                line1: None,
                line2: None,
                line3: Some(line),
            })
            .await;
        }
    }

    let gyro_x_bias = gyro_x_sum / num_samples as f32;
    let gyro_y_bias = gyro_y_sum / num_samples as f32;
    let gyro_z_bias = gyro_z_sum / num_samples as f32;
    let accel_x_bias = accel_x_sum / num_samples as f32;
    let accel_y_bias = accel_y_sum / num_samples as f32;
    let accel_z_bias = (accel_z_sum / num_samples as f32) - 1.0; // Subtract 1g for gravity

    info!("  ✓ Gyro bias calculated:");
    info!("    X: {} deg/s", gyro_x_bias);
    info!("    Y: {} deg/s", gyro_y_bias);
    info!("    Z: {} deg/s", gyro_z_bias);
    info!("  ✓ Accel bias calculated:");
    info!("    X: {} g", accel_x_bias);
    info!("    Y: {} g", accel_y_bias);
    info!("    Z: {} g (gravity removed)", accel_z_bias);

    // Step 2: Magnetometer Calibration (moving through all axes)
    info!("Step 2: Magnetometer Calibration (CRITICAL - Manual Rotation Required)");
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 2/9").unwrap()),
        line2: Some(String::try_from("MAG CALIBRATION").unwrap()),
        line3: Some(String::try_from("Prepare to move").unwrap()),
    })
    .await;
    Timer::after(Duration::from_secs(3)).await;

    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 2/9").unwrap()),
        line2: Some(String::try_from("ROTATE SLOWLY").unwrap()),
        line3: Some(String::try_from("All axes 60s").unwrap()),
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

    for i in 0..mag_samples {
        if let Some(mag_data) = wait_for_mag_event_timeout(200).await {
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

            // Update progress every 600 samples (every 6 seconds)
            if i % 600 == 0 {
                let seconds_left = (mag_samples - i) / 100;
                let mut line = String::new();
                let _ = write!(line, "{}s left", seconds_left);
                event::send_event(event::Events::CalibrationStatus {
                    header: None,
                    line1: None,
                    line2: None,
                    line3: Some(line),
                })
                .await;
                info!("  Progress: {} seconds remaining...", seconds_left);
            }
        }
    }

    // Calculate hard iron bias (center of min/max sphere)
    let mag_x_bias = (mag_x_max + mag_x_min) / 2.0;
    let mag_y_bias = (mag_y_max + mag_y_min) / 2.0;
    let mag_z_bias = (mag_z_max + mag_z_min) / 2.0;

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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 3/9").unwrap()),
        line2: Some(String::try_from("Baseline (OFF)").unwrap()),
        line3: Some(String::try_from("5 seconds...").unwrap()),
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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 4/9").unwrap()),
        line2: Some(String::try_from("All motors 50%").unwrap()),
        line3: Some(String::try_from("Measuring 6s...").unwrap()),
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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 4/9").unwrap()),
        line2: Some(String::try_from("All motors 100%").unwrap()),
        line3: Some(String::try_from("Measuring 6s...").unwrap()),
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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 5/9").unwrap()),
        line2: Some(String::try_from("Left track 50%").unwrap()),
        line3: Some(String::try_from("Measuring 6s...").unwrap()),
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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 6/9").unwrap()),
        line2: Some(String::try_from("Left track 100%").unwrap()),
        line3: Some(String::try_from("Measuring 6s...").unwrap()),
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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 7/9").unwrap()),
        line2: Some(String::try_from("Right track 50%").unwrap()),
        line3: Some(String::try_from("Measuring 6s...").unwrap()),
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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 8/9").unwrap()),
        line2: Some(String::try_from("Right track 100%").unwrap()),
        line3: Some(String::try_from("Measuring 6s...").unwrap()),
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

    // Step 10: Build calibration struct
    info!("Step 10: Building complete calibration data structure");
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Step 9/9").unwrap()),
        line2: Some(String::try_from("Finalizing...").unwrap()),
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
        mag_x_interference_50: [interference_all_50.x, interference_left_50.x, interference_right_50.x],
        mag_y_interference_50: [interference_all_50.y, interference_left_50.y, interference_right_50.y],
        mag_z_interference_50: [interference_all_50.z, interference_left_50.z, interference_right_50.z],
        mag_x_interference_100: [
            interference_all_100.x,
            interference_left_100.x,
            interference_right_100.x,
        ],
        mag_y_interference_100: [
            interference_all_100.y,
            interference_left_100.y,
            interference_right_100.y,
        ],
        mag_z_interference_100: [
            interference_all_100.z,
            interference_left_100.z,
            interference_right_100.z,
        ],
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
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Complete!").unwrap()),
        line2: Some(String::try_from("Saving...").unwrap()),
        line3: Some(String::try_from("Please wait").unwrap()),
    })
    .await;

    flash_storage::send_flash_command(flash_storage::FlashCommand::SaveData(
        flash_storage::CalibrationDataKind::Imu(imu_calibration),
    ))
    .await;

    // Brief delay to allow flash operation to complete
    Timer::after(Duration::from_millis(500)).await;

    // Step 12: Apply to IMU task immediately
    info!("Applying calibration to IMU task");
    imu_read::load_imu_calibration(imu_calibration);

    // Display completion
    event::send_event(event::Events::CalibrationStatus {
        header: None,
        line1: Some(String::try_from("Complete!").unwrap()),
        line2: Some(String::try_from("Calibration saved").unwrap()),
        line3: None,
    })
    .await;

    Timer::after(Duration::from_secs(2)).await;

    info!("=== IMU Calibration Complete ===");
}
