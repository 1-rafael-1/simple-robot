//! IMU calibration procedure
//!
//! Calibrates gyroscope, accelerometer, and magnetometer, plus measures
//! motor interference effects on magnetometer for runtime compensation.

use core::fmt::Write;

use defmt::info;
use embassy_time::{Duration, Instant, Timer};
use heapless::String;
use nalgebra::Vector3;

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

#[derive(Copy, Clone)]
/// Thresholds and timing used by magnetometer calibration.
struct MagCalibrationConfig {
    /// Max consecutive mag read timeouts before failing.
    timeout_limit: u32,
    /// Max time allowed for the rotation coverage phase.
    max_seconds: u64,
    /// Minimum axis range (μT) required per axis.
    range_min_ut: f32,
    /// Minimum samples required before accepting coverage.
    min_samples: usize,
    /// Samples per averaging window (baseline + interference).
    avg_samples: u16,
    /// Minimum acceptable corrected field magnitude (μT).
    verify_min_ut: f32,
    /// Maximum acceptable corrected field magnitude (μT).
    verify_max_ut: f32,
    /// Max allowed delta from baseline magnitude (μT).
    verify_max_delta_ut: f32,
    /// Status update cadence (seconds).
    status_interval_secs: u64,
}

/// Default magnetometer calibration thresholds.
const MAG_CALIBRATION_CONFIG: MagCalibrationConfig = MagCalibrationConfig {
    timeout_limit: 25, // ~5s at 200ms
    max_seconds: 90,
    range_min_ut: 30.0,
    min_samples: 500,
    avg_samples: 250, // ~5s at 50Hz
    verify_min_ut: 20.0,
    verify_max_ut: 200.0,
    verify_max_delta_ut: 20.0,
    status_interval_secs: 2,
};

#[derive(Copy, Clone)]
/// Motor command step used to measure or verify mag interference.
struct InterferenceStep {
    /// UI label for the step.
    label: &'static str,
    /// Left track command percentage.
    left: i8,
    /// Right track command percentage.
    right: i8,
    /// Nominal speed bucket (50 or 100).
    speed: u8,
    /// Slot index for storing results.
    slot: usize,
}

/// Sequence of steps for interference measurement and verification.
const INTERFERENCE_STEPS: [InterferenceStep; 6] = [
    InterferenceStep {
        label: "All 50%",
        left: 50,
        right: 50,
        speed: 50,
        slot: 0,
    },
    InterferenceStep {
        label: "All 100%",
        left: 100,
        right: 100,
        speed: 100,
        slot: 0,
    },
    InterferenceStep {
        label: "Left 50%",
        left: 50,
        right: 0,
        speed: 50,
        slot: 1,
    },
    InterferenceStep {
        label: "Left 100%",
        left: 100,
        right: 0,
        speed: 100,
        slot: 1,
    },
    InterferenceStep {
        label: "Right 50%",
        left: 0,
        right: 50,
        speed: 50,
        slot: 2,
    },
    InterferenceStep {
        label: "Right 100%",
        left: 0,
        right: 100,
        speed: 100,
        slot: 2,
    },
];

/// Tracks coverage of magnetometer readings during manual rotation.
struct MagCoverage {
    /// Minimum X reading observed.
    x_min: f32,
    /// Maximum X reading observed.
    x_max: f32,
    /// Minimum Y reading observed.
    y_min: f32,
    /// Maximum Y reading observed.
    y_max: f32,
    /// Minimum Z reading observed.
    z_min: f32,
    /// Maximum Z reading observed.
    z_max: f32,
    /// Total samples collected.
    samples: usize,
}

impl MagCoverage {
    /// Create an empty coverage tracker.
    const fn new() -> Self {
        Self {
            x_min: f32::MAX,
            x_max: f32::MIN,
            y_min: f32::MAX,
            y_max: f32::MIN,
            z_min: f32::MAX,
            z_max: f32::MIN,
            samples: 0,
        }
    }

    /// Update min/max ranges with a new magnetometer sample.
    fn update(&mut self, mag: Vector3<f32>) {
        self.samples += 1;
        self.x_min = self.x_min.min(mag.x);
        self.x_max = self.x_max.max(mag.x);
        self.y_min = self.y_min.min(mag.y);
        self.y_max = self.y_max.max(mag.y);
        self.z_min = self.z_min.min(mag.z);
        self.z_max = self.z_max.max(mag.z);
    }

    /// Return the span of each axis (max - min).
    const fn ranges(&self) -> (f32, f32, f32) {
        (
            self.x_max - self.x_min,
            self.y_max - self.y_min,
            self.z_max - self.z_min,
        )
    }

    /// Count how many axes meet the minimum coverage requirement.
    const fn axes_ok(&self, config: MagCalibrationConfig) -> u8 {
        let (x_range, y_range, z_range) = self.ranges();
        (if x_range >= config.range_min_ut { 1 } else { 0 })
            + (if y_range >= config.range_min_ut { 1 } else { 0 })
            + (if z_range >= config.range_min_ut { 1 } else { 0 })
    }
}

/// Captured motor interference vectors at 50% and 100% commands.
struct InterferenceData {
    /// X-axis interference at 50% (all, left, right).
    x_50: [f32; 3],
    /// Y-axis interference at 50% (all, left, right).
    y_50: [f32; 3],
    /// Z-axis interference at 50% (all, left, right).
    z_50: [f32; 3],
    /// X-axis interference at 100% (all, left, right).
    x_100: [f32; 3],
    /// Y-axis interference at 100% (all, left, right).
    y_100: [f32; 3],
    /// Z-axis interference at 100% (all, left, right).
    z_100: [f32; 3],
}

impl InterferenceData {
    /// Create empty interference buffers.
    const fn new() -> Self {
        Self {
            x_50: [0.0; 3],
            y_50: [0.0; 3],
            z_50: [0.0; 3],
            x_100: [0.0; 3],
            y_100: [0.0; 3],
            z_100: [0.0; 3],
        }
    }

    /// Store the measured interference vector for a step.
    fn set(&mut self, step: &InterferenceStep, interference: Vector3<f32>) {
        if step.speed == 50 {
            self.x_50[step.slot] = interference.x;
            self.y_50[step.slot] = interference.y;
            self.z_50[step.slot] = interference.z;
        } else {
            self.x_100[step.slot] = interference.x;
            self.y_100[step.slot] = interference.y;
            self.z_100[step.slot] = interference.z;
        }
    }

    /// Retrieve the interference vector for a step.
    const fn vector_for(&self, step: &InterferenceStep) -> Vector3<f32> {
        if step.speed == 50 {
            Vector3::new(self.x_50[step.slot], self.y_50[step.slot], self.z_50[step.slot])
        } else {
            Vector3::new(self.x_100[step.slot], self.y_100[step.slot], self.z_100[step.slot])
        }
    }
}

/// Output of magnetometer calibration.
struct MagCalibrationResult {
    /// Hard-iron bias in μT.
    bias: Vector3<f32>,
    /// Soft-iron scale factors.
    scale: Vector3<f32>,
    /// Motor interference measurements.
    interference: InterferenceData,
}

/// Collect stationary samples and compute gyro/accel bias vectors.
async fn run_gyro_accel_calibration(run_gyro: bool, run_accel: bool) -> (Vector3<f32>, Vector3<f32>) {
    use crate::system::event;

    let step_label = if run_gyro && run_accel {
        "Gyro+Accel cal"
    } else if run_gyro {
        "Gyro calibration"
    } else {
        "Accel calibration"
    };
    info!("Step 1: {=str}", step_label);
    let (line1, line2, line3) = (status_text(step_label), status_text("Keep still 10s"), None);
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1,
        line2,
        line3,
    })
    .await;

    info!("  Collecting calibration samples (stationary)...");
    let mut gyro_sum = Vector3::new(0.0f32, 0.0f32, 0.0f32);
    let mut accel_sum = Vector3::new(0.0f32, 0.0f32, 0.0f32);
    let num_samples: u16 = 500; // 500 samples at 50Hz = 10 seconds
    let num_samples_f32 = f32::from(num_samples);

    if run_gyro {
        clear_gyro_measurement().await;
    }
    if run_accel {
        clear_accel_measurement().await;
    }
    Timer::after(Duration::from_millis(100)).await;

    for i in 0..num_samples {
        if run_gyro {
            let start = embassy_time::Instant::now();
            while embassy_time::Instant::now().duration_since(start).as_millis() < 20 {
                if let Some(gyro_data) = get_latest_gyro_measurement().await {
                    gyro_sum += gyro_data;
                    break;
                }
                Timer::after(Duration::from_millis(1)).await;
            }
        }

        if run_accel {
            let start = embassy_time::Instant::now();
            while embassy_time::Instant::now().duration_since(start).as_millis() < 20 {
                if let Some(accel_data) = get_latest_accel_measurement().await {
                    accel_sum += accel_data;
                    break;
                }
                Timer::after(Duration::from_millis(1)).await;
            }
        }

        if i % 50 == 0 {
            let progress = (u32::from(i) * 100) / u32::from(num_samples);
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

    let computed_gyro_bias = if run_gyro {
        gyro_sum / num_samples_f32
    } else {
        Vector3::new(0.0, 0.0, 0.0)
    };
    let computed_accel_bias = if run_accel {
        let mut bias = accel_sum / num_samples_f32;
        bias.z -= 1.0;
        bias
    } else {
        Vector3::new(0.0, 0.0, 0.0)
    };

    if run_gyro {
        info!("  ✓ Gyro bias calculated:");
        info!("    X: {} deg/s", computed_gyro_bias.x);
        info!("    Y: {} deg/s", computed_gyro_bias.y);
        info!("    Z: {} deg/s", computed_gyro_bias.z);
    }

    if run_accel {
        info!("  ✓ Accel bias calculated:");
        info!("    X: {} g", computed_accel_bias.x);
        info!("    Y: {} g", computed_accel_bias.y);
        info!("    Z: {} g (gravity removed)", computed_accel_bias.z);
    }

    (computed_gyro_bias, computed_accel_bias)
}

/// Gather magnetometer samples until coverage and sample thresholds are met.
async fn measure_mag_coverage(config: MagCalibrationConfig) -> Option<MagCoverage> {
    use crate::system::event;

    let mut coverage = MagCoverage::new();
    let mut mag_timeout_streak: u32 = 0;
    let mut last_status_secs: u64 = 0;
    let start_time = Instant::now();
    let mut mag_failed = false;

    loop {
        let elapsed_secs = Instant::now().duration_since(start_time).as_secs();
        if elapsed_secs >= config.max_seconds {
            mag_failed = true;
            break;
        }

        clear_mag_measurement().await;
        if let Some(mag_data) = wait_for_mag_event_timeout(200).await {
            coverage.update(mag_data);
            mag_timeout_streak = 0;
        } else {
            mag_timeout_streak += 1;
        }

        let axes_ok = coverage.axes_ok(config);

        if axes_ok == 3 && coverage.samples >= config.min_samples {
            break;
        }

        if elapsed_secs != last_status_secs && elapsed_secs.is_multiple_of(config.status_interval_secs) {
            last_status_secs = elapsed_secs;
            let (x_range, y_range, z_range) = coverage.ranges();
            let mut line_axes = String::new();
            let _ = write!(line_axes, "Axes {axes_ok}/3");
            let mut line_ranges = String::new();
            let _ = write!(line_ranges, "X{x_range:.0} Y{y_range:.0} Z{z_range:.0}");
            event::raise_event(event::Events::CalibrationStatus {
                header: None,
                line1: None,
                line2: Some(line_axes),
                line3: Some(line_ranges),
            })
            .await;
        }

        if mag_timeout_streak >= config.timeout_limit {
            mag_failed = true;
            break;
        }
    }

    let axes_ok = coverage.axes_ok(config);
    if mag_failed || coverage.samples == 0 || axes_ok < 3 {
        info!(
            "Mag calibration failed (samples={}, axes_ok={})",
            coverage.samples, axes_ok
        );
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("MAG FAILED"),
            line2: status_text("Rotate more"),
            line3: status_text("All axes"),
        })
        .await;
        return None;
    }

    Some(coverage)
}

/// Measure motor-induced magnetometer interference across predefined steps.
async fn measure_mag_interference(config: MagCalibrationConfig, baseline_mag: Vector3<f32>) -> InterferenceData {
    use crate::system::event;

    let mut data = InterferenceData::new();

    for step in &INTERFERENCE_STEPS {
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("Mag interference"),
            line2: status_text(step.label),
            line3: status_text("Measuring 5s"),
        })
        .await;

        motor_driver::send_motor_command(MotorCommand::SetTrack {
            track: Track::Left,
            speed: step.left,
        })
        .await;
        motor_driver::send_motor_command(MotorCommand::SetTrack {
            track: Track::Right,
            speed: step.right,
        })
        .await;
        Timer::after(Duration::from_millis(1000)).await;
        clear_mag_measurement().await;
        let mag_avg = measure_mag_average(config.avg_samples).await;
        let interference = subtract_mag(mag_avg, baseline_mag);

        data.set(step, interference);

        motor_driver::send_motor_command(MotorCommand::CoastAll).await;
        Timer::after(Duration::from_millis(500)).await;
    }

    data
}

/// Verify interference compensation keeps the field within expected limits.
async fn verify_mag_interference(
    config: MagCalibrationConfig,
    bias: Vector3<f32>,
    scale: Vector3<f32>,
    baseline_mag: Vector3<f32>,
    interference: &InterferenceData,
) -> bool {
    use crate::system::event;

    let baseline_corrected = Vector3::new(
        (baseline_mag.x - bias.x) * scale.x,
        (baseline_mag.y - bias.y) * scale.y,
        (baseline_mag.z - bias.z) * scale.z,
    );
    let baseline_norm = baseline_corrected.norm();

    for (index, step) in INTERFERENCE_STEPS.iter().enumerate() {
        let mut line = String::new();
        let _ = write!(line, "Step {}/{}", index + 1, INTERFERENCE_STEPS.len());
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("Mag verify"),
            line2: status_text(step.label),
            line3: Some(line),
        })
        .await;

        motor_driver::send_motor_command(MotorCommand::SetTrack {
            track: Track::Left,
            speed: step.left,
        })
        .await;
        motor_driver::send_motor_command(MotorCommand::SetTrack {
            track: Track::Right,
            speed: step.right,
        })
        .await;
        Timer::after(Duration::from_millis(1000)).await;
        clear_mag_measurement().await;
        let mag_avg = measure_mag_average(config.avg_samples).await;

        let interference_vec = interference.vector_for(step);
        let mut corrected = subtract_mag(mag_avg, interference_vec);
        corrected.x = (corrected.x - bias.x) * scale.x;
        corrected.y = (corrected.y - bias.y) * scale.y;
        corrected.z = (corrected.z - bias.z) * scale.z;

        let mag_norm = corrected.norm();
        if mag_norm < config.verify_min_ut
            || mag_norm > config.verify_max_ut
            || (mag_norm - baseline_norm).abs() > config.verify_max_delta_ut
        {
            return false;
        }

        motor_driver::send_motor_command(MotorCommand::CoastAll).await;
        Timer::after(Duration::from_millis(500)).await;
    }

    true
}

/// Run the magnetometer calibration flow and return results on success.
async fn run_mag_calibration_steps(config: MagCalibrationConfig) -> Option<MagCalibrationResult> {
    use crate::system::event;

    info!("Step 2: Magnetometer Calibration (manual rotation)");
    let (line1, line2, line3) = (status_text("Mag calibration"), status_text("Prepare to move"), None);
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1,
        line2,
        line3,
    })
    .await;
    Timer::after(Duration::from_secs(3)).await;

    let (line1, line2, line3) = (
        status_text("Mag calibration"),
        status_text("Rotate slowly"),
        status_text("Pitch/Roll/Yaw"),
    );
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1,
        line2,
        line3,
    })
    .await;

    let coverage = measure_mag_coverage(config).await?;
    let (x_range, y_range, z_range) = coverage.ranges();

    let mag_bias = Vector3::new(
        f32::midpoint(coverage.x_max, coverage.x_min),
        f32::midpoint(coverage.y_max, coverage.y_min),
        f32::midpoint(coverage.z_max, coverage.z_min),
    );

    let x_radius = x_range / 2.0;
    let y_radius = y_range / 2.0;
    let z_radius = z_range / 2.0;
    let avg_radius = (x_radius + y_radius + z_radius) / 3.0;

    let mag_scale = Vector3::new(avg_radius / x_radius, avg_radius / y_radius, avg_radius / z_radius);

    info!("  ✓ Magnetometer coverage complete!");
    info!(
        "  Hard iron bias (μT): X={} Y={} Z={}",
        mag_bias.x, mag_bias.y, mag_bias.z
    );
    info!(
        "  Soft iron scale: X={} Y={} Z={}",
        mag_scale.x, mag_scale.y, mag_scale.z
    );

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

    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Mag interference"),
        line2: status_text("Baseline (OFF)"),
        line3: status_text("Measuring 5s"),
    })
    .await;

    clear_mag_measurement().await;
    Timer::after(Duration::from_millis(500)).await;
    let baseline_mag = measure_mag_average(config.avg_samples).await;

    let interference = measure_mag_interference(config, baseline_mag).await;
    let verify_ok = verify_mag_interference(config, mag_bias, mag_scale, baseline_mag, &interference).await;

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    Timer::after(Duration::from_millis(500)).await;

    if !verify_ok {
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("VERIFY FAIL"),
            line2: status_text("Recalibrate"),
            line3: status_text("Motors/mag"),
        })
        .await;
        return None;
    }

    Some(MagCalibrationResult {
        bias: mag_bias,
        scale: mag_scale,
        interference,
    })
}

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
async fn run_gyro_calibration() {
    use crate::{
        system::event,
        task::{io::flash_storage, sensors::imu as imu_read},
    };

    info!("=== Starting IMU Gyro Calibration ===");

    event::raise_event(event::Events::CalibrationStatus {
        header: status_text("IMU Calibration"),
        line1: status_text("Initializing"),
        line2: None,
        line3: None,
    })
    .await;

    imu_read::start_imu_readings();
    Timer::after(Duration::from_millis(500)).await;

    let cached_calibration = flash_storage::get_cached_imu_calibration().await.unwrap_or_default();
    let mut imu_calibration = cached_calibration;
    let mut imu_flags = flash_storage::get_cached_imu_flags().await.unwrap_or_default();

    let (gyro_bias, _) = run_gyro_accel_calibration(true, false).await;
    imu_calibration.gyro_x_bias = gyro_bias.x;
    imu_calibration.gyro_y_bias = gyro_bias.y;
    imu_calibration.gyro_z_bias = gyro_bias.z;
    imu_flags.gyro = true;

    info!("Saving gyro calibration to flash");
    info!("╔═══════════════════════════════════════════════════╗");
    info!("║     FINAL CALIBRATION SUMMARY (GYRO)             ║");
    info!("╠═══════════════════════════════════════════════════╣");
    info!("║  Gyro bias (deg/s):                              ║");
    info!(
        "║    X: {} Y: {} Z: {}",
        imu_calibration.gyro_x_bias, imu_calibration.gyro_y_bias, imu_calibration.gyro_z_bias
    );
    info!("╚═══════════════════════════════════════════════════╝");

    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Complete!"),
        line2: status_text("Saving to flash"),
        line3: status_text("Please wait"),
    })
    .await;

    flash_storage::send_flash_command(flash_storage::FlashCommand::SaveData(
        flash_storage::CalibrationDataKind::Imu(imu_calibration),
    ))
    .await;
    flash_storage::send_flash_command(flash_storage::FlashCommand::SaveImuFlags(imu_flags)).await;

    Timer::after(Duration::from_millis(500)).await;

    info!("Applying gyro calibration to IMU task");
    imu_read::load_imu_calibration(imu_calibration);

    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Complete!"),
        line2: status_text("Calibration saved"),
        line3: None,
    })
    .await;

    event::raise_event(event::Events::CalibrationCompleted).await;
    Timer::after(Duration::from_secs(2)).await;

    info!("=== IMU Gyro Calibration Complete ===");
}

#[allow(clippy::too_many_lines)]
#[allow(clippy::similar_names)]
#[allow(clippy::cast_precision_loss)]
/// Calibrate the accelerometer and persist results.
async fn run_accel_calibration() {
    use crate::{
        system::event,
        task::{io::flash_storage, sensors::imu as imu_read},
    };

    info!("=== Starting IMU Accel Calibration ===");

    event::raise_event(event::Events::CalibrationStatus {
        header: status_text("IMU Calibration"),
        line1: status_text("Initializing"),
        line2: None,
        line3: None,
    })
    .await;

    imu_read::start_imu_readings();
    Timer::after(Duration::from_millis(500)).await;

    let cached_calibration = flash_storage::get_cached_imu_calibration().await.unwrap_or_default();
    let mut imu_calibration = cached_calibration;
    let mut imu_flags = flash_storage::get_cached_imu_flags().await.unwrap_or_default();

    let (_, accel_bias) = run_gyro_accel_calibration(false, true).await;
    imu_calibration.accel_x_bias = accel_bias.x;
    imu_calibration.accel_y_bias = accel_bias.y;
    imu_calibration.accel_z_bias = accel_bias.z;
    imu_flags.accel = true;

    info!("Saving accel calibration to flash");
    info!("╔═══════════════════════════════════════════════════╗");
    info!("║    FINAL CALIBRATION SUMMARY (ACCEL)             ║");
    info!("╠═══════════════════════════════════════════════════╣");
    info!("║  Accel bias (g):                                 ║");
    info!(
        "║    X: {} Y: {} Z: {}",
        imu_calibration.accel_x_bias, imu_calibration.accel_y_bias, imu_calibration.accel_z_bias
    );
    info!("╚═══════════════════════════════════════════════════╝");

    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Complete!"),
        line2: status_text("Saving to flash"),
        line3: status_text("Please wait"),
    })
    .await;

    flash_storage::send_flash_command(flash_storage::FlashCommand::SaveData(
        flash_storage::CalibrationDataKind::Imu(imu_calibration),
    ))
    .await;
    flash_storage::send_flash_command(flash_storage::FlashCommand::SaveImuFlags(imu_flags)).await;

    Timer::after(Duration::from_millis(500)).await;

    info!("Applying accel calibration to IMU task");
    imu_read::load_imu_calibration(imu_calibration);

    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Complete!"),
        line2: status_text("Calibration saved"),
        line3: None,
    })
    .await;

    event::raise_event(event::Events::CalibrationCompleted).await;
    Timer::after(Duration::from_secs(2)).await;

    info!("=== IMU Accel Calibration Complete ===");
}

#[allow(clippy::too_many_lines)]
#[allow(clippy::similar_names)]
#[allow(clippy::cast_precision_loss)]
/// Calibrate the magnetometer and persist results.
async fn run_mag_calibration() {
    use crate::{
        system::event,
        task::{io::flash_storage, sensors::imu as imu_read},
    };

    info!("=== Starting IMU Mag Calibration ===");

    event::raise_event(event::Events::CalibrationStatus {
        header: status_text("IMU Calibration"),
        line1: status_text("Initializing"),
        line2: None,
        line3: None,
    })
    .await;

    imu_read::start_imu_readings();
    Timer::after(Duration::from_millis(500)).await;

    let cached_calibration = flash_storage::get_cached_imu_calibration().await.unwrap_or_default();
    let mut imu_calibration = cached_calibration;
    let mut imu_flags = flash_storage::get_cached_imu_flags().await.unwrap_or_default();

    if let Some(mag_result) = run_mag_calibration_steps(MAG_CALIBRATION_CONFIG).await {
        imu_calibration.mag_x_bias = mag_result.bias.x;
        imu_calibration.mag_y_bias = mag_result.bias.y;
        imu_calibration.mag_z_bias = mag_result.bias.z;
        imu_calibration.mag_x_scale = mag_result.scale.x;
        imu_calibration.mag_y_scale = mag_result.scale.y;
        imu_calibration.mag_z_scale = mag_result.scale.z;
        imu_calibration.mag_x_interference_50 = mag_result.interference.x_50;
        imu_calibration.mag_y_interference_50 = mag_result.interference.y_50;
        imu_calibration.mag_z_interference_50 = mag_result.interference.z_50;
        imu_calibration.mag_x_interference_100 = mag_result.interference.x_100;
        imu_calibration.mag_y_interference_100 = mag_result.interference.y_100;
        imu_calibration.mag_z_interference_100 = mag_result.interference.z_100;
        imu_flags.mag = true;
    } else {
        info!("Mag calibration failed; keeping previous values");
    }

    info!("Saving mag calibration to flash");
    info!("╔═══════════════════════════════════════════════════╗");
    info!("║     FINAL CALIBRATION SUMMARY (MAG)              ║");
    info!("╠═══════════════════════════════════════════════════╣");
    info!("║  Magnetometer hard iron bias (μT):               ║");
    info!(
        "║    X: {} Y: {} Z: {}",
        imu_calibration.mag_x_bias, imu_calibration.mag_y_bias, imu_calibration.mag_z_bias
    );
    info!("║  Magnetometer soft iron scale:                   ║");
    info!(
        "║    X: {} Y: {} Z: {}",
        imu_calibration.mag_x_scale, imu_calibration.mag_y_scale, imu_calibration.mag_z_scale
    );
    info!("║  Motor interference patterns captured ✓          ║");
    info!("╚═══════════════════════════════════════════════════╝");

    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Complete!"),
        line2: status_text("Saving to flash"),
        line3: status_text("Please wait"),
    })
    .await;

    flash_storage::send_flash_command(flash_storage::FlashCommand::SaveData(
        flash_storage::CalibrationDataKind::Imu(imu_calibration),
    ))
    .await;
    flash_storage::send_flash_command(flash_storage::FlashCommand::SaveImuFlags(imu_flags)).await;

    Timer::after(Duration::from_millis(500)).await;

    info!("Applying mag calibration to IMU task");
    imu_read::load_imu_calibration(imu_calibration);

    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Complete!"),
        line2: status_text("Calibration saved"),
        line3: None,
    })
    .await;

    event::raise_event(event::Events::CalibrationCompleted).await;
    Timer::after(Duration::from_secs(2)).await;

    info!("=== IMU Mag Calibration Complete ===");
}

/// Dispatch the requested IMU calibration routine.
pub async fn run_imu_calibration(kind: ImuCalibrationKind) {
    match kind {
        ImuCalibrationKind::Gyro => run_gyro_calibration().await,
        ImuCalibrationKind::Accel => run_accel_calibration().await,
        ImuCalibrationKind::Mag => run_mag_calibration().await,
    }
}
