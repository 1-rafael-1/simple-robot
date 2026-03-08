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
    #[allow(dead_code)]
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

/// Maximum time to wait for a fresh gyro/accel sample during calibration.
/// Must be comfortably above the IMU sampling period (~20.4 ms at 48.9 Hz).
const GYRO_ACCEL_SAMPLE_TIMEOUT: Duration = Duration::from_millis(30);

/// Ensures IMU readings are stopped (and fusion mode restored) after calibration completes.
struct ImuReadingsGuard {
    /// Fusion mode to restore after calibration (if any).
    restore_fusion_mode: Option<crate::task::sensors::imu::AhrsFusionMode>,
}

impl ImuReadingsGuard {
    /// Start IMU readings without changing fusion mode.
    fn start() -> Self {
        crate::task::sensors::imu::start_imu_readings();
        Self {
            restore_fusion_mode: None,
        }
    }

    /// Start IMU readings and set a temporary fusion mode.
    fn start_with_fusion_mode(
        target_mode: crate::task::sensors::imu::AhrsFusionMode,
        restore_mode: crate::task::sensors::imu::AhrsFusionMode,
    ) -> Self {
        crate::task::sensors::imu::start_imu_readings();
        crate::task::sensors::imu::set_ahrs_fusion_mode(target_mode);
        Self {
            restore_fusion_mode: Some(restore_mode),
        }
    }
}

impl Drop for ImuReadingsGuard {
    fn drop(&mut self) {
        if let Some(mode) = self.restore_fusion_mode {
            crate::task::sensors::imu::set_ahrs_fusion_mode(mode);
        }
        crate::task::sensors::imu::stop_imu_readings();
    }
}

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

/// Rotation guidance step for magnetometer coverage.
struct MagRotationStep {
    /// Label to show on the OLED for this step.
    label: &'static str,
    /// Require X-axis coverage for this step.
    require_x: bool,
    /// Require Y-axis coverage for this step.
    require_y: bool,
    /// Require Z-axis coverage for this step.
    require_z: bool,
    /// Minimum yaw heading span in degrees for this step (None = not used).
    min_heading_span_deg: Option<f32>,
}

/// Magnetometer calibration phase machine.
#[derive(Copy, Clone, Eq, PartialEq)]
#[allow(dead_code)]
enum MagCalibrationPhase {
    /// Manual yaw rotation phase.
    ManualYaw,
    /// Manual pitch rotation phase.
    ManualPitch,
    /// Manual roll rotation phase.
    ManualRoll,
    /// Settling delay before motor phase.
    SettleDelay,
    /// Baseline magnetometer measurement with motors off.
    MotorBaseline,
    /// Motor interference measurement phase.
    MotorMeasure,
    /// Motor interference verification phase.
    MotorVerify,
    /// Save calibration results.
    Save,
    /// Completed calibration flow.
    Done,
    /// Calibration failed.
    Failed,
}

impl MagCalibrationPhase {
    /// Human-readable phase name.
    const fn label(self) -> &'static str {
        match self {
            Self::ManualYaw => "P1 YAW",
            Self::ManualPitch => "P2 PITCH",
            Self::ManualRoll => "P3 ROLL",
            Self::SettleDelay => "P4 WAIT20",
            Self::MotorBaseline => "P5 BASE",
            Self::MotorMeasure => "P6 MOTOR",
            Self::MotorVerify => "P7 VERIFY",
            Self::Save => "P8 SAVE",
            Self::Done => "DONE",
            Self::Failed => "FAIL",
        }
    }

    /// 1-based phase number and total count for user-visible progress.
    const fn progress(self) -> Option<(u8, u8)> {
        match self {
            Self::ManualYaw => Some((1, 8)),
            Self::ManualPitch => Some((2, 8)),
            Self::ManualRoll => Some((3, 8)),
            Self::SettleDelay => Some((4, 8)),
            Self::MotorBaseline => Some((5, 8)),
            Self::MotorMeasure => Some((6, 8)),
            Self::MotorVerify => Some((7, 8)),
            Self::Save => Some((8, 8)),
            Self::Done | Self::Failed => None,
        }
    }
}

/// Sequence of user-guided rotations for magnetometer coverage.
const MAG_ROTATION_STEPS: [MagRotationStep; 3] = [
    MagRotationStep {
        label: "Yaw flat",
        require_x: false,
        require_y: false,
        require_z: false,
        min_heading_span_deg: Some(50.0),
    },
    MagRotationStep {
        label: "Pitch",
        require_x: true,
        require_y: false,
        require_z: true,
        min_heading_span_deg: None,
    },
    MagRotationStep {
        label: "Roll",
        require_x: false,
        require_y: true,
        require_z: true,
        min_heading_span_deg: None,
    },
];

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

/// Gyro/accel calibration output with optional biases.
struct GyroAccelCalibrationResult {
    /// Gyro bias if enough samples were captured.
    gyro_bias: Option<Vector3<f32>>,
    /// Accel bias if enough samples were captured.
    accel_bias: Option<Vector3<f32>>,
}

/// Collect stationary samples and compute gyro/accel bias vectors.
async fn run_gyro_accel_calibration(run_gyro: bool, run_accel: bool) -> GyroAccelCalibrationResult {
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
    let mut gyro_samples: u16 = 0;
    let mut accel_samples: u16 = 0;
    let num_samples: u16 = 500; // 500 samples at 50Hz = 10 seconds
    let min_samples = (num_samples * 8) / 10;

    Timer::after(Duration::from_millis(100)).await;

    for i in 0..num_samples {
        if run_gyro {
            clear_gyro_measurement().await;
            let deadline = embassy_time::Instant::now() + GYRO_ACCEL_SAMPLE_TIMEOUT;
            while embassy_time::Instant::now() < deadline {
                if let Some(gyro_data) = get_latest_gyro_measurement().await {
                    gyro_sum += gyro_data;
                    gyro_samples += 1;
                    break;
                }
                Timer::after(Duration::from_millis(1)).await;
            }
        }

        if run_accel {
            clear_accel_measurement().await;
            let deadline = embassy_time::Instant::now() + GYRO_ACCEL_SAMPLE_TIMEOUT;
            while embassy_time::Instant::now() < deadline {
                if let Some(accel_data) = get_latest_accel_measurement().await {
                    accel_sum += accel_data;
                    accel_samples += 1;
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

    if (run_gyro && gyro_samples < min_samples) || (run_accel && accel_samples < min_samples) {
        info!(
            "Calibration sample shortfall (gyro={}/{} accel={}/{})",
            gyro_samples, num_samples, accel_samples, num_samples
        );
    }

    let gyro_bias = if run_gyro && gyro_samples >= min_samples && gyro_samples > 0 {
        Some(gyro_sum / f32::from(gyro_samples))
    } else {
        None
    };
    let accel_bias = if run_accel && accel_samples >= min_samples && accel_samples > 0 {
        let mut bias = accel_sum / f32::from(accel_samples);
        bias.z -= 1.0;
        Some(bias)
    } else {
        None
    };

    if let Some(bias) = gyro_bias {
        info!("  ✓ Gyro bias calculated:");
        info!("    X: {} deg/s", bias.x);
        info!("    Y: {} deg/s", bias.y);
        info!("    Z: {} deg/s", bias.z);
    } else if run_gyro {
        info!("  ! Gyro calibration failed (insufficient samples)");
    }

    if let Some(bias) = accel_bias {
        info!("  ✓ Accel bias calculated:");
        info!("    X: {} g", bias.x);
        info!("    Y: {} g", bias.y);
        info!("    Z: {} g (gravity removed)", bias.z);
    } else if run_accel {
        info!("  ! Accel calibration failed (insufficient samples)");
    }

    GyroAccelCalibrationResult { gyro_bias, accel_bias }
}

/// Emit a phase transition log and update OLED with phase progress.
async fn enter_mag_phase(phase: MagCalibrationPhase, detail: Option<&'static str>) {
    use crate::system::event;

    if let Some((index, total)) = phase.progress() {
        info!("Mag phase transition -> {} ({}/{})", phase.label(), index, total);
        let mut line1 = String::new();
        let _ = write!(line1, "Phase {index}/{total}");
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: Some(line1),
            line2: status_text(phase.label()),
            line3: detail.and_then(status_text),
        })
        .await;
    } else {
        info!("Mag phase transition -> {}", phase.label());
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text(phase.label()),
            line2: detail.and_then(status_text),
            line3: None,
        })
        .await;
    }
}

/// Guide one explicit manual rotation phase and collect coverage.
#[allow(clippy::too_many_lines)]
#[allow(clippy::option_if_let_else)]
async fn measure_mag_rotation_phase(
    phase: MagCalibrationPhase,
    step: &MagRotationStep,
    config: MagCalibrationConfig,
    coverage: &mut MagCoverage,
) -> bool {
    use crate::system::event;

    enter_mag_phase(phase, Some("Rotate as shown")).await;

    let step_timeout = (config.max_seconds / MAG_ROTATION_STEPS.len() as u64).max(10);
    let min_step_samples = (config.min_samples / MAG_ROTATION_STEPS.len()).max(1);

    let mut step_coverage = MagCoverage::new();
    let mut mag_timeout_streak: u32 = 0;
    let start_time = Instant::now();
    let mut heading_min: f32 = f32::MAX;
    let mut heading_max: f32 = f32::MIN;
    let mut last_heading: Option<f32> = None;
    let mut heading_offset: f32 = 0.0;
    let mut last_log_ms: u32 = 0;

    loop {
        let elapsed_secs = Instant::now().duration_since(start_time).as_secs();
        if elapsed_secs >= step_timeout {
            info!("Mag phase timeout at {}", phase.label());
            return false;
        }

        clear_mag_measurement().await;
        if let Some(mag_data) = wait_for_mag_event_timeout(200).await {
            coverage.update(mag_data);
            step_coverage.update(mag_data);
            mag_timeout_streak = 0;

            if step.min_heading_span_deg.is_some() {
                let heading_deg = libm::atan2f(mag_data.y, mag_data.x).to_degrees();
                if let Some(prev) = last_heading {
                    let delta = heading_deg - prev;
                    if delta > 180.0 {
                        heading_offset -= 360.0;
                    } else if delta < -180.0 {
                        heading_offset += 360.0;
                    }
                }
                last_heading = Some(heading_deg);
                let unwrapped = heading_deg + heading_offset;
                heading_min = heading_min.min(unwrapped);
                heading_max = heading_max.max(unwrapped);
            }
        } else {
            mag_timeout_streak += 1;
        }

        let (x_range, y_range, z_range) = step_coverage.ranges();
        let heading_span = if step.min_heading_span_deg.is_some() {
            heading_max - heading_min
        } else {
            0.0
        };
        let heading_span_ok = if let Some(min_span) = step.min_heading_span_deg {
            heading_span >= min_span
        } else {
            true
        };

        let step_ok = (!step.require_x || x_range >= config.range_min_ut)
            && (!step.require_y || y_range >= config.range_min_ut)
            && (!step.require_z || z_range >= config.range_min_ut)
            && heading_span_ok;

        if step.min_heading_span_deg.is_some() {
            #[allow(clippy::cast_possible_truncation)]
            let now_ms = Instant::now().as_millis() as u32;
            if now_ms.wrapping_sub(last_log_ms) >= 1000 {
                last_log_ms = now_ms;
                info!(
                    "Yaw span {} deg (min={} max={}) samples={}",
                    heading_span, heading_min, heading_max, step_coverage.samples
                );
            }
        }

        if step_ok && step_coverage.samples >= min_step_samples {
            info!(
                "Mag phase complete: {} (samples={})",
                phase.label(),
                step_coverage.samples
            );
            event::raise_event(event::Events::CalibrationStatus {
                header: None,
                line1: status_text("Phase OK"),
                line2: status_text(step.label),
                line3: status_text("Continue"),
            })
            .await;
            Timer::after(Duration::from_millis(500)).await;
            return true;
        }

        let (line1, line2, line3) = match phase {
            MagCalibrationPhase::ManualYaw => (
                status_text("P1 YAW"),
                status_text("Keep flat"),
                status_text("Spin on table"),
            ),
            MagCalibrationPhase::ManualPitch => (
                status_text("P2 PITCH"),
                status_text("Tilt nose"),
                status_text("Up / down"),
            ),
            MagCalibrationPhase::ManualRoll => (status_text("P3 ROLL"), status_text("Tilt left"), status_text("Right")),
            _ => (
                status_text(phase.label()),
                status_text("Move slowly"),
                status_text("Follow prompt"),
            ),
        };

        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1,
            line2,
            line3,
        })
        .await;

        if mag_timeout_streak >= config.timeout_limit {
            info!("Mag phase timeout streak exceeded at {}", phase.label());
            return false;
        }
    }
}

/// Guide the user through explicit axis-specific phases and collect coverage.
async fn measure_mag_coverage(config: MagCalibrationConfig) -> Option<MagCoverage> {
    use crate::system::event;

    let mut coverage = MagCoverage::new();

    if !measure_mag_rotation_phase(
        MagCalibrationPhase::ManualYaw,
        &MAG_ROTATION_STEPS[0],
        config,
        &mut coverage,
    )
    .await
    {
        enter_mag_phase(MagCalibrationPhase::Failed, Some("Yaw incomplete")).await;
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("MAG FAILED"),
            line2: status_text("Yaw incomplete"),
            line3: status_text("Retry"),
        })
        .await;
        Timer::after(Duration::from_secs(2)).await;
        return None;
    }

    if !measure_mag_rotation_phase(
        MagCalibrationPhase::ManualPitch,
        &MAG_ROTATION_STEPS[1],
        config,
        &mut coverage,
    )
    .await
    {
        enter_mag_phase(MagCalibrationPhase::Failed, Some("Pitch incomplete")).await;
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("MAG FAILED"),
            line2: status_text("Pitch incomplete"),
            line3: status_text("Retry"),
        })
        .await;
        Timer::after(Duration::from_secs(2)).await;
        return None;
    }

    if !measure_mag_rotation_phase(
        MagCalibrationPhase::ManualRoll,
        &MAG_ROTATION_STEPS[2],
        config,
        &mut coverage,
    )
    .await
    {
        enter_mag_phase(MagCalibrationPhase::Failed, Some("Roll incomplete")).await;
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("MAG FAILED"),
            line2: status_text("Roll incomplete"),
            line3: status_text("Retry"),
        })
        .await;
        Timer::after(Duration::from_secs(2)).await;
        return None;
    }

    let axes_ok = coverage.axes_ok(config);
    if coverage.samples == 0 || axes_ok < 3 {
        info!(
            "Mag calibration failed after manual phases (samples={}, axes_ok={})",
            coverage.samples, axes_ok
        );
        enter_mag_phase(MagCalibrationPhase::Failed, Some("Coverage insufficient")).await;
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("MAG FAILED"),
            line2: status_text("Rotate more"),
            line3: status_text("All axes"),
        })
        .await;
        Timer::after(Duration::from_secs(2)).await;
        return None;
    }

    Some(coverage)
}

/// Enable or disable both motor driver chips during mag calibration.
async fn set_motor_drivers_enabled(enabled: bool) {
    motor_driver::send_motor_command(MotorCommand::SetDriverEnable {
        track: Track::Left,
        enabled,
    })
    .await;
    motor_driver::send_motor_command(MotorCommand::SetDriverEnable {
        track: Track::Right,
        enabled,
    })
    .await;
}

/// Measure motor-induced magnetometer interference across predefined steps.
async fn measure_mag_interference(config: MagCalibrationConfig, baseline_mag: Vector3<f32>) -> InterferenceData {
    use crate::system::event;

    let mut data = InterferenceData::new();

    for step in &INTERFERENCE_STEPS {
        info!("Motor interference step start: {=str}", step.label);
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
        info!("Motor interference step done: {=str}", step.label);

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
        info!(
            "Motor verify step {}/{}: {=str}",
            index + 1,
            INTERFERENCE_STEPS.len(),
            step.label
        );
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
#[allow(clippy::too_many_lines)]
async fn run_mag_calibration_steps(config: MagCalibrationConfig) -> Option<MagCalibrationResult> {
    use crate::system::event;

    info!("Step 2: Magnetometer Calibration (strict phase machine)");
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

    let Some(coverage) = measure_mag_coverage(config).await else {
        info!("MAG CAL RESULT: reached_motor_phase=false, reason=manual coverage failed");
        return None;
    };
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

    enter_mag_phase(MagCalibrationPhase::SettleDelay, Some("Set robot down")).await;
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Set down"),
        line2: status_text("Hold still"),
        line3: status_text("Motors in 20s"),
    })
    .await;

    for remaining in (1..=20).rev() {
        let mut line = String::new();
        let _ = write!(line, "Motors in {remaining}s");
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("Set down"),
            line2: status_text("Hold still"),
            line3: Some(line),
        })
        .await;
        Timer::after(Duration::from_secs(1)).await;
    }

    info!("Mag calibration entering motor phase");
    set_motor_drivers_enabled(true).await;
    Timer::after(Duration::from_millis(100)).await;

    enter_mag_phase(MagCalibrationPhase::MotorBaseline, Some("Motors off baseline")).await;
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

    enter_mag_phase(MagCalibrationPhase::MotorMeasure, Some("Run motor sequence")).await;
    let interference = measure_mag_interference(config, baseline_mag).await;

    enter_mag_phase(MagCalibrationPhase::MotorVerify, Some("Verify compensation")).await;
    let verify_ok = verify_mag_interference(config, mag_bias, mag_scale, baseline_mag, &interference).await;

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    Timer::after(Duration::from_millis(500)).await;
    set_motor_drivers_enabled(false).await;

    if !verify_ok {
        info!("Mag calibration failed in motor verify phase");
        info!("MAG CAL RESULT: reached_motor_phase=true, reason=motor verify failed");
        enter_mag_phase(MagCalibrationPhase::Failed, Some("Motor verify failed")).await;
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("VERIFY FAIL"),
            line2: status_text("Recalibrate"),
            line3: status_text("Motors/mag"),
        })
        .await;
        return None;
    }

    enter_mag_phase(MagCalibrationPhase::Save, Some("Ready to save")).await;
    info!("MAG CAL RESULT: reached_motor_phase=true, reason=success");

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

    let _imu_guard = ImuReadingsGuard::start();
    Timer::after(Duration::from_millis(500)).await;

    let cached_calibration = flash_storage::get_cached_imu_calibration().await.unwrap_or_default();
    let mut imu_calibration = cached_calibration;
    let mut imu_flags = flash_storage::get_cached_imu_flags().await.unwrap_or_default();

    let result = run_gyro_accel_calibration(true, false).await;
    let Some(gyro_bias) = result.gyro_bias else {
        info!("Gyro calibration failed; keeping previous values");
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("GYRO FAILED"),
            line2: status_text("Not saved"),
            line3: None,
        })
        .await;
        event::raise_event(event::Events::CalibrationCompleted).await;
        Timer::after(Duration::from_secs(2)).await;

        info!("=== IMU Gyro Calibration Complete ===");
        return;
    };

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

    let _imu_guard = ImuReadingsGuard::start();
    Timer::after(Duration::from_millis(500)).await;

    let cached_calibration = flash_storage::get_cached_imu_calibration().await.unwrap_or_default();
    let mut imu_calibration = cached_calibration;
    let mut imu_flags = flash_storage::get_cached_imu_flags().await.unwrap_or_default();

    let result = run_gyro_accel_calibration(false, true).await;
    let Some(accel_bias) = result.accel_bias else {
        info!("Accel calibration failed; keeping previous values");
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("ACCEL FAILED"),
            line2: status_text("Not saved"),
            line3: None,
        })
        .await;
        event::raise_event(event::Events::CalibrationCompleted).await;
        Timer::after(Duration::from_secs(2)).await;

        info!("=== IMU Accel Calibration Complete ===");
        return;
    };

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

    let _imu_guard =
        ImuReadingsGuard::start_with_fusion_mode(imu_read::AhrsFusionMode::Axis9, imu_read::DEFAULT_FUSION_MODE);
    Timer::after(Duration::from_millis(500)).await;

    let cached_calibration = flash_storage::get_cached_imu_calibration().await.unwrap_or_default();
    let mut imu_calibration = cached_calibration;
    let mut imu_flags = flash_storage::get_cached_imu_flags().await.unwrap_or_default();

    let Some(mag_result) = run_mag_calibration_steps(MAG_CALIBRATION_CONFIG).await else {
        info!("Mag calibration failed; keeping previous values");
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("MAG FAILED"),
            line2: status_text("Not saved"),
            line3: None,
        })
        .await;
        event::raise_event(event::Events::CalibrationCompleted).await;
        Timer::after(Duration::from_secs(2)).await;

        info!("=== IMU Mag Calibration Complete ===");
        return;
    };

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
