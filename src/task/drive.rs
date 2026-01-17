//! High-level drive control and coordination
//!
//! This module implements the control layer that coordinates between sensing and actuation:
//!
//! # Architecture
//!
//! ```text
//! encoder_read          drive (THIS)          motor_driver
//! ├── Sensing       →   ├── Calibration   →   ├── PWM actuation
//! └── Pulse counts      ├── Motion control    └── Direction control
//!                       ├── Feedback loops
//!                       └── Coordination
//!
//! imu_read          →   drive             →   motor_driver
//! ├── Orientation       ├── Rotation
//! └── Angles            └── Stabilization
//! ```
//!
//! # Responsibilities
//!
//! 1. **Motor Calibration**: Orchestrates the calibration procedure by coordinating
//!    encoder readings with motor commands and saving results to flash
//!
//! 2. **Motion Control**: Implements high-level driving behaviors:
//!    - Direct speed control using motor_driver's -100 to +100 convention
//!    - Differential steering (different left/right speeds)
//!    - Precise rotation using IMU feedback
//!    - Combined rotation + motion maneuvers
//!
//! 3. **Feedback Processing**: Receives sensor data from orchestrator and uses it for:
//!    - Speed adjustments based on encoder feedback
//!    - Rotation control using gyroscope/IMU data
//!    - Tilt compensation for inclines
//!    - Straight-line correction
//!
//! 4. **Task Coordination**: Sends commands to lower-level tasks:
//!    - `motor_driver`: PWM speed and direction commands
//!    - `encoder_read`: Start/stop/reset commands
//!    - `flash_storage`: Save calibration data
//!
//! # Speed Convention
//!
//! This module uses the same speed convention as `motor_driver`:
//! - **-100 to +100**: Full range of motor control
//! - **Positive values**: Forward motion
//! - **Negative values**: Backward motion
//! - **Zero**: Coast (freewheel)
//!
//! No unnecessary abstraction - the motor_driver's interface is clean and simple.
//!
//! # Data Flow
//!
//! ## Commands (from orchestrator or other high-level tasks)
//! ```rust
//! // Direct speed control
//! drive::send_drive_command(DriveCommand::Drive(
//!     DriveAction::SetSpeed { left: 50, right: 50 }
//! )).await;
//!
//! // Differential steering (turn right)
//! drive::send_drive_command(DriveCommand::Drive(
//!     DriveAction::SetSpeed { left: 60, right: 40 }
//! )).await;
//!
//! // Calibration
//! drive::send_drive_command(DriveCommand::RunMotorCalibration).await;
//! ```
//!
//! ## Sensor Feedback (from orchestrator)
//! ```rust
//! // Encoder measurements (forwarded by orchestrator from encoder_read events)
//! drive::send_encoder_measurement(measurement).await;
//!
//! // IMU measurements (forwarded by orchestrator from imu_read events)
//! drive::send_drive_command(DriveCommand::ImuFeedback(measurement)).await;
//! ```
//!
//! # Important: Event Channel Usage
//!
//! This task does NOT directly consume system events. All sensor data is forwarded
//! by the orchestrator via dedicated channels (`send_encoder_measurement()`) to
//! prevent multiple tasks from competing for events on the system event channel.
//!
//! # Control Modes
//!
//! - **Direct Control**: Manual speed commands (-100 to +100 per track)
//! - **Calibration Mode**: Automated motor matching procedure
//! - **Rotation Control**: IMU-based precise turning
//! - **Feedback Control**: Encoder-based speed adjustment (TODO)
//! - **Straight-Line**: IMU-based drift correction (TODO)

#![allow(clippy::too_many_arguments)]

use defmt::info;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};

use crate::task::{
    encoder_read::EncoderMeasurement,
    imu_read::ImuMeasurement,
    motor_driver::{self, MotorCommand, Track},
};

/// Command signal for drive control
///
/// Used by orchestrator and other high-level tasks to send drive commands
static DRIVE_CONTROL: Signal<CriticalSectionRawMutex, DriveCommand> = Signal::new();

/// Latest encoder measurement
///
/// The orchestrator forwards encoder measurements here. During calibration,
/// the drive task reads the latest measurement on demand rather than draining a queue.
/// This ensures we always get fresh data and don't miss measurements due to channel overflow.
static LATEST_ENCODER_MEASUREMENT: Mutex<CriticalSectionRawMutex, Option<EncoderMeasurement>> = Mutex::new(None);

/// Channel for receiving IMU measurements from orchestrator
/// Capacity of 16 allows buffering during calibration sequences (100Hz sampling)
const IMU_FEEDBACK_QUEUE_SIZE: usize = 16;
static IMU_FEEDBACK_CHANNEL: Channel<CriticalSectionRawMutex, ImuMeasurement, IMU_FEEDBACK_QUEUE_SIZE> = Channel::new();

/// Update the latest encoder measurement
///
/// Called by orchestrator to forward encoder events from the encoder_read task.
/// Stores the measurement in a mutex so the drive task can read it on demand.
pub async fn send_encoder_measurement(measurement: EncoderMeasurement) {
    let mut latest = LATEST_ENCODER_MEASUREMENT.lock().await;
    *latest = Some(measurement);
}

/// Try to update the latest encoder measurement without blocking
///
/// Returns true if updated successfully.
/// Used by orchestrator to avoid blocking when drive task is busy during calibration.
pub async fn try_send_encoder_measurement(measurement: EncoderMeasurement) -> bool {
    let mut latest = LATEST_ENCODER_MEASUREMENT.lock().await;
    *latest = Some(measurement);
    true
}

/// Clear the latest encoder measurement (internal use by drive task)
///
/// Used before each calibration step to ensure we wait for fresh data after a reset.
async fn clear_encoder_measurement() {
    let mut latest = LATEST_ENCODER_MEASUREMENT.lock().await;
    *latest = None;
}

/// Try to send IMU measurement to drive task without blocking
///
/// Returns true if sent, false if channel is full.
/// Used by orchestrator to avoid blocking when drive task is busy.
/// IMU measurements arrive at 100Hz, so dropping occasional readings is acceptable.
pub fn try_send_imu_measurement(measurement: ImuMeasurement) -> bool {
    IMU_FEEDBACK_CHANNEL.sender().try_send(measurement).is_ok()
}

/// Get the latest encoder measurement (internal use by drive task)
///
/// Returns the most recent encoder measurement, or None if no measurement available.
async fn get_latest_encoder_measurement() -> Option<EncoderMeasurement> {
    let latest = LATEST_ENCODER_MEASUREMENT.lock().await;
    *latest
}

/// Receive IMU measurement from channel (internal use by drive task)
async fn receive_imu_measurement() -> ImuMeasurement {
    IMU_FEEDBACK_CHANNEL.receive().await
}

/// Combined motor control and sensor feedback commands
#[derive(Debug, Clone)]
pub enum DriveCommand {
    /// Movement and control commands
    Drive(DriveAction),
    /// Encoder feedback for speed adjustment
    EncoderFeedback(EncoderMeasurement),
    /// Gyroscope feedback for rotation control
    ImuFeedback(ImuMeasurement),
    /// Run motor calibration procedure
    RunMotorCalibration,
}

/// Motion control commands with associated parameters
#[derive(Debug, Clone, PartialEq)]
pub enum DriveAction {
    /// Set motor speeds directly using motor_driver convention
    ///
    /// Speed range: -100 (full backward) to +100 (full forward)
    /// - Positive values: Forward motion
    /// - Negative values: Backward motion
    /// - Zero: Coast (freewheel)
    ///
    /// This directly maps to motor_driver's SetTracks command.
    SetSpeed {
        /// Left track speed (-100 to +100)
        left: i8,
        /// Right track speed (-100 to +100)
        right: i8,
    },
    /// Active electrical braking
    Brake,
    /// Passive stop (freewheeling)
    Coast,
    /// Enter low-power standby mode
    Standby,
    /// Precise rotation with optional forward/backward motion
    RotateExact {
        /// Rotation angle in degrees
        degrees: f32,
        /// Rotation direction
        direction: RotationDirection,
        /// Combined motion type
        motion: RotationMotion,
    },
}

/// Rotation direction for precise turning
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RotationDirection {
    /// Clockwise rotation (right turn)
    Clockwise,
    /// Counter-clockwise rotation (left turn)
    CounterClockwise,
}

/// Combined motion options during rotation
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RotationMotion {
    /// Rotate in place
    Stationary,
    /// Rotate while moving at specified speed (-100 to +100)
    /// Positive = forward, Negative = backward
    WhileMoving(i8),
}

// Control parameters
/// Number of encoder pulses per motor shaft revolution
const PULSES_PER_REV: u32 = 8;
/// Maximum rotation speed (0-100%)
const ROTATION_SPEED_MAX: u8 = 50;
/// Minimum rotation speed to overcome friction
const ROTATION_SPEED_MIN: u8 = 20;
/// Acceptable angle error in degrees
const ROTATION_TOLERANCE_DEG: f32 = 2.0;
/// Maximum speed differential during combined motion
const SPEED_DIFF_MAX: i8 = 30;

// Motor calibration constants
/// Calibration test speed for individual motors (100% forward)
const CALIBRATION_SPEED_INDIVIDUAL: i8 = 100;
/// Calibration test speed for track verification (60% forward)
const CALIBRATION_SPEED_TRACK: i8 = 60;
/// Calibration measurement duration in milliseconds
const CALIBRATION_SAMPLE_DURATION_MS: u64 = 10_000;
/// Coast time between calibration tests in milliseconds
const CALIBRATION_COAST_DURATION_MS: u64 = 500;

/// Send a drive command for execution
///
/// Non-blocking signal delivery for drive commands
pub fn send_drive_command(command: DriveCommand) {
    DRIVE_CONTROL.signal(command);
}

/// Wait for next drive command (internal use)
async fn wait() -> DriveCommand {
    DRIVE_CONTROL.wait().await
}

// Motor control now handled by motor_driver task
// Left/right motor state tracking removed - motor_driver task handles this

/// State tracking for precise rotation maneuvers
///
/// Maintains rotation progress and calculates differential motor speeds
/// needed to achieve the target rotation angle using IMU feedback.
struct RotationState {
    /// Target rotation angle in degrees
    target_angle: f32,
    /// Accumulated rotation so far (in degrees)
    accumulated_angle: f32,
    /// Last measured yaw angle (in degrees)
    last_yaw: Option<f32>,
    /// Last update timestamp
    last_update_ms: u32,
    /// Direction of rotation
    direction: RotationDirection,
    /// Type of motion during rotation
    motion: RotationMotion,
    /// Base speed before rotation adjustment
    base_speed: i8,
}

impl RotationState {
    /// Creates new rotation tracking state
    fn new(target_angle: f32, direction: RotationDirection, motion: RotationMotion) -> Self {
        let base_speed = match motion {
            RotationMotion::Stationary => 0,
            RotationMotion::WhileMoving(speed) => speed,
        };

        Self {
            target_angle,
            accumulated_angle: 0.0,
            last_yaw: None,
            last_update_ms: 0,
            direction,
            motion,
            base_speed,
        }
    }

    /// Updates rotation state with new IMU measurement
    /// Returns true if target angle has been reached
    fn update(&mut self, measurement: &ImuMeasurement) -> bool {
        // Calculate yaw change since last reading
        if let Some(last_yaw) = self.last_yaw {
            let mut yaw_change = measurement.orientation.yaw - last_yaw;

            // Handle wraparound at ±180 degrees
            if yaw_change > 180.0 {
                yaw_change -= 360.0;
            } else if yaw_change < -180.0 {
                yaw_change += 360.0;
            }

            // Accumulate based on desired direction
            self.accumulated_angle += match self.direction {
                RotationDirection::Clockwise => -yaw_change,
                RotationDirection::CounterClockwise => yaw_change,
            };
        }

        self.last_yaw = Some(measurement.orientation.yaw);
        self.last_update_ms = measurement.timestamp_ms;

        // Check if we've reached target angle within tolerance
        (self.accumulated_angle - self.target_angle).abs() <= ROTATION_TOLERANCE_DEG
    }

    /// Calculates appropriate motor speeds for current rotation state
    /// Returns (left_speed, right_speed)
    fn calculate_motor_speeds(&self) -> (i8, i8) {
        // Calculate rotation component based on remaining angle
        let remaining_degrees = (self.target_angle - self.accumulated_angle).abs();
        let rotation_speed = if remaining_degrees < 10.0 {
            // Linear interpolation between min and max speed
            let speed_range = ROTATION_SPEED_MAX - ROTATION_SPEED_MIN;
            let speed_factor = remaining_degrees / 10.0;
            (ROTATION_SPEED_MIN as f32 + (speed_range as f32 * speed_factor)) as u8
        } else {
            ROTATION_SPEED_MAX
        };

        match self.motion {
            RotationMotion::Stationary => {
                // Simple differential drive for in-place rotation
                match self.direction {
                    RotationDirection::Clockwise => (rotation_speed as i8, -(rotation_speed as i8)),
                    RotationDirection::CounterClockwise => (-(rotation_speed as i8), rotation_speed as i8),
                }
            }
            RotationMotion::WhileMoving(_) => {
                // Combine base motion with rotation
                let rotation_diff = rotation_speed.min(SPEED_DIFF_MAX as u8) as i8;

                match self.direction {
                    RotationDirection::Clockwise => {
                        (self.base_speed, (self.base_speed - rotation_diff).clamp(-100, 100))
                    }
                    RotationDirection::CounterClockwise => {
                        ((self.base_speed - rotation_diff).clamp(-100, 100), self.base_speed)
                    }
                }
            }
        }
    }

    /// Returns the base speed for maintaining motion after rotation
    fn continuation_speed(&self) -> Option<i8> {
        match self.motion {
            RotationMotion::Stationary => None,
            RotationMotion::WhileMoving(_) => Some(self.base_speed),
        }
    }
}

/// Motor power adjustment based on tilt
struct TiltCompensation {
    /// Maximum power adjustment factor
    max_adjustment: f32,
    /// Maximum tilt angle in degrees
    max_tilt_angle: f32,
}

impl TiltCompensation {
    /// Creates a new tilt compensation instance
    /// with the given maximum adjustment and tilt angle
    fn new(max_adjustment: f32, max_tilt_angle: f32) -> Self {
        Self {
            max_adjustment,
            max_tilt_angle,
        }
    }

    /// Calculate power adjustment factor based on tilt angle
    fn calculate_adjustment(&mut self, tilt_degrees: f32) -> f32 {
        (tilt_degrees / self.max_tilt_angle).clamp(-1.0, 1.0) * self.max_adjustment
    }
}

/// Tracks straight-line motion correction
struct StraightLineState {
    /// Initial yaw angle when straight motion started
    target_yaw: f32,
    /// Previous yaw error for derivative calculation
    last_error: Option<f32>,
    /// Last update timestamp for derivative calculation
    last_update_ms: u32,
}

impl StraightLineState {
    /// Maximum correction strength (as percentage of base speed)
    const MAX_CORRECTION: f32 = 0.2; // 20% max speed difference
    /// Proportional control factor
    const P_FACTOR: f32 = 0.1;
    /// Derivative control factor
    const D_FACTOR: f32 = 0.05;

    fn new(initial_yaw: f32) -> Self {
        Self {
            target_yaw: initial_yaw,
            last_error: None,
            last_update_ms: 0,
        }
    }

    /// Calculates motor speed corrections to maintain straight line
    /// Returns (left_adjustment, right_adjustment) as factors to multiply with base speed
    fn calculate_correction(&mut self, current_yaw: f32, timestamp_ms: u32) -> (f32, f32) {
        let mut yaw_error = current_yaw - self.target_yaw;

        // Handle wraparound at ±180 degrees
        if yaw_error > 180.0 {
            yaw_error -= 360.0;
        } else if yaw_error < -180.0 {
            yaw_error += 360.0;
        }

        // Calculate P term
        let p_correction = yaw_error * Self::P_FACTOR;

        // Calculate D term (rate of change of error)
        let d_correction = if let Some(last_error) = self.last_error {
            let dt = (timestamp_ms - self.last_update_ms) as f32 / 1000.0;
            if dt > 0.0 {
                let error_rate = (yaw_error - last_error) / dt;
                error_rate * Self::D_FACTOR
            } else {
                0.0
            }
        } else {
            0.0
        };

        // Update state for next calculation
        self.last_error = Some(yaw_error);
        self.last_update_ms = timestamp_ms;

        // Combine P and D terms and clamp to max correction
        let correction = (p_correction + d_correction).clamp(-Self::MAX_CORRECTION, Self::MAX_CORRECTION);

        if correction > 0.0 {
            // Drifting right, speed up right motor
            (1.0 - correction, 1.0 + correction)
        } else {
            // Drifting left, speed up left motor
            (1.0 + correction.abs(), 1.0 - correction.abs())
        }
    }
}

/// Motor state tracking (simplified - actual control via motor_driver task)
struct MotorState {
    current_speed: i8,
    forward: bool,
}

impl MotorState {
    fn new() -> Self {
        Self {
            current_speed: 0,
            forward: true,
        }
    }

    /// Converts encoder pulses to motor shaft RPM
    /// RPM is calculated for the motor shaft before the 120:1 gear reduction
    fn calculate_rpm(&self, pulses: u16, elapsed_ms: u32) -> f32 {
        let hz = (pulses as f32 * 1000.0) / elapsed_ms as f32;
        let motor_rpm = (hz / PULSES_PER_REV as f32) * 60.0;
        if self.forward { motor_rpm } else { -motor_rpm }
    }

    /// Sets motor speed and direction (-100 to +100)
    async fn set_speed(&mut self, track: Track, speed: i8) {
        self.current_speed = speed;
        self.forward = speed >= 0;

        motor_driver::send_motor_command(MotorCommand::SetTrack { track, speed }).await;
    }

    /// Sets motor speed with tilt compensation
    async fn set_speed_with_tilt(&mut self, track: Track, base_speed: i8, tilt_degrees: f32) {
        let mut tilt_compensation = TiltCompensation::new(0.3, 45.0);
        let tilt_adjustment = tilt_compensation.calculate_adjustment(tilt_degrees);
        let adjusted_speed = (base_speed as f32 * (1.0 + tilt_adjustment)) as i8;
        self.set_speed(track, adjusted_speed.clamp(-100, 100)).await;
    }

    /// Actively stops motor using electrical braking
    async fn brake(&mut self, track: Track) {
        self.current_speed = 0;
        motor_driver::send_motor_command(MotorCommand::SetTrack { track, speed: 0 }).await;
        motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
    }

    /// Stops motor by letting it spin freely
    async fn coast(&mut self, track: Track) {
        self.current_speed = 0;
        motor_driver::send_motor_command(MotorCommand::SetTrack { track, speed: 0 }).await;
    }

    /// Returns current motor speed setting (-100 to +100)
    pub fn current_speed(&self) -> i8 {
        self.current_speed
    }
}

/// Detect if robot is performing a stationary rotation (turn in place)
///
/// Returns true if left and right tracks have equal but opposite speeds.
fn is_turning_in_place(left_speed: i8, right_speed: i8) -> bool {
    left_speed == -right_speed && left_speed != 0
}

/// Wait for an encoder measurement with timeout
///
/// Receives encoder measurements forwarded by the orchestrator from encoder_read events.
/// Returns `None` if timeout expires before receiving a measurement.
///
/// # Important
/// Does NOT consume system events directly - measurements come via the dedicated
/// encoder feedback channel populated by the orchestrator.
/// Wait for a fresh encoder measurement with timeout
///
/// Polls for a new encoder measurement, waiting up to timeout_ms.
/// Returns None if timeout occurs before a measurement is available.
async fn wait_for_encoder_event_timeout(timeout_ms: u64) -> Option<EncoderMeasurement> {
    use embassy_time::{Duration, Timer};

    let start = Instant::now();
    let timeout_duration = Duration::from_millis(timeout_ms);

    loop {
        if let Some(measurement) = get_latest_encoder_measurement().await {
            return Some(measurement);
        }

        if start.elapsed() >= timeout_duration {
            return None;
        }

        // Small delay to avoid busy-waiting
        Timer::after(Duration::from_millis(10)).await;
    }
}

/// Run the motor calibration procedure
///
/// Coordinates a multi-step calibration process across three tasks:
/// - `encoder_read`: Provides pulse count measurements
/// - `motor_driver`: Applies speed commands and stores calibration factors
/// - `flash_storage`: Persists calibration data (TODO)
///
/// # Calibration Steps
///
/// 1. Test left track motors individually at 100% to match within-track
/// 2. Test right track motors individually at 100% to match within-track
/// 3. Test left track together at 60% to measure track performance
/// 4. Test right track together at 60% to measure track performance
/// 5. Scale stronger track down to match weaker track (between-track calibration)
/// 6. Final verification with all motors
///
/// # Requirements
///
/// - Robot must be elevated (wheels off ground)
/// - No interference during the ~30 second procedure
///
/// # Architecture Note
///
/// This procedure demonstrates the clean separation of concerns:
/// - Sensing (encoder_read): Just counts pulses, doesn't know about calibration
/// - Actuation (motor_driver): Just runs motors, receives calibration factors
/// - Control (drive/this): Orchestrates the procedure and calculates factors
async fn run_motor_calibration() {
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

    motor_driver::send_motor_command(MotorCommand::SetAllMotors {
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

    motor_driver::send_motor_command(MotorCommand::SetAllMotors {
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

    motor_driver::send_motor_command(MotorCommand::SetAllMotors {
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

    motor_driver::send_motor_command(MotorCommand::SetAllMotors {
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

    motor_driver::send_motor_command(MotorCommand::SetAllMotors {
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

    motor_driver::send_motor_command(MotorCommand::SetAllMotors {
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

/// Drive control task - coordinates motion and sensor feedback
///
/// # Architecture
///
/// This is a high-level control task that:
/// - Receives drive commands via signal
/// - Receives encoder feedback via channel (from orchestrator)
/// - Receives IMU feedback via command (from orchestrator)
/// - Sends motor commands to motor_driver task
/// - Coordinates calibration procedures
///
/// # Sensor Data Flow
///
/// Sensor tasks → Events → Orchestrator → Drive task (this) → Motor driver
///
/// The orchestrator forwards relevant sensor events to this task via dedicated
/// channels rather than having this task consume system events directly.
#[embassy_executor::task]
pub async fn drive() {
    // Initialize motor state tracking (for 2 motors on Driver1)
    let mut left_state = MotorState::new();
    let mut right_state = MotorState::new();

    // Motor driver standby control - now handled by port_expander task via motor_driver
    let mut standby_enabled = true;

    // Rotation state tracking
    let mut rotation_state: Option<RotationState>;

    // Add straight-line tracking state
    // let straight_line_state: Option<StraightLineState> = None;

    loop {
        // Process any pending commands
        let command = wait().await;

        match command {
            DriveCommand::Drive(action) => {
                // Wake from standby if movement requested
                if standby_enabled {
                    match action {
                        DriveAction::SetSpeed { .. } | DriveAction::RotateExact { .. } => {
                            motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true }).await;
                            standby_enabled = false;
                            Timer::after(Duration::from_millis(100)).await;
                        }
                        _ => {}
                    }
                }

                // Clear rotation state unless this is a rotation command
                if !matches!(action, DriveAction::RotateExact { .. }) {
                    // rotation_state = None;
                }

                // Execute drive action
                match action {
                    DriveAction::SetSpeed { left, right } => {
                        // Clamp speeds to valid range
                        let left_clamped = left.clamp(-100, 100);
                        let right_clamped = right.clamp(-100, 100);

                        left_state.set_speed(Track::Left, left_clamped).await;
                        right_state.set_speed(Track::Right, right_clamped).await;
                    }
                    DriveAction::RotateExact {
                        degrees,
                        direction,
                        motion,
                    } => {
                        // Initialize new rotation state
                        rotation_state = Some(RotationState::new(degrees, direction, motion));

                        // Apply initial motor speeds for rotation
                        let (left_speed, right_speed) = rotation_state.as_ref().unwrap().calculate_motor_speeds();

                        left_state.set_speed(Track::Left, left_speed).await;
                        right_state.set_speed(Track::Right, right_speed).await;
                    }
                    DriveAction::Coast => {
                        info!("coast");
                        left_state.coast(Track::Left).await;
                        right_state.coast(Track::Right).await;
                    }
                    DriveAction::Brake => {
                        info!("brake");
                        left_state.brake(Track::Left).await;
                        right_state.brake(Track::Right).await;
                    }
                    DriveAction::Standby => {
                        if !standby_enabled {
                            left_state.brake(Track::Left).await;
                            right_state.brake(Track::Right).await;
                            Timer::after(Duration::from_millis(100)).await;
                            left_state.coast(Track::Left).await;
                            right_state.coast(Track::Right).await;
                            Timer::after(Duration::from_millis(100)).await;
                            motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false })
                                .await;
                            standby_enabled = true;
                        }
                    }
                }

                // Drive command executed
            }

            DriveCommand::EncoderFeedback(_measurement) => {
                // // Skip encoder feedback during precise rotation
                // if rotation_state.is_some() {
                //     continue;
                // }

                // // Calculate motor RPMs
                // let left_rpm = left.calculate_rpm(measurement.left.pulse_count, measurement.left.elapsed_ms);
                // let right_rpm = right.calculate_rpm(measurement.right.pulse_count, measurement.right.elapsed_ms);

                // // Apply speed adjustments if motors are running
                // let left_speed = left.current_speed();
                // let right_speed = right.current_speed();

                // if left_speed != 0 || right_speed != 0 {
                //     // Compare raw motor RPMs since encoders are on motor shaft
                //     let rpm_ratio = if left_rpm != 0.0 { right_rpm / left_rpm } else { 1.0 };

                //     // Calculate how far we are from perfect ratio
                //     let ratio_error = (1.0 - rpm_ratio).abs();

                //     // Only adjust if error is above threshold
                //     if ratio_error > 0.05 {
                //         // 5% tolerance
                //         // Variable correction factor based on error magnitude
                //         let correction_factor = if ratio_error > 0.2 {
                //             0.8 // Aggressive correction when far off
                //         } else {
                //             0.4 // Fine adjustment when closer
                //         };

                //         let adjustment = 1.0 + ((1.0 - rpm_ratio) * correction_factor);
                //         let adjusted_speed = (right_speed as f32 * adjustment).clamp(-100.0, 100.0) as i8;
                //         right.set_speed(adjusted_speed).unwrap();

                //         // Signal that we're still adjusting
                //         event::send_event(Events::DriveCommandExecuted).await;
                //     }
                // }
            }

            DriveCommand::ImuFeedback(_measurement) => {
                // // Part 1: Tilt Compensation for Forward/Backward Motion
                // // --------------------------------------------------
                // // Only apply tilt compensation when:
                // // - Not in a rotation maneuver
                // // - Motors are running
                // // - Both motors at same speed (pure forward/backward motion)
                // //
                // // Example scenario:
                // // Robot driving forward at 50% speed encounters a 10° incline
                // // - Base speed: 50
                // // - Tilt: +10° (positive = climbing)
                // // - Adjustment factor: ~0.067 (10° / 45° * 0.3)
                // // - New speed: 50 * (1 + 0.067) = 53
                // // This increases power to maintain speed while climbing
                // if !rotation_state.is_some() && (left.current_speed() != 0 || right.current_speed() != 0) {
                //     if left.current_speed() == right.current_speed() {
                //         left.set_speed_with_tilt(left.current_speed(), measurement.orientation.pitch)
                //             .unwrap();
                //         right
                //             .set_speed_with_tilt(right.current_speed(), measurement.orientation.pitch)
                //             .unwrap();
                //     }
                // }

                // // Part 2: Rotation Control
                // // -----------------------
                // // Handle active rotation maneuvers using yaw measurements
                // if let Some(rot_state) = rotation_state.as_mut() {
                //     // Check if target angle reached using yaw tracking
                //     // Example: 90° clockwise turn
                //     // - Target: 90°
                //     // - Current accumulated: 88°
                //     // - Tolerance: ±2°
                //     // - Status: Almost complete, using reduced speed
                //     if rot_state.update(&measurement) {
                //         // Target angle reached
                //         info!("rotation complete");

                //         // Handle post-rotation motion:
                //         // 1. For combined movements (e.g., "rotate while moving forward")
                //         //    continue with the base motion including tilt compensation
                //         // 2. For stationary rotations, stop completely
                //         //
                //         // Example: After 90° turn while moving forward at 30%
                //         // - continue_speed returns Some(30)
                //         // - Apply tilt compensation to maintain 30% on any incline
                //         if let Some(continue_speed) = rot_state.continuation_speed() {
                //             left.set_speed_with_tilt(continue_speed, measurement.orientation.pitch)
                //                 .unwrap();
                //             right
                //                 .set_speed_with_tilt(continue_speed, measurement.orientation.pitch)
                //                 .unwrap();
                //         } else {
                //             // Stationary rotation complete - brake both motors
                //             left.brake().unwrap();
                //             right.brake().unwrap();
                //         }

                //         rotation_state = None;
                //         event::send_event(Events::RotationCompleted).await;
                //     } else {
                //         // Rotation still in progress
                //         // Calculate differential speeds based on:
                //         // - Remaining angle
                //         // - Current motion type (stationary or moving)
                //         // - Direction of rotation
                //         //
                //         // Example: Mid-way through 90° clockwise turn
                //         // - Accumulated: 45°
                //         // - Remaining: 45°
                //         // - Speed: ROTATION_SPEED_MAX (50%)
                //         // - Results in: Left=+50%, Right=-50% for stationary turn
                //         let (left_speed, right_speed) = rot_state.calculate_motor_speeds();
                //         left.set_speed(left_speed).unwrap();
                //         right.set_speed(right_speed).unwrap();
                //     }
                // }

                // // Part 3: Straight-Line Motion Correction
                // // --------------------------------------
                // // Adjust motor speeds to maintain straight line motion

                // // Check if we should initialize straight-line tracking
                // if straight_line_state.is_none()
                //     && left.current_speed() == right.current_speed()
                //     && left.current_speed() != 0
                // {
                //     // Starting new straight-line motion
                //     info!("Starting straight line control at yaw: {}", measurement.orientation.yaw);
                //     straight_line_state = Some(StraightLineState::new(measurement.orientation.yaw));
                // }

                // // Apply straight-line correction if active
                // if let Some(straight_state) = straight_line_state.as_mut() {
                //     if left.current_speed() == right.current_speed() && left.current_speed() != 0 {
                //         let (left_adj, right_adj) =
                //             straight_state.calculate_correction(measurement.orientation.yaw, measurement.timestamp_ms);

                //         let base_speed = left.current_speed();
                //         let left_corrected = (base_speed as f32 * left_adj) as i8;
                //         let right_corrected = (base_speed as f32 * right_adj) as i8;

                //         info!(
                //             "Straight correction L:{} R:{} (base:{})",
                //             left_corrected, right_corrected, base_speed
                //         );

                //         left.set_speed_with_tilt(left_corrected, measurement.orientation.pitch)
                //             .unwrap();
                //         right
                //             .set_speed_with_tilt(right_corrected, measurement.orientation.pitch)
                //             .unwrap();
                //     } else {
                //         // No longer in straight-line motion
                //         straight_line_state = None;
                //     }
                // }
            }

            DriveCommand::RunMotorCalibration => {
                info!("Starting motor calibration procedure");
                run_motor_calibration().await;
                info!("Motor calibration procedure completed");
            }
        }
    }
}
