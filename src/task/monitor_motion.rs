//!

use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal};
use embassy_time::{Duration, Instant};
use heapless::Vec;

use crate::{
    system::event::{Events, send_event},
    task::{drive::DriveAction, encoder_read::EncoderMeasurement, imu_read::ImuMeasurement},
};

// Constants for thresholds
/// Maximum tilt angle before emergency stop
const MAX_TILT_ANGLE: f32 = 45.0;
/// Maximum roll angle before emergency stop
const MAX_ROLL_ANGLE: f32 = 45.0;
/// Yaw threshold for what we consider not turning in degrees
const YAW_THRESHOLD: f32 = 1.0;
/// Encoder difference threshold for what we consider no difference
const ENCODER_DIFF_THRESHOLD: f32 = 0.01; // 1%
/// Degrees tilt before we want tilt compensation
const TILT_FLAT_THRESHOLD: f32 = 5.0;
/// Speed adjustment percent per 10° tilt
const TILT_COMPENSATION_FACTOR: f32 = 0.05;
/// Degrees roll before we want roll compensation
const ROLL_FLAT_THRESHOLD: f32 = 5.0;

/// Determines, how many historic measurements are held for motion correction
const MOTION_DATA_HISTORY: usize = 10;

/// What have we been sent?
enum MotionData {
    Imu(ImuMeasurement),
    Encoder(EncoderMeasurement),
    DriveAction(DriveAction),
}

enum MotionDataCommand {
    Start,
    Stop,
}

/// Control signal to trigger data collection & correction decisions
static MOTION_CONTROL: Signal<CriticalSectionRawMutex, MotionDataCommand> = Signal::new();

/// Start motion control
pub fn start_motion_control() {
    MOTION_CONTROL.signal(MotionDataCommand::Start);
}

/// Stop motion control
pub fn stop_motion_control() {
    MOTION_CONTROL.signal(MotionDataCommand::Stop);
}

/// Channel for sending motion data to the motion correction controller
static MOTION_DATA_CHANNEL: Channel<CriticalSectionRawMutex, MotionData, 10> = Channel::new();

/// Send motion data to the motion correction controller
pub async fn send_motion_data(data: MotionData) {
    MOTION_DATA_CHANNEL.sender().send(data).await;
}

/// Receive motion data
pub async fn receive_motion_data() -> MotionData {
    MOTION_DATA_CHANNEL.receiver().receive().await
}

/// Combined motion data for correction
/// - IMU measurements
/// - Encoder measurements
/// - Drive actions
/// - Match timestamps: To synchronize data from different sources
struct MotionFusion {
    imu: Vec<ImuMeasurement, MOTION_DATA_HISTORY>,
    encoder: Vec<EncoderMeasurement, MOTION_DATA_HISTORY>,
    drive_action: Vec<DriveAction, MOTION_DATA_HISTORY>,
    match_timestamp: Vec<Instant, MOTION_DATA_HISTORY>,
}

impl MotionFusion {
    /// Create a new empty history
    fn new() -> Self {
        Self {
            imu: Vec::new(),
            encoder: Vec::new(),
            drive_action: Vec::new(),
            match_timestamp: Vec::new(),
        }
    }

    /// Add a new IMU measurement to the history
    fn add_imu_measurement(&mut self, imu: ImuMeasurement) {
        if self.imu.is_full() {
            self.imu.remove(0);
        }
        let _ = self.imu.push(imu);
        self.update_timestamp();
    }

    /// Add a new encoder measurement to the history
    fn add_encoder_measurement(&mut self, encoder: EncoderMeasurement) {
        if self.encoder.is_full() {
            self.encoder.remove(0);
        }
        let _ = self.encoder.push(encoder);
        self.update_timestamp();
    }

    /// Add a new drive action to the history
    fn add_drive_action(&mut self, action: DriveAction) {
        if self.drive_action.is_full() {
            self.drive_action.remove(0);
        }
        let _ = self.drive_action.push(action);
        self.update_timestamp();
    }

    /// Add a new match timestamp to the history
    fn add_match_timestamp(&mut self, timestamp: Instant) {
        if self.match_timestamp.is_full() {
            self.match_timestamp.remove(0);
        }
        let _ = self.match_timestamp.push(timestamp);
    }

    /// If all data sources have a new measurement, but there is not yet a latest timestamp, then add the timestamp
    fn update_timestamp(&mut self) {
        if self.imu.len() == self.encoder.len() && self.encoder.len() == self.drive_action.len() {
            if self.match_timestamp.len() < self.imu.len() {
                let timestamp = Instant::now();
                self.add_match_timestamp(timestamp);
            }
        }
    }

    /// Clear all history
    fn clear(&mut self) {
        self.imu.clear();
        self.encoder.clear();
        self.drive_action.clear();
        self.match_timestamp.clear();
    }

    /// Ingest new motion data
    fn ingest_motion_data(&mut self, data: MotionData) {
        match data {
            MotionData::Imu(imu) => self.add_imu_measurement(imu),
            MotionData::Encoder(encoder) => self.add_encoder_measurement(encoder),
            MotionData::DriveAction(action) => self.add_drive_action(action),
        }
    }
}

impl MotionFusion {
    /// Check if we have enough data to correct motion
    fn is_ready(&self) -> bool {
        self.imu.len() > 0 && self.encoder.len() > 0 && self.drive_action.len() > 0
    }

    /// Correct motion based on sensor data
    fn correct_motion(&self) -> Option<MotionCorrectionInstruction> {
        // Check if we have enough data to correct motion
        if !self.is_ready() {
            return None;
        }

        let drive_action = self.drive_action.last()?;
        let imu = self.imu.last()?;
        let encoder = self.encoder.last()?;

        match *drive_action {
            DriveAction::Coast | DriveAction::Brake | DriveAction::Standby => {
                // we are not moving or stopping, no correction needed
                return None;
            }
            DriveAction::Backward(speed) | DriveAction::Forward(speed) => {
                // Get current angles
                let pitch = imu.orientation.pitch;
                let roll = imu.orientation.roll;
                let yaw = imu.orientation.yaw;

                // 1. Check for emergency stop conditions first
                if pitch.abs() > MAX_TILT_ANGLE || roll.abs() > MAX_ROLL_ANGLE {
                    return Some(MotionCorrectionInstruction {
                        left_speed: 0,
                        right_speed: 0,
                        duration: Duration::from_millis(100),
                        correction_type: CorrectionType::EmergencyStop,
                        emergency_stop: true,
                    });
                }

                // 2. Tilt compensation (straight climbing/descending).
                if let Some(correction) = MotionFusion::tilt_compensation(speed, pitch, roll) {
                    return Some(correction);
                }

                // Get current encoder measurements
                let left_speed = encoder.left.rpm;
                let right_speed = encoder.right.rpm;
                let encoder_ratio = right_speed / left_speed;
                let encoder_diff = (1.0 - encoder_ratio).abs();

                // 3. Combined roll and tilt (angled slope)
                if let Some(correction) = MotionFusion::angled_slope_compensation(speed, pitch, roll, yaw, encoder_diff)
                {
                    return Some(correction);
                }

                // 4. Pure roll (side slope)
                if roll.abs() > 5.0 && pitch.abs() < 5.0 {
                    // Apply slight uphill compensation to the lower motor
                    let roll_factor = (roll / MAX_ROLL_ANGLE).clamp(-0.2, 0.2);
                    let base_speed = speed;
                    let (left_speed, right_speed) = if roll > 0.0 {
                        (base_speed, (base_speed as f32 * (1.0 + roll_factor)) as u8)
                    } else {
                        ((base_speed as f32 * (1.0 + roll_factor.abs())) as u8, base_speed)
                    };

                    return Some(MotionCorrectionInstruction {
                        left_speed,
                        right_speed,
                        duration: Duration::from_millis(100),
                        correction_type: CorrectionType::StraightLineCorrection,
                        emergency_stop: false,
                    });
                }

                // 5. Straight line correction (flat surface)
                if encoder_diff > ENCODER_DIFF_THRESHOLD {
                    let correction_factor = 0.1; // 10% speed difference
                    let base_speed = speed;
                    let (left_speed, right_speed) = if encoder_ratio < 1.0 {
                        (base_speed, (base_speed as f32 * (1.0 - correction_factor)) as u8)
                    } else {
                        ((base_speed as f32 * (1.0 - correction_factor)) as u8, base_speed)
                    };

                    return Some(MotionCorrectionInstruction {
                        left_speed,
                        right_speed,
                        duration: Duration::from_millis(100),
                        correction_type: CorrectionType::StraightLineCorrection,
                        emergency_stop: false,
                    });
                }
            }
            DriveAction::RotateExact {
                degrees,
                direction,
                motion,
            } => {
                // we are rotating, check if we are rotating correctly
                let imu = self.imu.last()?;
                let encoder = self.encoder.last()?;

                // TODO
                // check if we are rotating correctly
                // if not, correct
            }
            DriveAction::TorqueBias {
                reduce_side,
                bias_amount,
            } => {
                // we are correcting torque bias, check if we are moving straight
                let imu = self.imu.last()?;
                let encoder = self.encoder.last()?;

                // TODO
                // check if we are moving straight
                // if not, correct
            }
        }

        // if nothing else, we need no correction
        None
    }

    /// If we are climbing or descending a slope at around 90° to the slope, we need to adjust the speed of the motors to maintain speed.
    fn tilt_compensation(speed: u8, pitch: f32, roll: f32) -> Option<MotionCorrectionInstruction> {
        if pitch.abs() > TILT_FLAT_THRESHOLD && roll.abs() < ROLL_FLAT_THRESHOLD {
            // Calculate compensation factor based on pitch angle (per 10°)
            let tilt_compensation = (pitch / 10.0) * TILT_COMPENSATION_FACTOR;
            let adjusted_speed = (speed as f32 * (1.0 + tilt_compensation)) as u8;

            return Some(MotionCorrectionInstruction {
                left_speed: adjusted_speed,
                right_speed: adjusted_speed,
                duration: Duration::from_millis(50),
                correction_type: CorrectionType::TiltCompensation,
                emergency_stop: false,
            });
        }
        None
    }

    /// If we are climbing or descending a slope at an angle we need to check if this has introduced a yaw angle or an encoder difference, and correct it.
    fn angled_slope_compensation(
        speed: u8,
        pitch: f32,
        roll: f32,
        yaw: f32,
        encoder_diff: f32,
    ) -> Option<MotionCorrectionInstruction> {
        // Only proceed if we're on an angled slope
        if pitch.abs() > TILT_FLAT_THRESHOLD && roll.abs() > ROLL_FLAT_THRESHOLD {
            // Calculate absolute yaw error in range 0-360°
            let yaw_error = yaw.abs() % 360.0;

            // Only correct if we're experiencing either:
            // a) Deviation from straight line (yaw error)
            // b) Different wheel speeds (encoder difference)
            if yaw_error > YAW_THRESHOLD || encoder_diff > ENCODER_DIFF_THRESHOLD {
                // Calculate correction parameters
                let correction_factor = 0.15; // 15% speed difference for correction
                let base_speed = speed;

                // Two forces are at play here:
                // 1. Gravity pulling robot sideways (roll)
                // 2. Gravity affecting forward motion (pitch)
                //
                // For example, on a 20° angled slope:
                // - Roll might be 15° (sideways tilt)
                // - Pitch might be 13° (forward/backward tilt)
                // - Robot will naturally try to turn downhill
                // - Lower wheel needs more power to maintain straight line

                // Determine which side needs compensation based on yaw deviation
                // - Positive yaw error means robot is turning right (downhill)
                // - Negative yaw error means robot is turning left (downhill)
                let (left_speed, right_speed) = if yaw_error > 0.0 {
                    // Robot drifting right (clockwise):
                    // - Reduce left motor speed to compensate
                    // - Keep right motor at base speed (uphill side needs more power)
                    ((base_speed as f32 * (1.0 - correction_factor)) as u8, base_speed)
                } else {
                    // Robot drifting left (counter-clockwise):
                    // - Keep left motor at base speed (uphill side needs more power)
                    // - Reduce right motor speed to compensate
                    (base_speed, (base_speed as f32 * (1.0 - correction_factor)) as u8)
                };

                // Return correction instruction:
                // - Asymmetric motor speeds to counter downhill drift
                // - Short duration to allow frequent reassessment
                // - Marks this as straight line correction type
                return Some(MotionCorrectionInstruction {
                    left_speed,
                    right_speed,
                    duration: Duration::from_millis(100), // Frequent updates needed on slopes
                    correction_type: CorrectionType::StraightLineCorrection,
                    emergency_stop: false,
                });
            }
        }
        None
    }
}

/// Instructions for motion correction
#[derive(Debug, Clone)]
pub struct MotionCorrectionInstruction {
    /// Speed setting for left motor (0-100)
    pub left_speed: u8,
    /// Speed setting for right motor (0-100)
    pub right_speed: u8,
    /// How long this correction should be applied
    pub duration: Duration,
    /// Type of correction being performed
    pub correction_type: CorrectionType,
    /// Whether this is an emergency stop situation
    pub emergency_stop: bool,
}

/// Types of motion corrections that can be performed
#[derive(Debug, Clone)]
pub enum CorrectionType {
    /// Compensation for driving up/down slopes
    TiltCompensation,
    /// Correction to maintain straight line motion
    StraightLineCorrection,
    /// Correction during rotation maneuvers
    RotationCorrection,
    /// Emergency stop due to unsafe conditions
    EmergencyStop,
}

/// Main task for motion correction control that:
/// - Receives motion data from various sensors
/// - Fuses sensor data together
/// - Determines if corrections are needed
/// - Sends correction commands
///
/// The task runs in a loop waiting for start/stop commands and processes
/// motion data when running. It will:
///
/// 1. Wait for a start command
/// 2. Begin collecting motion data
/// 3. Process incoming sensor measurements and drive commands
/// 4. Calculate needed corrections based on fused data
/// 5. Send correction events when needed
/// 6. Stop when commanded and clear history
#[embassy_executor::task]
pub async fn motion_correction_control() {
    'command: loop {
        let mut motion_fusion = MotionFusion::new();

        match MOTION_CONTROL.wait().await {
            MotionDataCommand::Start => {
                info!("Starting motion correction control");
                send_event(Events::StartStopMotionDataCollection(true)).await;

                loop {
                    match select(MOTION_CONTROL.wait(), receive_motion_data()).await {
                        Either::First(MotionDataCommand::Stop) => {
                            info!("Motion control stopped");
                            motion_fusion.clear();
                            send_event(Events::StartStopMotionDataCollection(false)).await;
                            continue 'command;
                        }
                        Either::First(_) => continue,
                        Either::Second(data) => {
                            motion_fusion.ingest_motion_data(data);
                            if let Some(correction) = motion_fusion.correct_motion() {
                                send_event(Events::MotionCorrectionRequired(correction)).await;
                            }
                        }
                    }
                }
            }
            MotionDataCommand::Stop => {
                info!("Motion control stopped");
                motion_fusion.clear();
                send_event(Events::StartStopMotionDataCollection(false)).await;
                continue 'command;
            }
        }
    }
}
