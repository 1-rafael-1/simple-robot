//! Motor Driver Task
//!
//! This module provides a task that controls motor speed and direction through PWM
//! and coordinates with the PCA9555 port expander for direction pin control.
//!
//! # Terminology
//!
//! - **Track**: Left or right side of the robot (each has 2 motors mechanically coupled via belt/chain)
//! - **Motor**: Front or rear position within a track
//! - **Driver**: Physical TB6612FNG motor driver chip (one per track, controls 2 motors)
//!
//! # Architecture
//!
//! The motor driver uses a dual-layer control system:
//! 1. **PWM Control (this module)**: Speed control via PWM duty cycle on GPIO 0-3
//! 2. **Direction Control (port_expander)**: Direction pins via PCA9555 I2C expander
//! 3. **Encoder Feedback (this module)**: Pulse counting via PWM input on GPIO 7, 9, 21, 27
//!
//! # Hardware Configuration
//!
//! ## Driver Layout (2 TB6612FNG drivers, 4 motors total)
//!
//! **Left Track Driver (PWM Slice 0, GPIO 0-1):**
//! - Front motor: GPIO 0 (PWM0A)
//! - Rear motor: GPIO 1 (PWM0B)
//!
//! **Right Track Driver (PWM Slice 1, GPIO 2-3):**
//! - Front motor: GPIO 2 (PWM1A)
//! - Rear motor: GPIO 3 (PWM1B)
//!
//! ## Direction Pins (via PCA9555 Port 0)
//! - Left Track, Front Motor: Bits 0-1 (forward/backward)
//! - Left Track, Rear Motor: Bits 2-3 (forward/backward)
//! - Right Track, Front Motor: Bits 4-5 (forward/backward)
//! - Right Track, Rear Motor: Bits 6-7 (forward/backward)
//!
//! ## Standby/Enable Pins (via PCA9555 Port 1)
//! - Left Driver Chip: Bit 4
//! - Right Driver Chip: Bit 5
//!
//! ## Encoder Inputs (PWM Input Mode)
//! - Left Front Motor: GPIO 7 (PWM Slice 3B)
//! - Left Rear Motor: GPIO 21 (PWM Slice 2B)
//! - Right Front Motor: GPIO 9 (PWM Slice 4B)
//! - Right Rear Motor: GPIO 27 (PWM Slice 5B)
//!
//! # Motor Calibration
//!
//! Each motor has an independent calibration factor (0.5 to 1.5) to compensate
//! for manufacturing variations. The calibration is applied as a multiplier
//! to the commanded speed before converting to PWM duty cycle.
//!
//! ## Automatic Calibration Procedure
//!
//! The `RunCalibration` command performs a 9-step automated calibration:
//! 1. Test left track motors individually at 50% speed (2 seconds each)
//! 2. Match left track motors to the weaker one
//! 3. Verify left track by running both together
//! 4. Test right track motors individually at 50% speed
//! 5. Match right track motors to the weaker one
//! 6. Verify right track by running both together
//! 7. Match both tracks to the weakest overall motor
//! 8. Final verification with all 4 motors running
//! 9. Save calibration factors to flash storage
//!
//! **Important**: Robot must be elevated (wheels off ground) during calibration!
//!
//! The calibration uses encoder feedback to measure actual motor speeds and
//! automatically calculates correction factors. Single-point calibration at 50%
//! speed is used, with the assumption that motor speed relationships are
//! approximately linear in the operational range.
//!
//! # Usage
//!
//! ```rust
//! // Set single motor speed
//! motor_driver::send_motor_command(MotorCommand::SetSpeed {
//!     track: Track::Left,
//!     motor: Motor::Front,
//!     speed: 50,  // 50% forward
//! }).await;
//!
//! // Set all motors at once (atomic operation)
//! motor_driver::send_motor_command(MotorCommand::SetAllMotors {
//!     left_front: 50,
//!     left_rear: 50,
//!     right_front: 50,
//!     right_rear: 50,
//! }).await;
//!
//! // Brake a motor
//! motor_driver::send_motor_command(MotorCommand::Brake {
//!     track: Track::Left,
//!     motor: Motor::Front,
//! }).await;
//!
//! // Run motor calibration (robot must be elevated with wheels off ground)
//! motor_driver::send_motor_command(MotorCommand::RunCalibration).await;
//! ```

use defmt::{Format, debug, info, warn};
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};

use crate::task::flash_storage;
use crate::task::port_expander::{self, PortExpanderCommand, PortNumber};

/// Track selection (left or right side of robot)
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum Track {
    Left,
    Right,
}

/// Motor position within a track (front or rear)
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum Motor {
    Front,
    Rear,
}

/// Motor direction states
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum MotorDirection {
    Forward,
    Backward,
    Coast, // Both pins low - freewheeling
    Brake, // Both pins high - short circuit braking
}

/// Motor-specific port expander helpers
/// These functions prepare generic port expander commands for motor control
mod motor_port_mapping {
    use super::*;

    /// Get the bit positions for a specific motor's direction pins on Port 0
    ///
    /// Returns (forward_bit, backward_bit) positions
    pub fn get_motor_direction_bits(track: Track, motor: Motor) -> (u8, u8) {
        let base = match (track, motor) {
            (Track::Left, Motor::Front) => 0,  // bits 0,1
            (Track::Left, Motor::Rear) => 2,   // bits 2,3
            (Track::Right, Motor::Front) => 4, // bits 4,5
            (Track::Right, Motor::Rear) => 6,  // bits 6,7
        };
        (base, base + 1) // (forward_bit, backward_bit)
    }

    /// Get the bit position for a driver's enable/standby pin on Port 1
    pub fn get_driver_enable_bit(track: Track) -> u8 {
        match track {
            Track::Left => 4,
            Track::Right => 5,
        }
    }

    /// Create port expander command to set a motor's direction
    pub fn set_motor_direction_cmd(track: Track, motor: Motor, direction: MotorDirection) -> PortExpanderCommand {
        let (fwd_bit, bwd_bit) = get_motor_direction_bits(track, motor);

        let (fwd_state, bwd_state) = match direction {
            MotorDirection::Forward => (true, false),
            MotorDirection::Backward => (false, true),
            MotorDirection::Coast => (false, false),
            MotorDirection::Brake => (true, true),
        };

        // Create a mask for the two bits we're controlling
        let mask = (1 << fwd_bit) | (1 << bwd_bit);
        let value = ((fwd_state as u8) << fwd_bit) | ((bwd_state as u8) << bwd_bit);

        PortExpanderCommand::SetOutputBits {
            port: PortNumber::Port0,
            mask,
            value,
        }
    }

    /// Create port expander command to set all motor directions at once
    pub fn set_all_motor_directions_cmd(
        left_front: MotorDirection,
        left_rear: MotorDirection,
        right_front: MotorDirection,
        right_rear: MotorDirection,
    ) -> PortExpanderCommand {
        let mut port0_value = 0u8;

        // Helper to set direction bits
        let set_direction = |value: &mut u8, track: Track, motor: Motor, motor_dir: MotorDirection| {
            let (fwd_bit, bwd_bit) = get_motor_direction_bits(track, motor);
            let (fwd_state, bwd_state) = match motor_dir {
                MotorDirection::Forward => (true, false),
                MotorDirection::Backward => (false, true),
                MotorDirection::Coast => (false, false),
                MotorDirection::Brake => (true, true),
            };
            if fwd_state {
                *value |= 1 << fwd_bit;
            }
            if bwd_state {
                *value |= 1 << bwd_bit;
            }
        };

        set_direction(&mut port0_value, Track::Left, Motor::Front, left_front);
        set_direction(&mut port0_value, Track::Left, Motor::Rear, left_rear);
        set_direction(&mut port0_value, Track::Right, Motor::Front, right_front);
        set_direction(&mut port0_value, Track::Right, Motor::Rear, right_rear);

        PortExpanderCommand::SetOutputByte {
            port: PortNumber::Port0,
            value: port0_value,
        }
    }

    /// Create port expander command to enable/disable a motor driver
    pub fn set_driver_enable_cmd(track: Track, enabled: bool) -> PortExpanderCommand {
        let bit = get_driver_enable_bit(track);

        PortExpanderCommand::SetOutputPin {
            port: PortNumber::Port1,
            pin: bit,
            state: enabled,
        }
    }

    /// Create port expander command to enable/disable both drivers at once
    pub fn set_all_drivers_enable_cmd(enabled: bool) -> PortExpanderCommand {
        let mask = (1 << 4) | (1 << 5); // Bits 4 and 5 (left and right drivers)
        let value = if enabled { mask } else { 0 };

        PortExpanderCommand::SetOutputBits {
            port: PortNumber::Port1,
            mask,
            value,
        }
    }
}

/// Maximum number of pending motor commands
const COMMAND_QUEUE_SIZE: usize = 16;

/// Command channel for motor control operations
static MOTOR_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, MotorCommand, COMMAND_QUEUE_SIZE> = Channel::new();

/// Send a motor command to the driver task
pub async fn send_motor_command(command: MotorCommand) {
    MOTOR_COMMAND_CHANNEL.sender().send(command).await;
}

/// Try to send a motor command without blocking
/// Returns true if the command was queued, false if the queue is full
pub fn try_send_motor_command(command: MotorCommand) -> bool {
    MOTOR_COMMAND_CHANNEL.sender().try_send(command).is_ok()
}

/// Receive a motor command (internal use by motor_driver task)
async fn receive_motor_command() -> MotorCommand {
    MOTOR_COMMAND_CHANNEL.receiver().receive().await
}

/// Motor control commands
/// Motor commands for controlling speed, direction, and calibration
///
/// # Calibration Behavior
///
/// Commands are divided into two categories:
///
/// **Calibrated Commands** (normal operation):
/// - `SetTrack` - ✅ Calibration applied
/// - `SetTracks` - ✅ Calibration applied
/// - `SetAllMotors` - ✅ Calibration applied
///
/// **Raw Commands** (calibration & testing):
/// - `SetSpeed` - ❌ No calibration (for testing individual motors)
/// - `Brake`, `Coast`, `BrakeAll`, `CoastAll` - ❌ No calibration (safety commands)
/// - `UpdateCalibration` - ❌ No calibration (sets calibration values)
///
/// For normal operation, use `SetTrack` or `SetTracks` commands.
#[derive(Debug, Clone, Copy, Format)]
pub enum MotorCommand {
    // === CALIBRATION COMMAND ===
    /// Run motor calibration procedure
    ///
    /// This command performs an automated calibration of all 4 motors at 50% speed.
    /// The calibration process:
    /// 1. Tests each motor individually and reads encoder feedback
    /// 2. Matches motors within each track (left front/rear, right front/rear)
    /// 3. Matches both tracks to the weakest motor
    /// 4. Saves the calibration to flash storage
    ///
    /// The robot must be elevated (wheels off ground) during calibration.
    RunCalibration,

    // === NORMAL OPERATION COMMANDS (Calibrated) ===
    /// Set speed for an entire track (left or right side)
    ///
    /// **Calibration: APPLIED** ✅
    ///
    /// This is the primary command for normal operation. Both motors on the
    /// specified track will run at the same speed, with calibration applied.
    ///
    /// Speed range: -100 (full backward) to +100 (full forward)
    ///
    /// # Example
    /// ```rust
    /// // Drive left track at 50% forward, right track at 60% forward
    /// motor_driver::send_motor_command(MotorCommand::SetTrack {
    ///     track: Track::Left,
    ///     speed: 50,
    /// }).await;
    /// ```
    SetTrack { track: Track, speed: i8 },

    /// Set speeds for both tracks at once (most common operation)
    ///
    /// **Calibration: APPLIED** ✅
    ///
    /// Efficiently sets both left and right tracks in a single operation.
    /// This is the most efficient way to control the robot during normal driving.
    ///
    /// Speed range: -100 (full backward) to +100 (full forward)
    ///
    /// # Example
    /// ```rust
    /// // Drive straight at 50%
    /// motor_driver::send_motor_command(MotorCommand::SetTracks {
    ///     left_speed: 50,
    ///     right_speed: 50,
    /// }).await;
    ///
    /// // Turn right (left faster than right)
    /// motor_driver::send_motor_command(MotorCommand::SetTracks {
    ///     left_speed: 60,
    ///     right_speed: 40,
    /// }).await;
    /// ```
    SetTracks { left_speed: i8, right_speed: i8 },

    /// Set all motors individually (advanced operation)
    ///
    /// **Calibration applied**
    ///
    /// Allows independent control of all 4 motors with calibration applied.
    /// Useful for advanced maneuvers or testing, but `SetTracks` is preferred
    /// for normal operation.
    ///
    /// This updates all motor directions and PWM values in an optimized way:
    /// - Single I2C transaction for all direction pins
    /// - Rapid PWM updates for all channels
    SetAllMotors {
        left_front: i8,
        left_rear: i8,
        right_front: i8,
        right_rear: i8,
    },

    // === CALIBRATION & STOPPING COMMANDS (Raw, No Calibration) ===
    /// Set individual motor speed and direction (calibration/stopping only)
    ///
    /// **No calibration applied**
    ///
    /// This is a raw command used for motor testing and calibration determination.
    /// For normal operation, use `SetTrack` or `SetTracks` instead.
    ///
    /// Speed range: -100 (full backward) to +100 (full forward)
    /// Speed 0 will coast the motor (both direction pins LOW)
    SetSpeed { track: Track, motor: Motor, speed: i8 },

    /// Brake individual motor (testing/safety)
    ///
    /// Actively stops motor using electrical braking (both direction pins HIGH).
    /// For normal operation, use `BrakeAll` instead.
    Brake { track: Track, motor: Motor },

    /// Coast individual motor (testing/safety)
    ///
    /// **Calibration: NOT APPLIED** ❌
    ///
    /// Stops motor by freewheeling (both direction pins LOW).
    /// For normal operation, use `CoastAll` instead.
    Coast { track: Track, motor: Motor },

    /// Brake all motors simultaneously (safety/emergency stop)
    ///
    /// Emergency stop using electrical braking on all motors.
    BrakeAll,

    /// Coast all motors simultaneously (safety/power-down)
    ///
    /// Stops all motors by freewheeling (minimal electrical load).
    CoastAll,

    // === DRIVER ENABLE/DISABLE COMMANDS ===
    /// Enable or disable entire motor driver chip
    ///
    /// When disabled, the driver chip enters standby mode (low power).
    /// All motor outputs are disabled regardless of direction pins.
    SetDriverEnable { track: Track, enabled: bool },

    /// Enable or disable both motor driver chips
    SetAllDriversEnable { enabled: bool },

    // === CALIBRATION MANAGEMENT COMMANDS ===
    /// Update calibration factor for a specific motor
    ///
    /// Calibration factors are multipliers (0.5 to 1.5) applied to commanded speeds
    /// to compensate for manufacturing variations between motors.
    UpdateCalibration { track: Track, motor: Motor, factor: f32 },

    /// Update all calibration factors at once
    ///
    /// Use this to load a complete calibration profile.
    UpdateAllCalibration {
        left_front: f32,
        left_rear: f32,
        right_front: f32,
        right_rear: f32,
    },
}

/// Motor calibration factors for speed balancing
///
/// Each factor is a multiplier applied to the commanded speed.
/// Default value is 1.0 (no correction).
///
/// Valid range: 0.5 to 1.5 (clamped)
#[derive(Debug, Clone, Copy, Format)]
pub struct MotorCalibration {
    /// Left track, Front motor calibration factor
    pub left_front: f32,
    /// Left track, Rear motor calibration factor
    pub left_rear: f32,
    /// Right track, Front motor calibration factor
    pub right_front: f32,
    /// Right track, Rear motor calibration factor
    pub right_rear: f32,
}

impl Default for MotorCalibration {
    fn default() -> Self {
        Self {
            left_front: 1.0,
            left_rear: 1.0,
            right_front: 1.0,
            right_rear: 1.0,
        }
    }
}

impl MotorCalibration {
    /// Clamp calibration factor to somewhat safe range
    const MIN_FACTOR: f32 = 0.5;
    const MAX_FACTOR: f32 = 1.5;

    /// Create new calibration with specified factors
    pub fn new(left_front: f32, left_rear: f32, right_front: f32, right_rear: f32) -> Self {
        let mut cal = Self {
            left_front,
            left_rear,
            right_front,
            right_rear,
        };
        cal.clamp_all();
        cal
    }

    /// Get calibration factor for a specific motor
    pub fn get_factor(&self, track: Track, motor: Motor) -> f32 {
        match (track, motor) {
            (Track::Left, Motor::Front) => self.left_front,
            (Track::Left, Motor::Rear) => self.left_rear,
            (Track::Right, Motor::Front) => self.right_front,
            (Track::Right, Motor::Rear) => self.right_rear,
        }
    }

    /// Set calibration factor for a specific motor
    pub fn set_factor(&mut self, track: Track, motor: Motor, factor: f32) {
        let clamped = factor.clamp(Self::MIN_FACTOR, Self::MAX_FACTOR);
        if clamped != factor {
            warn!("Calibration factor {} clamped to {}", factor, clamped);
        }

        match (track, motor) {
            (Track::Left, Motor::Front) => self.left_front = clamped,
            (Track::Left, Motor::Rear) => self.left_rear = clamped,
            (Track::Right, Motor::Front) => self.right_front = clamped,
            (Track::Right, Motor::Rear) => self.right_rear = clamped,
        }
    }

    /// Apply calibration to a commanded speed
    ///
    /// Returns the calibrated speed, clamped to [-100, 100]
    pub fn apply(&self, track: Track, motor: Motor, speed: i8) -> i8 {
        let factor = self.get_factor(track, motor);
        let calibrated_f32 = speed as f32 * factor;
        let calibrated = if calibrated_f32 >= 0.0 {
            (calibrated_f32 + 0.5) as i8
        } else {
            (calibrated_f32 - 0.5) as i8
        };
        calibrated.clamp(-100, 100)
    }

    /// Clamp all factors to valid range
    fn clamp_all(&mut self) {
        self.left_front = self.left_front.clamp(Self::MIN_FACTOR, Self::MAX_FACTOR);
        self.left_rear = self.left_rear.clamp(Self::MIN_FACTOR, Self::MAX_FACTOR);
        self.right_front = self.right_front.clamp(Self::MIN_FACTOR, Self::MAX_FACTOR);
        self.right_rear = self.right_rear.clamp(Self::MIN_FACTOR, Self::MAX_FACTOR);
    }
}

/// PWM channel mapping - uses PWM slices with A/B channels split into separate outputs
struct PwmChannels {
    left_front: embassy_rp::pwm::PwmOutput<'static>, // Left driver, SLICE0 Channel A
    left_rear: embassy_rp::pwm::PwmOutput<'static>,  // Left driver, SLICE0 Channel B
    right_front: embassy_rp::pwm::PwmOutput<'static>, // Right driver, SLICE1 Channel A
    right_rear: embassy_rp::pwm::PwmOutput<'static>, // Right driver, SLICE1 Channel B
}

impl PwmChannels {
    /// Set PWM duty cycle based on speed percentage
    ///
    /// Speed range: -100 to +100
    /// Duty cycle: 0 to 100%
    ///
    /// Each motor has independent PWM control via the split channels.
    fn set_speed(&mut self, track: Track, motor: Motor, speed: i8) {
        let abs_speed = speed.abs() as u8;

        let pwm_output = match (track, motor) {
            (Track::Left, Motor::Front) => &mut self.left_front,
            (Track::Left, Motor::Rear) => &mut self.left_rear,
            (Track::Right, Motor::Front) => &mut self.right_front,
            (Track::Right, Motor::Rear) => &mut self.right_rear,
        };

        let _ = pwm_output.set_duty_cycle_percent(abs_speed);
    }

    /// Set all PWM channels at once
    fn set_all_speeds(&mut self, left_front: i8, left_rear: i8, right_front: i8, right_rear: i8) {
        self.set_speed(Track::Left, Motor::Front, left_front);
        self.set_speed(Track::Left, Motor::Rear, left_rear);
        self.set_speed(Track::Right, Motor::Front, right_front);
        self.set_speed(Track::Right, Motor::Rear, right_rear);
    }
}

/// Convert speed value to motor direction
fn speed_to_direction(speed: i8) -> MotorDirection {
    match speed {
        s if s > 0 => MotorDirection::Forward,
        s if s < 0 => MotorDirection::Backward,
        _ => MotorDirection::Coast,
    }
}

/// Process a single motor speed command
async fn process_set_speed(
    pwm_channels: &mut PwmChannels,
    calibration: &MotorCalibration,
    track: Track,
    motor: Motor,
    speed: i8,
) {
    // Apply calibration
    let calibrated_speed = calibration.apply(track, motor, speed);

    debug!(
        "Motor {:?} {:?}: speed {} -> calibrated {}",
        track, motor, speed, calibrated_speed
    );

    // Set direction via port expander using motor-specific helper
    let direction = speed_to_direction(calibrated_speed);
    let cmd = motor_port_mapping::set_motor_direction_cmd(track, motor, direction);
    port_expander::send_command(cmd).await;

    // Set PWM duty cycle
    pwm_channels.set_speed(track, motor, calibrated_speed);
}

/// Process bulk motor command (all 4 motors at once)
async fn process_set_all_motors(
    pwm_channels: &mut PwmChannels,
    calibration: &MotorCalibration,
    left_front: i8,
    left_rear: i8,
    right_front: i8,
    right_rear: i8,
) {
    // Apply calibration to all motors
    let cal_left_front = calibration.apply(Track::Left, Motor::Front, left_front);
    let cal_left_rear = calibration.apply(Track::Left, Motor::Rear, left_rear);
    let cal_right_front = calibration.apply(Track::Right, Motor::Front, right_front);
    let cal_right_rear = calibration.apply(Track::Right, Motor::Rear, right_rear);

    debug!(
        "All motors: [{}, {}, {}, {}] -> calibrated [{}, {}, {}, {}]",
        left_front, left_rear, right_front, right_rear, cal_left_front, cal_left_rear, cal_right_front, cal_right_rear
    );

    // Set all directions via port expander (atomic bulk operation)
    let cmd = motor_port_mapping::set_all_motor_directions_cmd(
        speed_to_direction(cal_left_front),
        speed_to_direction(cal_left_rear),
        speed_to_direction(cal_right_front),
        speed_to_direction(cal_right_rear),
    );
    port_expander::send_command(cmd).await;

    // Set all PWM duty cycles
    pwm_channels.set_all_speeds(cal_left_front, cal_left_rear, cal_right_front, cal_right_rear);
}

/// Process a motor command
async fn process_command(pwm_channels: &mut PwmChannels, calibration: &mut MotorCalibration, command: MotorCommand) {
    match command {
        // Normal operation commands (calibrated)
        MotorCommand::SetTrack { track, speed } => {
            // Set both motors on the track to the same speed (with calibration)
            process_set_speed(pwm_channels, calibration, track, Motor::Front, speed).await;
            process_set_speed(pwm_channels, calibration, track, Motor::Rear, speed).await;
        }

        MotorCommand::SetTracks {
            left_speed,
            right_speed,
        } => {
            // Set both tracks efficiently (with calibration)
            process_set_all_motors(
                pwm_channels,
                calibration,
                left_speed,
                left_speed,
                right_speed,
                right_speed,
            )
            .await;
        }

        MotorCommand::SetAllMotors {
            left_front,
            left_rear,
            right_front,
            right_rear,
        } => {
            process_set_all_motors(
                pwm_channels,
                calibration,
                left_front,
                left_rear,
                right_front,
                right_rear,
            )
            .await;
        }

        // Raw commands (no calibration)
        MotorCommand::SetSpeed { track, motor, speed } => {
            // Raw speed command - no calibration applied
            debug!("Raw motor {:?} {:?}: speed {}", track, motor, speed);

            let direction = speed_to_direction(speed);
            let cmd = motor_port_mapping::set_motor_direction_cmd(track, motor, direction);
            port_expander::send_command(cmd).await;

            pwm_channels.set_speed(track, motor, speed);
        }

        MotorCommand::Brake { track, motor } => {
            debug!("Braking motor {:?} {:?}", track, motor);
            let cmd = motor_port_mapping::set_motor_direction_cmd(track, motor, MotorDirection::Brake);
            port_expander::send_command(cmd).await;
            pwm_channels.set_speed(track, motor, 0);
        }

        MotorCommand::Coast { track, motor } => {
            debug!("Coasting motor {:?} {:?}", track, motor);
            let cmd = motor_port_mapping::set_motor_direction_cmd(track, motor, MotorDirection::Coast);
            port_expander::send_command(cmd).await;
            pwm_channels.set_speed(track, motor, 0);
        }

        MotorCommand::BrakeAll => {
            debug!("Braking all motors");
            let cmd = motor_port_mapping::set_all_motor_directions_cmd(
                MotorDirection::Brake,
                MotorDirection::Brake,
                MotorDirection::Brake,
                MotorDirection::Brake,
            );
            port_expander::send_command(cmd).await;
            pwm_channels.set_all_speeds(0, 0, 0, 0);
        }

        MotorCommand::CoastAll => {
            debug!("Coasting all motors");
            let cmd = motor_port_mapping::set_all_motor_directions_cmd(
                MotorDirection::Coast,
                MotorDirection::Coast,
                MotorDirection::Coast,
                MotorDirection::Coast,
            );
            port_expander::send_command(cmd).await;
            pwm_channels.set_all_speeds(0, 0, 0, 0);
        }

        MotorCommand::SetDriverEnable { track, enabled } => {
            debug!("Setting track {:?} enable: {}", track, enabled);
            let cmd = motor_port_mapping::set_driver_enable_cmd(track, enabled);
            port_expander::send_command(cmd).await;
        }

        MotorCommand::SetAllDriversEnable { enabled } => {
            debug!("Setting all drivers enable: {}", enabled);
            let cmd = motor_port_mapping::set_all_drivers_enable_cmd(enabled);
            port_expander::send_command(cmd).await;
        }

        MotorCommand::UpdateCalibration { track, motor, factor } => {
            info!("Updating calibration for {:?} {:?}: {}", track, motor, factor);
            calibration.set_factor(track, motor, factor);
        }

        MotorCommand::UpdateAllCalibration {
            left_front,
            left_rear,
            right_front,
            right_rear,
        } => {
            info!(
                "Updating all calibration: [{}, {}, {}, {}]",
                left_front, left_rear, right_front, right_rear
            );
            *calibration = MotorCalibration::new(left_front, left_rear, right_front, right_rear);
        }

        MotorCommand::RunCalibration => {
            // This command is handled in the main task loop, not here
            warn!("RunCalibration command received in process_command - should be handled in main loop");
        }
    }
}

/// Encoder configuration constants
const ENCODER_PULSES_PER_MOTOR_REV: u16 = 8;
const ENCODER_GEAR_RATIO: u16 = 120;
const ENCODER_PULSES_PER_OUTPUT_REV: u16 = ENCODER_PULSES_PER_MOTOR_REV * ENCODER_GEAR_RATIO;

/// Calibration test parameters
const CALIBRATION_SPEED: i8 = 50; // 50% forward speed for calibration
const CALIBRATION_SAMPLE_DURATION_MS: u64 = 2000; // 2 second measurement window
const CALIBRATION_COAST_DURATION_MS: u64 = 500; // 0.5 second coast between tests

/// Encoder channels for all 4 motors
struct EncoderChannels {
    left_front: Pwm<'static>,  // GPIO 7, PWM Slice 3B
    left_rear: Pwm<'static>,   // GPIO 21, PWM Slice 2B
    right_front: Pwm<'static>, // GPIO 9, PWM Slice 4B
    right_rear: Pwm<'static>,  // GPIO 27, PWM Slice 5B
}

impl EncoderChannels {
    /// Reset all encoder counters to zero
    fn reset_all(&self) {
        self.left_front.set_counter(0);
        self.left_rear.set_counter(0);
        self.right_front.set_counter(0);
        self.right_rear.set_counter(0);
    }

    /// Read encoder count for a specific motor
    fn read(&self, track: Track, motor: Motor) -> u16 {
        match (track, motor) {
            (Track::Left, Motor::Front) => self.left_front.counter(),
            (Track::Left, Motor::Rear) => self.left_rear.counter(),
            (Track::Right, Motor::Front) => self.right_front.counter(),
            (Track::Right, Motor::Rear) => self.right_rear.counter(),
        }
    }

    /// Read all encoder counts at once
    fn read_all(&self) -> [u16; 4] {
        [
            self.left_front.counter(),
            self.left_rear.counter(),
            self.right_front.counter(),
            self.right_rear.counter(),
        ]
    }
}

/// Run the motor calibration procedure
///
/// This function performs automated calibration of all 4 motors:
/// 1. Tests left front motor individually, then left rear motor individually
/// 2. Matches left track motors (adjusts stronger to match weaker)
/// 3. Verifies left track by running both motors together
/// 4. Tests right front motor individually, then right rear motor individually
/// 5. Matches right track motors (adjusts stronger to match weaker)
/// 6. Verifies right track by running both motors together
/// 7. Compares tracks and matches to the weakest overall
/// 8. Final verification with all 4 motors
/// 9. Saves calibration to flash storage
///
/// Note: Each motor must be tested individually because front and rear motors
/// on the same track are mechanically coupled. Running them together before
/// matching would cause them to fight each other.
async fn run_motor_calibration(
    pwm_channels: &mut PwmChannels,
    encoders: &EncoderChannels,
    calibration: &mut MotorCalibration,
) {
    info!("=== Starting Motor Calibration ===");
    info!("Initial calibration: {:?}", calibration);

    // Step 1: Test left track motors individually
    info!("Step 1: Testing left track motors individually");

    // Test left front motor alone
    info!("  Testing left front motor");
    let cmd = motor_port_mapping::set_motor_direction_cmd(Track::Left, Motor::Front, MotorDirection::Forward);
    port_expander::send_command(cmd).await;
    let coast_cmd = motor_port_mapping::set_motor_direction_cmd(Track::Left, Motor::Rear, MotorDirection::Coast);
    port_expander::send_command(coast_cmd).await;
    let enable_cmd = motor_port_mapping::set_driver_enable_cmd(Track::Left, true);
    port_expander::send_command(enable_cmd).await;

    encoders.reset_all();
    pwm_channels.set_speed(Track::Left, Motor::Front, CALIBRATION_SPEED);
    pwm_channels.set_speed(Track::Left, Motor::Rear, 0);

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let left_front_count = encoders.read(Track::Left, Motor::Front);
    info!("    Left front encoder count: {}", left_front_count);

    // Coast left front
    let coast_cmd = motor_port_mapping::set_motor_direction_cmd(Track::Left, Motor::Front, MotorDirection::Coast);
    port_expander::send_command(coast_cmd).await;
    pwm_channels.set_speed(Track::Left, Motor::Front, 0);
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // Test left rear motor alone
    info!("  Testing left rear motor");
    let cmd = motor_port_mapping::set_motor_direction_cmd(Track::Left, Motor::Rear, MotorDirection::Forward);
    port_expander::send_command(cmd).await;

    encoders.reset_all();
    pwm_channels.set_speed(Track::Left, Motor::Rear, CALIBRATION_SPEED);

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let left_rear_count = encoders.read(Track::Left, Motor::Rear);
    info!("    Left rear encoder count: {}", left_rear_count);

    // Coast left rear
    let coast_cmd = motor_port_mapping::set_motor_direction_cmd(Track::Left, Motor::Rear, MotorDirection::Coast);
    port_expander::send_command(coast_cmd).await;
    pwm_channels.set_speed(Track::Left, Motor::Rear, 0);
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // Step 2: Match left track motors
    info!("Step 2: Matching left track motors");
    let left_weaker_count = if left_front_count < left_rear_count {
        info!("  Left front is weaker (reference)");
        let factor = left_front_count as f32 / left_rear_count as f32;
        calibration.left_rear *= factor;
        info!("  Adjustment factor: {}", factor);
        left_front_count
    } else {
        info!("  Left rear is weaker (reference)");
        let factor = left_rear_count as f32 / left_front_count as f32;
        calibration.left_front *= factor;
        info!("  Adjustment factor: {}", factor);
        left_rear_count
    };

    // Step 3: Verify left track match by running both motors together
    info!("Step 3: Verifying left track match");
    let cmd = motor_port_mapping::set_all_motor_directions_cmd(
        MotorDirection::Forward,
        MotorDirection::Forward,
        MotorDirection::Coast,
        MotorDirection::Coast,
    );
    port_expander::send_command(cmd).await;

    encoders.reset_all();
    let cal_left_front = calibration.apply(Track::Left, Motor::Front, CALIBRATION_SPEED);
    let cal_left_rear = calibration.apply(Track::Left, Motor::Rear, CALIBRATION_SPEED);
    pwm_channels.set_speed(Track::Left, Motor::Front, cal_left_front);
    pwm_channels.set_speed(Track::Left, Motor::Rear, cal_left_rear);

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let verify_left_front = encoders.read(Track::Left, Motor::Front);
    let verify_left_rear = encoders.read(Track::Left, Motor::Rear);
    info!("  Left front: {}, Left rear: {}", verify_left_front, verify_left_rear);

    // Coast left motors
    let coast_cmd = motor_port_mapping::set_all_motor_directions_cmd(
        MotorDirection::Coast,
        MotorDirection::Coast,
        MotorDirection::Coast,
        MotorDirection::Coast,
    );
    port_expander::send_command(coast_cmd).await;
    pwm_channels.set_all_speeds(0, 0, 0, 0);
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // Step 4: Test right track motors individually
    info!("Step 4: Testing right track motors individually");

    // Test right front motor alone
    info!("  Testing right front motor");
    let cmd = motor_port_mapping::set_motor_direction_cmd(Track::Right, Motor::Front, MotorDirection::Forward);
    port_expander::send_command(cmd).await;
    let coast_cmd = motor_port_mapping::set_motor_direction_cmd(Track::Right, Motor::Rear, MotorDirection::Coast);
    port_expander::send_command(coast_cmd).await;
    let enable_cmd = motor_port_mapping::set_driver_enable_cmd(Track::Right, true);
    port_expander::send_command(enable_cmd).await;

    encoders.reset_all();
    pwm_channels.set_speed(Track::Right, Motor::Front, CALIBRATION_SPEED);
    pwm_channels.set_speed(Track::Right, Motor::Rear, 0);

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let right_front_count = encoders.read(Track::Right, Motor::Front);
    info!("    Right front encoder count: {}", right_front_count);

    // Coast right front
    let coast_cmd = motor_port_mapping::set_motor_direction_cmd(Track::Right, Motor::Front, MotorDirection::Coast);
    port_expander::send_command(coast_cmd).await;
    pwm_channels.set_speed(Track::Right, Motor::Front, 0);
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // Test right rear motor alone
    info!("  Testing right rear motor");
    let cmd = motor_port_mapping::set_motor_direction_cmd(Track::Right, Motor::Rear, MotorDirection::Forward);
    port_expander::send_command(cmd).await;

    encoders.reset_all();
    pwm_channels.set_speed(Track::Right, Motor::Rear, CALIBRATION_SPEED);

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let right_rear_count = encoders.read(Track::Right, Motor::Rear);
    info!("    Right rear encoder count: {}", right_rear_count);

    // Coast right rear
    let coast_cmd = motor_port_mapping::set_motor_direction_cmd(Track::Right, Motor::Rear, MotorDirection::Coast);
    port_expander::send_command(coast_cmd).await;
    pwm_channels.set_speed(Track::Right, Motor::Rear, 0);
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // Step 5: Match right track motors
    info!("Step 5: Matching right track motors");
    let right_weaker_count = if right_front_count < right_rear_count {
        info!("  Right front is weaker (reference)");
        let factor = right_front_count as f32 / right_rear_count as f32;
        calibration.right_rear *= factor;
        info!("  Adjustment factor: {}", factor);
        right_front_count
    } else {
        info!("  Right rear is weaker (reference)");
        let factor = right_rear_count as f32 / right_front_count as f32;
        calibration.right_front *= factor;
        info!("  Adjustment factor: {}", factor);
        right_rear_count
    };

    // Step 6: Verify right track match by running both motors together
    info!("Step 6: Verifying right track match");
    let cmd = motor_port_mapping::set_all_motor_directions_cmd(
        MotorDirection::Coast,
        MotorDirection::Coast,
        MotorDirection::Forward,
        MotorDirection::Forward,
    );
    port_expander::send_command(cmd).await;
    let enable_cmd = motor_port_mapping::set_driver_enable_cmd(Track::Right, true);
    port_expander::send_command(enable_cmd).await;

    encoders.reset_all();
    let cal_right_front = calibration.apply(Track::Right, Motor::Front, CALIBRATION_SPEED);
    let cal_right_rear = calibration.apply(Track::Right, Motor::Rear, CALIBRATION_SPEED);
    pwm_channels.set_speed(Track::Right, Motor::Front, cal_right_front);
    pwm_channels.set_speed(Track::Right, Motor::Rear, cal_right_rear);

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let verify_right_front = encoders.read(Track::Right, Motor::Front);
    let verify_right_rear = encoders.read(Track::Right, Motor::Rear);
    info!(
        "  Right front: {}, Right rear: {}",
        verify_right_front, verify_right_rear
    );

    // Coast right motors
    let coast_cmd = motor_port_mapping::set_all_motor_directions_cmd(
        MotorDirection::Coast,
        MotorDirection::Coast,
        MotorDirection::Coast,
        MotorDirection::Coast,
    );
    port_expander::send_command(coast_cmd).await;
    pwm_channels.set_all_speeds(0, 0, 0, 0);
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // Step 7: Match both tracks to the weakest overall
    // Now we can safely run both motors per track together since they're matched
    info!("Step 7: Matching tracks to weakest overall motor");
    let track_adjustment = if left_weaker_count < right_weaker_count {
        info!("  Left track is weaker (reference)");
        let factor = left_weaker_count as f32 / right_weaker_count as f32;
        calibration.right_front *= factor;
        calibration.right_rear *= factor;
        factor
    } else {
        info!("  Right track is weaker (reference)");
        let factor = right_weaker_count as f32 / left_weaker_count as f32;
        calibration.left_front *= factor;
        calibration.left_rear *= factor;
        factor
    };
    info!("  Track adjustment factor: {}", track_adjustment);

    // Step 8: Final verification with all motors
    info!("Step 8: Final verification with all motors");
    let cmd = motor_port_mapping::set_all_motor_directions_cmd(
        MotorDirection::Forward,
        MotorDirection::Forward,
        MotorDirection::Forward,
        MotorDirection::Forward,
    );
    port_expander::send_command(cmd).await;
    let enable_cmd = motor_port_mapping::set_all_drivers_enable_cmd(true);
    port_expander::send_command(enable_cmd).await;

    encoders.reset_all();
    let cal_left_front = calibration.apply(Track::Left, Motor::Front, CALIBRATION_SPEED);
    let cal_left_rear = calibration.apply(Track::Left, Motor::Rear, CALIBRATION_SPEED);
    let cal_right_front = calibration.apply(Track::Right, Motor::Front, CALIBRATION_SPEED);
    let cal_right_rear = calibration.apply(Track::Right, Motor::Rear, CALIBRATION_SPEED);
    pwm_channels.set_all_speeds(cal_left_front, cal_left_rear, cal_right_front, cal_right_rear);

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let final_counts = encoders.read_all();
    info!("  Final encoder counts:");
    info!("    Left front:  {}", final_counts[0]);
    info!("    Left rear:   {}", final_counts[1]);
    info!("    Right front: {}", final_counts[2]);
    info!("    Right rear:  {}", final_counts[3]);

    // Stop all motors
    let coast_cmd = motor_port_mapping::set_all_motor_directions_cmd(
        MotorDirection::Coast,
        MotorDirection::Coast,
        MotorDirection::Coast,
        MotorDirection::Coast,
    );
    port_expander::send_command(coast_cmd).await;
    pwm_channels.set_all_speeds(0, 0, 0, 0);

    // Step 9: Save calibration to flash
    info!("Step 9: Saving calibration to flash");
    info!("Final calibration factors: {:?}", calibration);
    flash_storage::send_flash_command(flash_storage::FlashCommand::SaveMotorCalibration(*calibration)).await;

    info!("=== Calibration Complete ===");
}

/// Motor driver task - manages PWM control and coordinates with port expander
///
/// This task processes motor commands from a channel and controls:
/// - PWM duty cycle for speed control (via hardware PWM on GPIO 0-3)
/// - Motor direction (via port expander task)
/// - Driver enable/standby (via port expander task)
/// - Encoder feedback for calibration (via PWM input on GPIO 7, 9, 21, 27)
///
/// # Arguments
///
/// * `pwm_driver_left` - PWM slice for Left track motors (GPIO 0/1, PWM0 A/B channels)
///   - Channel A: Front motor (GPIO 0)
///   - Channel B: Rear motor (GPIO 1)
/// * `pwm_driver_right` - PWM slice for Right track motors (GPIO 2/3, PWM1 A/B channels)
///   - Channel A: Front motor (GPIO 2)
///   - Channel B: Rear motor (GPIO 3)
/// * `encoder_left_front` - Encoder input for left front motor (GPIO 7, PWM3B)
/// * `encoder_left_rear` - Encoder input for left rear motor (GPIO 21, PWM2B)
/// * `encoder_right_front` - Encoder input for right front motor (GPIO 9, PWM4B)
/// * `encoder_right_rear` - Encoder input for right rear motor (GPIO 27, PWM5B)
///
/// # Note
///
/// Direction pins and driver chip enable pins are controlled via the PCA9555
/// port expander task. This task only manages PWM speed control and
/// coordinates with the port expander for direction changes.
#[embassy_executor::task]
pub async fn motor_driver(
    pwm_driver_left: Pwm<'static>,
    pwm_driver_right: Pwm<'static>,
    encoder_left_front: Pwm<'static>,
    encoder_left_rear: Pwm<'static>,
    encoder_right_front: Pwm<'static>,
    encoder_right_rear: Pwm<'static>,
) {
    info!("Motor driver task starting");

    // Split PWM slices into separate channel outputs for independent control
    let (left_a, left_b) = pwm_driver_left.split();
    let (right_a, right_b) = pwm_driver_right.split();

    // Initialize PWM channels - unwrap is safe because we created outputs for both A and B
    let mut pwm_channels = PwmChannels {
        left_front: left_a.expect("Left driver channel A not configured"),
        left_rear: left_b.expect("Left driver channel B not configured"),
        right_front: right_a.expect("Right driver channel A not configured"),
        right_rear: right_b.expect("Right driver channel B not configured"),
    };

    // Initialize encoder channels
    let encoders = EncoderChannels {
        left_front: encoder_left_front,
        left_rear: encoder_left_rear,
        right_front: encoder_right_front,
        right_rear: encoder_right_rear,
    };

    // Initialize calibration - try to load from flash, otherwise use defaults
    let mut calibration = flash_storage::get_calibration().await.motor;
    info!("Motor calibration loaded: {:?}", calibration);

    // Set all motors to coast initially
    pwm_channels.set_all_speeds(0, 0, 0, 0);
    info!("Motor driver initialized - all motors coasting");

    // Main command processing loop
    loop {
        let command = receive_motor_command().await;

        // Handle calibration command specially since it needs encoder access
        match command {
            MotorCommand::RunCalibration => {
                run_motor_calibration(&mut pwm_channels, &encoders, &mut calibration).await;
            }
            _ => {
                process_command(&mut pwm_channels, &mut calibration, command).await;
            }
        }
    }
}
