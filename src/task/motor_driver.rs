//! Motor Driver Task
//!
//! This module provides a task that controls motor speed and direction through PWM
//! and coordinates with the PCA9555 port expander for direction pin control.
//!
//! # Architecture
//!
//! The motor driver uses a dual-layer control system:
//! 1. **PWM Control (this module)**: Speed control via PWM duty cycle on GPIO 0-3
//! 2. **Direction Control (port_expander)**: Direction pins via PCA9555 I2C expander
//!
//! # Hardware Configuration
//!
//! ## Driver Layout (2 TB6612FNG drivers, 4 motors total)
//!
//! **Left Driver (PWM Slice 0, GPIO 0-1):**
//! - Front motor: GPIO 0 (PWM0A)
//! - Rear motor: GPIO 1 (PWM0B)
//!
//! **Right Driver (PWM Slice 1, GPIO 2-3):**
//! - Front motor: GPIO 2 (PWM1A)
//! - Rear motor: GPIO 3 (PWM1B)
//!
//! ## Direction Pins (via PCA9555 Port 0)
//! - Left Driver, Front Motor: Bits 0-1 (forward/backward)
//! - Left Driver, Rear Motor: Bits 2-3 (forward/backward)
//! - Right Driver, Front Motor: Bits 4-5 (forward/backward)
//! - Right Driver, Rear Motor: Bits 6-7 (forward/backward)
//!
//! ## Standby/Enable Pins (via PCA9555 Port 1)
//! - Left Driver: Bit 4
//! - Right Driver: Bit 5
//!
//! # Motor Calibration
//!
//! Each motor has an independent calibration factor (0.5 to 1.5) to compensate
//! for manufacturing variations. The calibration is applied as a multiplier
//! to the commanded speed before converting to PWM duty cycle.
//!
//! # Usage
//!
//! ```rust
//! // Set single motor speed
//! motor_driver::send_motor_command(MotorCommand::SetSpeed {
//!     driver: MotorDriver::Left,
//!     motor: MotorSelection::Front,
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
//!     driver: MotorDriver::Left,
//!     motor: MotorSelection::Front,
//! }).await;
//! ```

use defmt::{Format, debug, info, warn};
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

use crate::task::port_expander::{self, PortExpanderCommand, PortNumber};

/// Motor driver selection (left or right side of robot)
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum MotorDriver {
    Left,
    Right,
}

/// Motor position within a driver (front or rear)
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum MotorSelection {
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
    pub fn get_motor_direction_bits(driver: MotorDriver, motor: MotorSelection) -> (u8, u8) {
        let base = match (driver, motor) {
            (MotorDriver::Left, MotorSelection::Front) => 0,  // bits 0,1
            (MotorDriver::Left, MotorSelection::Rear) => 2,   // bits 2,3
            (MotorDriver::Right, MotorSelection::Front) => 4, // bits 4,5
            (MotorDriver::Right, MotorSelection::Rear) => 6,  // bits 6,7
        };
        (base, base + 1) // (forward_bit, backward_bit)
    }

    /// Get the bit position for a driver's enable/standby pin on Port 1
    pub fn get_driver_enable_bit(driver: MotorDriver) -> u8 {
        match driver {
            MotorDriver::Left => 4,
            MotorDriver::Right => 5,
        }
    }

    /// Create port expander command to set a motor's direction
    pub fn set_motor_direction_cmd(
        driver: MotorDriver,
        motor: MotorSelection,
        direction: MotorDirection,
    ) -> PortExpanderCommand {
        let (fwd_bit, bwd_bit) = get_motor_direction_bits(driver, motor);

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
        let set_direction = |value: &mut u8, driver: MotorDriver, motor: MotorSelection, dir: MotorDirection| {
            let (fwd_bit, bwd_bit) = get_motor_direction_bits(driver, motor);
            let (fwd_state, bwd_state) = match dir {
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

        set_direction(&mut port0_value, MotorDriver::Left, MotorSelection::Front, left_front);
        set_direction(&mut port0_value, MotorDriver::Left, MotorSelection::Rear, left_rear);
        set_direction(&mut port0_value, MotorDriver::Right, MotorSelection::Front, right_front);
        set_direction(&mut port0_value, MotorDriver::Right, MotorSelection::Rear, right_rear);

        PortExpanderCommand::SetOutputByte {
            port: PortNumber::Port0,
            value: port0_value,
        }
    }

    /// Create port expander command to enable/disable a motor driver
    pub fn set_driver_enable_cmd(driver: MotorDriver, enabled: bool) -> PortExpanderCommand {
        let bit = get_driver_enable_bit(driver);

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
    ///     track: MotorDriver::Left,
    ///     speed: 50,
    /// }).await;
    /// ```
    SetTrack { track: MotorDriver, speed: i8 },

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
    SetSpeed {
        driver: MotorDriver,
        motor: MotorSelection,
        speed: i8,
    },

    /// Brake individual motor (testing/safety)
    ///
    /// Actively stops motor using electrical braking (both direction pins HIGH).
    /// For normal operation, use `BrakeAll` instead.
    Brake { driver: MotorDriver, motor: MotorSelection },

    /// Coast individual motor (testing/safety)
    ///
    /// **Calibration: NOT APPLIED** ❌
    ///
    /// Stops motor by freewheeling (both direction pins LOW).
    /// For normal operation, use `CoastAll` instead.
    Coast { driver: MotorDriver, motor: MotorSelection },

    /// Brake all motors simultaneously (safety/emergency stop)
    ///
    /// Emergency stop using electrical braking on all motors.
    BrakeAll,

    /// Coast all motors simultaneously (safety/power-down)
    ///
    /// Stops all motors by freewheeling (minimal electrical load).
    CoastAll,

    // === DRIVER ENABLE/DISABLE COMMANDS ===
    /// Enable or disable entire motor driver
    ///
    /// When disabled, the driver enters standby mode (low power).
    /// All motor outputs are disabled regardless of direction pins.
    SetDriverEnable { driver: MotorDriver, enabled: bool },

    /// Enable or disable both motor drivers
    SetAllDriversEnable { enabled: bool },

    // === CALIBRATION MANAGEMENT COMMANDS ===
    /// Update calibration factor for a specific motor
    ///
    /// Calibration factors are multipliers (0.5 to 1.5) applied to commanded speeds
    /// to compensate for manufacturing variations between motors.
    UpdateCalibration {
        driver: MotorDriver,
        motor: MotorSelection,
        factor: f32,
    },

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
    /// Left driver, Front motor calibration factor
    pub left_front: f32,
    /// Left driver, Rear motor calibration factor
    pub left_rear: f32,
    /// Right driver, Front motor calibration factor
    pub right_front: f32,
    /// Right driver, Rear motor calibration factor
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
    pub fn get_factor(&self, driver: MotorDriver, motor: MotorSelection) -> f32 {
        match (driver, motor) {
            (MotorDriver::Left, MotorSelection::Front) => self.left_front,
            (MotorDriver::Left, MotorSelection::Rear) => self.left_rear,
            (MotorDriver::Right, MotorSelection::Front) => self.right_front,
            (MotorDriver::Right, MotorSelection::Rear) => self.right_rear,
        }
    }

    /// Set calibration factor for a specific motor
    pub fn set_factor(&mut self, driver: MotorDriver, motor: MotorSelection, factor: f32) {
        let clamped = factor.clamp(Self::MIN_FACTOR, Self::MAX_FACTOR);
        if clamped != factor {
            warn!("Calibration factor {} clamped to {}", factor, clamped);
        }

        match (driver, motor) {
            (MotorDriver::Left, MotorSelection::Front) => self.left_front = clamped,
            (MotorDriver::Left, MotorSelection::Rear) => self.left_rear = clamped,
            (MotorDriver::Right, MotorSelection::Front) => self.right_front = clamped,
            (MotorDriver::Right, MotorSelection::Rear) => self.right_rear = clamped,
        }
    }

    /// Apply calibration to a commanded speed
    ///
    /// Returns the calibrated speed, clamped to [-100, 100]
    pub fn apply(&self, driver: MotorDriver, motor: MotorSelection, speed: i8) -> i8 {
        let factor = self.get_factor(driver, motor);
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
    fn set_speed(&mut self, driver: MotorDriver, motor: MotorSelection, speed: i8) {
        let abs_speed = speed.abs() as u8;

        let pwm_output = match (driver, motor) {
            (MotorDriver::Left, MotorSelection::Front) => &mut self.left_front,
            (MotorDriver::Left, MotorSelection::Rear) => &mut self.left_rear,
            (MotorDriver::Right, MotorSelection::Front) => &mut self.right_front,
            (MotorDriver::Right, MotorSelection::Rear) => &mut self.right_rear,
        };

        let _ = pwm_output.set_duty_cycle_percent(abs_speed);
    }

    /// Set all PWM channels at once
    fn set_all_speeds(&mut self, left_front: i8, left_rear: i8, right_front: i8, right_rear: i8) {
        self.set_speed(MotorDriver::Left, MotorSelection::Front, left_front);
        self.set_speed(MotorDriver::Left, MotorSelection::Rear, left_rear);
        self.set_speed(MotorDriver::Right, MotorSelection::Front, right_front);
        self.set_speed(MotorDriver::Right, MotorSelection::Rear, right_rear);
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
    driver: MotorDriver,
    motor: MotorSelection,
    speed: i8,
) {
    // Apply calibration
    let calibrated_speed = calibration.apply(driver, motor, speed);

    debug!(
        "Motor {:?} {:?}: speed {} -> calibrated {}",
        driver, motor, speed, calibrated_speed
    );

    // Set direction via port expander using motor-specific helper
    let direction = speed_to_direction(calibrated_speed);
    let cmd = motor_port_mapping::set_motor_direction_cmd(driver, motor, direction);
    port_expander::send_command(cmd).await;

    // Set PWM duty cycle
    pwm_channels.set_speed(driver, motor, calibrated_speed);
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
    let cal_left_front = calibration.apply(MotorDriver::Left, MotorSelection::Front, left_front);
    let cal_left_rear = calibration.apply(MotorDriver::Left, MotorSelection::Rear, left_rear);
    let cal_right_front = calibration.apply(MotorDriver::Right, MotorSelection::Front, right_front);
    let cal_right_rear = calibration.apply(MotorDriver::Right, MotorSelection::Rear, right_rear);

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
            process_set_speed(pwm_channels, calibration, track, MotorSelection::Front, speed).await;
            process_set_speed(pwm_channels, calibration, track, MotorSelection::Rear, speed).await;
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
        MotorCommand::SetSpeed { driver, motor, speed } => {
            // Raw speed command - no calibration applied
            debug!("Raw motor {:?} {:?}: speed {}", driver, motor, speed);

            let direction = speed_to_direction(speed);
            let cmd = motor_port_mapping::set_motor_direction_cmd(driver, motor, direction);
            port_expander::send_command(cmd).await;

            pwm_channels.set_speed(driver, motor, speed);
        }

        MotorCommand::Brake { driver, motor } => {
            debug!("Braking motor {:?} {:?}", driver, motor);
            let cmd = motor_port_mapping::set_motor_direction_cmd(driver, motor, MotorDirection::Brake);
            port_expander::send_command(cmd).await;
            pwm_channels.set_speed(driver, motor, 0);
        }

        MotorCommand::Coast { driver, motor } => {
            debug!("Coasting motor {:?} {:?}", driver, motor);
            let cmd = motor_port_mapping::set_motor_direction_cmd(driver, motor, MotorDirection::Coast);
            port_expander::send_command(cmd).await;
            pwm_channels.set_speed(driver, motor, 0);
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

        MotorCommand::SetDriverEnable { driver, enabled } => {
            debug!("Setting driver {:?} enable: {}", driver, enabled);
            let cmd = motor_port_mapping::set_driver_enable_cmd(driver, enabled);
            port_expander::send_command(cmd).await;
        }

        MotorCommand::SetAllDriversEnable { enabled } => {
            debug!("Setting all drivers enable: {}", enabled);
            let cmd = motor_port_mapping::set_all_drivers_enable_cmd(enabled);
            port_expander::send_command(cmd).await;
        }

        MotorCommand::UpdateCalibration { driver, motor, factor } => {
            info!("Updating calibration for {:?} {:?}: {}", driver, motor, factor);
            calibration.set_factor(driver, motor, factor);
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
    }
}

/// Motor driver task - manages PWM control and coordinates with port expander
///
/// This task processes motor commands from a channel and controls:
/// - PWM duty cycle for speed control (via hardware PWM on GPIO 0-3)
/// - Motor direction (via port expander task)
/// - Driver enable/standby (via port expander task)
///
/// # Arguments
///
/// * `pwm_driver_left` - PWM slice for Left driver motors (GPIO 0/1, PWM0 A/B channels)
///   - Channel A: Front motor (GPIO 0)
///   - Channel B: Rear motor (GPIO 1)
/// * `pwm_driver_right` - PWM slice for Right driver motors (GPIO 2/3, PWM1 A/B channels)
///   - Channel A: Front motor (GPIO 2)
///   - Channel B: Rear motor (GPIO 3)
///
/// # Note
///
/// Direction pins and driver enable pins are controlled via the PCA9555
/// port expander task. This task only manages PWM speed control and
/// coordinates with the port expander for direction changes.
#[embassy_executor::task]
pub async fn motor_driver(pwm_driver_left: Pwm<'static>, pwm_driver_right: Pwm<'static>) {
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

    // Initialize calibration with defaults
    let mut calibration = MotorCalibration::default();
    info!("Motor calibration initialized: {:?}", calibration);

    // Set all motors to coast initially
    pwm_channels.set_all_speeds(0, 0, 0, 0);
    info!("Motor driver initialized - all motors coasting");

    // Main command processing loop
    loop {
        let command = receive_motor_command().await;
        process_command(&mut pwm_channels, &mut calibration, command).await;
    }
}
