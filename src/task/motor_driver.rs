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
//! ## Motor Calibration
//!
//! Motor calibration is now handled by the `drive` task, which coordinates
//! encoder readings from the `encoder_read` task with motor commands to
//! this task. This provides better separation of concerns:
//! - `motor_driver`: Pure actuation (PWM control)
//! - `encoder_read`: Pure sensing (encoder pulse counting)
//! - `drive`: Control logic (calibration procedures)
//!
//! Calibration factors are stored in the `MotorCalibration` struct and can be
//! loaded from flash storage or updated dynamically using the `UpdateCalibration`
//! and `LoadCalibration` commands.
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
//! // Load calibration from flash storage
//! motor_driver::send_motor_command(MotorCommand::LoadCalibration(calibration)).await;
//! ```

use defmt::{Format, debug, info, warn};
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

use crate::{
    system::state,
    task::port_expander::{self, PortExpanderCommand, PortNumber},
};

/// Target motor voltage (we compensate battery voltage down to this)
const TARGET_MOTOR_VOLTAGE: f32 = 6.0;

/// Get current battery voltage from system state (truly non-blocking)
/// Returns None if no voltage reading available yet OR if mutex is busy
fn try_get_battery_voltage() -> Option<f32> {
    // Use try_lock to avoid blocking - if mutex is busy, just return None
    state::SYSTEM_STATE.try_lock().ok()?.battery_voltage
}

/// Wait for first battery voltage reading from system state
async fn wait_for_battery_voltage() -> f32 {
    loop {
        if let Some(voltage) = try_get_battery_voltage() {
            return voltage;
        }
        // Small delay before checking again
        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
    }
}

/// Calculate voltage compensation factor
/// This scales duty cycle to maintain 6V output to motors as battery drains
///
/// Examples:
/// - At 8.4V battery: factor = 6.0/8.4 = 0.714 (71.4% duty cycle for 100% speed)
/// - At 7.2V battery: factor = 6.0/7.2 = 0.833 (83.3% duty cycle for 100% speed)
/// - At 6.0V battery: factor = 6.0/6.0 = 1.000 (100% duty cycle for 100% speed)
fn calculate_voltage_compensation(battery_voltage: f32) -> f32 {
    if battery_voltage < TARGET_MOTOR_VOLTAGE {
        // Battery below target - can't compensate, use full range
        warn!(
            "Battery voltage {} below target {}V",
            battery_voltage, TARGET_MOTOR_VOLTAGE
        );
        1.0
    } else {
        TARGET_MOTOR_VOLTAGE / battery_voltage
    }
}

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

        PortExpanderCommand::OutputBits {
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

        PortExpanderCommand::OutputByte {
            port: PortNumber::Port0,
            value: port0_value,
        }
    }

    /// Create port expander command to enable/disable a motor driver
    pub fn set_driver_enable_cmd(track: Track, enabled: bool) -> PortExpanderCommand {
        let bit = get_driver_enable_bit(track);

        PortExpanderCommand::OutputPin {
            port: PortNumber::Port1,
            pin: bit,
            state: enabled,
        }
    }

    /// Create port expander command to enable/disable both drivers at once
    pub fn set_all_drivers_enable_cmd(enabled: bool) -> PortExpanderCommand {
        let mask = (1 << 4) | (1 << 5); // Bits 4 and 5 (left and right drivers)
        let value = if enabled { mask } else { 0 };

        PortExpanderCommand::OutputBits {
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

    /// Load calibration data from provided calibration struct
    ///
    /// This command receives calibration data (typically from flash storage)
    /// and applies it to the motor driver. Used during system initialization
    /// or when calibration data is updated.
    LoadCalibration(MotorCalibration),

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

    /// Apply both calibration AND voltage compensation to a commanded speed
    ///
    /// This applies two corrections in sequence:
    /// 1. Motor calibration factor (compensates for motor variations)
    /// 2. Voltage compensation factor (compensates for battery voltage)
    ///
    /// The voltage compensation ensures motors always receive ~6V regardless of battery level.
    ///
    /// Returns the fully compensated speed, clamped to [-100, 100]
    pub fn apply_with_voltage_compensation(
        &self,
        track: Track,
        motor: Motor,
        speed: i8,
        voltage_compensation: f32,
    ) -> i8 {
        // First apply motor calibration
        let calibrated = self.apply(track, motor, speed);

        // Then apply voltage compensation
        let final_f32 = calibrated as f32 * voltage_compensation;
        let final_speed = if final_f32 >= 0.0 {
            (final_f32 + 0.5) as i8
        } else {
            (final_f32 - 0.5) as i8
        };

        final_speed.clamp(-100, 100)
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
        let abs_speed = speed.unsigned_abs();

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
    voltage_compensation: f32,
    track: Track,
    motor: Motor,
    speed: i8,
) {
    // Apply both calibration and voltage compensation
    let final_speed = calibration.apply_with_voltage_compensation(track, motor, speed, voltage_compensation);

    debug!(
        "Motor {:?} {:?}: speed {} -> final {} (voltage comp: {})",
        track, motor, speed, final_speed, voltage_compensation
    );

    // Set direction via port expander using motor-specific helper
    let direction = speed_to_direction(final_speed);
    let cmd = motor_port_mapping::set_motor_direction_cmd(track, motor, direction);
    port_expander::send_command(cmd).await;

    // Set PWM duty cycle
    pwm_channels.set_speed(track, motor, final_speed);
}

/// Process bulk motor command (all 4 motors at once)
async fn process_set_all_motors(
    pwm_channels: &mut PwmChannels,
    calibration: &MotorCalibration,
    voltage_compensation: f32,
    left_front: i8,
    left_rear: i8,
    right_front: i8,
    right_rear: i8,
) {
    // Apply both calibration AND voltage compensation to all motors
    let final_left_front =
        calibration.apply_with_voltage_compensation(Track::Left, Motor::Front, left_front, voltage_compensation);
    let final_left_rear =
        calibration.apply_with_voltage_compensation(Track::Left, Motor::Rear, left_rear, voltage_compensation);
    let final_right_front =
        calibration.apply_with_voltage_compensation(Track::Right, Motor::Front, right_front, voltage_compensation);
    let final_right_rear =
        calibration.apply_with_voltage_compensation(Track::Right, Motor::Rear, right_rear, voltage_compensation);

    debug!(
        "All motors: [{}, {}, {}, {}] -> final [{}, {}, {}, {}] (voltage comp: {})",
        left_front,
        left_rear,
        right_front,
        right_rear,
        final_left_front,
        final_left_rear,
        final_right_front,
        final_right_rear,
        voltage_compensation
    );

    // Set all directions via port expander (atomic bulk operation)
    let cmd = motor_port_mapping::set_all_motor_directions_cmd(
        speed_to_direction(final_left_front),
        speed_to_direction(final_left_rear),
        speed_to_direction(final_right_front),
        speed_to_direction(final_right_rear),
    );
    port_expander::send_command(cmd).await;

    // Set all PWM duty cycles
    pwm_channels.set_all_speeds(final_left_front, final_left_rear, final_right_front, final_right_rear);
}

/// Process a motor command
async fn process_command(
    pwm_channels: &mut PwmChannels,
    calibration: &mut MotorCalibration,
    voltage_compensation: f32,
    command: MotorCommand,
) {
    match command {
        // Normal operation commands (calibrated + voltage compensated)
        MotorCommand::SetTrack { track, speed } => {
            // Set both motors on the track to the same speed (with calibration + voltage compensation)
            process_set_speed(
                pwm_channels,
                calibration,
                voltage_compensation,
                track,
                Motor::Front,
                speed,
            )
            .await;
            process_set_speed(
                pwm_channels,
                calibration,
                voltage_compensation,
                track,
                Motor::Rear,
                speed,
            )
            .await;
        }

        MotorCommand::SetTracks {
            left_speed,
            right_speed,
        } => {
            // Set both tracks efficiently (with calibration + voltage compensation)
            process_set_all_motors(
                pwm_channels,
                calibration,
                voltage_compensation,
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
                voltage_compensation,
                left_front,
                left_rear,
                right_front,
                right_rear,
            )
            .await;
        }

        // Raw commands (no calibration, no voltage compensation)
        MotorCommand::SetSpeed { track, motor, speed } => {
            // Raw speed command - no calibration or voltage compensation applied
            // Used for testing and calibration procedures
            debug!("Raw motor {:?} {:?}: speed {} (no compensation)", track, motor, speed);

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

        MotorCommand::LoadCalibration(new_calibration) => {
            info!(
                "Loading calibration: [{}, {}, {}, {}]",
                new_calibration.left_front,
                new_calibration.left_rear,
                new_calibration.right_front,
                new_calibration.right_rear
            );
            *calibration = new_calibration;
        }
    }
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
pub async fn motor_driver(pwm_driver_left: Pwm<'static>, pwm_driver_right: Pwm<'static>) {
    info!("Motor driver task starting");

    // Split PWM slices into separate channel outputs for independent control
    let (left_a, left_b) = pwm_driver_left.split();
    let (right_a, right_b) = pwm_driver_right.split();

    // Initialize PWM channels
    let mut pwm_channels = PwmChannels {
        left_front: left_a.expect("Left driver channel A not configured"),
        left_rear: left_b.expect("Left driver channel B not configured"),
        right_front: right_a.expect("Right driver channel A not configured"),
        right_rear: right_b.expect("Right driver channel B not configured"),
    };

    // Initialize calibration with defaults - will be updated when flash sends data
    let mut calibration = MotorCalibration::default();
    info!("Motor driver initialized with default calibration");

    // Set all motors to coast initially
    pwm_channels.set_all_speeds(0, 0, 0, 0);
    info!("Motor driver initialized - all motors coasting");

    // Wait for first battery voltage reading before allowing motor commands
    info!("Waiting for battery voltage reading...");
    let initial_voltage = wait_for_battery_voltage().await;
    let mut voltage_compensation = calculate_voltage_compensation(initial_voltage);
    info!(
        "Battery voltage: {}V, voltage compensation factor: {}",
        initial_voltage, voltage_compensation
    );

    // Main command processing loop
    loop {
        let command = receive_motor_command().await;

        // Try to get fresh battery voltage reading non-blocking
        // If available and mutex not busy, update compensation; otherwise keep using previous value
        if let Some(current_voltage) = try_get_battery_voltage() {
            voltage_compensation = calculate_voltage_compensation(current_voltage);
        }

        process_command(&mut pwm_channels, &mut calibration, voltage_compensation, command).await;
    }
}
