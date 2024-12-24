//! Motor control implementation with encoder feedback
//!
//! Controls the robot's movement through a TB6612FNG dual motor driver with
//! DFRobot FIT0450 DC motors that include quadrature encoders:
//! - Manages motor speeds (0-100%) and directions for both left and right motors
//! - Uses encoder feedback to synchronize motor speeds
//! - Handles complex movement commands like turning while moving
//! - Provides power management through standby mode
//!
//! # Motor Control Logic
//! - Forward/Backward: Both motors run at equal speeds
//! - Turning: Differential speed between left/right motors
//! - Speed changes are incremental and clamped to valid ranges
//! - Conflicting motions (e.g. forward while moving backward) trigger a stop first
//! - Turn handling:
//!   * Pure rotation: Equal speeds in opposite directions
//!   * Turn while moving: Maintain average speed, adjust ratio
//!   * Turn recovery: Detect opposing turn and restore motion
//!   * Speed ratios maintained using encoder feedback
//!
//! # Encoder Feedback
//! - Uses PWM input to count encoder pulses
//! - Measures speed every 100ms
//! - Left motor used as reference
//! - Right motor speed adjusted to match left motor RPM
//! - Speed adjustment calculation:
//!   * Compute RPM ratio = right_rpm / left_rpm
//!   * Target ratio is 1.0 for straight motion
//!   * Adjustment = 1.0 + ((1.0 - ratio) * 0.5)
//!   * 50% correction factor for smooth adjustments
//!   * Example: if right is 10% slow (ratio=0.9):
//!     - Adjustment = 1.0 + ((1.0 - 0.9) * 0.5) = 1.05
//!     - Right speed increased by 5%
//!
//! # Power Management
//! - Standby mode disables motor driver for power saving
//! - Wake-up sequence ensures clean transitions from standby
//! - Brake command actively stops motors vs Coast which lets them spin down

use crate::system::resources::{MotorDriverResources, MotorEncoderResources};
use defmt::info;
use embassy_rp::{
    gpio::{self, Pull},
    pwm::{self, Config, InputMode, Pwm},
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
use tb6612fng::{DriveCommand, Motor, Tb6612fng};

/// Drive control signal for sending commands to the motor task
static DRIVE_CONTROL: Signal<CriticalSectionRawMutex, Command> = Signal::new();

/// Motor control commands with speed parameters in range 0-100
#[derive(Debug, Clone)]
pub enum Command {
    /// Turn left motor only, speed 0-100
    Left(u8),
    /// Turn right motor only, speed 0-100     
    Right(u8),
    /// Drive both motors forward, speed 0-100
    Forward(u8),
    /// Drive both motors backward, speed 0-100
    Backward(u8),
    /// Actively brake motors (high resistance stop)
    Brake,
    /// Let motors coast to stop (low resistance)
    Coast,
    /// Enter power saving mode
    Standby,
}

/// Sends a drive command to the motor control task
pub fn send_command(command: Command) {
    DRIVE_CONTROL.signal(command);
}

/// Waits for next motor command
async fn wait_command() -> Command {
    DRIVE_CONTROL.wait().await
}

/// Checks if robot is executing a pure rotation (turning in place).
/// This occurs when motors run at equal speeds in opposite directions,
/// causing the robot to rotate around its center axis.
///
/// Turn types:
/// 1. Pure rotation (this function):
///    - Left motor: -50 (backward), Right motor: 50 (forward) = true
///    - Left motor: 30 (forward), Right motor: -30 (backward) = true
///
/// 2. Turn while moving (not this function):
///    - Left motor: 40 (forward), Right motor: 30 (forward) = false
///    - Maintains average speed while adjusting ratio
///
/// 3. No motion:
///    - Left motor: 0, Right motor: 0 = false
fn is_turning_in_place(left_speed: i8, right_speed: i8) -> bool {
    left_speed == -right_speed && left_speed != 0
}

// DFRobot FIT0450 motor specifications
// Each motor has a Hall effect quadrature encoder that outputs 8 pulses per revolution
const PULSES_PER_REV: u32 = 8; // Encoder pulses per motor revolution
const GEAR_RATIO: u32 = 120; // 120:1 gear reduction

// At 100ms measurement interval and 8 pulses/rev:
// 1 pulse = 75 RPM (motor) = 0.625 RPM (wheel)
// Max measurable speed at 65535 pulses/100ms = 491,512 RPM (motor) = 4,096 RPM (wheel)

/// Main motor control task that processes movement commands
#[embassy_executor::task]
pub async fn drive(d: MotorDriverResources, e: MotorEncoderResources) {
    // Configure PWM input for encoder pulse counting
    let config = Config::default();
    let left_encoder = Pwm::new_input(
        e.left_encoder_slice,
        e.left_encoder_pin,
        Pull::None,
        InputMode::RisingEdge,
        config.clone(),
    );
    let right_encoder = Pwm::new_input(
        e.right_encoder_slice,
        e.right_encoder_pin,
        Pull::None,
        InputMode::RisingEdge,
        config,
    );

    // Track motor directions
    let mut left_forward = true;
    let mut right_forward = true;

    // PWM config for motor control (10kHz)
    let desired_freq_hz = 10_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    let mut pwm_config = Config::default();
    pwm_config.divider = divider.into();
    pwm_config.top = period;

    // Initialize motor driver pins
    let stby = gpio::Output::new(d.standby_pin, gpio::Level::Low);

    // Left motor
    let left_fwd = gpio::Output::new(d.left_forward_pin, gpio::Level::Low);
    let left_bckw = gpio::Output::new(d.left_backward_pin, gpio::Level::Low);
    let left_pwm = pwm::Pwm::new_output_a(d.left_slice, d.left_pwm_pin, pwm_config.clone());
    let left_motor = Motor::new(left_fwd, left_bckw, left_pwm).unwrap();

    // Right motor
    let right_fwd = gpio::Output::new(d.right_forward_pin, gpio::Level::Low);
    let right_bckw = gpio::Output::new(d.right_backward_pin, gpio::Level::Low);
    let right_pwm = pwm::Pwm::new_output_b(d.right_slice, d.right_pwm_pin, pwm_config.clone());
    let right_motor = Motor::new(right_fwd, right_bckw, right_pwm).unwrap();

    // Motor driver
    let mut control = Tb6612fng::new(left_motor, right_motor, stby).unwrap();

    // Create ticker for speed measurement (100ms interval)
    let mut ticker = Ticker::every(Duration::from_millis(100));

    loop {
        // Wait for next 100ms interval
        ticker.next().await;

        // Update speed measurements
        let left_pulses = left_encoder.counter();
        left_encoder.set_counter(0);
        let right_pulses = right_encoder.counter();
        right_encoder.set_counter(0);

        // Convert pulses to RPM
        let left_hz = left_pulses as f32 * 10.0;
        let right_hz = right_pulses as f32 * 10.0;
        let left_motor_rpm = (left_hz / PULSES_PER_REV as f32) * 60.0;
        let right_motor_rpm = (right_hz / PULSES_PER_REV as f32) * 60.0;
        let left_wheel_rpm = left_motor_rpm / GEAR_RATIO as f32;
        let right_wheel_rpm = right_motor_rpm / GEAR_RATIO as f32;

        // Apply direction
        let left_rpm = if left_forward {
            left_wheel_rpm
        } else {
            -left_wheel_rpm
        };
        let right_rpm = if right_forward {
            right_wheel_rpm
        } else {
            -right_wheel_rpm
        };

        // Log encoder data
        info!(
            "Left Encoder: pulses={} freq={}Hz motor_rpm={} wheel_rpm={}",
            left_pulses, left_hz, left_motor_rpm, left_wheel_rpm
        );
        info!(
            "Right Encoder: pulses={} freq={}Hz motor_rpm={} wheel_rpm={}",
            right_pulses, right_hz, right_motor_rpm, right_wheel_rpm
        );
        info!(
            "Speed measurements - Left: {} RPM, Right: {} RPM",
            left_rpm, right_rpm
        );

        // Process any pending commands
        let drive_command = wait_command().await;
        // Get current state
        let is_standby = control.current_standby().unwrap();
        let left_speed_cmd = control.motor_a.current_speed();
        let right_speed_cmd = control.motor_b.current_speed();

        // Wake from standby if movement requested
        if is_standby {
            match drive_command {
                Command::Forward(_)
                | Command::Backward(_)
                | Command::Left(_)
                | Command::Right(_) => {
                    control.disable_standby().unwrap();
                    Timer::after(Duration::from_millis(100)).await;
                }
                _ => {}
            }
        }

        match drive_command {
            Command::Forward(speed) => {
                // Stop first if currently moving backward
                if left_speed_cmd < 0 || right_speed_cmd < 0 {
                    info!("in conflicting motion, stopping");
                    control.motor_a.drive(DriveCommand::Stop).unwrap();
                    control.motor_b.drive(DriveCommand::Stop).unwrap();
                } else {
                    // Set direction for RPM calculation
                    left_forward = true;
                    right_forward = true;

                    // Use left motor as reference
                    let new_speed = (left_speed_cmd + speed as i8).clamp(0, 100) as u8;
                    info!(
                        "drive forward {} (L:{} R:{} RPM)",
                        new_speed, left_rpm, right_rpm
                    );

                    // Set left motor (reference)
                    control
                        .motor_a
                        .drive(DriveCommand::Forward(new_speed))
                        .unwrap();

                    // Calculate speed adjustment based on RPM difference
                    let rpm_ratio = if left_rpm != 0.0 {
                        right_rpm / left_rpm
                    } else {
                        1.0
                    };
                    let adjustment = 1.0 + ((1.0 - rpm_ratio) * 0.5); // 50% correction factor

                    let adjusted_speed = (new_speed as f32 * adjustment).clamp(0.0, 100.0) as u8;

                    control
                        .motor_b
                        .drive(DriveCommand::Forward(adjusted_speed))
                        .unwrap();
                }
            }
            Command::Backward(speed) => {
                // Stop first if currently moving forward
                if left_speed_cmd > 0 || right_speed_cmd > 0 {
                    info!("in conflicting motion, stopping");
                    control.motor_a.drive(DriveCommand::Stop).unwrap();
                    control.motor_b.drive(DriveCommand::Stop).unwrap();
                } else {
                    // Set direction for RPM calculation
                    left_forward = false;
                    right_forward = false;

                    // Use left motor as reference
                    let new_speed = (-left_speed_cmd + speed as i8).clamp(0, 100) as u8;
                    info!(
                        "drive backward {} (L:{} R:{} RPM)",
                        new_speed, left_rpm, right_rpm
                    );

                    // Set left motor (reference)
                    control
                        .motor_a
                        .drive(DriveCommand::Backward(new_speed))
                        .unwrap();

                    // Calculate speed adjustment based on RPM difference
                    let rpm_ratio = if left_rpm != 0.0 {
                        right_rpm / left_rpm
                    } else {
                        1.0
                    };
                    let adjustment = 1.0 + ((1.0 - rpm_ratio) * 0.5); // 50% correction factor

                    let adjusted_speed = (new_speed as f32 * adjustment).clamp(0.0, 100.0) as u8;

                    control
                        .motor_b
                        .drive(DriveCommand::Backward(adjusted_speed))
                        .unwrap();
                }
            }
            Command::Left(speed) => {
                if left_speed_cmd > right_speed_cmd {
                    // We're in a right turn, need to counter it
                    if is_turning_in_place(left_speed_cmd, right_speed_cmd) {
                        // If turning in place, stop completely
                        info!("stopping turn-in-place maneuver");
                        control.motor_a.drive(DriveCommand::Stop).unwrap();
                        control.motor_b.drive(DriveCommand::Stop).unwrap();
                    } else {
                        // If turning while moving, restore original motion speed with encoder feedback
                        let original_speed = (left_speed_cmd + right_speed_cmd) / 2;
                        info!(
                            "restoring original motion speed {} (L:{} R:{} RPM)",
                            original_speed, left_rpm, right_rpm
                        );

                        if original_speed > 0 {
                            // Forward motion
                            left_forward = true;
                            right_forward = true;

                            control
                                .motor_a
                                .drive(DriveCommand::Forward(original_speed as u8))
                                .unwrap();

                            let rpm_ratio = if left_rpm != 0.0 {
                                right_rpm / left_rpm
                            } else {
                                1.0
                            };
                            let adjustment = 1.0 + ((1.0 - rpm_ratio) * 0.5);
                            let adjusted_speed =
                                (original_speed as f32 * adjustment).clamp(0.0, 100.0) as u8;

                            control
                                .motor_b
                                .drive(DriveCommand::Forward(adjusted_speed))
                                .unwrap();
                        } else if original_speed < 0 {
                            // Backward motion
                            left_forward = false;
                            right_forward = false;

                            control
                                .motor_a
                                .drive(DriveCommand::Backward(-original_speed as u8))
                                .unwrap();

                            let rpm_ratio = if left_rpm != 0.0 {
                                right_rpm / left_rpm
                            } else {
                                1.0
                            };
                            let adjustment = 1.0 + ((1.0 - rpm_ratio) * 0.5);
                            let adjusted_speed =
                                (-original_speed as f32 * adjustment).clamp(0.0, 100.0) as u8;

                            control
                                .motor_b
                                .drive(DriveCommand::Backward(adjusted_speed))
                                .unwrap();
                        }
                    }
                } else {
                    // Initiate or continue left turn with encoder feedback
                    let new_left_speed = (left_speed_cmd - speed as i8).clamp(-100, 100);
                    let target_right_speed = (right_speed_cmd + speed as i8).clamp(-100, 100);

                    info!(
                        "turn left L:{} R:{} (L:{} R:{} RPM)",
                        new_left_speed, target_right_speed, left_rpm, right_rpm
                    );

                    // Set direction for RPM calculation
                    left_forward = new_left_speed > 0;
                    right_forward = target_right_speed > 0;

                    // Set left motor (reference)
                    if new_left_speed > 0 {
                        control
                            .motor_a
                            .drive(DriveCommand::Forward(new_left_speed as u8))
                            .unwrap();
                    } else if new_left_speed < 0 {
                        control
                            .motor_a
                            .drive(DriveCommand::Backward(-new_left_speed as u8))
                            .unwrap();
                    } else {
                        control.motor_a.drive(DriveCommand::Stop).unwrap();
                    }

                    // Calculate desired RPM ratio for turn
                    let target_rpm_ratio = if new_left_speed != 0 {
                        target_right_speed as f32 / new_left_speed as f32
                    } else {
                        1.0
                    };

                    // Adjust right motor speed to maintain desired RPM ratio
                    let current_rpm_ratio = if left_rpm != 0.0 {
                        right_rpm / left_rpm
                    } else {
                        1.0
                    };

                    let speed_adjustment = (target_rpm_ratio - current_rpm_ratio) * 0.5;
                    let adjusted_speed = ((target_right_speed.abs() as f32)
                        * (1.0 + speed_adjustment))
                        .clamp(0.0, 100.0) as u8;

                    // Set right motor with adjusted speed
                    if target_right_speed > 0 {
                        control
                            .motor_b
                            .drive(DriveCommand::Forward(adjusted_speed))
                            .unwrap();
                    } else if target_right_speed < 0 {
                        control
                            .motor_b
                            .drive(DriveCommand::Backward(adjusted_speed))
                            .unwrap();
                    } else {
                        control.motor_b.drive(DriveCommand::Stop).unwrap();
                    }
                }
            }
            Command::Right(speed) => {
                if right_speed_cmd > left_speed_cmd {
                    // We're in a left turn, need to counter it
                    if is_turning_in_place(left_speed_cmd, right_speed_cmd) {
                        // If turning in place, stop completely
                        info!("stopping turn-in-place maneuver");
                        control.motor_a.drive(DriveCommand::Stop).unwrap();
                        control.motor_b.drive(DriveCommand::Stop).unwrap();
                    } else {
                        // If turning while moving, restore original motion speed with encoder feedback
                        let original_speed = (left_speed_cmd + right_speed_cmd) / 2;
                        info!(
                            "restoring original motion speed {} (L:{} R:{} RPM)",
                            original_speed, left_rpm, right_rpm
                        );

                        if original_speed > 0 {
                            // Forward motion
                            left_forward = true;
                            right_forward = true;

                            control
                                .motor_a
                                .drive(DriveCommand::Forward(original_speed as u8))
                                .unwrap();

                            let rpm_ratio = if left_rpm != 0.0 {
                                right_rpm / left_rpm
                            } else {
                                1.0
                            };
                            let adjustment = 1.0 + ((1.0 - rpm_ratio) * 0.5);
                            let adjusted_speed =
                                (original_speed as f32 * adjustment).clamp(0.0, 100.0) as u8;

                            control
                                .motor_b
                                .drive(DriveCommand::Forward(adjusted_speed))
                                .unwrap();
                        } else if original_speed < 0 {
                            // Backward motion
                            left_forward = false;
                            right_forward = false;

                            control
                                .motor_a
                                .drive(DriveCommand::Backward(-original_speed as u8))
                                .unwrap();

                            let rpm_ratio = if left_rpm != 0.0 {
                                right_rpm / left_rpm
                            } else {
                                1.0
                            };
                            let adjustment = 1.0 + ((1.0 - rpm_ratio) * 0.5);
                            let adjusted_speed =
                                (-original_speed as f32 * adjustment).clamp(0.0, 100.0) as u8;

                            control
                                .motor_b
                                .drive(DriveCommand::Backward(adjusted_speed))
                                .unwrap();
                        }
                    }
                } else {
                    // Initiate or continue right turn with encoder feedback
                    let new_left_speed = (left_speed_cmd + speed as i8).clamp(-100, 100);
                    let target_right_speed = (right_speed_cmd - speed as i8).clamp(-100, 100);

                    info!(
                        "turn right L:{} R:{} (L:{} R:{} RPM)",
                        new_left_speed, target_right_speed, left_rpm, right_rpm
                    );

                    // Set direction for RPM calculation
                    left_forward = new_left_speed > 0;
                    right_forward = target_right_speed > 0;

                    // Set left motor (reference)
                    if new_left_speed > 0 {
                        control
                            .motor_a
                            .drive(DriveCommand::Forward(new_left_speed as u8))
                            .unwrap();
                    } else if new_left_speed < 0 {
                        control
                            .motor_a
                            .drive(DriveCommand::Backward(-new_left_speed as u8))
                            .unwrap();
                    } else {
                        control.motor_a.drive(DriveCommand::Stop).unwrap();
                    }

                    // Calculate desired RPM ratio for turn
                    let target_rpm_ratio = if new_left_speed != 0 {
                        target_right_speed as f32 / new_left_speed as f32
                    } else {
                        1.0
                    };

                    // Adjust right motor speed to maintain desired RPM ratio
                    let current_rpm_ratio = if left_rpm != 0.0 {
                        right_rpm / left_rpm
                    } else {
                        1.0
                    };

                    let speed_adjustment = (target_rpm_ratio - current_rpm_ratio) * 0.5;
                    let adjusted_speed = ((target_right_speed.abs() as f32)
                        * (1.0 + speed_adjustment))
                        .clamp(0.0, 100.0) as u8;

                    // Set right motor with adjusted speed
                    if target_right_speed > 0 {
                        control
                            .motor_b
                            .drive(DriveCommand::Forward(adjusted_speed))
                            .unwrap();
                    } else if target_right_speed < 0 {
                        control
                            .motor_b
                            .drive(DriveCommand::Backward(adjusted_speed))
                            .unwrap();
                    } else {
                        control.motor_b.drive(DriveCommand::Stop).unwrap();
                    }
                }
            }
            Command::Coast => {
                info!("coast");
                control.motor_a.drive(DriveCommand::Stop).unwrap();
                control.motor_b.drive(DriveCommand::Stop).unwrap();
            }
            Command::Brake => {
                info!("brake");
                control.motor_a.drive(DriveCommand::Brake).unwrap();
                control.motor_b.drive(DriveCommand::Brake).unwrap();
            }
            Command::Standby => {
                if !is_standby {
                    control.motor_a.drive(DriveCommand::Brake).unwrap();
                    control.motor_b.drive(DriveCommand::Brake).unwrap();
                    Timer::after(Duration::from_millis(100)).await;
                    control.motor_a.drive(DriveCommand::Stop).unwrap();
                    control.motor_b.drive(DriveCommand::Stop).unwrap();
                    Timer::after(Duration::from_millis(100)).await;
                    control.enable_standby().unwrap();
                }
            }
        }
    }
}
