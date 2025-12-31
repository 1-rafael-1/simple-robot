//! Robot firmware entry point
//!
//! Initializes system and spawns control tasks.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    adc::{Adc, Channel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler},
    bind_interrupts,
    block::ImageDef,
    config::Config,
    gpio::{Input, Level, Output, Pull},
    i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler},
    peripherals::{I2C0, PIO0},
    pio::InterruptHandler as PioInterruptHandler,
    pwm::{InputMode, Pwm},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use panic_probe as _;
use static_cell::StaticCell;

use crate::{
    system::event::ButtonId,
    task::{
        autonomous_drive::autonomous_drive,
        battery_charge_read::battery_charge_read,
        display::display,
        drive::drive,
        encoder_read::{self, encoder_read},
        imu_read::inertial_measurement_read,
        ir_obstacle_detect::ir_obstacle_detect,
        monitor_motion::motion_correction_control,
        orchestrate::orchestrate,
        rc_control::rc_button_handle,
        rgb_led_indicate::rgb_led_indicate,
        sweep_ultrasonic::{self, ultrasonic_sweep},
        track_inactivity::track_inactivity,
    },
};

/// Firmware image type for bootloader
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

/// System core modules
mod system;
/// Task implementations
mod task;

// Bind interrupts on global scope for convenience
bind_interrupts!(pub struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
});

/// Public type for shared I2C bus
pub type I2cBusShared = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, embassy_rp::i2c::Async>>;

/// Firmware entry point
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Config::default());

    // Orchestrator task
    spawner.must_spawn(orchestrate());

    // Battery monitoring
    let adc = Adc::new(p.ADC, Irqs, AdcConfig::default());
    let vsys_channel = Channel::new_pin(p.PIN_29, Pull::None);
    spawner.must_spawn(battery_charge_read(adc, vsys_channel));

    // RGB LED indicator
    let pwm_red_config = embassy_rp::pwm::Config::default();
    let pwm_red = Pwm::new_output_a(p.PWM_SLICE1, p.PIN_2, pwm_red_config.clone());
    let pwm_green = Pwm::new_output_a(p.PWM_SLICE2, p.PIN_4, pwm_red_config.clone());
    spawner.must_spawn(rgb_led_indicate(pwm_red, pwm_green));

    // RC button A
    let btn_a = Input::new(p.PIN_10, Pull::Down);
    spawner.must_spawn(rc_button_handle(btn_a, ButtonId::A));

    // RC button B
    let btn_b = Input::new(p.PIN_16, Pull::Down);
    spawner.must_spawn(rc_button_handle(btn_b, ButtonId::B));

    // RC button C
    let btn_c = Input::new(p.PIN_11, Pull::Down);
    spawner.must_spawn(rc_button_handle(btn_c, ButtonId::C));

    // RC button D
    let btn_d = Input::new(p.PIN_17, Pull::Down);
    spawner.must_spawn(rc_button_handle(btn_d, ButtonId::D));

    // Motor driver
    let motor_standby = Output::new(p.PIN_22, Level::Low);

    // Configure PWM for motor control at 10kHz
    let desired_freq_hz = 10_000u32;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;
    let mut motor_pwm_config = embassy_rp::pwm::Config::default();
    motor_pwm_config.divider = divider.into();
    motor_pwm_config.top = period;

    // Create configured PWM outputs for motors
    let left_pwm = embassy_rp::pwm::Pwm::new_output_a(p.PWM_SLICE6, p.PIN_28, motor_pwm_config.clone());
    let right_pwm = embassy_rp::pwm::Pwm::new_output_b(p.PWM_SLICE5, p.PIN_27, motor_pwm_config);

    // Create direction control GPIO pins
    let left_forward = Output::new(p.PIN_21, Level::Low);
    let left_backward = Output::new(p.PIN_20, Level::Low);
    let right_forward = Output::new(p.PIN_19, Level::Low);
    let right_backward = Output::new(p.PIN_18, Level::Low);

    spawner.must_spawn(drive(
        motor_standby,
        left_pwm,
        left_forward,
        left_backward,
        right_pwm,
        right_forward,
        right_backward,
    ));

    // Autonomous drive task
    spawner.must_spawn(autonomous_drive());

    // IR obstacle detection
    let ir_right = Input::new(p.PIN_26, Pull::Down);
    spawner.must_spawn(ir_obstacle_detect(ir_right));

    // Ultrasonic sensor sweep
    let us_trigger = Output::new(p.PIN_15, Level::Low);
    let us_echo = Input::new(p.PIN_14, Pull::None);
    let sensor = sweep_ultrasonic::setup_ultrasonic_sensor(us_trigger, us_echo);
    let servo = sweep_ultrasonic::setup_servo(p.PIO0, p.PIN_5);
    spawner.must_spawn(ultrasonic_sweep(sensor, servo));

    // I2C bus (shared between display and IMU)
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, i2c_config);
    static I2C_BUS: StaticCell<I2cBusShared> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    // Display
    spawner.must_spawn(display(i2c_bus));

    // Motor encoders
    let encoder_pwm_config = encoder_read::configure_encoder_pwm();
    let left_encoder = Pwm::new_input(
        p.PWM_SLICE3,
        p.PIN_7,
        Pull::None,
        InputMode::RisingEdge,
        encoder_pwm_config.clone(),
    );
    let right_encoder = Pwm::new_input(
        p.PWM_SLICE4,
        p.PIN_9,
        Pull::None,
        InputMode::RisingEdge,
        encoder_pwm_config,
    );
    spawner.must_spawn(encoder_read(left_encoder, right_encoder));

    // IMU (Inertial Measurement Unit)
    let imu_int = p.PIN_8;
    let imu_add = p.PIN_3;
    spawner.must_spawn(inertial_measurement_read(i2c_bus, imu_int, imu_add));

    // Motion correction control
    spawner.must_spawn(motion_correction_control());

    // Inactivity tracker
    spawner.must_spawn(track_inactivity());
}
