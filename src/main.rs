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
    gpio::{Level, Output, Pull},
    i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler},
    peripherals::{I2C0, PIO0},
    pio::InterruptHandler as PioInterruptHandler,
    pwm::Pwm,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use panic_probe as _;
use static_cell::StaticCell;

use crate::{
    system::event::ButtonId,
    task::{
        autonomous_drive::autonomous_drive, battery_charge_read::battery_charge_read, display::display, drive::drive,
        encoder_read::encoder_read, imu_read::inertial_measurement_read, ir_obstacle_detect::ir_obstacle_detect,
        monitor_motion::motion_correction_control, orchestrate::orchestrate, rc_control::rc_button_handle,
        rgb_led_indicate::rgb_led_indicate, sweep_ultrasonic::ultrasonic_sweep, track_inactivity::track_inactivity,
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
    let btn_a = p.PIN_10;
    spawner.must_spawn(rc_button_handle(btn_a.into(), ButtonId::A));

    // RC button B
    let btn_b = p.PIN_16;
    spawner.must_spawn(rc_button_handle(btn_b.into(), ButtonId::B));

    // RC button C
    let btn_c = p.PIN_11;
    spawner.must_spawn(rc_button_handle(btn_c.into(), ButtonId::C));

    // RC button D
    let btn_d = p.PIN_17;
    spawner.must_spawn(rc_button_handle(btn_d.into(), ButtonId::D));

    // Motor driver
    let motor_standby = Output::new(p.PIN_22, Level::Low);
    let left_pwm_slice = p.PWM_SLICE6;
    let left_pwm_pin = p.PIN_28;
    let left_forward = Output::new(p.PIN_21, Level::Low);
    let left_backward = Output::new(p.PIN_20, Level::Low);
    let right_pwm_slice = p.PWM_SLICE5;
    let right_pwm_pin = p.PIN_27;
    let right_forward = Output::new(p.PIN_19, Level::Low);
    let right_backward = Output::new(p.PIN_18, Level::Low);
    spawner.must_spawn(drive(
        motor_standby,
        left_pwm_slice,
        left_pwm_pin,
        left_forward,
        left_backward,
        right_pwm_slice,
        right_pwm_pin,
        right_forward,
        right_backward,
    ));

    // Autonomous drive task
    spawner.must_spawn(autonomous_drive());

    // IR obstacle detection
    let ir_left = p.PIN_6;
    let ir_right = p.PIN_26;
    spawner.must_spawn(ir_obstacle_detect(ir_left, ir_right));

    // Ultrasonic sensor sweep
    let servo_pio = p.PIO0;
    let servo_pin = p.PIN_5;
    let us_trigger = p.PIN_15;
    let us_echo = p.PIN_14;
    spawner.must_spawn(ultrasonic_sweep(servo_pio, servo_pin, us_trigger, us_echo));

    // I2C bus (shared between display and IMU)
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, i2c_config);
    static I2C_BUS: StaticCell<I2cBusShared> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    // Display
    spawner.must_spawn(display(i2c_bus));

    // Motor encoders
    let left_encoder_slice = p.PWM_SLICE3;
    let left_encoder_pin = p.PIN_7;
    let right_encoder_slice = p.PWM_SLICE4;
    let right_encoder_pin = p.PIN_9;
    spawner.must_spawn(encoder_read(
        left_encoder_slice,
        left_encoder_pin,
        right_encoder_slice,
        right_encoder_pin,
    ));

    // IMU (Inertial Measurement Unit)
    let imu_int = p.PIN_8;
    let imu_add = p.PIN_3;
    spawner.must_spawn(inertial_measurement_read(i2c_bus, imu_int, imu_add));

    // Motion correction control
    spawner.must_spawn(motion_correction_control());

    // Inactivity tracker
    spawner.must_spawn(track_inactivity());
}
