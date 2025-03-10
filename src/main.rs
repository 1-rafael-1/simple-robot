//! Robot firmware entry point
//!
//! Initializes system and spawns control tasks.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    block::ImageDef,
    config::Config,
    i2c::{Config as I2cConfig, I2c},
};
use embassy_sync::mutex::Mutex;
use panic_probe as _;
use static_cell::StaticCell;
use system::resources::{
    AdcResources, AssignedResources, BatteryChargeResources, I2c0BusResources, I2c0BusShared, IRSensorResources,
    InertialMeasurementUnitResources, Irqs, MotorDriverResources, MotorEncoderResources, RCResources, RGBLedResources,
    SweepServoResources, UltrasonicDistanceSensorResources, init_adc,
};

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

/// Firmware entry point
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Config::default());

    // Split the resources into separate groups for each task, for all the resources that we do not share between tasks.
    let r = split_resources!(p);

    // Initialize the ADC instance before spawning any tasks
    init_adc(r.adc.adc);

    // Initialize the I2C bus before spawning any tasks
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(r.i2c.i2c0, r.i2c.scl, r.i2c.sda, Irqs, i2c_config);
    static I2C_BUS: StaticCell<I2c0BusShared> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    spawner.spawn(orchestrate()).unwrap();
    spawner.spawn(battery_charge_read(r.battery_charge)).unwrap();
    spawner.spawn(rgb_led_indicate(r.rgb_led)).unwrap();
    spawner.spawn(rc_button_handle(r.rc.btn_a.into(), ButtonId::A)).unwrap();
    spawner.spawn(rc_button_handle(r.rc.btn_b.into(), ButtonId::B)).unwrap();
    spawner.spawn(rc_button_handle(r.rc.btn_c.into(), ButtonId::C)).unwrap();
    spawner.spawn(rc_button_handle(r.rc.btn_d.into(), ButtonId::D)).unwrap();
    spawner.spawn(drive(r.motor_driver)).unwrap();
    spawner.spawn(autonomous_drive()).unwrap();
    spawner.spawn(ir_obstacle_detect(r.ir_sensor)).unwrap();
    spawner
        .spawn(ultrasonic_sweep(r.sweep_servo, r.us_distance_sensor))
        .unwrap();
    spawner.spawn(display(i2c_bus)).unwrap();
    spawner.spawn(encoder_read(r.motor_encoders)).unwrap();
    spawner.spawn(inertial_measurement_read(i2c_bus)).unwrap();
    spawner.spawn(motion_correction_control()).unwrap();
    spawner.spawn(track_inactivity()).unwrap();
}
