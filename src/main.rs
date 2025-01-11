//! Robot firmware entry point
//!
//! Initializes system and spawns control tasks.

#![no_std]
#![no_main]

use crate::{
    system::event::ButtonId,
    task::{
        autonomous_drive::autonomous_drive, battery_charge_read::battery_charge_read,
        display::display, drive::drive, encoder_read::read_encoder,
        imu_read::inertial_measurement_handle, ir_obstacle_detect::ir_obstacle_detect,
        orchestrate::orchestrate, rc_control::rc_button_handle, rgb_led_indicate::rgb_led_indicate,
        sweep_ultrasonic::ultrasonic_sweep, track_inactivity::track_inactivity,
    },
};
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use embassy_rp::config::Config;
use system::resources::{
    self, AssignedResources, BatteryChargeResources, IRSensorResources,
    InertialMeasurementUnitResources, MotorDriverResources, MotorEncoderResources, RCResources,
    RGBLedResources, SweepServoResources, UltrasonicDistanceSensorResources,
};
use {defmt_rtt as _, panic_probe as _};

/// Firmware image type for bootloader
#[link_section = ".start_block"]
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

    // Initialize the global ADC instance before spawning any tasks.
    // This initialization must happen here to ensure:
    // 1. The ADC is ready before any tasks that need it (e.g., battery monitoring)
    // 2. We only initialize once, as multiple initializations could corrupt the hardware state
    // 3. No race conditions can occur since this happens before any tasks are spawned
    resources::init_adc(p.ADC);

    // Initialize the I2C bus before spawning any tasks
    resources::init_i2c(p.I2C0, p.PIN_13, p.PIN_12);

    // Split the resources into separate groups for each task, for all the resources that we do not share between tasks.
    let r = split_resources!(p);

    spawner.spawn(orchestrate()).unwrap();
    spawner
        .spawn(battery_charge_read(r.battery_charge))
        .unwrap();
    spawner.spawn(rgb_led_indicate(r.rgb_led)).unwrap();
    spawner
        .spawn(rc_button_handle(r.rc.btn_a.into(), ButtonId::A))
        .unwrap();
    spawner
        .spawn(rc_button_handle(r.rc.btn_b.into(), ButtonId::B))
        .unwrap();
    spawner
        .spawn(rc_button_handle(r.rc.btn_c.into(), ButtonId::C))
        .unwrap();
    spawner
        .spawn(rc_button_handle(r.rc.btn_d.into(), ButtonId::D))
        .unwrap();
    spawner.spawn(read_encoder(r.motor_encoders)).unwrap();
    spawner.spawn(drive(r.motor_driver)).unwrap();
    spawner.spawn(autonomous_drive()).unwrap();
    spawner.spawn(track_inactivity()).unwrap();
    spawner
        .spawn(ultrasonic_sweep(r.sweep_servo, r.us_distance_sensor))
        .unwrap();
    spawner.spawn(display()).unwrap();
    spawner.spawn(inertial_measurement_handle()).unwrap();
    spawner.spawn(ir_obstacle_detect(r.ir_sensor)).unwrap();
}
