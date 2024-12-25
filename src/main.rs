//! Robot firmware entry point
//!
//! Initializes system and spawns control tasks.

#![no_std]
#![no_main]

use crate::task::{
    autonomous_drive::autonomous_drive,
    battery_charge_read::battery_charge_read,
    drive::drive,
    encoder::encoder,
    ir_obstacle_detect::ir_obstacle_detect,
    orchestrate::orchestrate,
    rc_control::{rc_button_a_handle, rc_button_b_handle, rc_button_c_handle, rc_button_d_handle},
    rgb_led_indicate::rgb_led_indicate,
    track_inactivity::track_inactivity,
};
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use embassy_rp::config::Config;
use system::resources::{
    self, AssignedResources, BatteryChargeResources, DistanceSensorResources, IRSensorResources,
    MotorDriverResources, MotorEncoderResources, RCResourcesA, RCResourcesB, RCResourcesC,
    RCResourcesD, RGBLedResources,
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

    // Split the resources into separate groups for each task, for all the resources that we do not share between tasks.
    let r = split_resources!(p);

    // Finally spawn all the tasks
    spawner.spawn(orchestrate()).unwrap();
    spawner.spawn(ir_obstacle_detect(r.ir_sensor)).unwrap();
    spawner
        .spawn(battery_charge_read(r.battery_charge))
        .unwrap();
    spawner.spawn(rgb_led_indicate(r.rgb_led)).unwrap();
    spawner.spawn(rc_button_a_handle(r.rc_a)).unwrap();
    spawner.spawn(rc_button_b_handle(r.rc_b)).unwrap();
    spawner.spawn(rc_button_c_handle(r.rc_c)).unwrap();
    spawner.spawn(rc_button_d_handle(r.rc_d)).unwrap();
    // Spawn encoder task first to ensure it's ready to provide measurements
    spawner.spawn(encoder(r.motor_encoders)).unwrap();
    // Then spawn drive task which will consume encoder measurements
    spawner.spawn(drive(r.motor_driver)).unwrap();
    spawner.spawn(autonomous_drive()).unwrap();
    spawner.spawn(track_inactivity()).unwrap();
}
