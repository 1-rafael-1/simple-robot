//! Robot firmware entry point
//!
//! Initializes system and spawns control tasks.

#![no_std]
#![no_main]

use crate::task::autonomous_drive::autonomous_drive;
use crate::task::battery_charge_read::battery_charge_read;
use crate::task::distance_measure::distance_measure;
use crate::task::drive::drive;
use crate::task::orchestrate::orchestrate;
use crate::task::rc_control::{
    rc_button_a_handle, rc_button_b_handle, rc_button_c_handle, rc_button_d_handle,
};
use crate::task::rgb_led_indicate::rgb_led_indicate;
use crate::task::track_inactivity::track_inactivity;
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use embassy_rp::config::Config;
use system::resources::{
    AssignedResources, BatteryChargeResources, DistanceSensorResources, MotorResources,
    RCResourcesA, RCResourcesB, RCResourcesC, RCResourcesD, RGBLedResources,
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
    let r = split_resources!(p);

    spawner.spawn(orchestrate()).unwrap();
    spawner.spawn(distance_measure(r.distance_sensor)).unwrap();
    spawner
        .spawn(battery_charge_read(r.battery_charge))
        .unwrap();
    spawner.spawn(rgb_led_indicate(r.rgb_led)).unwrap();
    spawner.spawn(rc_button_a_handle(r.rc_a)).unwrap();
    spawner.spawn(rc_button_b_handle(r.rc_b)).unwrap();
    spawner.spawn(rc_button_c_handle(r.rc_c)).unwrap();
    spawner.spawn(rc_button_d_handle(r.rc_d)).unwrap();
    spawner.spawn(drive(r.motor)).unwrap();
    spawner.spawn(autonomous_drive()).unwrap();
    spawner.spawn(track_inactivity()).unwrap();
}
