//! Main Entry Point for the Simple Robot Control System
//!
//! This module serves as the entry point for the robot's firmware.
//! It initializes the system, spawns the main tasks, and sets up
//! the necessary hardware resources.

#![no_std]
#![no_main]

use crate::task::battery_charge_reader::battery_charge_reader;
use crate::task::distance_measure::distance_measure;
use crate::task::orchestrator::orchestrator;
use crate::task::rgb_led_indicator::rgb_led_indicator;
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use task::resources::{
    AssignedResources, BatteryChargeResources, DistanceSensorResources, RCResourcesA, RCResourcesB,
    RCResourcesC, RCResourcesD, RGBLedResources, StatusLedResources,
};
use {defmt_rtt as _, panic_probe as _};

/// Defines the image type for the firmware
///
/// This is used by the bootloader to determine how to load and execute the firmware.
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

/// Module containing all task-related code
mod task;

/// Main function and entry point of the firmware
///
/// This function is called by the runtime after basic chip initialization.
/// It's responsible for setting up the system and spawning the main tasks.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);

    spawner.spawn(orchestrator()).unwrap();
    spawner.spawn(distance_measure(r.distance_sensor)).unwrap();
    spawner
        .spawn(battery_charge_reader(r.battery_charge))
        .unwrap();
    spawner.spawn(rgb_led_indicator(r.rgb_led)).unwrap();
}
