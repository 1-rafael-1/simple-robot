#![no_std]
#![no_main]

use crate::task::battery_indicator::battery_indicator;
use crate::task::distance::distance_measurement;
use crate::task::orchestrator::orchestrator;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use task::resources::*;
use {defmt_rtt as _, panic_probe as _};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

mod task;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Program start");

    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);

    spawner.spawn(orchestrator()).unwrap();
    spawner
        .spawn(distance_measurement(r.distance_sensor))
        .unwrap();
    spawner
        .spawn(battery_indicator(r.battery_indicator))
        .unwrap();
}
