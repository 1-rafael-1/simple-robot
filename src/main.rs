#![no_std]
#![no_main]

use crate::task::battery_charge_reader::battery_charge_reader;
use crate::task::distance_measure::distance_measure;
use crate::task::orchestrator::orchestrator;
use crate::task::rgb_led_indicator::rgb_led_indicator;
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
    spawner.spawn(distance_measure(r.distance_sensor)).unwrap();
    spawner
        .spawn(battery_charge_reader(r.battery_charge))
        .unwrap();
    spawner.spawn(rgb_led_indicator(r.rgb_led)).unwrap();
}
