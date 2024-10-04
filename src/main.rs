#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use {defmt_rtt as _, panic_probe as _};

mod task;
use task::resources::*;
use task::scheduler::scheduler;
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Program start");

    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);

    // Start the scheduler
    spawner.spawn(scheduler(spawner)).unwrap();
}
