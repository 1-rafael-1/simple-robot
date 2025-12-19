//! Minimal test firmware for module testing
//!
//! Initializes RP2350 with Embassy executor and runs a simple main loop.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{block::ImageDef, config::Config};
use embassy_time::Timer;
use panic_probe as _;

/// Firmware image type for bootloader
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

/// Firmware entry point
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_rp::init(Config::default());

    info!("RP2350 initialized - test-modules running");

    let mut counter = 0;
    loop {
        info!("Main loop iteration: {}", counter);
        counter += 1;
        Timer::after_secs(1).await;
    }
}

