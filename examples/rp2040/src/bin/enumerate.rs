//! Initializes all 10 chips on the shared bus and reports each one's
//! detected variant — the first thing to run on new hardware.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Delay;
use mcp251xfd::MCP251xFdAsync;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let devices = common::setup(p);
    let mut ok = 0;
    for (i, dev) in devices.into_iter().enumerate() {
        let mut can = MCP251xFdAsync::new(dev);
        match can.init(&common::CAN_CONFIG, &mut Delay).await {
            Ok(variant) => {
                info!("chip {}: init OK, variant {}", i, variant);
                ok += 1;
            }
            Err(_) => error!("chip {}: init FAILED (wiring/CS/SPI clock?)", i),
        }
    }
    info!("{}/10 chips initialized", ok);
}
