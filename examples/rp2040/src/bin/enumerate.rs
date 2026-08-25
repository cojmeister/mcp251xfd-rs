//! Initializes all 10 chips on the shared bus and reports each one's
//! detected variant -- the first thing to run on new hardware.
//!
//! Output leaves over USB CDC-ACM serial: open the board's COM port in any
//! terminal. The sweep repeats every 5 s, so the report is still coming when
//! you connect.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use log::{error, info};
use mcp251xfd::MCP251xFdAsync;
use panic_halt as _;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut chips = devices.map(MCP251xFdAsync::new);
    loop {
        info!("--- enumerate: probing 10 chips ---");
        let mut ok = 0;
        for (i, can) in chips.iter_mut().enumerate() {
            common::ensure_configuration(can).await;
            match can.init(&common::CAN_CONFIG, &mut Delay).await {
                Ok(variant) => {
                    info!("chip {i}: init OK, variant {variant:?}");
                    ok += 1;
                }
                // The discriminant is the whole diagnostic: `ClockNotReady`
                // means the crystal is not what `CAN_CONFIG` claims,
                // `CommunicationCheckFailed` means CS wiring or an over-spec
                // SPI clock, `Spi(_)` means the RP2040 peripheral itself.
                Err(e) => error!("chip {i}: init FAILED: {e:?}"),
            }
        }
        info!("{ok}/10 chips initialized");
        Timer::after_secs(5).await;
    }
}
