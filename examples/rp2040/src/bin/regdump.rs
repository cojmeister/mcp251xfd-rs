//! Dumps every configuration register the driver writes, for all ten chips.
//!
//! The status registers were always readable (`fifo_status`,
//! `interrupt_flags`, `error_counters`); the *configuration* registers were
//! not, so there was no way to check whether a chip agreed with what `init`
//! believed it had written. This dumps both, and diffs `NBTCFG` (the nominal
//! bit-timing register) against the value `CAN_CONFIG` implies -- `DBTCFG`
//! is dumped alongside it but not diffed.
//!
//! Needs SPI wiring only -- nothing here touches the CAN bus.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use log::{error, info};
use mcp251xfd::registers::addr;
use mcp251xfd::{Fifo, FifoLayout, MCP251xFdAsync, PayloadSize};
use panic_halt as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B64, 8);

type Can = MCP251xFdAsync<common::Device>;

async fn dump(index: usize, can: &mut Can) -> Result<(), common::CanError> {
    let variant = can.init(&common::CAN_CONFIG, &mut Delay).await?;
    can.apply_layout(&LAYOUT).await?;

    let cfg = can.read_back_config().await?;
    info!(
        "chip {index}: {variant:?} CiCON={:#010X} mode={:?} isocrc={} rtxat={}",
        cfg.con.0,
        cfg.con.op_mode(),
        cfg.con.iso_crc_enabled(),
        cfg.con.restrict_retx(),
    );
    info!(
        "chip {index}: NBTCFG={:#010X} DBTCFG={:#010X} TDC={:#010X}",
        cfg.nominal.0, cfg.data.0, cfg.tdc.0
    );

    // What init should have written, derived from the same config the driver
    // was handed -- so a mismatch means the chip disagrees, not that these
    // literals drifted.
    let want_nbt = common::CAN_CONFIG.nominal.to_reg().0;
    if cfg.nominal.0 != want_nbt {
        error!(
            "chip {index}: NBTCFG mismatch: chip has {:#010X}, config implies {:#010X}",
            cfg.nominal.0, want_nbt
        );
    }

    for fifo in [Fifo::F1, Fifo::F2] {
        let con = can.fifo_config(fifo).await?;
        let sta = can.fifo_status(fifo).await?;
        // This chip is still in Configuration mode (nothing above requests
        // another one), so per `fifo_user_address`'s own docs this value is
        // not meaningful -- it's dumped anyway for a complete register
        // picture, not because it's expected to mean anything here.
        let ua = can.fifo_user_address(fifo).await?;
        info!(
            "chip {index} {fifo:?}: CON={:#010X} tx={} txreq={} | STA={:#010X} ready={} empty_or_full={} | UA={:#06X}",
            con.0,
            con.tx(),
            con.txreq(),
            sta.0,
            sta.not_full_or_not_empty(),
            sta.tx_empty_or_rx_full(),
            ua,
        );
    }

    // The raw escape hatch, on a register the typed API does not cover.
    let iocon = can.read_register_raw(addr::IOCON).await?;
    info!("chip {index}: IOCON={iocon:#010X}");

    Ok(())
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut chips: [Can; 10] = devices.map(MCP251xFdAsync::new);
    loop {
        info!("--- register dump ---");
        for (i, can) in chips.iter_mut().enumerate() {
            common::ensure_configuration(can).await;
            if let Err(e) = dump(i, can).await {
                error!("chip {i}: {e:?}");
            }
        }
        Timer::after_secs(5).await;
    }
}
