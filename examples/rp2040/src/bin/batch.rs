//! Times `transmit` in a loop against `transmit_batch`, ten chips, three
//! frames each, `CYCLES` times back to back with no pacing between
//! iterations -- a throughput measurement, not a reproduction of a paced
//! cycle that motivated the API.
//!
//! Both should come out the same: after the paired status/user-address read,
//! the readiness check shares a transaction with the user-address fetch, so
//! there is nothing further for a batch to fold. Three chip-select
//! transactions per frame is the floor without the driver mirroring the
//! chip's RAM allocator.
//!
//! What `transmit_batch` actually buys is the accepted-count contract. The
//! partial-fill probe below is the part worth reading: it fills the FIFO
//! deliberately and checks the returned count is the accepted prefix.
//!
//! If the two timings differ by more than noise, that is worth investigating
//! -- it would mean the transaction accounting in the driver docs is wrong.
//!
//! Runs on internal loopback: SPI wiring only.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Instant, Timer};
use embedded_can::{Frame as _, StandardId};
use log::{error, info};
use mcp251xfd::{
    Fifo, FifoLayout, Filter, FilterMatch, Frame, MCP251xFdAsync, OperationMode, PayloadSize,
};
use panic_halt as _;

const TX: Fifo = Fifo::F1;
const RX: Fifo = Fifo::F2;

/// Depth 16: three frames per cycle cannot fill
/// a FIFO that drains within the cycle.
const LAYOUT: FifoLayout =
    FifoLayout::new()
        .tx_fifo(TX, PayloadSize::B8, 16)
        .rx_fifo(RX, PayloadSize::B8, 8);

/// A short TX FIFO used only by the partial-fill probe.
const SHALLOW: FifoLayout =
    FifoLayout::new()
        .tx_fifo(TX, PayloadSize::B8, 2)
        .rx_fifo(RX, PayloadSize::B8, 8);

const MODE: OperationMode = OperationMode::InternalLoopback;
const CYCLES: u32 = 500;

type Can = MCP251xFdAsync<common::Device>;

async fn setup(can: &mut Can, layout: &FifoLayout) -> Result<(), common::CanError> {
    can.set_mode(OperationMode::Configuration, &mut Delay)
        .await?;
    can.apply_layout(layout).await?;
    can.set_filter(Filter::F0, FilterMatch::accept_all(), RX)
        .await?;
    can.set_mode(MODE, &mut Delay).await?;
    Ok(())
}

/// Fills a two-deep FIFO with a four-frame batch and checks the count.
async fn partial_fill_probe(can: &mut Can, frames: &[Frame; 4]) {
    if let Err(e) = setup(can, &SHALLOW).await {
        error!("partial-fill setup: {e:?}");
        return;
    }
    match can.transmit_batch(TX, frames).await {
        Ok(n) if (n as usize) < frames.len() => info!(
            "partial fill: {n} of {} accepted, remainder correctly refused",
            frames.len()
        ),
        Ok(n) => info!("partial fill: all {n} accepted (the FIFO drained mid-batch)"),
        Err(e) => error!("partial fill: {e:?}"),
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut chips: [Can; 10] = devices.map(MCP251xFdAsync::new);
    for (i, can) in chips.iter_mut().enumerate() {
        common::ensure_configuration(can).await;
        if let Err(e) = can.init(&common::CAN_CONFIG, &mut Delay).await {
            error!("chip {i}: init: {e:?}");
        }
        if let Err(e) = setup(can, &LAYOUT).await {
            error!("chip {i}: setup: {e:?}");
        }
    }

    let three = [
        Frame::new(StandardId::new(0x101).unwrap(), &[1; 8]).unwrap(),
        Frame::new(StandardId::new(0x102).unwrap(), &[2; 8]).unwrap(),
        Frame::new(StandardId::new(0x103).unwrap(), &[3; 8]).unwrap(),
    ];
    let four = [
        Frame::new(StandardId::new(0x201).unwrap(), &[1; 8]).unwrap(),
        Frame::new(StandardId::new(0x202).unwrap(), &[2; 8]).unwrap(),
        Frame::new(StandardId::new(0x203).unwrap(), &[3; 8]).unwrap(),
        Frame::new(StandardId::new(0x204).unwrap(), &[4; 8]).unwrap(),
    ];

    loop {
        // Individual transmits.
        let t0 = Instant::now();
        for _ in 0..CYCLES {
            for can in chips.iter_mut() {
                for f in &three {
                    let _ = can.transmit(TX, f).await;
                }
                while can.receive(RX).await.is_ok() {}
            }
        }
        let individual = t0.elapsed().as_micros();

        // Batched.
        let t1 = Instant::now();
        for _ in 0..CYCLES {
            for can in chips.iter_mut() {
                let _ = can.transmit_batch(TX, &three).await;
                while can.receive(RX).await.is_ok() {}
            }
        }
        let batched = t1.elapsed().as_micros();

        info!(
            "{CYCLES} cycles x 10 chips x 3 frames: transmit {individual} us ({} us/cycle), transmit_batch {batched} us ({} us/cycle)",
            individual / CYCLES as u64,
            batched / CYCLES as u64,
        );

        partial_fill_probe(&mut chips[0], &four).await;
        if let Err(e) = setup(&mut chips[0], &LAYOUT).await {
            error!("restore chip 0: {e:?}");
        }

        Timer::after_secs(5).await;
    }
}
