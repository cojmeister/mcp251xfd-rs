//! Internal-loopback smoke test: every chip transmits to itself, classic
//! and FD. Verifies the entire driver stack per chip in isolation, without
//! touching the CAN pins.
//!
//! Output leaves over USB CDC-ACM serial: open the board's COM port in any
//! terminal. The sweep repeats every 5 s.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use embedded_can::{Frame as _, StandardId};
use log::{error, info};
use mcp251xfd::{
    FdFrame, Fifo, FifoLayout, Filter, FilterMatch, Frame, FrameFlags, MCP251xFdAsync,
    OperationMode, PayloadSize, ReceivedFrame,
};
use panic_halt as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B64, 8);

/// Classic + FD-64 loopback for one chip. Returns `Err` on any SPI/driver
/// fault so the caller can move on to the next chip; a wrong or missing frame
/// is logged in place and is not an error, because the remaining sub-test
/// still carries information.
async fn test_chip(
    can: &mut MCP251xFdAsync<common::Device>,
    i: usize,
) -> Result<(), common::CanError> {
    can.apply_layout(&LAYOUT).await?;
    can.set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2)
        .await?;
    can.set_mode(OperationMode::InternalLoopback, &mut Delay)
        .await?;

    // Classic frame. The `unwrap`s below are on literal constants that are
    // valid by construction -- 0x123 is a legal 11-bit ID, 4 bytes a legal
    // classic payload -- so they cannot fire at runtime.
    let tx = Frame::new(StandardId::new(0x123).unwrap(), &[1, 2, 3, 4]).unwrap();
    can.transmit(Fifo::F1, &tx).await?;
    match common::recv_timeout(can, Fifo::F2).await {
        Some(rx) => match rx.frame {
            ReceivedFrame::Classic(f) if f.data() == [1, 2, 3, 4] => {
                info!("chip {i}: classic loopback OK")
            }
            _ => error!("chip {i}: classic loopback WRONG FRAME"),
        },
        None => error!("chip {i}: classic loopback TIMEOUT"),
    }

    // FD frame with bit-rate switch.
    let mut payload = [0u8; 64];
    for (n, b) in payload.iter_mut().enumerate() {
        *b = n as u8;
    }
    let tx = FdFrame::new(
        StandardId::new(0x456).unwrap(),
        &payload,
        FrameFlags {
            brs: true,
            esi: false,
        },
    )
    .unwrap();
    can.transmit_fd(Fifo::F1, &tx).await?;
    match common::recv_timeout(can, Fifo::F2).await {
        Some(rx) => match rx.frame {
            ReceivedFrame::Fd(f) if f.data() == payload => info!("chip {i}: FD-64 loopback OK"),
            _ => error!("chip {i}: FD loopback WRONG FRAME"),
        },
        None => error!("chip {i}: FD loopback TIMEOUT"),
    }
    Ok(())
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut chips = devices.map(MCP251xFdAsync::new);
    loop {
        info!("--- loopback: 10 chips, classic + FD-64 ---");
        for (i, can) in chips.iter_mut().enumerate() {
            common::ensure_configuration(can).await;
            if let Err(e) = can.init(&common::CAN_CONFIG, &mut Delay).await {
                error!("chip {i}: init FAILED: {e:?}");
                continue;
            }
            if let Err(e) = test_chip(can, i).await {
                error!("chip {i}: aborted: {e:?}");
            }
        }
        info!("loopback test complete");
        Timer::after_secs(5).await;
    }
}
