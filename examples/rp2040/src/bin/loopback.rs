//! Internal-loopback smoke test: every chip transmits to itself, classic
//! and FD. Verifies the entire driver stack per chip in isolation.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use embedded_can::{Frame as _, StandardId};
use mcp251xfd::{
    FdFrame, Fifo, FifoLayout, Filter, FilterMatch, Frame, FrameFlags, MCP251xFdAsync,
    OperationMode, PayloadSize, ReceivedFrame,
};
use panic_probe as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B64, 8);

/// Polls an RX FIFO for up to ~100 ms.
async fn recv_timeout(
    can: &mut MCP251xFdAsync<common::Device>,
    fifo: Fifo,
) -> Option<mcp251xfd::RxFrame> {
    for _ in 0..100 {
        match can.receive(fifo).await {
            Ok(rx) => return Some(rx),
            Err(_) => Timer::after_millis(1).await,
        }
    }
    None
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let devices = common::setup(p);
    for (i, dev) in devices.into_iter().enumerate() {
        let mut can = MCP251xFdAsync::new(dev);
        if can.init(&common::CAN_CONFIG, &mut Delay).await.is_err() {
            error!("chip {}: init failed", i);
            continue;
        }
        can.apply_layout(&LAYOUT).await.unwrap();
        can.set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2)
            .await
            .unwrap();
        can.set_mode(OperationMode::InternalLoopback, &mut Delay)
            .await
            .unwrap();

        // Classic frame.
        let tx = Frame::new(StandardId::new(0x123).unwrap(), &[1, 2, 3, 4]).unwrap();
        can.transmit(Fifo::F1, &tx).await.unwrap();
        match recv_timeout(&mut can, Fifo::F2).await {
            Some(rx) => match rx.frame {
                ReceivedFrame::Classic(f) if f.data() == [1, 2, 3, 4] => {
                    info!("chip {}: classic loopback OK", i)
                }
                _ => error!("chip {}: classic loopback WRONG FRAME", i),
            },
            None => error!("chip {}: classic loopback TIMEOUT", i),
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
        can.transmit_fd(Fifo::F1, &tx).await.unwrap();
        match recv_timeout(&mut can, Fifo::F2).await {
            Some(rx) => match rx.frame {
                ReceivedFrame::Fd(f) if f.data() == payload => {
                    info!("chip {}: FD-64 loopback OK", i)
                }
                _ => error!("chip {}: FD loopback WRONG FRAME", i),
            },
            None => error!("chip {}: FD loopback TIMEOUT", i),
        }
    }
    info!("loopback test complete");
}
