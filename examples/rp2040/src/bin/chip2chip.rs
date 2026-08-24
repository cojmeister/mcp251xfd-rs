//! Two chips on the shared CAN bus: chip 0 transmits, chip 1 receives.
//! Classic at the nominal rate, then FD with bit-rate switch.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Delay;
use embedded_can::{Frame as _, StandardId};
use mcp251xfd::{
    FdFrame, Fifo, FifoLayout, Filter, FilterMatch, Frame, FrameFlags, MCP251xFdAsync,
    OperationMode, PayloadSize, ReceivedFrame,
};
use panic_probe as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B64, 8);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut devices = common::setup(p).into_iter();
    let dev_a = devices.next().unwrap();
    let dev_b = devices.next().unwrap();

    let mut a = MCP251xFdAsync::new(dev_a);
    let mut b = MCP251xFdAsync::new(dev_b);
    for (name, can) in [("A", &mut a), ("B", &mut b)] {
        can.init(&common::CAN_CONFIG, &mut Delay).await.expect(name);
        can.apply_layout(&LAYOUT).await.unwrap();
        can.set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2)
            .await
            .unwrap();
        can.set_mode(OperationMode::NormalFd, &mut Delay)
            .await
            .unwrap();
    }

    // Classic 500 kbit/s.
    let tx = Frame::new(StandardId::new(0x100).unwrap(), &[0xDE, 0xAD]).unwrap();
    a.transmit(Fifo::F1, &tx).await.unwrap();
    match common::recv_timeout(&mut b, Fifo::F2).await {
        Some(rx) => match rx.frame {
            ReceivedFrame::Classic(f) if f.data() == [0xDE, 0xAD] => info!("classic A->B OK"),
            _ => error!("classic A->B wrong frame"),
        },
        None => error!("classic A->B TIMEOUT (transceivers? termination?)"),
    }

    // FD 500k/2M with BRS.
    let payload: [u8; 48] = core::array::from_fn(|i| i as u8);
    let tx = FdFrame::new(
        StandardId::new(0x200).unwrap(),
        &payload,
        FrameFlags {
            brs: true,
            esi: false,
        },
    )
    .unwrap();
    a.transmit_fd(Fifo::F1, &tx).await.unwrap();
    match common::recv_timeout(&mut b, Fifo::F2).await {
        Some(rx) => match rx.frame {
            ReceivedFrame::Fd(f) if f.data() == payload => info!("FD-48 BRS A->B OK"),
            _ => error!("FD A->B wrong frame"),
        },
        None => error!("FD A->B TIMEOUT"),
    }
    info!("chip2chip complete");
}
