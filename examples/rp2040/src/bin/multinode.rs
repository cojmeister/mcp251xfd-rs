//! Multi-node bus test with chips 0 (A), 1 (B), 2 (C):
//! 1. Broadcast: A transmits once; B and C (filters wide open) both receive.
//! 2. Selective: B accepts only 0x0B0, C only 0x0C0 (+ a broadcast ID all
//!    accept); A sends all three; each node sees exactly what its filters
//!    admit.
//! 3. Arbitration: A (ID 0x010) and B (ID 0x700) queue frames back-to-back;
//!    C must receive both intact, lower ID typically first. Repeated 10x,
//!    all 20 frames must arrive.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Delay;
use embedded_can::{Frame as _, Id, StandardId};
use mcp251xfd::{
    Fifo, FifoLayout, Filter, FilterMatch, Frame, MCP251xFdAsync, OperationMode, PayloadSize,
    ReceivedFrame,
};
use panic_probe as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B8, 8)
    .rx_fifo(Fifo::F2, PayloadSize::B8, 16);

const BROADCAST: u16 = 0x7DF;

fn sid(raw: u16) -> Id {
    Id::Standard(StandardId::new(raw).unwrap())
}

async fn drain_ids(can: &mut MCP251xFdAsync<common::Device>, got: &mut [Option<Id>]) -> usize {
    let mut n = 0;
    while n < got.len() {
        match common::recv_timeout(can, Fifo::F2).await {
            Some(rx) => {
                let id = match rx.frame {
                    ReceivedFrame::Classic(f) => f.id(),
                    ReceivedFrame::Fd(f) => f.id(),
                };
                got[n] = Some(id);
                n += 1;
            }
            None => break,
        }
    }
    n
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut devices = common::setup(p).into_iter();
    let mut a = MCP251xFdAsync::new(devices.next().unwrap());
    let mut b = MCP251xFdAsync::new(devices.next().unwrap());
    let mut c = MCP251xFdAsync::new(devices.next().unwrap());

    for (name, can) in [("A", &mut a), ("B", &mut b), ("C", &mut c)] {
        can.init(&common::CAN_CONFIG, &mut Delay).await.expect(name);
        can.apply_layout(&LAYOUT).await.unwrap();
        can.set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2)
            .await
            .unwrap();
        can.set_mode(OperationMode::Normal20, &mut Delay)
            .await
            .unwrap();
    }

    // --- 1. Broadcast: one TX, every other node receives it. ---
    a.transmit(Fifo::F1, &Frame::new(sid(0x123), &[0x42]).unwrap())
        .await
        .unwrap();
    let mut got_b = [None; 1];
    let mut got_c = [None; 1];
    let nb = drain_ids(&mut b, &mut got_b).await;
    let nc = drain_ids(&mut c, &mut got_c).await;
    if nb == 1 && nc == 1 && got_b[0] == Some(sid(0x123)) && got_c[0] == Some(sid(0x123)) {
        info!("broadcast OK: B and C both received");
    } else {
        error!("broadcast FAILED: B got {}, C got {}", nb, nc);
    }

    // --- 2. Selective delivery via filters. ---
    for (name, can, own) in [("B", &mut b, 0x0B0u16), ("C", &mut c, 0x0C0)] {
        can.set_mode(OperationMode::Configuration, &mut Delay)
            .await
            .expect(name);
        can.apply_layout(&LAYOUT).await.unwrap(); // FRESET drains stale frames
        can.set_filter(Filter::F0, FilterMatch::exact(sid(own)), Fifo::F2)
            .await
            .unwrap();
        can.set_filter(Filter::F1, FilterMatch::exact(sid(BROADCAST)), Fifo::F2)
            .await
            .unwrap();
        can.set_mode(OperationMode::Normal20, &mut Delay)
            .await
            .unwrap();
    }
    for id in [0x0B0, 0x0C0, BROADCAST] {
        a.transmit(Fifo::F1, &Frame::new(sid(id), &[id as u8]).unwrap())
            .await
            .unwrap();
    }
    let mut got_b = [None; 3];
    let mut got_c = [None; 3];
    let nb = drain_ids(&mut b, &mut got_b).await;
    let nc = drain_ids(&mut c, &mut got_c).await;
    let b_ok = nb == 2
        && got_b[..2].contains(&Some(sid(0x0B0)))
        && got_b[..2].contains(&Some(sid(BROADCAST)));
    let c_ok = nc == 2
        && got_c[..2].contains(&Some(sid(0x0C0)))
        && got_c[..2].contains(&Some(sid(BROADCAST)));
    if b_ok && c_ok {
        info!("selective delivery OK: each node saw its ID + broadcast only");
    } else {
        error!(
            "selective delivery FAILED: B {} frames, C {} frames",
            nb, nc
        );
    }

    // --- 3. Arbitration: A (high prio 0x010) vs B (low prio 0x700). ---
    // Re-open C wide before the arbitration rounds; step 2 narrowed it to
    // 0x0C0/0x7DF. FRESET also drains the stale selective-test frames.
    c.set_mode(OperationMode::Configuration, &mut Delay)
        .await
        .unwrap();
    c.apply_layout(&LAYOUT).await.unwrap();
    c.set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2)
        .await
        .unwrap();
    c.disable_filter(Filter::F1).await.unwrap();
    c.set_mode(OperationMode::Normal20, &mut Delay)
        .await
        .unwrap();

    // A's RX FIFO F2 (still accept_all via filter F0) is untouched here: it
    // quietly accumulates B's ten 0x700 frames over these rounds, leaving 6
    // of its 16 slots free. Harmless for this test, which never reads A's
    // FIFO.
    let mut received = 0usize;
    let mut high_first = 0usize;
    for round in 0..10u8 {
        b.transmit(Fifo::F1, &Frame::new(sid(0x700), &[round]).unwrap())
            .await
            .unwrap();
        a.transmit(Fifo::F1, &Frame::new(sid(0x010), &[round]).unwrap())
            .await
            .unwrap();
        let mut got = [None; 2];
        let n = drain_ids(&mut c, &mut got).await;
        received += n;
        if n == 2 && got[0] == Some(sid(0x010)) {
            high_first += 1;
        }
    }
    if received == 20 {
        info!(
            "arbitration OK: all 20 frames arrived; high-priority first in {}/10 rounds",
            high_first
        );
    } else {
        error!("arbitration FAILED: only {}/20 frames arrived", received);
    }
    info!("multinode complete");
}
