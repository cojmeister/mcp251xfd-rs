//! Multi-node bus test with chips 0 (A), 1 (B), 2 (C):
//! 1. Broadcast: A transmits once; B and C (filters wide open) both receive.
//! 2. Selective: B accepts only 0x0B0, C only 0x0C0 (+ a broadcast ID all
//!    accept); A sends all three; each node sees exactly what its filters
//!    admit.
//! 3. Back-to-back: A (ID 0x010) and B (ID 0x700) queue frames one after the
//!    other; C must receive both intact. Repeated 10x, all 20 must arrive.
//!
//! Needs transceivers and a terminated bus. Output leaves over USB CDC-ACM
//! serial: open the board's COM port in any terminal. The test repeats every
//! 5 s.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use embedded_can::{Frame as _, Id, StandardId};
use log::{error, info};
use mcp251xfd::{
    Fifo, FifoLayout, Filter, FilterMatch, Frame, MCP251xFdAsync, OperationMode, PayloadSize,
    ReceivedFrame,
};
use panic_halt as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B8, 8)
    .rx_fifo(Fifo::F2, PayloadSize::B8, 16);

const BROADCAST: u16 = 0x7DF;

type Can = MCP251xFdAsync<common::Device>;

fn sid(raw: u16) -> Id {
    Id::Standard(StandardId::new(raw).unwrap())
}

async fn drain_ids(can: &mut Can, got: &mut [Option<Id>]) -> usize {
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

/// Brings one chip up to `Normal20` with a wide-open RX filter.
async fn bring_up(can: &mut Can) -> Result<(), common::CanError> {
    common::ensure_configuration(can).await;
    can.init(&common::CAN_CONFIG, &mut Delay).await?;
    can.apply_layout(&LAYOUT).await?;
    can.set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2)
        .await?;
    can.set_mode(OperationMode::Normal20, &mut Delay).await?;
    Ok(())
}

async fn test_broadcast(a: &mut Can, b: &mut Can, c: &mut Can) -> Result<(), common::CanError> {
    a.transmit(Fifo::F1, &Frame::new(sid(0x123), &[0x42]).unwrap())
        .await?;
    let mut got_b = [None; 1];
    let mut got_c = [None; 1];
    let nb = drain_ids(b, &mut got_b).await;
    let nc = drain_ids(c, &mut got_c).await;
    if nb == 1 && nc == 1 && got_b[0] == Some(sid(0x123)) && got_c[0] == Some(sid(0x123)) {
        info!("broadcast OK: B and C both received");
    } else {
        error!("broadcast FAILED: B got {nb}, C got {nc}");
        match a.error_counters().await {
            Ok(trec) => error!("  bus health on A: {trec:?}"),
            Err(e) => error!("  bus health on A unreadable: {e:?}"),
        }
    }
    Ok(())
}

async fn test_selective(b: &mut Can, c: &mut Can) -> Result<(), common::CanError> {
    for (can, own) in [(b, 0x0B0u16), (c, 0x0C0)] {
        can.set_mode(OperationMode::Configuration, &mut Delay)
            .await?;
        can.apply_layout(&LAYOUT).await?; // FRESET drains stale frames
        can.set_filter(Filter::F0, FilterMatch::exact(sid(own)), Fifo::F2)
            .await?;
        can.set_filter(Filter::F1, FilterMatch::exact(sid(BROADCAST)), Fifo::F2)
            .await?;
        can.set_mode(OperationMode::Normal20, &mut Delay).await?;
    }
    Ok(())
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut devices = devices.into_iter();
    let mut a = MCP251xFdAsync::new(devices.next().unwrap());
    let mut b = MCP251xFdAsync::new(devices.next().unwrap());
    let mut c = MCP251xFdAsync::new(devices.next().unwrap());

    loop {
        info!("--- multinode: chips 0/1/2 on the real bus ---");
        if let Err(e) = run_all(&mut a, &mut b, &mut c).await {
            error!("multinode aborted: {e:?}");
        }
        Timer::after_secs(5).await;
    }
}

async fn run_all(a: &mut Can, b: &mut Can, c: &mut Can) -> Result<(), common::CanError> {
    for (name, can) in [("A", &mut *a), ("B", &mut *b), ("C", &mut *c)] {
        if let Err(e) = bring_up(can).await {
            error!("chip {name}: bring-up FAILED: {e:?}");
            return Ok(());
        }
    }

    // --- 1. Broadcast: one TX, every other node receives it. ---
    test_broadcast(a, b, c).await?;

    // --- 2. Selective delivery via filters. ---
    test_selective(b, c).await?;
    for id in [0x0B0, 0x0C0, BROADCAST] {
        a.transmit(Fifo::F1, &Frame::new(sid(id), &[id as u8]).unwrap())
            .await?;
    }
    let mut got_b = [None; 3];
    let mut got_c = [None; 3];
    let nb = drain_ids(b, &mut got_b).await;
    let nc = drain_ids(c, &mut got_c).await;
    let b_ok = nb == 2
        && got_b[..2].contains(&Some(sid(0x0B0)))
        && got_b[..2].contains(&Some(sid(BROADCAST)));
    let c_ok = nc == 2
        && got_c[..2].contains(&Some(sid(0x0C0)))
        && got_c[..2].contains(&Some(sid(BROADCAST)));
    if b_ok && c_ok {
        info!("selective delivery OK: each node saw its ID + broadcast only");
    } else {
        error!("selective delivery FAILED: B {nb} frames, C {nc} frames");
    }

    // --- 3. Back-to-back TX from two nodes. ---
    // Re-open C wide; step 2 narrowed it to 0x0C0/0x7DF. FRESET also drains
    // the stale selective-test frames.
    c.set_mode(OperationMode::Configuration, &mut Delay).await?;
    c.apply_layout(&LAYOUT).await?;
    c.set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2)
        .await?;
    c.disable_filter(Filter::F1).await?;
    c.set_mode(OperationMode::Normal20, &mut Delay).await?;

    // A's RX FIFO F2 (still accept_all via filter F0) is untouched here: it
    // quietly accumulates B's ten 0x700 frames over these rounds, leaving 6
    // of its 16 slots free. Harmless for this test, which never reads A's
    // FIFO.
    //
    // Note these rounds are *not* contended: on an idle bus B starts
    // transmitting the moment TXREQ is set, and A's four SPI transactions
    // take far less than one 500 kbit/s frame time, so the two frames
    // serialize. `low_id_first` therefore measures ordering, not arbitration.
    let mut received = 0usize;
    let mut low_id_first = 0usize;
    for round in 0..10u8 {
        b.transmit(Fifo::F1, &Frame::new(sid(0x700), &[round]).unwrap())
            .await?;
        a.transmit(Fifo::F1, &Frame::new(sid(0x010), &[round]).unwrap())
            .await?;
        let mut got = [None; 2];
        let n = drain_ids(c, &mut got).await;
        received += n;
        if n == 2 && got[0] == Some(sid(0x010)) {
            low_id_first += 1;
        }
    }
    if received == 20 {
        info!("back-to-back OK: all 20 frames arrived; low ID first in {low_id_first}/10 rounds");
    } else {
        error!("back-to-back FAILED: only {received}/20 frames arrived");
    }
    info!("multinode complete");
    Ok(())
}
