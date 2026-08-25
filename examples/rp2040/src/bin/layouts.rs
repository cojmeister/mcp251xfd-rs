//! Verifies **multi-FIFO layouts**: several RX FIFOs of different payload
//! sizes, each fed by its own acceptance filter.
//!
//! Needs SPI wiring only -- everything runs through internal loopback.
//!
//! Compile-time RAM budgeting is the crate's headline feature, yet every other
//! test here uses exactly one TX and one RX FIFO. The risk this covers is the
//! chip's own RAM address generation: `CiFIFOUA` for FIFO *n* depends on the
//! sizes of all preceding FIFOs, so a layout with mixed `PLSIZE` values is
//! where an addressing error shows up -- as a frame read from the wrong offset,
//! or one FIFO's objects overwriting another's.
//!
//! Filters route a distinct identifier to each RX FIFO, so "the frame arrived
//! in the FIFO we expected, with the payload we sent, and the other FIFOs
//! stayed empty" checks routing and addressing together.
//!
//! Layouts here are contiguous from `F1`: [`FifoLayout`]'s own docs note that
//! gapped layouts are not validated against the chip's address generation, so
//! they are out of scope.
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

/// Four RX FIFOs with ascending payload sizes behind one TX FIFO.
///
/// 2*(8+64) + 4*(8+8) + 4*(8+16) + 4*(8+32) + 2*(8+64) = 608 bytes of the
/// 2048-byte window.
const MIXED: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 2)
    .rx_fifo(Fifo::F2, PayloadSize::B8, 4)
    .rx_fifo(Fifo::F3, PayloadSize::B16, 4)
    .rx_fifo(Fifo::F4, PayloadSize::B32, 4)
    .rx_fifo(Fifo::F5, PayloadSize::B64, 2);

/// A wider layout: one TX plus six RX FIFOs, to push the address generation
/// further down the chain.
///
/// 2*(8+64) + 6 FIFOs * 6 deep * (8+12) = 144 + 720 = 864 bytes.
const WIDE: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 2)
    .rx_fifo(Fifo::F2, PayloadSize::B12, 6)
    .rx_fifo(Fifo::F3, PayloadSize::B12, 6)
    .rx_fifo(Fifo::F4, PayloadSize::B12, 6)
    .rx_fifo(Fifo::F5, PayloadSize::B12, 6)
    .rx_fifo(Fifo::F6, PayloadSize::B12, 6)
    .rx_fifo(Fifo::F7, PayloadSize::B12, 6);

// Budget arithmetic pinned at compile time -- if `bytes()` or the layout ever
// drifts, this fails the build rather than the board.
const _: () = assert!(MIXED.total_bytes() == 608);
const _: () = assert!(WIDE.total_bytes() == 864);

/// `(fifo, filter slot, identifier, payload length)` per RX FIFO. Each length
/// is exactly that FIFO's `PLSIZE`, which is the largest frame it may legally
/// carry.
const MIXED_ROUTES: [(Fifo, Filter, u16, usize); 4] = [
    (Fifo::F2, Filter::F0, 0x201, 8),
    (Fifo::F3, Filter::F1, 0x202, 16),
    (Fifo::F4, Filter::F2, 0x203, 32),
    (Fifo::F5, Filter::F3, 0x204, 64),
];

const WIDE_ROUTES: [(Fifo, Filter, u16, usize); 6] = [
    (Fifo::F2, Filter::F0, 0x301, 12),
    (Fifo::F3, Filter::F1, 0x302, 12),
    (Fifo::F4, Filter::F2, 0x303, 12),
    (Fifo::F5, Filter::F3, 0x304, 12),
    (Fifo::F6, Filter::F4, 0x305, 12),
    (Fifo::F7, Filter::F5, 0x306, 12),
];

type Can = MCP251xFdAsync<common::Device>;

fn payload_for(id: u16, len: usize) -> [u8; 64] {
    let mut p = [0u8; 64];
    for (i, b) in p.iter_mut().enumerate().take(len) {
        // Seeded by the identifier so a frame landing in the wrong FIFO is
        // detected by its contents, not only by its address.
        *b = (id as u8).wrapping_add(i as u8);
    }
    p
}

/// Applies `layout` and arms one exact filter per route.
async fn configure(
    can: &mut Can,
    layout: &FifoLayout,
    routes: &[(Fifo, Filter, u16, usize)],
) -> bool {
    if can
        .set_mode(OperationMode::Configuration, &mut Delay)
        .await
        .is_err()
    {
        return false;
    }
    if let Err(e) = can.apply_layout(layout).await {
        error!("    apply_layout failed: {e:?}");
        return false;
    }
    for &(fifo, filter, id, _) in routes {
        let m = FilterMatch::exact(StandardId::new(id).unwrap().into());
        if let Err(e) = can.set_filter(filter, m, fifo).await {
            error!("    set_filter failed: {e:?}");
            return false;
        }
    }
    can.set_mode(OperationMode::InternalLoopback, &mut Delay)
        .await
        .is_ok()
}

/// Reads `fifo` once, returning the payload length and first byte if a frame
/// was waiting.
async fn try_recv(can: &mut Can, fifo: Fifo) -> Option<(usize, u8, u32)> {
    match can.receive(fifo).await {
        Ok(rx) => {
            let (data, id) = match &rx.frame {
                ReceivedFrame::Fd(g) => (g.data(), g.id()),
                ReceivedFrame::Classic(g) => (g.data(), g.id()),
            };
            let raw = match id {
                embedded_can::Id::Standard(s) => s.as_raw() as u32,
                embedded_can::Id::Extended(e) => e.as_raw(),
            };
            Some((data.len(), data.first().copied().unwrap_or(0), raw))
        }
        Err(_) => None,
    }
}

async fn run_layout(
    can: &mut Can,
    label: &str,
    layout: &FifoLayout,
    routes: &[(Fifo, Filter, u16, usize)],
) -> (usize, usize) {
    let mut pass = 0;
    let mut fail = 0;
    info!("  {label}: {} bytes of message RAM", layout.total_bytes());
    if !configure(can, layout, routes).await {
        error!("    configure FAILED");
        return (0, 1);
    }
    // Clear anything left from a previous pass.
    for &(fifo, _, _, _) in routes {
        while try_recv(can, fifo).await.is_some() {}
    }

    for &(fifo, _, id, len) in routes {
        let payload = payload_for(id, len);
        let sid = StandardId::new(id).unwrap();
        let sent = if len <= 8 {
            can.transmit(Fifo::F1, &Frame::new(sid, &payload[..len]).unwrap())
                .await
        } else {
            let f = FdFrame::new(
                sid,
                &payload[..len],
                FrameFlags {
                    brs: true,
                    esi: false,
                },
            )
            .unwrap();
            can.transmit_fd(Fifo::F1, &f).await
        };
        if let Err(e) = sent {
            error!("    {id:#05x}: transmit failed: {e:?}");
            fail += 1;
            continue;
        }
        Timer::after_millis(10).await;

        // The frame must be in its own FIFO...
        match try_recv(can, fifo).await {
            Some((glen, gfirst, gid)) => {
                if glen == len && gfirst == payload[0] && gid == id as u32 {
                    pass += 1;
                } else {
                    error!(
                        "    {id:#05x}: wrong content in FIFO {} -- len {glen} first {gfirst:#04x} id {gid:#05x}",
                        fifo.index()
                    );
                    fail += 1;
                }
            }
            None => {
                error!(
                    "    {id:#05x}: nothing in FIFO {} (routing or RAM addressing?)",
                    fifo.index()
                );
                fail += 1;
            }
        }

        // ...and nowhere else. A RAM-addressing error would show up as a
        // frame appearing in a neighbouring FIFO.
        for &(other, _, other_id, _) in routes {
            if other.index() == fifo.index() {
                continue;
            }
            if let Some((olen, _, oid)) = try_recv(can, other).await {
                error!(
                    "    {id:#05x}: LEAKED into FIFO {} (expected empty; got len {olen} id {oid:#05x}, that FIFO owns {other_id:#05x})",
                    other.index()
                );
                fail += 1;
            } else {
                pass += 1;
            }
        }
    }
    if fail == 0 {
        info!("    OK ({} checks)", pass);
    }
    (pass, fail)
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut chips = devices.map(MCP251xFdAsync::new);

    loop {
        info!("--- layouts: multi-FIFO RAM budgeting and routing ---");
        let can = &mut chips[0];
        common::ensure_configuration(can).await;
        if let Err(e) = can.init(&common::CAN_CONFIG, &mut Delay).await {
            error!("chip 0: init FAILED: {e:?}");
            Timer::after_secs(5).await;
            continue;
        }

        let (p1, f1) =
            run_layout(can, "mixed  (1 TX + 4 RX, B8..B64)", &MIXED, &MIXED_ROUTES).await;
        let (p2, f2) = run_layout(can, "wide   (1 TX + 6 RX, B12)    ", &WIDE, &WIDE_ROUTES).await;

        // Re-applying a layout in Configuration mode must leave the chip
        // usable: `apply_layout`'s docs warn it does not clear FIFOs the
        // previous layout configured, so switching back is worth exercising.
        let (p3, f3) = run_layout(can, "mixed again (re-apply)      ", &MIXED, &MIXED_ROUTES).await;

        let pass = p1 + p2 + p3;
        let fail = f1 + f2 + f3;
        let _ = can.set_mode(OperationMode::Configuration, &mut Delay).await;
        if fail == 0 {
            info!("layouts: all {pass} checks OK");
        } else {
            error!("layouts: {fail} FAILED, {pass} OK");
        }
        Timer::after_secs(5).await;
    }
}
