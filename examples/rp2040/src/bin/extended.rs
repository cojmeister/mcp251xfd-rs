//! Verifies 29-bit **extended** identifiers: the SID/EID split, and the
//! standard-vs-extended distinction (`EXIDE`/`MIDE`).
//!
//! Needs SPI wiring only -- everything runs through internal loopback.
//!
//! # Why this uses exact filters rather than a plain round trip
//!
//! A round trip alone proves nothing about the split. `pack_id` writes the
//! transmit object and `unpack_id` reads the receive object, so if the two
//! 11-bit and 18-bit halves were swapped, the swap would cancel and the frame
//! would come back looking correct -- the same blindness that let a wrong
//! crystal pass every loopback test.
//!
//! An **exact acceptance filter** breaks the symmetry. The chip builds the
//! receive object's `R0` from the bits actually on the wire, then compares it
//! against `CiFLTOBJ`, which the driver packs with the very same `pack_id`. A
//! mispacked filter therefore does *not* match the chip's own canonical
//! layout, and the frame is silently dropped. So a timeout here means the
//! split is wrong, and a delivered frame means the driver and the chip agree.
//!
//! The IDs below are chosen to make a swap unmissable: `0x0000_07FF` lives
//! entirely in the low 18 bits (the EID field) and `0x1FFC_0000` entirely in
//! the high 11 (the SID field), so swapping the halves turns one into the
//! other.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use embedded_can::{ExtendedId, Frame as _, Id, StandardId};
use log::{error, info};
use mcp251xfd::{
    FdFrame, Fifo, FifoLayout, FilterMatch, Frame, FrameFlags, MCP251xFdAsync, OperationMode,
    PayloadSize, ReceivedFrame,
};
use panic_halt as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B64, 8);

/// Extended IDs worth pinning. The last two isolate one half of the split
/// each, so a SID/EID swap maps one onto the other.
const IDS: [(u32, &str); 8] = [
    (0x0000_0000, "all zero          "),
    (0x1FFF_FFFF, "all 29 bits set   "),
    (0x1234_5678, "reference value   "),
    (0x0000_07FF, "low 11 bits only  "),
    (0x0003_FFFF, "EID field only    "),
    (0x1FFC_0000, "SID field only    "),
    (0x1555_5555, "alternating 0101  "),
    (0x0AAA_AAAA, "alternating 1010  "),
];

type Can = MCP251xFdAsync<common::Device>;

fn raw_id(id: Id) -> u32 {
    match id {
        Id::Standard(s) => s.as_raw() as u32,
        Id::Extended(e) => e.as_raw(),
    }
}

fn ext(raw: u32) -> Id {
    Id::Extended(ExtendedId::new(raw).unwrap())
}

/// Sends `send_id` with filter `F0` armed for `filter_id`, and reports whether
/// a frame arrived and with which identifier.
///
/// Returns `Some(received_raw_id)` on delivery, `None` on timeout.
async fn attempt(can: &mut Can, filter_id: Id, send_id: Id, payload: &[u8]) -> Option<u32> {
    if let Err(e) = common::arm_filter(can, &LAYOUT, FilterMatch::exact(filter_id)).await {
        error!("  arm_filter failed: {e:?}");
        return None;
    }
    let f = Frame::new(send_id, payload).unwrap();
    if let Err(e) = can.transmit(Fifo::F1, &f).await {
        error!("  transmit failed: {e:?}");
        return None;
    }
    common::recv_timeout(can, Fifo::F2)
        .await
        .map(|rx| match rx.frame {
            ReceivedFrame::Classic(g) => raw_id(g.id()),
            ReceivedFrame::Fd(g) => raw_id(g.id()),
        })
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut chips = devices.map(MCP251xFdAsync::new);

    loop {
        info!("--- extended: 29-bit identifiers through exact filters ---");
        let can = &mut chips[0];
        common::ensure_configuration(can).await;
        if let Err(e) = can.init(&common::CAN_CONFIG, &mut Delay).await {
            error!("chip 0: init FAILED: {e:?}");
            Timer::after_secs(5).await;
            continue;
        }

        let mut pass = 0usize;
        let mut fail = 0usize;

        // 1. Each ID must survive the round trip *and* satisfy a filter
        // packed from the same value.
        for (raw, label) in IDS {
            let id = ext(raw);
            match attempt(can, id, id, &[0xA5, raw as u8]).await {
                Some(got) if got == raw => {
                    info!("  {label} {raw:#010x}: OK");
                    pass += 1;
                }
                Some(got) => {
                    error!("  {label} {raw:#010x}: WRONG ID BACK {got:#010x}");
                    fail += 1;
                }
                None => {
                    error!("  {label} {raw:#010x}: FILTERED OUT (SID/EID split wrong?)");
                    fail += 1;
                }
            }
        }

        // 2. Negative control: a filter for one ID must reject another.
        // Without this, an accept-everything bug would make part 1 pass.
        let want = ext(0x1234_5678);
        let other = ext(0x1234_5679);
        match attempt(can, want, other, &[1]).await {
            None => {
                info!("  negative control: 0x12345679 correctly rejected by 0x12345678 filter");
                pass += 1;
            }
            Some(got) => {
                error!("  negative control: filter LEAKED, got {got:#010x}");
                fail += 1;
            }
        }

        // 3. `EXIDE`/`MIDE`: the same numeric value as a standard and an
        // extended ID must not cross-match. `FilterMatch::exact` sets `MIDE`
        // so the frame kind is part of the comparison.
        let std_123 = Id::Standard(StandardId::new(0x123).unwrap());
        let ext_123 = ext(0x123);
        match attempt(can, ext_123, std_123, &[2]).await {
            None => {
                info!("  MIDE: standard 0x123 correctly rejected by extended 0x123 filter");
                pass += 1;
            }
            Some(got) => {
                error!("  MIDE: standard frame LEAKED through extended filter, got {got:#010x}");
                fail += 1;
            }
        }
        match attempt(can, std_123, ext_123, &[3]).await {
            None => {
                info!("  MIDE: extended 0x123 correctly rejected by standard 0x123 filter");
                pass += 1;
            }
            Some(got) => {
                error!("  MIDE: extended frame LEAKED through standard filter, got {got:#010x}");
                fail += 1;
            }
        }

        // 4. An extended FD frame with bit-rate switch: the ID path is shared
        // with the classic case, but `T1` differs, so confirm the two do not
        // interfere.
        let id = ext(0x1ABC_DEF0);
        if let Err(e) = common::arm_filter(can, &LAYOUT, FilterMatch::exact(id)).await {
            error!("  FD arm_filter failed: {e:?}");
            fail += 1;
        } else {
            let payload: [u8; 48] = core::array::from_fn(|i| i as u8);
            let f = FdFrame::new(
                match id {
                    Id::Extended(e) => e,
                    Id::Standard(_) => unreachable!(),
                },
                &payload,
                FrameFlags {
                    brs: true,
                    esi: false,
                },
            )
            .unwrap();
            if let Err(e) = can.transmit_fd(Fifo::F1, &f).await {
                error!("  FD transmit failed: {e:?}");
                fail += 1;
            } else {
                match common::recv_timeout(can, Fifo::F2).await {
                    Some(rx) => match rx.frame {
                        ReceivedFrame::Fd(g)
                            if raw_id(g.id()) == 0x1ABC_DEF0 && g.data() == payload =>
                        {
                            info!("  extended FD-48 BRS 0x1ABCDEF0: OK");
                            pass += 1;
                        }
                        ReceivedFrame::Fd(g) => {
                            error!("  extended FD: wrong frame, id {:#010x}", raw_id(g.id()));
                            fail += 1;
                        }
                        ReceivedFrame::Classic(g) => {
                            error!(
                                "  extended FD: came back classic, id {:#010x}",
                                raw_id(g.id())
                            );
                            fail += 1;
                        }
                    },
                    None => {
                        error!("  extended FD-48 BRS: TIMEOUT");
                        fail += 1;
                    }
                }
            }
        }

        let _ = can.set_mode(OperationMode::Configuration, &mut Delay).await;
        if fail == 0 {
            info!("extended: all {pass} checks OK");
        } else {
            error!("extended: {fail} FAILED, {pass} OK");
        }
        Timer::after_secs(5).await;
    }
}
