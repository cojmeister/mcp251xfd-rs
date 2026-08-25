//! Verifies [`FilterMatch::with_mask`] -- masked acceptance filtering.
//!
//! Needs SPI wiring only -- everything runs through internal loopback.
//!
//! `with_mask` carries the crate's trickiest bit expression: for an extended
//! identifier it has to scatter a flat 29-bit mask into the register's split
//! layout, `((mask >> 18) & 0x7FF) | ((mask & 0x3_FFFF) << 11)`. Nothing else
//! exercises it, so each case below sends identifiers that must be accepted
//! *and* identifiers that must be rejected: a filter that is accidentally
//! wide-open passes the accept cases on its own, and only the reject cases
//! catch it.
//!
//! The last two cases mask exactly one half of the split each -- the high 11
//! bits (SID field) and the low 18 (EID field) -- which is where a mispacked
//! mask shows up as a filter that ignores the wrong end of the identifier.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use embedded_can::{ExtendedId, Frame as _, Id, StandardId};
use log::{error, info};
use mcp251xfd::{
    Fifo, FifoLayout, FilterMatch, Frame, MCP251xFdAsync, OperationMode, PayloadSize, ReceivedFrame,
};
use panic_halt as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B8, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B8, 8);

type Can = MCP251xFdAsync<common::Device>;

fn raw_id(id: Id) -> u32 {
    match id {
        Id::Standard(s) => s.as_raw() as u32,
        Id::Extended(e) => e.as_raw(),
    }
}

fn sid(raw: u16) -> Id {
    Id::Standard(StandardId::new(raw).unwrap())
}

fn ext(raw: u32) -> Id {
    Id::Extended(ExtendedId::new(raw).unwrap())
}

/// One masked-filter case: the filter, then identifiers that must be accepted
/// and identifiers that must be rejected.
struct Case<'a> {
    label: &'a str,
    id: Id,
    mask: u32,
    accept: &'a [Id],
    reject: &'a [Id],
}

/// Sends one frame and reports whether it was delivered.
async fn delivered(can: &mut Can, id: Id) -> Option<u32> {
    let f = Frame::new(id, &[0x5A]).unwrap();
    if let Err(e) = can.transmit(Fifo::F1, &f).await {
        error!("    transmit failed: {e:?}");
        return None;
    }
    common::recv_timeout(can, Fifo::F2)
        .await
        .map(|rx| match rx.frame {
            ReceivedFrame::Classic(g) => raw_id(g.id()),
            ReceivedFrame::Fd(g) => raw_id(g.id()),
        })
}

async fn run_case(can: &mut Can, c: &Case<'_>) -> (usize, usize) {
    let mut pass = 0;
    let mut fail = 0;
    info!(
        "  {}: id {:#010x} mask {:#010x}",
        c.label,
        raw_id(c.id),
        c.mask
    );
    if let Err(e) = common::arm_filter(can, &LAYOUT, FilterMatch::with_mask(c.id, c.mask)).await {
        error!("    arm_filter failed: {e:?}");
        return (0, 1);
    }
    for &id in c.accept {
        match delivered(can, id).await {
            Some(got) if got == raw_id(id) => pass += 1,
            Some(got) => {
                error!(
                    "    {:#010x}: expected accept, got id {got:#010x}",
                    raw_id(id)
                );
                fail += 1;
            }
            None => {
                error!("    {:#010x}: expected ACCEPT, was rejected", raw_id(id));
                fail += 1;
            }
        }
    }
    for &id in c.reject {
        match delivered(can, id).await {
            None => pass += 1,
            Some(got) => {
                error!(
                    "    {:#010x}: expected REJECT, was delivered as {got:#010x}",
                    raw_id(id)
                );
                fail += 1;
            }
        }
    }
    if fail == 0 {
        info!(
            "    OK ({} accepted, {} rejected as expected)",
            c.accept.len(),
            c.reject.len()
        );
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
        info!("--- filters: FilterMatch::with_mask ---");
        let can = &mut chips[0];
        common::ensure_configuration(can).await;
        if let Err(e) = can.init(&common::CAN_CONFIG, &mut Delay).await {
            error!("chip 0: init FAILED: {e:?}");
            Timer::after_secs(5).await;
            continue;
        }

        let cases = [
            // Standard: match the top 7 bits, ignore the low 4.
            Case {
                label: "std  0x7Ex   ",
                id: sid(0x7E0),
                mask: 0x7F0,
                accept: &[sid(0x7E0), sid(0x7E5), sid(0x7EF)],
                reject: &[sid(0x7D0), sid(0x7F0), sid(0x123)],
            },
            // Standard: a single don't-care bit, the tightest useful mask.
            Case {
                label: "std  0x100/1 ",
                id: sid(0x100),
                mask: 0x7FE,
                accept: &[sid(0x100), sid(0x101)],
                reject: &[sid(0x102), sid(0x000)],
            },
            // Extended: ignore the low 4 bits. Spans nothing interesting but
            // confirms the ordinary extended path.
            Case {
                label: "ext  low nib ",
                id: ext(0x1234_5670),
                mask: 0x1FFF_FFF0,
                accept: &[ext(0x1234_5670), ext(0x1234_5675), ext(0x1234_567F)],
                reject: &[ext(0x1234_5680), ext(0x1234_5660)],
            },
            // Extended: match ONLY the high 11 bits -- the SID field. Every
            // low-18-bit value must be accepted. A mask packed into the wrong
            // half rejects these.
            Case {
                label: "ext  SID only",
                id: ext(0x1FFC_0000),
                mask: 0x1FFC_0000,
                accept: &[ext(0x1FFC_0000), ext(0x1FFF_FFFF), ext(0x1FFC_5678)],
                reject: &[ext(0x0FFC_0000), ext(0x0000_0000)],
            },
            // Extended: match ONLY the low 18 bits -- the EID field. The
            // mirror image of the case above.
            Case {
                label: "ext  EID only",
                id: ext(0x0000_5678),
                mask: 0x0003_FFFF,
                accept: &[ext(0x0000_5678), ext(0x1FFC_5678), ext(0x1234_5678)],
                reject: &[ext(0x0000_5679), ext(0x0000_1678)],
            },
        ];

        let mut pass = 0usize;
        let mut fail = 0usize;
        for c in &cases {
            let (p, f) = run_case(can, c).await;
            pass += p;
            fail += f;
        }

        // Control: `accept_all` must let both kinds through, so a systematic
        // "everything is rejected" fault cannot masquerade as passing rejects.
        if let Err(e) = common::arm_filter(can, &LAYOUT, FilterMatch::accept_all()).await {
            error!("  accept_all arm failed: {e:?}");
            fail += 1;
        } else {
            for id in [sid(0x123), ext(0x1234_5678)] {
                match delivered(can, id).await {
                    Some(got) if got == raw_id(id) => pass += 1,
                    _ => {
                        error!("  accept_all: {:#010x} was NOT delivered", raw_id(id));
                        fail += 1;
                    }
                }
            }
            if fail == 0 {
                info!("  accept_all control: both kinds delivered");
            }
        }

        let _ = can.set_mode(OperationMode::Configuration, &mut Delay).await;
        if fail == 0 {
            info!("filters: all {pass} checks OK");
        } else {
            error!("filters: {fail} FAILED, {pass} OK");
        }
        Timer::after_secs(5).await;
    }
}
