//! Validates `transmit_batch_contiguous` on hardware: ten chips, three frames
//! per cycle, internal loopback, paced one cycle at a time.
//!
//! This is the check the mock-SPI unit tests cannot make. They prove the
//! driver emits the bytes it means to; this proves the *chip* places the
//! three objects where the driver wrote them. Every frame is read back from
//! the RX FIFO and compared for ID, payload and order, so a wrong element
//! stride, a wrong `UA` or a `TXREQ` that does not release the whole burst
//! shows up as a mismatch rather than as a timing number.
//!
//! The TX FIFO is six deep and a batch is three frames, so the head
//! alternates between the FIFO base and base + 48 and every second batch ends
//! exactly on the FIFO end: the wrap rule in the method's docs (depth a
//! multiple of the batch length, nothing else writing the FIFO) is exercised
//! on every chip, 1000 batches each, without ever crossing it.
//!
//! Three phases per pass:
//! 1. `transmit_batch_contiguous`, verified frame by frame, with the time
//!    spent inside the transmit call accumulated separately from the drain.
//! 2. `transmit_batch` on the same traffic, timed the same way: the baseline.
//!    The contiguous call should cost roughly `n + 2` chip-select assertions
//!    against `3n`, so expect it clearly faster at n = 3.
//! 3. Two refusal probes: the call on an RX FIFO must fail with
//!    `CommunicationCheckFailed`, and a second call issued straight after a
//!    first, while the burst is still on the wire, must return 0 (the FIFO is
//!    not empty). The second probe reports rather than asserts: if the
//!    SPI round trip is slower than three frames on the wire the FIFO has
//!    already drained, which is a legitimate outcome and is logged as such.
//!
//! Expect `mismatches 0`, `short 0`, `not_empty 0`, `timeouts 0`, and the
//! pass line ending `ALL OK`. Anything else is a finding.
//!
//! Runs on internal loopback: SPI wiring only, no transceivers.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Instant, Timer};
use embedded_can::{Frame as _, Id, StandardId};
use log::{error, info};
use mcp251xfd::{
    Error, Fifo, FifoLayout, Filter, FilterMatch, Frame, MCP251xFdAsync, OperationMode,
    PayloadSize, ReceivedFrame,
};
use panic_halt as _;

const TX: Fifo = Fifo::F1;
const RX: Fifo = Fifo::F2;

/// Six deep: a multiple of the three-frame batch, so a batch never straddles
/// the FIFO end (the driver cannot detect that; see the method's docs).
const LAYOUT: FifoLayout =
    FifoLayout::new()
        .tx_fifo(TX, PayloadSize::B8, 6)
        .rx_fifo(RX, PayloadSize::B8, 8);

const CYCLES: u32 = 500;
const BATCH: usize = 3;

type Can = MCP251xFdAsync<common::Device>;

#[derive(Default)]
struct Tally {
    /// Frames whose ID, payload or position differ from what was sent.
    mismatches: u32,
    /// Batches that queued fewer than three frames, including zero.
    short: u32,
    /// Of those, batches refused because the FIFO was not empty.
    not_empty: u32,
    /// Frames that never arrived within the receive timeout.
    timeouts: u32,
    /// Driver errors other than the above.
    errors: u32,
    /// Microseconds spent inside the transmit call only.
    transmit_us: u64,
}

async fn setup(can: &mut Can) -> Result<(), common::CanError> {
    can.set_mode(OperationMode::Configuration, &mut Delay)
        .await?;
    can.apply_layout(&LAYOUT).await?;
    can.set_filter(Filter::F0, FilterMatch::accept_all(), RX)
        .await?;
    can.set_mode(OperationMode::InternalLoopback, &mut Delay)
        .await?;
    Ok(())
}

/// The three frames of one cycle. The payload carries the cycle number so a
/// stale frame from an earlier cycle cannot pass as a current one.
fn frames_for(cycle: u32) -> [Frame; BATCH] {
    let tag = cycle as u8;
    [
        Frame::new(StandardId::new(0x301).unwrap(), &[tag, 1, 1, 1, 1, 1, 1, 1]).unwrap(),
        Frame::new(StandardId::new(0x302).unwrap(), &[tag, 2, 2, 2, 2, 2, 2, 2]).unwrap(),
        Frame::new(StandardId::new(0x303).unwrap(), &[tag, 3, 3, 3]).unwrap(),
    ]
}

/// Reads the three frames of one cycle back and compares them in order.
async fn drain_and_verify(can: &mut Can, sent: &[Frame; BATCH], t: &mut Tally) {
    for want in sent {
        match common::recv_timeout(can, RX).await {
            Some(rx) => {
                let ok = matches!(
                    rx.frame,
                    ReceivedFrame::Classic(got)
                        if got.id() == Id::Standard(match want.id() {
                            Id::Standard(s) => s,
                            Id::Extended(_) => unreachable!(),
                        }) && got.data() == want.data()
                );
                if !ok {
                    t.mismatches += 1;
                }
            }
            None => t.timeouts += 1,
        }
    }
    // Loopback delivery implies the frame left the TX FIFO, but TFERFFIF can
    // trail it by a moment; give it a bounded wait so the next cycle's
    // not-empty refusal means what it says.
    for _ in 0..20 {
        match can.fifo_status(TX).await {
            Ok(s) if s.tx_empty_or_rx_full() => return,
            Ok(_) => Timer::after_micros(50).await,
            Err(_) => {
                t.errors += 1;
                return;
            }
        }
    }
}

/// One timed phase: `CYCLES` cycles across all ten chips. `contiguous`
/// selects the method under test; otherwise the per-frame baseline runs.
async fn phase(chips: &mut [Can; 10], contiguous: bool) -> Tally {
    let mut t = Tally::default();
    for cycle in 0..CYCLES {
        let sent = frames_for(cycle);
        for can in chips.iter_mut() {
            let t0 = Instant::now();
            let r = if contiguous {
                can.transmit_batch_contiguous(TX, &sent).await
            } else {
                can.transmit_batch(TX, &sent).await
            };
            t.transmit_us += t0.elapsed().as_micros();
            match r {
                Ok(n) if n as usize == BATCH => {}
                Ok(n) => {
                    t.short += 1;
                    if n == 0 {
                        t.not_empty += 1;
                    }
                }
                Err(e) => {
                    t.errors += 1;
                    error!("transmit: {e:?}");
                }
            }
        }
        for can in chips.iter_mut() {
            drain_and_verify(can, &sent, &mut t).await;
        }
    }
    t
}

/// Calls the method on the RX FIFO, which must be refused.
async fn probe_rx_fifo(can: &mut Can) -> bool {
    let sent = frames_for(0);
    match can.transmit_batch_contiguous(RX, &sent).await {
        Err(Error::CommunicationCheckFailed) => {
            info!("probe: RX FIFO refused with CommunicationCheckFailed (expected)");
            true
        }
        other => {
            error!("probe: RX FIFO call returned {other:?}, expected CommunicationCheckFailed");
            false
        }
    }
}

/// Fires two batches back to back; the second should see a non-empty FIFO.
async fn probe_not_empty(can: &mut Can) -> bool {
    let sent = frames_for(1);
    let first = can.transmit_batch_contiguous(TX, &sent).await;
    let second = can.transmit_batch_contiguous(TX, &sent).await;
    let mut t = Tally::default();
    drain_and_verify(can, &sent, &mut t).await;
    // Second batch, if it was ever queued, produces three more frames.
    if let Ok(3) = second {
        drain_and_verify(can, &sent, &mut t).await;
    }
    match (first, second) {
        (Ok(3), Ok(0)) => {
            info!("probe: second batch refused while the first was in flight (expected)");
            true
        }
        (Ok(3), Ok(3)) => {
            info!("probe: first batch drained before the second call (SPI slower than the wire); not a failure");
            true
        }
        other => {
            error!("probe: unexpected results {other:?}");
            false
        }
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
        if let Err(e) = setup(can).await {
            error!("chip {i}: setup: {e:?}");
        }
    }

    let mut pass = 0u32;
    loop {
        pass += 1;
        let c = phase(&mut chips, true).await;
        let b = phase(&mut chips, false).await;
        let probes_ok = probe_rx_fifo(&mut chips[0]).await & probe_not_empty(&mut chips[0]).await;

        let n = CYCLES as u64 * 10;
        info!(
            "pass {pass}: contiguous {} us/call, per-frame batch {} us/call ({n} calls each)",
            c.transmit_us / n,
            b.transmit_us / n,
        );
        info!(
            "pass {pass}: contiguous mismatches {} short {} not_empty {} timeouts {} errors {}",
            c.mismatches, c.short, c.not_empty, c.timeouts, c.errors
        );
        info!(
            "pass {pass}: baseline   mismatches {} short {} not_empty {} timeouts {} errors {}",
            b.mismatches, b.short, b.not_empty, b.timeouts, b.errors
        );
        let clean = |t: &Tally| {
            t.mismatches == 0 && t.short == 0 && t.timeouts == 0 && t.errors == 0
        };
        if clean(&c) && clean(&b) && probes_ok {
            info!("pass {pass}: ALL OK");
        } else {
            error!("pass {pass}: FAILED");
        }

        // The probes leave chip 0 mid-traffic state clean (drained), but
        // re-arm it anyway so a failed probe cannot poison the next pass.
        if let Err(e) = setup(&mut chips[0]).await {
            error!("re-arm chip 0: {e:?}");
        }
        Timer::after_secs(5).await;
    }
}
