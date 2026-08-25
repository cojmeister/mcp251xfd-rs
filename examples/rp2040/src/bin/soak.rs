//! Sustained-traffic soak test at the production SPI clock.
//!
//! Needs SPI wiring only -- everything runs through internal loopback.
//!
//! The other binaries send a handful of frames per pass, which is why an
//! intermittent SPI fault hid so well: a 50%-per-frame corruption was
//! invisible to `enumerate` (one 32-bit echo per chip), and a 1-in-10,000
//! fault would be invisible to all of them. This one pushes thousands of
//! frames and reports the corruption rate, so a rare fault becomes a number
//! instead of a coin flip.
//!
//! Every frame carries a distinct payload derived from a running counter, so a
//! stale frame delivered out of order is detected as corruption rather than
//! silently accepted -- a fixed payload would match its own predecessor.
//!
//! Three things only volume reaches:
//! - `SEQ` wraparound (7 bits on the MCP2517FD, so every 128 transmits)
//! - RX/TX FIFO index wraparound
//! - [`Error::TxFifoFull`] and RX overflow, which the probes below force
//!   deliberately rather than waiting for
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Instant, Timer};
use embedded_can::{Frame as _, StandardId};
use log::{error, info};
use mcp251xfd::{
    FdFrame, Fifo, FifoLayout, Filter, FilterMatch, Frame, FrameFlags, MCP251xFdAsync,
    OperationMode, PayloadSize, ReceivedFrame,
};
use panic_halt as _;

/// TX depth 4 so the TX-full probe can outrun the chip; RX depth 8 so the
/// overflow probe has a definite threshold to cross.
const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B64, 8);

/// Verified frames per reporting cycle.
const FRAMES_PER_CYCLE: u32 = 300;

/// The rotation of frame shapes. Mixed lengths and BRS settings so the soak
/// covers the padding boundaries (a 5-byte classic frame pads to 8) rather
/// than only the aligned sizes the other tests happen to use.
const SHAPES: [(bool, bool, usize); 10] = [
    (false, false, 0), // classic, empty
    (false, false, 1), // classic, 1 byte (pads to 4)
    (false, false, 5), // classic, 5 bytes (pads to 8)
    (false, false, 8), // classic, full
    (true, false, 8),  // FD, no BRS
    (true, true, 12),  // FD, BRS
    (true, true, 20),  // FD, BRS, non-power-of-two DLC
    (true, false, 32), // FD, no BRS
    (true, true, 48),  // FD, BRS
    (true, true, 64),  // FD, BRS, maximum
];

type Can = MCP251xFdAsync<common::Device>;

#[derive(Default)]
struct Stats {
    sent: u32,
    ok: u32,
    corrupt: u32,
    timeout: u32,
    tx_full_seen: u32,
    rx_overflow_seen: u32,
    overflow_clear_failed: u32,
    max_tec: u8,
    max_rec: u8,
}

/// Payload for frame `n`: a ramp offset by the counter, so consecutive frames
/// never share a payload and a stale delivery is visible.
fn payload_for(n: u32, len: usize) -> [u8; 64] {
    let mut p = [0u8; 64];
    let base = n as u8;
    for (i, b) in p.iter_mut().enumerate().take(len) {
        *b = base.wrapping_add(i as u8);
    }
    p
}

async fn drain(can: &mut Can) -> u32 {
    let mut n = 0;
    while n < 64 {
        match can.receive(Fifo::F2).await {
            Ok(_) => n += 1,
            Err(_) => break,
        }
    }
    n
}

async fn bring_up(can: &mut Can) -> bool {
    common::ensure_configuration(can).await;
    can.init(&common::CAN_CONFIG, &mut Delay).await.is_ok()
        && can.apply_layout(&LAYOUT).await.is_ok()
        && can
            .set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2)
            .await
            .is_ok()
        && can
            .set_mode(OperationMode::InternalLoopback, &mut Delay)
            .await
            .is_ok()
}

/// Sends one frame and verifies the echo byte-for-byte.
async fn one_frame(can: &mut Can, n: u32, s: &mut Stats) {
    let (fd, brs, len) = SHAPES[(n as usize) % SHAPES.len()];
    let payload = payload_for(n, len);
    let id = StandardId::new(0x100 + (n % 0x100) as u16).unwrap();

    s.sent += 1;
    let sent = if fd {
        let f = FdFrame::new(id, &payload[..len], FrameFlags { brs, esi: false }).unwrap();
        can.transmit_fd(Fifo::F1, &f).await
    } else {
        let f = Frame::new(id, &payload[..len]).unwrap();
        can.transmit(Fifo::F1, &f).await
    };
    if let Err(mcp251xfd::Error::TxFifoFull) = sent {
        // Not corruption: the chip simply has not drained yet. Counted so the
        // report distinguishes back-pressure from a fault.
        s.tx_full_seen += 1;
        s.sent -= 1;
        Timer::after_millis(2).await;
        return;
    }
    if let Err(e) = sent {
        error!("  transmit failed at frame {n}: {e:?}");
        s.corrupt += 1;
        return;
    }

    match common::recv_timeout(can, Fifo::F2).await {
        None => s.timeout += 1,
        Some(rx) => {
            let good = match &rx.frame {
                ReceivedFrame::Fd(g) => fd && g.data() == &payload[..len] && g.id() == id.into(),
                ReceivedFrame::Classic(g) => {
                    !fd && g.data() == &payload[..len] && g.id() == id.into()
                }
            };
            if good {
                s.ok += 1;
            } else {
                s.corrupt += 1;
                let (kind, gid, glen) = match &rx.frame {
                    ReceivedFrame::Fd(g) => ("Fd", g.id(), g.data().len()),
                    ReceivedFrame::Classic(g) => ("Clsc", g.id(), g.data().len()),
                };
                error!(
                    "  frame {n}: CORRUPT -- sent {}{} len {len}, got {kind} len {glen} id {gid:?}",
                    if fd { "FD" } else { "classic" },
                    if brs { "+BRS" } else { "" }
                );
            }
        }
    }
}

/// Queues frames back to back without draining, to reach `TxFifoFull`.
///
/// Deliberately **without** BRS. With the bit-rate switch on, a 64-byte frame
/// clears the wire in roughly 800 us while queueing four costs about 600 us of
/// SPI, so the chip keeps up and the FIFO never fills -- measured: 0 hits in
/// 144 attempts. Running the data phase at the nominal rate instead roughly
/// doubles the on-wire time, so the queue outruns the transmitter.
///
/// Reported rather than asserted: the margin depends on the SPI clock and bit
/// rate, so failing to fill is not a defect.
async fn probe_tx_full(can: &mut Can, s: &mut Stats) {
    let payload = payload_for(0xAA, 64);
    let id = StandardId::new(0x7A0).unwrap();
    let mut hit = false;
    for _ in 0..12 {
        let f = FdFrame::new(
            id,
            &payload,
            FrameFlags {
                brs: false,
                esi: false,
            },
        )
        .unwrap();
        if let Err(mcp251xfd::Error::TxFifoFull) = can.transmit_fd(Fifo::F1, &f).await {
            hit = true;
            break;
        }
    }
    if hit {
        s.tx_full_seen += 1;
    }
    // Long enough for a full 4-deep FIFO of nominal-rate 64-byte frames.
    Timer::after_millis(60).await;
    drain(can).await;
}

/// Overfills the 8-deep RX FIFO, then checks `RXOVIF` is set and that
/// `clear_rx_overflow` actually clears it.
async fn probe_rx_overflow(can: &mut Can, s: &mut Stats) {
    let payload = payload_for(0x55, 8);
    let id = StandardId::new(0x7B0).unwrap();
    // 14 frames into 8 slots, never reading. Paced so the TX FIFO drains.
    for _ in 0..14 {
        let f = Frame::new(id, &payload[..8]).unwrap();
        let _ = can.transmit(Fifo::F1, &f).await;
        Timer::after_millis(1).await;
    }
    Timer::after_millis(20).await;

    match can.fifo_status(Fifo::F2).await {
        Ok(sta) if sta.rx_overflow() => {
            s.rx_overflow_seen += 1;
            if let Err(e) = can.clear_rx_overflow(Fifo::F2).await {
                error!("  clear_rx_overflow failed: {e:?}");
                s.overflow_clear_failed += 1;
            } else {
                drain(can).await;
                match can.fifo_status(Fifo::F2).await {
                    Ok(after) if after.rx_overflow() => {
                        error!("  RXOVIF still set after clear_rx_overflow");
                        s.overflow_clear_failed += 1;
                    }
                    Ok(_) => {}
                    Err(e) => error!("  fifo_status after clear failed: {e:?}"),
                }
            }
        }
        Ok(_) => {
            // Not a defect: the loopback path may drain faster than we fill.
            drain(can).await;
        }
        Err(e) => error!("  fifo_status failed: {e:?}"),
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut chips = devices.map(MCP251xFdAsync::new);
    let can = &mut chips[0];

    info!("--- soak: sustained traffic on chip 0 ---");
    info!(
        "  SPI {} Hz, {FRAMES_PER_CYCLE} verified frames per cycle",
        { mcp251xfd::max_spi_hz(common::CAN_CONFIG.clock.sysclk_hz()) }
    );
    if !bring_up(can).await {
        error!("  bring-up FAILED");
        return;
    }
    drain(can).await;

    let mut s = Stats::default();
    let mut counter: u32 = 0;
    let mut cycle: u32 = 0;
    let start = Instant::now();

    loop {
        let cycle_start = Instant::now();
        for _ in 0..FRAMES_PER_CYCLE {
            one_frame(can, counter, &mut s).await;
            counter = counter.wrapping_add(1);
        }
        probe_tx_full(can, &mut s).await;
        probe_rx_overflow(can, &mut s).await;

        if let Ok(t) = can.error_counters().await {
            s.max_tec = s.max_tec.max(t.tec());
            s.max_rec = s.max_rec.max(t.rec());
            if t.tx_bus_off() || t.tx_error_passive() || t.rx_error_passive() {
                error!(
                    "  bus state degraded: TEC={} REC={} busoff={} txpassive={} rxpassive={}",
                    t.tec(),
                    t.rec(),
                    t.tx_bus_off(),
                    t.tx_error_passive(),
                    t.rx_error_passive()
                );
            }
        }

        cycle += 1;
        let secs = start.elapsed().as_secs().max(1);
        let cycle_ms = cycle_start.elapsed().as_millis().max(1);
        let bad = s.corrupt + s.timeout;
        // Corruption in parts-per-million, so a rare fault is still legible.
        let ppm = if s.sent > 0 {
            (bad as u64 * 1_000_000) / s.sent as u64
        } else {
            0
        };
        if bad == 0 {
            info!(
                "  cycle {cycle}: {} sent, ALL OK -- {} f/s, {}s elapsed, seq wraps ~{}",
                s.sent,
                (FRAMES_PER_CYCLE as u64 * 1000) / cycle_ms,
                secs,
                s.sent / 128
            );
        } else {
            error!(
                "  cycle {cycle}: {} sent, {} corrupt, {} timeout ({ppm} ppm bad)",
                s.sent, s.corrupt, s.timeout
            );
        }
        info!(
            "    tx_full={} rx_overflow={} overflow_clear_failed={} max TEC={} REC={}",
            s.tx_full_seen, s.rx_overflow_seen, s.overflow_clear_failed, s.max_tec, s.max_rec
        );
    }
}
