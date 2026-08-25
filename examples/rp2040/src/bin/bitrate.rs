//! Measures the **actual on-wire CAN bit rate** and checks it against what
//! [`common::CAN_CONFIG`] intends.
//!
//! This is the only test here that can catch a wrong crystal. Every other
//! binary uses internal loopback, where both ends of the link share the same
//! oscillator -- so a board whose crystal is half the configured frequency
//! passes all of them while transmitting at half the intended bit rate. That
//! is exactly what happened on this board: `ClockConfig::MHZ40` with 40 MHz
//! presets on a 20 MHz part ran the bus at 250 kbit/s instead of 500, and only
//! showed up here.
//!
//! Method: time a frame's loopback round trip at two payload sizes and take
//! the difference. The fixed cost -- SPI transactions, driver work, poll
//! granularity -- is identical for both, so it cancels, leaving the on-wire
//! time of the extra payload. No estimate of that overhead enters the result.
//!
//! Output leaves over USB CDC-ACM serial; open the board's COM port.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_rp::spi::{Config as SpiConfig, Phase, Polarity};
use embassy_time::{Delay, Instant, Timer};
use embedded_can::StandardId;
use log::{error, info};
use mcp251xfd::{
    FdFrame, Fifo, FifoLayout, Filter, FilterMatch, FrameFlags, MCP251xFdAsync, OperationMode,
    PayloadSize, ReceivedFrame,
};
use panic_halt as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B64, 8);

/// SPI clock for the measurement. Deliberately well below the erratum cap so
/// corrupted reads cannot skew the timing, and low enough that its own cost is
/// stable between the two payload sizes.
const MEASURE_SPI_HZ: u32 = 4_000_000;

/// The two payload sizes. The gap between them is what gets measured.
const SHORT_LEN: usize = 8;
const LONG_LEN: usize = 64;

/// On-wire bits added by going from `SHORT_LEN` to `LONG_LEN` bytes:
/// 56 bytes = 448 data bits, +4 for CRC21-vs-CRC17, plus roughly 20 dynamic
/// stuff bits over a 0..63 ramp. The stuff-bit term is an estimate, which is
/// why the tolerance below is generous -- this test is built to catch a factor
/// of two, not to calibrate an oscillator.
const EXTRA_BITS: u64 = 472;

/// Accept +/-15%: comfortably tighter than the 2x error it exists to catch,
/// and looser than the stuff-bit estimate's uncertainty.
const TOLERANCE_PERCENT: u64 = 15;

type Can = MCP251xFdAsync<common::Device>;

async fn set_bus(bus: &'static common::Bus, hz: u32) {
    let mut cfg = SpiConfig::default();
    cfg.frequency = hz;
    cfg.phase = Phase::CaptureOnFirstTransition;
    cfg.polarity = Polarity::IdleLow;
    bus.lock().await.set_config(&cfg);
    Timer::after_millis(20).await;
}

async fn drain(can: &mut Can) {
    for _ in 0..32 {
        if can.receive(Fifo::F2).await.is_err() {
            return;
        }
    }
}

fn ramp() -> [u8; 64] {
    let mut p = [0u8; 64];
    let mut n = 0;
    while n < 64 {
        p[n] = n as u8;
        n += 1;
    }
    p
}

/// Round trip for one FD frame without BRS, in microseconds. BRS is off so the
/// whole frame runs at the nominal rate, which is what this measures.
async fn roundtrip(can: &mut Can, len: usize) -> Option<u64> {
    drain(can).await;
    let payload = ramp();
    let f = FdFrame::new(
        StandardId::new(0x456).unwrap(),
        &payload[..len],
        FrameFlags {
            brs: false,
            esi: false,
        },
    )
    .unwrap();

    let t0 = Instant::now();
    if can.transmit_fd(Fifo::F1, &f).await.is_err() {
        return None;
    }
    loop {
        match can.receive(Fifo::F2).await {
            Ok(rx) => {
                // A corrupted read would poison the timing, so only count a
                // frame that came back intact.
                return match &rx.frame {
                    ReceivedFrame::Fd(g) if g.data() == &payload[..len] => {
                        Some(t0.elapsed().as_micros())
                    }
                    _ => None,
                };
            }
            Err(mcp251xfd::Error::RxFifoEmpty) => {
                if t0.elapsed().as_millis() > 80 {
                    return None;
                }
            }
            Err(_) => return None,
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let sysclk = common::CAN_CONFIG.clock.sysclk_hz();
    let expect_bps = common::CAN_CONFIG.nominal.bit_rate_hz(sysclk);
    let mut chips = devices.map(MCP251xFdAsync::new);

    loop {
        info!("--- bitrate: measuring actual nominal bit rate ---");
        info!(
            "  CAN_CONFIG: sysclk {} Hz -> nominal {} bit/s, sample point {}permille",
            sysclk,
            expect_bps,
            common::CAN_CONFIG.nominal.sample_point_permille()
        );
        set_bus(bus, MEASURE_SPI_HZ).await;

        let can = &mut chips[0];
        let _ = can.set_mode(OperationMode::Configuration, &mut Delay).await;
        let up = can.init(&common::CAN_CONFIG, &mut Delay).await.is_ok()
            && can.apply_layout(&LAYOUT).await.is_ok()
            && can
                .set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2)
                .await
                .is_ok()
            && can
                .set_mode(OperationMode::InternalLoopback, &mut Delay)
                .await
                .is_ok();
        if !up {
            error!("  chip 0: bring-up FAILED");
            Timer::after_secs(5).await;
            continue;
        }

        // Minimum over repeats: scheduling jitter only ever adds time, so the
        // fastest observed round trip is the closest to the true cost.
        let mut short = u64::MAX;
        let mut long = u64::MAX;
        for _ in 0..10 {
            if let Some(us) = roundtrip(can, SHORT_LEN).await {
                short = short.min(us);
            }
            if let Some(us) = roundtrip(can, LONG_LEN).await {
                long = long.min(us);
            }
        }
        if short == u64::MAX || long == u64::MAX {
            error!("  no intact frames -- check SPI clock and wiring first");
            Timer::after_secs(5).await;
            continue;
        }

        // Subtract the extra SPI traffic the larger frame needs: 56 payload
        // bytes written plus 56 read back, 8 bits each.
        let extra_spi_us = (112 * 8 * 1_000_000u64) / MEASURE_SPI_HZ as u64;
        let wire_us = long.saturating_sub(short).saturating_sub(extra_spi_us);
        let measured_bps = (EXTRA_BITS * 1_000_000).checked_div(wire_us).unwrap_or(0);

        let lo = expect_bps as u64 * (100 - TOLERANCE_PERCENT) / 100;
        let hi = expect_bps as u64 * (100 + TOLERANCE_PERCENT) / 100;
        info!("  t{SHORT_LEN}={short}us t{LONG_LEN}={long}us wire={wire_us}us");
        if measured_bps >= lo && measured_bps <= hi {
            info!("  measured {measured_bps} bit/s -- OK (expected {expect_bps}, +/-{TOLERANCE_PERCENT}%)");
        } else {
            error!("  measured {measured_bps} bit/s -- MISMATCH (expected {expect_bps})");
            error!("  CAN_CONFIG's clock does not match the board's crystal.");
            if measured_bps * 2 >= lo && measured_bps * 2 <= hi {
                error!("  measured ~half: the crystal is half what CAN_CONFIG claims.");
            } else if measured_bps >= lo * 2 && measured_bps <= hi * 2 {
                error!("  measured ~double: the crystal is twice what CAN_CONFIG claims.");
            }
        }

        Timer::after_secs(5).await;
    }
}
