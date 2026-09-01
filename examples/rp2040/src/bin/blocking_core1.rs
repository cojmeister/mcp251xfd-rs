//! The blocking driver on core 1, with core 0 measuring its own jitter.
//!
//! # Why blocking on a dedicated core
//!
//! `embassy_rp::init` calls `dma::init`, which enables `DMA_IRQ_0` in the
//! calling core's NVIC -- and `init` runs on core 0. The handler loops over
//! all twelve DMA channels on every completion. `embassy-rp` never uses
//! `DMA_IRQ_1`, so there is no second line to give core 1. Every SPI DMA
//! completion raised by core 1 is therefore serviced on core 0, at arbitrary
//! phase relative to core 0's own real-time cycle.
//!
//! `Spi::new_blocking` takes no DMA channels and raises no completion
//! interrupt, so *that* interrupt stays off core 0 and two DMA channels come
//! back. Core 0 is not otherwise left alone: `embassy_time`'s `Ticker` below
//! still wakes via `TIMER_IRQ_0`, which `embassy_rp::init` enables on
//! whichever core called it (core 0) regardless of which core's task uses
//! the timer, so other cross-core interrupt traffic remains. For the 3-18
//! byte transfers this driver issues, DMA setup overhead dominates the
//! transfer anyway.
//!
//! There may also be a correctness gain. DS80000792D item 1 is triggered by
//! delays between SPI bytes and between the last byte and nCS rising; a DMA
//! completion serviced late on another core is one way to produce exactly
//! that. If `stall` faults on the async driver and this binary does not under
//! the same load, that is the cross-core interrupt being the mechanism.
//!
//! # What it reports
//!
//! Core 0 runs a fixed 2 ms cycle and counts how many starts land late, plus
//! the worst overshoot. Core 1 runs the CAN load and counts its own cycles.
//! Compare the late count here against the same measurement with the async
//! driver.
//!
//! # Wiring
//!
//! **Needs the CAN bus wired**, same as `stall` and for the same reason: it
//! runs `Normal20` so its fault rate is directly comparable with `stall`'s.
//! Run the two back to back and compare — that comparison is the experiment.
//!
//! # Offered load
//!
//! Same reasoning as `stall`: all ten chips transmitting one 8-byte classic
//! frame every 2 ms cycle would be ~125% of the bus's airtime, making a full
//! TX FIFO the steady state rather than a symptom. Only [`ACTIVE_PER_CYCLE`]
//! chips transmit each cycle, rotating through all ten in fixed groups on the
//! same schedule `stall` uses, for a 100 Hz effective per-chip send rate and
//! ~25% offered load per active cycle -- kept identical between the two
//! binaries so the fault-rate comparison is apples to apples.
#![no_std]
#![no_main]

// This binary is the only one on the blocking path, so it uses none of
// `common.rs`'s async-side items (`Bus`, `init_board`, ...) -- the reverse
// of the existing `#[allow(dead_code)]`s on the blocking-side items there
// for every other, async, binary.
#[allow(dead_code)]
#[path = "../common.rs"]
mod common;

use embassy_executor::Executor;
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_time::{Delay, Instant, Ticker, Timer};
use embedded_can::{Frame as _, StandardId};
use log::{error, info};
use mcp251xfd::{
    Fifo, FifoLayout, Filter, FilterMatch, Frame, MCP251xFd, OperationMode, PayloadSize,
};
use panic_halt as _;
// `core::sync::atomic::AtomicU32::fetch_add` needs native CAS, which
// thumbv6m (Cortex-M0+) does not have. `portable-atomic`'s critical-section
// fallback -- already wired up via `embassy-rp`'s `critical-section-impl`
// feature -- provides it instead.
use portable_atomic::{AtomicU32, Ordering};
use static_cell::StaticCell;

const TX: Fifo = Fifo::F1;
const RX: Fifo = Fifo::F2;

const LAYOUT: FifoLayout =
    FifoLayout::new()
        .tx_fifo(TX, PayloadSize::B8, 8)
        .rx_fifo(RX, PayloadSize::B8, 8);

/// Classic CAN on the real bus, matching `stall` so the two runs compare.
const MODE: OperationMode = OperationMode::Normal20;

/// Core 0's cycle period, and core 1's.
const CYCLE_US: u64 = 2000;

/// How many of the ten chips transmit each cycle -- see "Offered load"
/// above. Must divide 10 evenly so the rotation covers every chip on a fixed
/// cadence, and kept equal to `stall`'s constant of the same name so the two
/// binaries are comparable.
const ACTIVE_PER_CYCLE: usize = 2;

static CORE1_CYCLES: AtomicU32 = AtomicU32::new(0);
static CORE1_ERRORS: AtomicU32 = AtomicU32::new(0);
/// Times core 1 found a chip parked in Restricted Operation or Listen Only —
/// the DS80000792D item 1 signature. If this stays at zero under a load that
/// makes `stall` fault, the cross-core DMA interrupt was the mechanism.
static CORE1_STALLS: AtomicU32 = AtomicU32::new(0);

static mut CORE1_STACK: Stack<8192> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

type Can = MCP251xFd<common::BlockingDevice>;

#[cortex_m_rt::entry]
fn main() -> ! {
    let (devices, usb, core1) = common::init_board_blocking();

    spawn_core1(
        core1,
        // The stack is only ever touched by core 1 after this point.
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| spawner.must_spawn(can_task(devices)));
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.must_spawn(common::logger_task(usb));
        spawner.must_spawn(jitter_task());
    });
}

/// Core 1: the entire CAN workload, all of it blocking SPI.
#[embassy_executor::task]
async fn can_task(devices: [common::BlockingDevice; 10]) {
    let mut chips: [Can; 10] = devices.map(MCP251xFd::new);

    for can in chips.iter_mut() {
        let _ = can.set_mode(OperationMode::Configuration, &mut Delay);
        if let Err(e) = setup(can) {
            CORE1_ERRORS.fetch_add(1, Ordering::Relaxed);
            error!("core1 setup: {e:?}");
        }
    }

    let frame = Frame::new(StandardId::new(0x100).unwrap(), &[0xAA; 8]).unwrap();
    let mut ticker = Ticker::every(embassy_time::Duration::from_micros(CYCLE_US));
    let groups = chips.len() / ACTIVE_PER_CYCLE;
    let mut cycle: usize = 0;

    loop {
        // Only a rotating subset transmits each cycle -- see "Offered load"
        // above -- but every chip's RX FIFO is still drained below, since any
        // of them may have received from whichever chips were active.
        let group = cycle % groups;
        let start = group * ACTIVE_PER_CYCLE;
        for can in chips[start..start + ACTIVE_PER_CYCLE].iter_mut() {
            if can.transmit(TX, &frame).is_err() {
                CORE1_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
        }
        cycle += 1;
        for can in chips.iter_mut() {
            while can.receive(RX).is_ok() {}
        }
        // Same fault check `stall` makes, so the two binaries' counts mean
        // the same thing. Recover in place and keep going.
        for can in chips.iter_mut() {
            if let Ok(con) = can.control_register() {
                if matches!(
                    con.op_mode(),
                    OperationMode::RestrictedOperation | OperationMode::ListenOnly
                ) {
                    CORE1_STALLS.fetch_add(1, Ordering::Relaxed);
                    let _ = can.recover_system_error(MODE, &mut Delay);
                }
            }
        }
        CORE1_CYCLES.fetch_add(1, Ordering::Relaxed);
        ticker.next().await;
    }
}

fn setup(can: &mut Can) -> Result<(), common::BlockingCanError> {
    can.init(&common::CAN_CONFIG, &mut Delay)?;
    can.apply_layout(&LAYOUT)?;
    can.set_filter(Filter::F0, FilterMatch::accept_all(), RX)?;
    can.set_mode(MODE, &mut Delay)?;
    Ok(())
}

/// Core 0: a fixed cycle that does nothing but notice when it starts late.
#[embassy_executor::task]
async fn jitter_task() {
    common::wait_for_host().await;
    info!("blocking driver on core 1; core 0 measuring its own jitter");

    let period = embassy_time::Duration::from_micros(CYCLE_US);
    let mut ticker = Ticker::every(period);
    let mut expected = Instant::now() + period;
    let mut late: u32 = 0;
    let mut worst_us: u64 = 0;
    let mut cycles: u32 = 0;

    loop {
        ticker.next().await;
        let now = Instant::now();
        if now > expected {
            let over = (now - expected).as_micros();
            // One tick of slack: the timer itself has finite resolution.
            if over > 100 {
                late += 1;
                worst_us = worst_us.max(over);
            }
        }
        expected += period;
        cycles += 1;

        // Every ten minutes at 500 Hz.
        if cycles.is_multiple_of(300_000) {
            info!(
                "core0: {late} late starts in {cycles} cycles, worst {worst_us} us | core1: {} cycles, {} errors, {} stalls",
                CORE1_CYCLES.load(Ordering::Relaxed),
                CORE1_ERRORS.load(Ordering::Relaxed),
                CORE1_STALLS.load(Ordering::Relaxed),
            );
        }
        // Keep the log alive on a shorter cadence too.
        if cycles.is_multiple_of(15_000) {
            Timer::after_micros(0).await;
        }
    }
}
