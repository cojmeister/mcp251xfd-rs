//! The control for [`bench_async`]: identical workload, **blocking** driver.
//!
//! Run this back to back with `bench_async`. The two binaries offer the same
//! CAN load, on the same chip, at the same rate, from the same core — they
//! differ in exactly one thing:
//!
//! | configuration | SPI DMA completion serviced on | expectation |
//! |---|---|---|
//! | `bench_async` (dual-core async) | core 0, remotely | fault |
//! | **`bench_blocking` (dual-core blocking)** | **no interrupt at all** | **no fault** |
//!
//! `Spi::new_blocking` takes no DMA channels and raises no completion
//! interrupt, so the gap between the last SPI byte and nCS rising is bounded by
//! core 1's own instruction stream rather than by whenever core 0 gets around
//! to servicing `DMA_IRQ_0`. If `bench_async` faults under this load and this
//! binary does not, the cross-core DMA interrupt is the mechanism, and the stall and the
//! cross-core jitter are one defect rather than two.
//!
//! A null result here is only meaningful if `bench_async` faulted first — read
//! them as a pair, never alone.
//!
//! # Bench topology
//!
//! Same as `bench_async`: only the chip on `GP15` (index 9) is wired to the
//! PCAN adapter, so it carries all the CAN traffic; the other nine stay in
//! Configuration mode and are polled over SPI purely to reproduce the ten-chip
//! bus occupancy. Feed the bussed chip from the Pi so `receive` runs:
//!
//! ```text
//! cangen can0 -g 2 -I 321 -L 8 -D r
//! ```
#![no_std]
#![no_main]

#[allow(dead_code)]
#[path = "../common.rs"]
mod common;

use embassy_executor::Executor;
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_time::{Delay, Duration, Instant, Ticker};
use embedded_can::{Frame as _, StandardId};
use log::{error, info, warn};
use mcp251xfd::{
    CiInt, Fifo, FifoLayout, Filter, FilterMatch, Frame, MCP251xFd, OperationMode, PayloadSize,
};
use panic_halt as _;
use portable_atomic::{AtomicU32, Ordering};
use static_cell::StaticCell;

const TX: Fifo = Fifo::F1;
const RX: Fifo = Fifo::F2;

/// The only chip whose CAN connector is wired to the PCAN adapter (`GP15`).
const BUSSED: usize = 9;

const LAYOUT: FifoLayout =
    FifoLayout::new()
        .tx_fifo(TX, PayloadSize::B8, 8)
        .rx_fifo(RX, PayloadSize::B8, 8);

const MODE: OperationMode = OperationMode::Normal20;
const CYCLE_US: u64 = 2000;

static CYCLES: AtomicU32 = AtomicU32::new(0);
static TX_ERRORS: AtomicU32 = AtomicU32::new(0);
static RX_FRAMES: AtomicU32 = AtomicU32::new(0);
static STALLS: AtomicU32 = AtomicU32::new(0);
static RECOVERED: AtomicU32 = AtomicU32::new(0);
static LAST_RECOVERY_US: AtomicU32 = AtomicU32::new(0);

static mut CORE1_STACK: Stack<8192> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

type Can = MCP251xFd<common::BlockingDevice>;

#[cortex_m_rt::entry]
fn main() -> ! {
    let (devices, usb, core1) = common::init_board_blocking();

    spawn_core1(
        core1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| spawner.must_spawn(can_task(devices)));
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.must_spawn(common::logger_task(usb));
        spawner.must_spawn(report_task());
    });
}

/// Core 1: the whole CAN workload, over **blocking** SPI — no DMA, so no
/// completion interrupt is raised on either core.
#[embassy_executor::task]
async fn can_task(devices: [common::BlockingDevice; 10]) {
    let mut chips: [Can; 10] = devices.map(MCP251xFd::new);

    for (i, can) in chips.iter_mut().enumerate() {
        let _ = can.set_mode(OperationMode::Configuration, &mut Delay);
        if let Err(e) = init_chip(can, i == BUSSED) {
            error!("core1: chip {i} setup: {e:?}");
        }
    }

    let frame = Frame::new(StandardId::new(0x100).unwrap(), &[0xAA; 8]).unwrap();
    let mut ticker = Ticker::every(Duration::from_micros(CYCLE_US));
    let mut reported = false;

    loop {
        if chips[BUSSED].transmit(TX, &frame).is_err() {
            TX_ERRORS.fetch_add(1, Ordering::Relaxed);
        }
        while chips[BUSSED].receive(RX).is_ok() {
            RX_FRAMES.fetch_add(1, Ordering::Relaxed);
        }

        for (i, can) in chips.iter_mut().enumerate() {
            if i != BUSSED {
                let _ = can.control_register();
            }
        }

        if let Ok(con) = chips[BUSSED].control_register() {
            if matches!(
                con.op_mode(),
                OperationMode::RestrictedOperation | OperationMode::ListenOnly
            ) {
                STALLS.fetch_add(1, Ordering::Relaxed);
                if !reported {
                    reported = true;
                    report_signature(&mut chips[BUSSED]);
                }
                let t0 = Instant::now();
                let did = chips[BUSSED].recover_system_error(MODE, &mut Delay);
                let us = t0.elapsed().as_micros() as u32;
                LAST_RECOVERY_US.store(us, Ordering::Relaxed);
                match did {
                    Ok(true) => {
                        RECOVERED.fetch_add(1, Ordering::Relaxed);
                        info!("recover_system_error: recovered in {us} us");
                    }
                    Ok(false) => warn!("recover_system_error: nothing to do ({us} us)"),
                    Err(e) => error!("recover_system_error: {e:?}"),
                }
            }
        }

        CYCLES.fetch_add(1, Ordering::Relaxed);
        ticker.next().await;
    }
}

/// Dumps the full DS80000792D item 1 signature, once, the first time it is seen.
fn report_signature(can: &mut Can) {
    let int = can.interrupt_flags();
    let con = can.control_register();
    let sta = can.fifo_status(TX);
    let fcon = can.fifo_config(TX);
    let trec = can.error_counters();
    match (int, con, sta, fcon, trec) {
        (Ok(int), Ok(con), Ok(sta), Ok(fcon), Ok(trec)) => {
            warn!(
                "STALL SIGNATURE: CiINT={:#010X} serrif={} modif={} ivmif={} | OPMOD={:?}",
                int.0,
                int.serrif(),
                int.modif(),
                int.ivmif(),
                con.op_mode()
            );
            warn!(
                "STALL SIGNATURE: FIFOSTA={:#010X} room={} empty={} txreq={} | TEC={} REC={} bo={} bp={}",
                sta.0,
                sta.not_full_or_not_empty(),
                sta.tx_empty_or_rx_full(),
                fcon.txreq(),
                trec.tec(),
                trec.rec(),
                trec.tx_bus_off(),
                trec.tx_error_passive()
            );
        }
        _ => error!("STALL SIGNATURE: a register read failed"),
    }
}

/// Initialises one chip. Only the bussed chip leaves Configuration mode.
fn init_chip(can: &mut Can, bussed: bool) -> Result<(), common::BlockingCanError> {
    can.init(&common::CAN_CONFIG, &mut Delay)?;
    can.apply_layout(&LAYOUT)?;
    can.set_filter(Filter::F0, FilterMatch::accept_all(), RX)?;
    can.configure_interrupts(CiInt(0).with_serrie(true).with_ivmie(true).with_modie(true))?;
    if bussed {
        can.set_mode(MODE, &mut Delay)?;
    }
    Ok(())
}

/// Core 0: same measurement as `bench_async`, so the two are comparable.
#[embassy_executor::task]
async fn report_task() {
    common::wait_for_host().await;
    info!("bench_blocking: BLOCKING driver on core 1 (the control)");
    info!("bussed chip = {BUSSED} (GP15); others held in Configuration mode as SPI load");

    let period = Duration::from_micros(CYCLE_US);
    let mut ticker = Ticker::every(period);
    let mut expected = Instant::now() + period;
    let mut late: u32 = 0;
    let mut worst_us: u64 = 0;
    let mut n: u32 = 0;

    loop {
        ticker.next().await;
        let now = Instant::now();
        if now > expected {
            let over = (now - expected).as_micros();
            if over > 100 {
                late += 1;
                worst_us = worst_us.max(over);
            }
        }
        expected += period;
        n += 1;

        if n.is_multiple_of(2500) {
            info!(
                "cycles={} rx={} txerr={} stalls={} recovered={} last_recovery_us={} | core0 late={} worst={}us",
                CYCLES.load(Ordering::Relaxed),
                RX_FRAMES.load(Ordering::Relaxed),
                TX_ERRORS.load(Ordering::Relaxed),
                STALLS.load(Ordering::Relaxed),
                RECOVERED.load(Ordering::Relaxed),
                LAST_RECOVERY_US.load(Ordering::Relaxed),
                late,
                worst_us
            );
        }
    }
}
