//! **The field-faulting configuration**: the *async* driver on core 1.
//!
//! This is the missing middle of the experiment matrix. `stall` is single-core
//! async (DMA completions land on the core that issued them) and
//! `blocking_core1` raises no DMA completion at all. Neither recreates what the
//! field report describes, which is:
//!
//! | configuration | SPI DMA completion serviced on | expectation |
//! |---|---|---|
//! | `stall` (single-core async) | the issuing core | no fault |
//! | **`bench_async` (dual-core async)** | **core 0, remotely** | **fault** |
//! | `bench_blocking` (dual-core blocking) | no interrupt at all | no fault |
//!
//! `embassy_rp::init` calls `dma::init`, which enables `DMA_IRQ_0` in the
//! calling core's NVIC -- and `init` runs on core 0. `embassy-rp` never uses
//! `DMA_IRQ_1`. So every SPI DMA completion raised by core 1 here is serviced
//! on core 0, at a phase core 1 cannot predict. A completion serviced late
//! stretches the gap between the last SPI byte and nCS rising, which is exactly
//! the window MCP2517FD errata DS80000792D item 1 names: held off longer than
//! T_SPIMAXDLY (5 nominal bit times, 10 us at 500 kbit/s), the chip suffers a
//! TX MAB underflow, sets `SERRIF`/`MODIF`, and drops into Restricted Operation
//! or Listen Only where `TXREQ` is ignored.
//!
//! # Bench topology
//!
//! Only the chip on `GP15` -- index 9 -- has its CAN connector wired to the
//! PCAN adapter, so it is the only chip that can transmit and be ACKed. The
//! other nine are initialised and then left in **Configuration mode**: they
//! generate no CAN traffic (so they cannot go bus-off unacknowledged and flood
//! the log) but they are still polled over SPI every cycle, which reproduces
//! the ten-chip SPI bus occupancy the field report ran with. That occupancy
//! matters: the erratum is about SPI holding the CAN FSM off the RAM.
//!
//! Feed the bussed chip from the Pi so `receive` actually runs -- only
//! `receive` issues the RAM reads the erratum needs:
//!
//! ```text
//! cangen can0 -g 2 -I 321 -L 8 -D r
//! ```
//!
//! # What it reports
//!
//! Per fault: the full DS80000792D signature (latched `CiINT`, `CiCON.OPMOD`,
//! TX `CiFIFOSTA` and `TXREQ`, and `CiTREC`), then `recover_system_error`
//! timed in microseconds. Core 0 separately counts its own late cycle starts,
//! so the jitter cost of servicing core 1's completions is visible too.
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
    CiInt, Fifo, FifoLayout, Filter, FilterMatch, Frame, MCP251xFdAsync, OperationMode, PayloadSize,
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

type Can = MCP251xFdAsync<common::AsyncCsDevice>;

#[cortex_m_rt::entry]
fn main() -> ! {
    let (devices, usb, core1) = common::init_board_async_cs();

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

/// Core 1: the whole CAN workload, over **async** SPI, so its DMA completions
/// are raised here and serviced on core 0.
#[embassy_executor::task]
async fn can_task(devices: [common::AsyncCsDevice; 10]) {
    let mut chips: [Can; 10] = devices.map(MCP251xFdAsync::new);

    for (i, can) in chips.iter_mut().enumerate() {
        let _ = can.set_mode(OperationMode::Configuration, &mut Delay).await;
        if let Err(e) = init_chip(can, i == BUSSED).await {
            error!("core1: chip {i} setup: {e:?}");
        }
    }

    let frame = Frame::new(StandardId::new(0x100).unwrap(), &[0xAA; 8]).unwrap();
    let mut ticker = Ticker::every(Duration::from_micros(CYCLE_US));
    let mut reported = false;

    loop {
        // The bussed chip carries the receive-then-echo path that trips the
        // erratum. `receive` is the half that issues RAM reads.
        if chips[BUSSED].transmit(TX, &frame).await.is_err() {
            TX_ERRORS.fetch_add(1, Ordering::Relaxed);
        }
        while chips[BUSSED].receive(RX).await.is_ok() {
            RX_FRAMES.fetch_add(1, Ordering::Relaxed);
        }

        // The other nine stay in Configuration mode -- no CAN traffic, so no
        // unacknowledged retries -- but are still read every cycle to
        // reproduce the ten-chip SPI bus occupancy.
        for (i, can) in chips.iter_mut().enumerate() {
            if i != BUSSED {
                let _ = can.control_register().await;
            }
        }

        if let Ok(con) = chips[BUSSED].control_register().await {
            if matches!(
                con.op_mode(),
                OperationMode::RestrictedOperation | OperationMode::ListenOnly
            ) {
                STALLS.fetch_add(1, Ordering::Relaxed);
                if !reported {
                    reported = true;
                    report_signature(&mut chips[BUSSED]).await;
                }
                let t0 = Instant::now();
                let did = chips[BUSSED].recover_system_error(MODE, &mut Delay).await;
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
async fn report_signature(can: &mut Can) {
    let int = can.interrupt_flags().await;
    let con = can.control_register().await;
    let sta = can.fifo_status(TX).await;
    let fcon = can.fifo_config(TX).await;
    let trec = can.error_counters().await;
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
async fn init_chip(can: &mut Can, bussed: bool) -> Result<(), common::CanError> {
    can.init(&common::CAN_CONFIG, &mut Delay).await?;
    can.apply_layout(&LAYOUT).await?;
    can.set_filter(Filter::F0, FilterMatch::accept_all(), RX)
        .await?;
    can.configure_interrupts(CiInt(0).with_serrie(true).with_ivmie(true).with_modie(true))
        .await?;
    if bussed {
        can.set_mode(MODE, &mut Delay).await?;
    }
    Ok(())
}

/// Core 0: measures its own lateness while servicing core 1's DMA completions,
/// and prints the running totals.
#[embassy_executor::task]
async fn report_task() {
    common::wait_for_host().await;
    info!("bench_async: ASYNC driver on core 1 (the field-faulting configuration)");
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

        // Every 5 s at 500 Hz.
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
