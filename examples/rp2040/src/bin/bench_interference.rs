//! Measures T_SPIMAXDLY directly: sweep core 0's interrupt-mask time and watch
//! for the MCP2517FD TX MAB underflow to appear.
//!
//! # Why this exists
//!
//! `bench_async` runs the field-faulting configuration -- async driver on core
//! 1, so its SPI DMA completions are serviced on core 0 -- and did **not**
//! fault: 76,000 cycles and 77,000 received frames, zero stalls. The reason is
//! that core 0 was idle, so it serviced `DMA_IRQ_0` the instant it fired. The
//! field rig's core 0 was busy running a flight-control cycle.
//!
//! That missing variable is the whole mechanism, and it can be controlled
//! rather than approximated. MCP2517FD errata DS80000792D item 1 says the chip
//! suffers a TX MAB underflow when SPI holds the CAN FSM off the message RAM
//! for longer than T_SPIMAXDLY -- in the gaps between SPI bytes, and between
//! the last byte and nCS rising. On this driver's async path the nCS rising
//! edge waits on the DMA completion, and that completion is serviced on core 0.
//! So masking interrupts on core 0 for `D` microseconds delays nCS by up to
//! `D`.
//!
//! Table 1 of the erratum puts T_SPIMAXDLY at 5 nominal bit times for a classic
//! base frame: **10 us at 500 kbit/s**. So the prediction is sharp: sweep `D`
//! and faults should appear as `D` crosses ~10 us, and not below it.
//!
//! A clean step in that sweep is a quantitative confirmation of the erratum on
//! silicon -- considerably stronger evidence than the field report's
//! observational correlation. A flat zero across the whole sweep refutes the
//! nCS-delay mechanism and sends the investigation elsewhere. Either outcome is
//! worth having; that is what makes this the right experiment.
//!
//! # Method
//!
//! - Core 1: async driver, chip 9 (the only one wired to the PCAN adapter),
//!   transmit-then-receive every 2 ms. `receive` is what issues the RAM reads.
//! - Core 0: masks its own interrupts with `cortex_m::interrupt::free` for `D`
//!   us, then yields for a fixed gap, repeatedly. `DMA_IRQ_0` cannot be
//!   serviced while masked.
//! - `D` steps through [`SWEEP`], holding each value for [`PHASE_SECS`], and
//!   the stall count is attributed to the value in force at the time.
//!
//! Feed the bussed chip from the Pi so `receive` runs:
//!
//! ```text
//! cangen can0 -g 1 -I 321 -L 8 -D r
//! ```
//!
//! # Reading the result
//!
//! The per-phase line reports `d_us`, stalls seen in that phase, and core 0's
//! own lateness. Faults concentrated at and above 10 us, with none below,
//! confirm both the mechanism and the erratum's stated budget.
#![no_std]
#![no_main]

#[allow(dead_code)]
#[path = "../common.rs"]
mod common;

use embassy_executor::Executor;
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
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

/// Interrupt-mask durations to sweep, in microseconds. Straddles the 10 us
/// T_SPIMAXDLY the erratum specifies for a classic base frame at 500 kbit/s.
const SWEEP: [u32; 9] = [0, 2, 4, 8, 10, 12, 16, 24, 40];

/// Seconds to hold each sweep value.
const PHASE_SECS: u64 = 12;

/// Idle gap between mask windows, in microseconds. Short enough that mask
/// windows overlap core 1's RAM reads often; long enough that core 0 still
/// makes progress.
const GAP_US: u64 = 60;

/// RP2040 system clock after `embassy_rp::init(Default::default())`.
const SYSCLK_MHZ: u32 = 125;

static CYCLES: AtomicU32 = AtomicU32::new(0);
static RX_FRAMES: AtomicU32 = AtomicU32::new(0);
static TX_ERRORS: AtomicU32 = AtomicU32::new(0);
static STALLS: AtomicU32 = AtomicU32::new(0);
static RECOVERED: AtomicU32 = AtomicU32::new(0);
static LAST_RECOVERY_US: AtomicU32 = AtomicU32::new(0);
/// The mask duration currently in force, so core 1's log lines can be
/// attributed to the right phase.
static MASK_US: AtomicU32 = AtomicU32::new(0);

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
        spawner.must_spawn(sweep_task());
    });
}

/// Core 1: the CAN workload whose DMA completions core 0 is about to delay.
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
        if chips[BUSSED].transmit(TX, &frame).await.is_err() {
            TX_ERRORS.fetch_add(1, Ordering::Relaxed);
        }
        while chips[BUSSED].receive(RX).await.is_ok() {
            RX_FRAMES.fetch_add(1, Ordering::Relaxed);
        }
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
                let d = MASK_US.load(Ordering::Relaxed);
                if !reported {
                    reported = true;
                    report_signature(&mut chips[BUSSED], d).await;
                }
                let t0 = Instant::now();
                let did = chips[BUSSED].recover_system_error(MODE, &mut Delay).await;
                let us = t0.elapsed().as_micros() as u32;
                LAST_RECOVERY_US.store(us, Ordering::Relaxed);
                match did {
                    Ok(true) => {
                        RECOVERED.fetch_add(1, Ordering::Relaxed);
                        info!("d={d}us STALL -> recovered in {us} us");
                    }
                    Ok(false) => warn!("d={d}us recover_system_error: nothing to do"),
                    Err(e) => error!("d={d}us recover_system_error: {e:?}"),
                }
            }
        }

        CYCLES.fetch_add(1, Ordering::Relaxed);
        ticker.next().await;
    }
}

/// Dumps the full DS80000792D item 1 signature, once, the first time it is seen.
async fn report_signature(can: &mut Can, d_us: u32) {
    let int = can.interrupt_flags().await;
    let con = can.control_register().await;
    let sta = can.fifo_status(TX).await;
    let fcon = can.fifo_config(TX).await;
    let trec = can.error_counters().await;
    match (int, con, sta, fcon, trec) {
        (Ok(int), Ok(con), Ok(sta), Ok(fcon), Ok(trec)) => {
            warn!(
                "FIRST STALL at d={d_us}us: CiINT={:#010X} serrif={} modif={} ivmif={} | OPMOD={:?}",
                int.0,
                int.serrif(),
                int.modif(),
                int.ivmif(),
                con.op_mode()
            );
            warn!(
                "FIRST STALL at d={d_us}us: FIFOSTA={:#010X} room={} empty={} txreq={} | TEC={} REC={} bo={} bp={}",
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
        _ => error!("FIRST STALL at d={d_us}us: a register read failed"),
    }
}

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

/// Core 0: holds interrupts off for `d` microseconds at a time, stepping `d`
/// through [`SWEEP`], and reports the stall count attributable to each step.
#[embassy_executor::task]
async fn sweep_task() {
    common::wait_for_host().await;
    info!("bench_interference: sweeping core-0 interrupt-mask time against the stall rate");
    info!(
        "T_SPIMAXDLY for a classic base frame at 500 kbit/s is 5 nominal bit times = 10 us;\
         faults are predicted to appear as d crosses that and not below it"
    );
    info!("d_us,phase_s,stalls_in_phase,cycles,rx,txerr,core0_late,worst_us,last_recovery_us");

    for d in SWEEP {
        let before_stalls = STALLS.load(Ordering::Relaxed);
        MASK_US.store(d, Ordering::Relaxed);

        let cycles_to_burn = d * SYSCLK_MHZ;
        let phase_end = Instant::now() + Duration::from_secs(PHASE_SECS);
        let mut late: u32 = 0;
        let mut worst_us: u64 = 0;
        let mut expected = Instant::now() + Duration::from_micros(GAP_US);

        while Instant::now() < phase_end {
            if cycles_to_burn > 0 {
                // Interrupts off here, so DMA_IRQ_0 -- and therefore the nCS
                // rising edge for whatever transfer core 1 has in flight --
                // waits until this returns.
                cortex_m::interrupt::free(|_| {
                    cortex_m::asm::delay(cycles_to_burn);
                });
            }
            Timer::after(Duration::from_micros(GAP_US)).await;

            let now = Instant::now();
            if now > expected {
                let over = (now - expected).as_micros();
                // Anything past the mask duration itself plus a tick of slack
                // is genuine scheduling lateness.
                if over > d as u64 + 100 {
                    late += 1;
                    worst_us = worst_us.max(over);
                }
            }
            expected = now + Duration::from_micros(GAP_US);
        }

        let stalls = STALLS.load(Ordering::Relaxed) - before_stalls;
        info!(
            "{},{},{},{},{},{},{},{},{}",
            d,
            PHASE_SECS,
            stalls,
            CYCLES.load(Ordering::Relaxed),
            RX_FRAMES.load(Ordering::Relaxed),
            TX_ERRORS.load(Ordering::Relaxed),
            late,
            worst_us,
            LAST_RECOVERY_US.load(Ordering::Relaxed)
        );
    }

    info!(
        "sweep complete: total stalls={} recovered={}",
        STALLS.load(Ordering::Relaxed),
        RECOVERED.load(Ordering::Relaxed)
    );
    loop {
        Timer::after(Duration::from_secs(5)).await;
    }
}
