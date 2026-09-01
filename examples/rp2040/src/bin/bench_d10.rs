//! Why did `d = 10 us` produce zero stalls when 8 us gave 7 and 12 us gave 18?
//!
//! # The hypothesis
//!
//! In `bench_interference` each core-0 mask iteration costs roughly
//! `d + GAP_US + loop overhead`. With `GAP_US = 60` and ~10 us of overhead,
//! `d = 10` lands on an ~80 us period -- and **80 divides core 1's 2000 us
//! cycle exactly, 25 times**. A mask cadence phase-locked to core 1 would put
//! every mask window at the same point in that cycle; if that point misses the
//! RAM reads, the fault rate collapses to zero for structural reasons rather
//! than because 10 us is below the erratum's threshold. No neighbouring `d`
//! divides 2000 evenly.
//!
//! # The design
//!
//! Two arms, run back to back on one flash:
//!
//! - **A, fixed gap** (`GAP_FIXED_US`): reproduces the original conditions.
//! - **B, jittered gap**: the gap walks over a 19-value span, so the mask
//!   cadence cannot stay phase-locked to a 2 ms cycle.
//!
//! Both arms **round-robin** the `d` values across [`ROUNDS`] rounds instead of
//! holding each for one long phase. Interleaving matters: a single ascending
//! pass confounds `d` with elapsed time, so anything that drifts -- die
//! temperature, accumulated recoveries -- would masquerade as a `d` effect.
//!
//! # Reading it
//!
//! If `d = 10` is near-zero in arm A and in line with its neighbours in arm B,
//! the resonance explains the notch and the underlying curve is monotonic. If
//! it is near-zero in **both** arms, the notch is real and something about that
//! delay genuinely suppresses the fault -- which would matter, because it would
//! mean the fault rate is not monotonic in the delay.
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

/// Mask durations to test, in microseconds: finer resolution around the notch.
const SWEEP: [u32; 7] = [6, 8, 9, 10, 11, 12, 14];

/// Round-robin passes over [`SWEEP`] within each arm.
const ROUNDS: usize = 3;

/// Seconds spent at each (arm, d) per round.
const PHASE_SECS: u64 = 4;

/// Arm A's fixed idle gap, in microseconds -- the original conditions.
const GAP_FIXED_US: u64 = 60;

/// Arm B walks the gap over `GAP_JITTER_BASE_US .. + GAP_JITTER_SPAN_US`, so
/// the mask cadence cannot phase-lock to core 1's 2 ms cycle.
const GAP_JITTER_BASE_US: u64 = 53;
const GAP_JITTER_SPAN_US: u64 = 19;

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

/// Core 0: two arms x round-robin over `d`, reporting a stall count per cell.
#[embassy_executor::task]
async fn sweep_task() {
    common::wait_for_host().await;
    info!("bench_d10: is the d=10us notch a phase-lock artefact or a real effect?");
    info!("arm A = fixed {GAP_FIXED_US}us gap (original); arm B = jittered gap");
    info!("arm,round,d_us,stalls,cycles,rx,core0_late");

    // [arm][d index] accumulated stalls.
    let mut totals = [[0u32; SWEEP.len()]; 2];
    let mut gap_step: u64 = 0;

    for (arm, arm_totals) in totals.iter_mut().enumerate() {
        for round in 0..ROUNDS {
            for (di, &d) in SWEEP.iter().enumerate() {
                let before = STALLS.load(Ordering::Relaxed);
                MASK_US.store(d, Ordering::Relaxed);

                let burn = d * SYSCLK_MHZ;
                let end = Instant::now() + Duration::from_secs(PHASE_SECS);
                let mut late: u32 = 0;
                let mut expected = Instant::now() + Duration::from_micros(GAP_FIXED_US);

                while Instant::now() < end {
                    if burn > 0 {
                        cortex_m::interrupt::free(|_| {
                            cortex_m::asm::delay(burn);
                        });
                    }
                    let gap = if arm == 0 {
                        GAP_FIXED_US
                    } else {
                        gap_step = gap_step.wrapping_add(1);
                        GAP_JITTER_BASE_US + (gap_step % GAP_JITTER_SPAN_US)
                    };
                    Timer::after(Duration::from_micros(gap)).await;

                    let now = Instant::now();
                    if now > expected && (now - expected).as_micros() > d as u64 + 100 {
                        late += 1;
                    }
                    expected = now + Duration::from_micros(gap);
                }

                let stalls = STALLS.load(Ordering::Relaxed) - before;
                arm_totals[di] += stalls;
                info!(
                    "{},{},{},{},{},{},{}",
                    if arm == 0 { "A-fixed" } else { "B-jitter" },
                    round,
                    d,
                    stalls,
                    CYCLES.load(Ordering::Relaxed),
                    RX_FRAMES.load(Ordering::Relaxed),
                    late
                );
            }
        }
    }

    info!("=== TOTALS over {ROUNDS} rounds x {PHASE_SECS}s per cell ===");
    for (di, &d) in SWEEP.iter().enumerate() {
        info!(
            "d={}us: A-fixed={} B-jitter={}",
            d, totals[0][di], totals[1][di]
        );
    }
    info!(
        "grand total stalls={} recovered={}",
        STALLS.load(Ordering::Relaxed),
        RECOVERED.load(Ordering::Relaxed)
    );
    loop {
        Timer::after(Duration::from_secs(5)).await;
    }
}
