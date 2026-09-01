//! Reproduces the MCP2517FD transmit stall and times four recovery ladders.
//!
//! # What this is testing
//!
//! DS80000792D item 1: during an SPI READ that accesses message RAM, the SPI
//! interface can block the CAN FSM from reaching RAM -- in the gaps between
//! bytes and between the last byte and nCS rising. Held off longer than
//! T_SPIMAXDLY (5 nominal bit times for a classic base frame, so 10 us at
//! 500 kbit/s), the chip suffers a TX MAB underflow: it sets SERRIF and MODIF
//! and drops into Restricted Operation or Listen Only, where TXREQ is
//! ignored. The TX FIFO fills, nothing drains, and CiTREC stays perfectly
//! clean -- so it looks nothing like a bus fault.
//!
//! Only `receive` issues RAM reads, which is why a transmit-only load does not
//! reproduce this and a receive-then-echo load does.
//!
//! # What it reports
//!
//! Per fault: the latched CiINT flags, CiCON.OPMOD, the TX FIFO's CiFIFOSTA
//! and TXREQ, and CiTREC -- i.e. the full signature, so it can be compared
//! against the table in the driver docs.
//!
//! Then it recovers, rotating through four ladders and timing each:
//!
//! 1. clear the latched flags only        -- expected to never work
//! 2. `recover_system_error`              -- expected to always work, cheaply
//! 3. `reset_fifo` then re-request Normal -- works, but discards queued frames
//! 4. full Configuration-mode cycle       -- works, and is the expensive one
//!
//! Ladder 1 failing while 2 succeeds is the evidence that the operation mode,
//! not the interrupt flags, is what is wrong. Ladders 2 and 4 differing by
//! roughly two orders of magnitude in microseconds is the argument for
//! putting recovery in a production path.
//!
//! # Wiring
//!
//! **This one needs the CAN bus wired**, unlike most binaries here —
//! transceivers and termination, exactly as `multinode` needs. It cannot use
//! internal loopback: DS20005678E Figure 2-1 shows the System Error
//! transition leaving the *Normal* modes, and internal loopback is a *Debug*
//! mode. A loopback run might never reproduce the fault, and would prove
//! nothing either way.
//!
//! `Normal20` also matches the conditions the fault was reported under:
//! classic CAN at 500 kbit/s, where T_SPIMAXDLY is at its tightest (5 nominal
//! bit times, 10 us).
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Instant, Timer};
use embedded_can::{Frame as _, StandardId};
use log::{error, info, warn};
use mcp251xfd::{
    CiInt, Fifo, FifoLayout, Filter, FilterMatch, Frame, MCP251xFdAsync, OperationMode, PayloadSize,
};
use panic_halt as _;

const TX: Fifo = Fifo::F1;
const RX: Fifo = Fifo::F2;

const LAYOUT: FifoLayout =
    FifoLayout::new()
        .tx_fifo(TX, PayloadSize::B8, 8)
        .rx_fifo(RX, PayloadSize::B8, 8);

/// Classic CAN on the real bus. Must be a Normal mode — see the wiring note.
const MODE: OperationMode = OperationMode::Normal20;

/// Where recovery returns to. Figure 2-1 makes Restricted Operation and
/// Listen Only exit directly to the Normal modes, so this needs no
/// Configuration-mode round trip.
const RECOVER_TO: OperationMode = MODE;

/// The two modes a system error parks the chip in (`CiCON.SERR2LOM` picks
/// which). Neither is reachable any other way in this binary, so seeing
/// either one *is* the fault.
fn is_stalled(mode: OperationMode) -> bool {
    matches!(
        mode,
        OperationMode::RestrictedOperation | OperationMode::ListenOnly
    )
}

type Can = MCP251xFdAsync<common::Device>;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Ladder {
    ClearFlagsOnly,
    RecoverSystemError,
    ResetFifoThenMode,
    FullConfigCycle,
}

impl Ladder {
    const ALL: [Ladder; 4] = [
        Ladder::ClearFlagsOnly,
        Ladder::RecoverSystemError,
        Ladder::ResetFifoThenMode,
        Ladder::FullConfigCycle,
    ];

    fn name(self) -> &'static str {
        match self {
            Ladder::ClearFlagsOnly => "clear-flags-only",
            Ladder::RecoverSystemError => "recover_system_error",
            Ladder::ResetFifoThenMode => "reset_fifo+mode",
            Ladder::FullConfigCycle => "full-config-cycle",
        }
    }
}

async fn run_ladder(can: &mut Can, ladder: Ladder) -> Result<(), common::CanError> {
    match ladder {
        Ladder::ClearFlagsOnly => {
            let flags = can.interrupt_flags().await?;
            can.clear_interrupts(flags).await?;
        }
        Ladder::RecoverSystemError => {
            can.recover_system_error(RECOVER_TO, &mut Delay).await?;
        }
        Ladder::ResetFifoThenMode => {
            can.reset_fifo(TX).await?;
            can.recover_system_error(RECOVER_TO, &mut Delay).await?;
        }
        Ladder::FullConfigCycle => {
            can.set_mode(OperationMode::Configuration, &mut Delay)
                .await?;
            can.apply_layout(&LAYOUT).await?;
            can.set_filter(Filter::F0, FilterMatch::accept_all(), RX)
                .await?;
            can.set_mode(MODE, &mut Delay).await?;
        }
    }
    Ok(())
}

/// True once the chip is transmitting again: out of the stalled modes, with
/// the TX FIFO showing room.
async fn is_recovered(can: &mut Can) -> Result<bool, common::CanError> {
    if is_stalled(can.control_register().await?.op_mode()) {
        return Ok(false);
    }
    Ok(can.fifo_status(TX).await?.not_full_or_not_empty())
}

async fn report_signature(can: &mut Can, faults: u32) -> Result<(), common::CanError> {
    let int = can.interrupt_flags().await?;
    let con = can.control_register().await?;
    let sta = can.fifo_status(TX).await?;
    let fcon = can.fifo_config(TX).await?;
    let trec = can.error_counters().await?;
    warn!(
        "fault {faults}: CiINT={:#010X} serrif={} modif={} ivmif={} | OPMOD={:?}",
        int.0,
        int.serrif(),
        int.modif(),
        int.ivmif(),
        con.op_mode(),
    );
    warn!(
        "fault {faults}: FIFOSTA={:#010X} room={} txreq={} | TEC={} REC={} bo={} bp={}",
        sta.0,
        sta.not_full_or_not_empty(),
        fcon.txreq(),
        trec.tec(),
        trec.rec(),
        trec.tx_bus_off(),
        trec.tx_error_passive(),
    );
    Ok(())
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut chips: [Can; 10] = devices.map(MCP251xFdAsync::new);

    for (i, can) in chips.iter_mut().enumerate() {
        common::ensure_configuration(can).await;
        if let Err(e) = setup(can).await {
            error!("chip {i}: setup failed: {e:?}");
        }
    }

    let frame = Frame::new(StandardId::new(0x100).unwrap(), &[0xAA; 8]).unwrap();
    let mut cycles: u32 = 0;
    let mut faults: u32 = 0;
    let mut ladder_index = 0usize;

    loop {
        cycles += 1;
        // The receive-then-echo path: transmit, then read the frame back out
        // of RAM. The read is the half that trips the erratum.
        for can in chips.iter_mut() {
            let _ = can.transmit(TX, &frame).await;
        }
        for can in chips.iter_mut() {
            // Drain whatever arrived. Each `receive` is two RAM READs.
            while can.receive(RX).await.is_ok() {}
        }

        for (i, can) in chips.iter_mut().enumerate() {
            let stalled = match can.control_register().await {
                Ok(con) => is_stalled(con.op_mode()),
                Err(e) => {
                    error!("chip {i}: mode read failed: {e:?}");
                    continue;
                }
            };
            if !stalled {
                continue;
            }

            faults += 1;
            if let Err(e) = report_signature(can, faults).await {
                error!("chip {i}: signature read failed: {e:?}");
            }

            let ladder = Ladder::ALL[ladder_index % Ladder::ALL.len()];
            ladder_index += 1;

            let t0 = Instant::now();
            let outcome = run_ladder(can, ladder).await;
            let elapsed = t0.elapsed().as_micros();

            match outcome {
                Ok(()) => match is_recovered(can).await {
                    Ok(true) => info!(
                        "fault {faults} chip {i}: ladder {} RECOVERED in {elapsed} us",
                        ladder.name()
                    ),
                    Ok(false) => {
                        warn!(
                            "fault {faults} chip {i}: ladder {} DID NOT RECOVER ({elapsed} us)",
                            ladder.name()
                        );
                        // Fall back to the ladder known to work so the sweep
                        // can continue.
                        let _ = run_ladder(can, Ladder::FullConfigCycle).await;
                    }
                    Err(e) => error!("chip {i}: recovery check failed: {e:?}"),
                },
                Err(e) => error!("chip {i}: ladder {} errored: {e:?}", ladder.name()),
            }
        }

        if cycles.is_multiple_of(500) {
            info!("{cycles} cycles, {faults} faults");
        }
        // 500 Hz, matching the load that reproduced this in the field.
        Timer::after_micros(2000).await;
    }
}

async fn setup(can: &mut Can) -> Result<(), common::CanError> {
    can.init(&common::CAN_CONFIG, &mut Delay).await?;
    can.apply_layout(&LAYOUT).await?;
    can.set_filter(Filter::F0, FilterMatch::accept_all(), RX)
        .await?;
    can.configure_interrupts(CiInt(0).with_serrie(true).with_ivmie(true).with_modie(true))
        .await?;
    can.set_mode(MODE, &mut Delay).await?;
    Ok(())
}
