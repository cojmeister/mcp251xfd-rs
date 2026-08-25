//! Two chips on the shared CAN bus: chip 0 transmits, chip 1 receives.
//! Classic at the nominal rate, then FD with bit-rate switch.
//!
//! Needs transceivers and a terminated bus. Output leaves over USB CDC-ACM
//! serial: open the board's COM port in any terminal. The test repeats every
//! 5 s.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use embedded_can::{Frame as _, StandardId};
use log::{error, info};
use mcp251xfd::{
    FdFrame, Fifo, FifoLayout, Filter, FilterMatch, Frame, FrameFlags, MCP251xFdAsync,
    OperationMode, PayloadSize, ReceivedFrame,
};
use panic_halt as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B64, 8);

/// Brings one chip up to `NormalFd` with a wide-open RX filter.
async fn bring_up(can: &mut MCP251xFdAsync<common::Device>) -> Result<(), common::CanError> {
    common::ensure_configuration(can).await;
    can.init(&common::CAN_CONFIG, &mut Delay).await?;
    can.apply_layout(&LAYOUT).await?;
    can.set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2)
        .await?;
    can.set_mode(OperationMode::NormalFd, &mut Delay).await?;
    Ok(())
}

/// On a timeout, reads the transmitter's error counters: a TEC pegged high or
/// a bus-off/error-passive flag is the definitive "no transceiver / no
/// termination" answer, and distinguishes it from a receive-side problem.
async fn report_bus_health(a: &mut MCP251xFdAsync<common::Device>) {
    match a.error_counters().await {
        Ok(trec) => error!("  bus health on A: {trec:?}"),
        Err(e) => error!("  bus health on A unreadable: {e:?}"),
    }
}

async fn run_test(
    a: &mut MCP251xFdAsync<common::Device>,
    b: &mut MCP251xFdAsync<common::Device>,
) -> Result<(), common::CanError> {
    // Classic 500 kbit/s.
    let tx = Frame::new(StandardId::new(0x100).unwrap(), &[0xDE, 0xAD]).unwrap();
    a.transmit(Fifo::F1, &tx).await?;
    match common::recv_timeout(b, Fifo::F2).await {
        Some(rx) => match rx.frame {
            ReceivedFrame::Classic(f) if f.data() == [0xDE, 0xAD] => info!("classic A->B OK"),
            _ => error!("classic A->B wrong frame"),
        },
        None => {
            error!("classic A->B TIMEOUT (transceivers? termination?)");
            report_bus_health(a).await;
        }
    }

    // FD 500k/2M with BRS.
    let payload: [u8; 48] = core::array::from_fn(|i| i as u8);
    let tx = FdFrame::new(
        StandardId::new(0x200).unwrap(),
        &payload,
        FrameFlags {
            brs: true,
            esi: false,
        },
    )
    .unwrap();
    a.transmit_fd(Fifo::F1, &tx).await?;
    match common::recv_timeout(b, Fifo::F2).await {
        Some(rx) => match rx.frame {
            ReceivedFrame::Fd(f) if f.data() == payload => info!("FD-48 BRS A->B OK"),
            _ => error!("FD A->B wrong frame"),
        },
        None => {
            error!("FD A->B TIMEOUT");
            report_bus_health(a).await;
        }
    }
    Ok(())
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut devices = devices.into_iter();
    let mut a = MCP251xFdAsync::new(devices.next().unwrap());
    let mut b = MCP251xFdAsync::new(devices.next().unwrap());

    loop {
        info!("--- chip2chip: chip 0 -> chip 1 over the real bus ---");
        let mut up = true;
        for (name, can) in [("A", &mut a), ("B", &mut b)] {
            if let Err(e) = bring_up(can).await {
                error!("chip {name}: bring-up FAILED: {e:?}");
                up = false;
            }
        }
        if up {
            if let Err(e) = run_test(&mut a, &mut b).await {
                error!("chip2chip aborted: {e:?}");
            } else {
                info!("chip2chip complete");
            }
        }
        Timer::after_secs(5).await;
    }
}
