//! Verifies classic **remote (RTR) frames**.
//!
//! Needs SPI wiring only -- everything runs through internal loopback.
//!
//! This exists for one specific regression. A remote frame carries no data on
//! the wire, but the chip's message object still reserves the payload slot, so
//! whatever occupied that RAM previously is still sitting there. The driver
//! therefore skips the payload read for remote frames and leaves the buffer
//! zeroed (commit "skip payload read for remote frames so RTR data stays
//! zeroed"). That fix had never run on hardware.
//!
//! The trick to catching a regression is the **RX FIFO of depth 1**: every
//! frame reuses the same RAM slot, so a remote frame always lands on top of
//! the previous data frame's payload. If the driver read the payload anyway,
//! the received frame's backing array would hold the stale bytes.
//!
//! `Frame::data()` returns `&[]` for a remote frame, so a stale payload is
//! invisible through it. The comparison below is against a locally built
//! `Frame::new_remote`, and `Frame`'s derived `PartialEq` covers the whole
//! 8-byte array -- which is what makes the stale bytes detectable.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use embedded_can::{ExtendedId, Frame as _, Id, StandardId};
use log::{error, info};
use mcp251xfd::{
    Fifo, FifoLayout, FilterMatch, Frame, MCP251xFdAsync, OperationMode, PayloadSize, ReceivedFrame,
};
use panic_halt as _;

/// RX depth **1** on purpose: every frame reuses one RAM slot, so a remote
/// frame always follows a data frame into the same bytes.
const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B8, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B8, 1);

/// A payload chosen to be conspicuous if it leaks into a remote frame.
const POISON: [u8; 8] = [0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE];

type Can = MCP251xFdAsync<common::Device>;

async fn drain(can: &mut Can) {
    for _ in 0..16 {
        if can.receive(Fifo::F2).await.is_err() {
            return;
        }
    }
}

/// Sends one frame and returns the classic frame that came back.
async fn echo(can: &mut Can, f: &Frame) -> Option<Frame> {
    if let Err(e) = can.transmit(Fifo::F1, f).await {
        error!("    transmit failed: {e:?}");
        return None;
    }
    match common::recv_timeout(can, Fifo::F2).await {
        Some(rx) => match rx.frame {
            ReceivedFrame::Classic(g) => Some(g),
            ReceivedFrame::Fd(_) => {
                error!("    remote frame came back as FD");
                None
            }
        },
        None => None,
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut chips = devices.map(MCP251xFdAsync::new);

    loop {
        info!("--- remote: classic RTR frames ---");
        let can = &mut chips[0];
        common::ensure_configuration(can).await;
        if let Err(e) = can.init(&common::CAN_CONFIG, &mut Delay).await {
            error!("chip 0: init FAILED: {e:?}");
            Timer::after_secs(5).await;
            continue;
        }
        if let Err(e) = common::arm_filter(can, &LAYOUT, FilterMatch::accept_all()).await {
            error!("  arm_filter failed: {e:?}");
            Timer::after_secs(5).await;
            continue;
        }
        drain(can).await;

        let mut pass = 0usize;
        let mut fail = 0usize;
        let id = Id::Standard(StandardId::new(0x321).unwrap());

        // 1. Every DLC: a remote frame must come back remote, with the DLC
        // preserved and no data. Each is preceded by a data frame carrying
        // POISON into the one RX slot the remote frame will reuse.
        for dlc in 0..=8usize {
            let poison = Frame::new(id, &POISON).unwrap();
            if echo(can, &poison).await.is_none() {
                error!("  dlc {dlc}: poison frame did not echo");
                fail += 1;
                continue;
            }

            let want = Frame::new_remote(id, dlc).unwrap();
            match echo(can, &want).await {
                None => {
                    error!("  dlc {dlc}: remote frame TIMEOUT");
                    fail += 1;
                }
                Some(got) => {
                    if !got.is_remote_frame() {
                        error!("  dlc {dlc}: RTR bit lost -- came back as a data frame");
                        fail += 1;
                    } else if got.dlc() != dlc {
                        error!("  dlc {dlc}: DLC changed to {}", got.dlc());
                        fail += 1;
                    } else if !got.data().is_empty() {
                        error!(
                            "  dlc {dlc}: data() should be empty, got {} bytes",
                            got.data().len()
                        );
                        fail += 1;
                    } else if got != want {
                        // Whole-array comparison: this is the stale-payload
                        // case that data() cannot show.
                        error!("  dlc {dlc}: backing buffer not zeroed -- stale payload leaked");
                        fail += 1;
                    } else {
                        info!("  dlc {dlc}: remote OK, no data, buffer clean");
                        pass += 1;
                    }
                }
            }
        }

        // 2. A data frame straight after a remote frame must still work, so
        // the remote path leaves no state behind.
        let after = Frame::new(id, &[1, 2, 3]).unwrap();
        match echo(can, &after).await {
            Some(got) if !got.is_remote_frame() && got.data() == [1, 2, 3] => {
                info!("  data frame after remote: OK");
                pass += 1;
            }
            Some(got) => {
                error!(
                    "  data frame after remote: WRONG (remote={}, {} bytes)",
                    got.is_remote_frame(),
                    got.data().len()
                );
                fail += 1;
            }
            None => {
                error!("  data frame after remote: TIMEOUT");
                fail += 1;
            }
        }

        // 3. Remote frames with a 29-bit identifier: RTR and IDE are adjacent
        // bits in T1 (5 and 4), so confirm neither disturbs the other.
        let ext_id = Id::Extended(ExtendedId::new(0x1234_5678).unwrap());
        let poison = Frame::new(ext_id, &POISON).unwrap();
        let _ = echo(can, &poison).await;
        let want = Frame::new_remote(ext_id, 8).unwrap();
        match echo(can, &want).await {
            Some(got) if got.is_remote_frame() && got.id() == ext_id && got == want => {
                info!("  extended remote 0x12345678: OK");
                pass += 1;
            }
            Some(got) => {
                error!(
                    "  extended remote: WRONG (remote={}, id={:?})",
                    got.is_remote_frame(),
                    got.id()
                );
                fail += 1;
            }
            None => {
                error!("  extended remote: TIMEOUT");
                fail += 1;
            }
        }

        // 4. An out-of-range DLC must be refused at construction, not sent.
        if Frame::new_remote(id, 9).is_none() {
            info!("  DLC 9 correctly refused by Frame::new_remote");
            pass += 1;
        } else {
            error!("  DLC 9 was accepted by Frame::new_remote");
            fail += 1;
        }

        let _ = can.set_mode(OperationMode::Configuration, &mut Delay).await;
        if fail == 0 {
            info!("remote: all {pass} checks OK");
        } else {
            error!("remote: {fail} FAILED, {pass} OK");
        }
        Timer::after_secs(5).await;
    }
}
