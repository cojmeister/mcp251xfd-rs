//! Async-variant integration tests (compiled only with the `async` feature).
#![cfg(feature = "async")]

use embedded_hal_mock::eh1::spi::{Mock, Transaction};
use mcp251xfd::{Error, Fifo, MCP251xFdAsync, ReceivedFrame};

/// An interrupt pin that is always asserted (returns immediately).
struct ReadyPin;

impl embedded_hal::digital::ErrorType for ReadyPin {
    type Error = core::convert::Infallible;
}

impl embedded_hal_async::digital::Wait for ReadyPin {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn r32(addr: u16, val: u32) -> Vec<Transaction<u8>> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x30 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::read_vec(val.to_le_bytes().to_vec()),
        Transaction::transaction_end(),
    ]
}

fn rram(addr: u16, data: &[u8]) -> Vec<Transaction<u8>> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x30 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::read_vec(data.to_vec()),
        Transaction::transaction_end(),
    ]
}

fn w8(addr: u16, val: u8) -> Vec<Transaction<u8>> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x20 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::write_vec(vec![val]),
        Transaction::transaction_end(),
    ]
}

#[tokio::test]
async fn wait_rx_polls_until_frame_arrives() {
    let mut e = Vec::new();
    e.extend(r32(0x06C, 0x0000_0000)); // empty -> waits on the pin once
    e.extend(r32(0x06C, 0x0000_0001)); // now a frame is there
    e.extend(r32(0x070, 0x0000_0000));
    e.extend(rram(
        0x400,
        &[0x23, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00],
    ));
    e.extend(rram(0x408, &[0xBE, 0xEF, 0x00, 0x00]));
    e.extend(w8(0x069, 0x01));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFdAsync::new(&mut spi);
    let rx = can.wait_rx(Fifo::F2, &mut ReadyPin).await.unwrap();
    match rx.frame {
        ReceivedFrame::Classic(f) => assert_eq!(f.data(), &[0xBE, 0xEF]),
        ReceivedFrame::Fd(_) => panic!("expected classic"),
    }
    spi.done();
}

#[tokio::test]
async fn async_receive_empty_errors() {
    let mut e = Vec::new();
    e.extend(r32(0x06C, 0x0000_0000));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFdAsync::new(&mut spi);
    assert!(matches!(
        can.receive(Fifo::F2).await,
        Err(Error::RxFifoEmpty)
    ));
    spi.done();
}
