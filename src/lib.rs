//! Driver for the Microchip MCP2517FD / MCP2518FD / MCP251863 external SPI
//! CAN FD controllers.
//!
//! - Generic over [`embedded_hal::spi::SpiDevice`] — works on shared SPI
//!   buses; the driver never touches chip select.
//! - The `async` feature adds [`MCP251xFdAsync`] over
//!   [`embedded_hal_async::spi::SpiDevice`] (embassy-compatible), generated
//!   from the same source. Sync and async coexist in one binary.
//! - Classic CAN 2.0 ([`Frame`], with [`embedded_can::Frame`] interop) and
//!   CAN FD up to 64 bytes ([`FdFrame`]).
//! - Compile-time message-RAM budgeting: build a [`FifoLayout`] in a
//!   `const` and overflowing the 2 KiB RAM is a compile error.
//!
//! # SPI clock limit (silicon erratum)
//!
//! RAM reads corrupt above `0.85 * SYSCLK / 2` — 17 MHz at the recommended
//! 40 MHz SYSCLK. The driver cannot observe your bus clock; size it with
//! [`max_spi_hz`]. [`MCP251xFd::init`] verifies communication with a RAM
//! echo test and fails with
//! [`Error::CommunicationCheckFailed`] on an over-clocked bus.
//!
//! # Example (sync; the async API is identical plus `.await`)
//!
//! ```ignore
//! # // Not compiled: requires a real SpiDevice + DelayNs implementation.
//! use mcp251xfd::{
//!     ClockConfig, Config, DataBitTiming, Fifo, FifoLayout, Filter,
//!     FilterMatch, Frame, MCP251xFd, NominalBitTiming, OperationMode,
//!     PayloadSize,
//! };
//! use embedded_can::{Frame as _, StandardId};
//!
//! const LAYOUT: FifoLayout = FifoLayout::new()
//!     .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
//!     .rx_fifo(Fifo::F2, PayloadSize::B64, 8);
//!
//! let mut can = MCP251xFd::new(spi_device);
//! let variant = can.init(
//!     &Config {
//!         clock: ClockConfig::MHZ40,
//!         nominal: NominalBitTiming::KBPS500_40MHZ,
//!         data: Some(DataBitTiming::MBPS2_40MHZ),
//!     },
//!     &mut delay,
//! )?;
//! can.apply_layout(&LAYOUT)?;
//! can.set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2)?;
//! can.set_mode(OperationMode::NormalFd, &mut delay)?;
//!
//! let frame = Frame::new(StandardId::new(0x123).unwrap(), &[1, 2, 3, 4]).unwrap();
//! can.transmit(Fifo::F1, &frame)?;
//! ```
#![cfg_attr(not(test), no_std)]
#![deny(missing_docs)]

mod bus;
pub mod config;
mod driver;
mod error;
pub mod frame;
pub mod registers;

pub use config::{ClockConfig, Config, DataBitTiming, FilterMatch, NominalBitTiming, max_spi_hz};
pub use driver::Event;
pub use driver::MCP251xFd;
#[cfg(feature = "async")]
pub use driver::MCP251xFdAsync;
pub use error::{ConfigError, Error};
pub use frame::{FdFrame, Frame, FrameFlags, ReceivedFrame, RxFrame};
pub use registers::ram::FifoLayout;
pub use registers::{CiFifoSta, CiInt, CiTrec};
pub use registers::{Fifo, Filter, OperationMode, PayloadSize, Variant};
