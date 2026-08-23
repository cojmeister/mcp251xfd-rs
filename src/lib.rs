//! Driver for the Microchip MCP2517FD / MCP2518FD / MCP251863 external SPI
//! CAN FD controllers.
//!
//! See the crate README for a usage example. The driver is generic over
//! [`embedded_hal::spi::SpiDevice`] (and its async twin behind the `async`
//! feature) and never manages the chip-select line itself.
#![cfg_attr(not(test), no_std)]
#![deny(missing_docs)]

mod bus;
pub mod config;
mod error;
pub mod frame;
pub mod registers;

pub use config::{ClockConfig, Config, DataBitTiming, FilterMatch, NominalBitTiming, max_spi_hz};
pub use error::{ConfigError, Error};
pub use frame::{FdFrame, Frame, FrameFlags, RxFrame};
pub use registers::ram::FifoLayout;
pub use registers::{Fifo, Filter, OperationMode, PayloadSize, Variant};
