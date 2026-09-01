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
//! # SPI mode
//!
//! The MCP251xFD requires **SPI mode (0,0)** — clock idle low, data sampled
//! on the first (rising) edge. Set it explicitly. It happens to match the
//! default `Config` on some HALs (`embassy-rp` among them), so an
//! unconfigured bus will appear to work right up until you move to a HAL
//! whose default differs, and then fail in a way that reads like a wiring
//! fault.
//!
//! # Choosing blocking or async
//!
//! Both APIs are generated from one source, so they are feature-identical and
//! the choice is purely about how the SPI transfer should wait.
//!
//! Prefer **async** when the core issuing SPI also runs other work: the await
//! points let the executor do that work while a transfer is in flight.
//!
//! Prefer **blocking** when the core is dedicated to CAN, and specifically on
//! **any target where DMA completion interrupts are serviced on a different
//! core than the one that issued the transfer**. There the async path exports
//! its completion cost to a core that did not ask for it, at a phase that
//! core cannot predict.
//!
//! The RP2040 under `embassy-rp` is the common case, and it is surprising
//! enough to name. `embassy_rp::init` calls `dma::init`, which enables
//! `DMA_IRQ_0` in the calling core's NVIC — and `init` runs on core 0. The
//! handler loops over all twelve DMA channels on every completion.
//! `embassy-rp` does not use `DMA_IRQ_1`, so there is no second line to give
//! core 1. Every SPI DMA completion raised by core 1 is therefore serviced on
//! core 0, at arbitrary phase relative to core 0's own timing. A project
//! running this driver on core 1 measured 23 late cycle starts per ten
//! minutes on core 0 that a single-core build did not have.
//!
//! For a dedicated core the blocking driver is simply better: it removes
//! *this driver's own* cross-core interrupt — the SPI DMA completion — and
//! frees two DMA channels, and for the 3-18 byte transfers this driver
//! issues, DMA setup overhead dominates the transfer anyway. If the core has
//! slack against its deadline, busy-waiting on the SPI FIFO costs it nothing
//! it needs.
//!
//! That is not the same as isolating the core from cross-core interrupts
//! generally. Other traffic can still land there — on the RP2040 under
//! `embassy-rp`, a `Ticker`/`Timer` used to pace a task still wakes via
//! `TIMER_IRQ_0`, which `embassy_rp::init` enables on whichever core called
//! it (core 0), regardless of which core the timed task runs on.
//!
//! There is a correctness dimension too — see the next section.
//!
//! # Known hardware anomalies
//!
//! ## MCP2517FD: transmit stalls under a receive-heavy load
//!
//! On the **MCP2517FD only** (DS80000792D item 1; the MCP2518FD and
//! MCP251863 errata carry no equivalent), the SPI interface can block the CAN
//! FSM from reaching RAM during an SPI **READ** that accesses message RAM —
//! in the gaps between bytes, and between the last byte and nCS rising. Held
//! off for longer than T_SPIMAXDLY, the chip suffers a TX MAB underflow.
//!
//! The signature is distinctive, and it looks nothing like a bus fault:
//!
//! | Where | What you see |
//! |---|---|
//! | `CiINT` | `SERRIF` (12) latched, usually with `MODIF` (3) and `IVMIF` (15) |
//! | `CiCON.OPMOD` | Restricted Operation, or Listen Only if `SERR2LOM` is set |
//! | TX FIFO | reports full and stops draining — both modes ignore `TXREQ` |
//! | `CiTREC` | completely clean: `TEC` 0, `REC` 0, not bus-off, not error-passive |
//!
//! T_SPIMAXDLY is short. The erratum's worst case for a classic base frame is
//! 5 nominal bit times — 10 us at 500 kbit/s, against roughly 1 us per SPI
//! byte at 7.5 MHz.
//!
//! **Recovery** is [`MCP251xFd::recover_system_error`]: clear the flags and
//! request Normal mode. The chip then retransmits the offending message
//! itself, and the erratum states explicitly that resetting the device is not
//! necessary. Clearing the interrupt flags alone never works — the flags are
//! not what is wrong, the operation mode is.
//!
//! **Avoiding it** means keeping SPI byte gaps and the last-byte-to-nCS gap
//! short. Anything that can stall mid-transaction is a risk: a shared bus
//! whose arbitration can preempt, a DMA completion serviced on another core
//! (see the previous section), or a debugger halt. Only [`MCP251xFd::receive`]
//! issues RAM reads, so a transmit-only workload does not trigger this — one
//! project saw zero faults in 86,901 sustained transmits and then roughly 5.4
//! faults per second once the same load included the receive path.
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
pub use driver::ChipConfig;
pub use driver::Event;
pub use driver::MCP251xFd;
#[cfg(feature = "async")]
pub use driver::MCP251xFdAsync;
pub use error::{ConfigError, Error};
pub use frame::{FdFrame, Frame, FrameFlags, ReceivedFrame, RxFrame};
pub use registers::ram::FifoLayout;
pub use registers::{CiCon, CiDbtCfg, CiFifoCon, CiFifoSta, CiInt, CiNbtCfg, CiTdc, CiTrec};
pub use registers::{Fifo, Filter, OperationMode, PayloadSize, Variant};
