# mcp251xfd Driver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A published-quality `no_std` Rust driver crate `mcp251xfd` for the Microchip MCP2517FD/MCP2518FD/MCP251863 SPI CAN FD controllers, with sync + async APIs generated from one codebase.

**Architecture:** Three layers — pure-data `registers/` (bitfields, message-object codecs, const-fn RAM planner), `bus.rs` (SPI transactions over `SpiDevice`, sync/async via `maybe-async-cfg`), and `driver.rs` (init recipe, FIFO layout, filters, TX/RX, events). Host tests use `embedded-hal-mock` with byte-exact SPI expectations; hardware examples live in a standalone `examples/rp2040` crate.

**Tech Stack:** Rust edition 2024, `embedded-hal` 1.0, `embedded-hal-async` 1.0 (feature `async`), `embedded-can` 0.4, `maybe-async-cfg`, `embedded-hal-mock` 0.11, `trybuild`, embassy-rp (examples only).

**Spec:** `docs/superpowers/specs/2026-08-19-mcp251xfd-driver-design.md` — read it before starting any task. Register bit layouts in this plan were transcribed from the MCP2518FD datasheet (DS20006027B) and the MCP25XXFD Family Reference Manual (DS20005678E); when in doubt, the datasheet wins.

## Global Constraints

- Crate name `mcp251xfd`; `#![no_std]` (via `#![cfg_attr(not(test), no_std)]`), `#![deny(missing_docs)]`.
- Every public struct, enum, trait, function, method, constant, and **every public field** has a doc comment. Doc comments state units and valid ranges (e.g. `1..=256, stored as value − 1`).
- Zero warnings: every `cargo build`/`test` step in this plan implicitly requires no compiler warnings; `cargo clippy --all-features -- -D warnings` must pass before every commit.
- Features: `default = []`, `async`, `defmt`, `log` — all additive. Sync API always available.
- Dependencies limited to: embedded-hal 1.0, embedded-can 0.4, embedded-hal-async 1.0 (optional), maybe-async-cfg (pinned `=0.2.4`), defmt (optional), log (optional). Dev-deps: embedded-hal-mock, tokio, trybuild.
- The driver never manages CS and never depends on embassy. SPI I/O only via `SpiDevice::transaction` / its convenience methods.
- MSRV 1.85 (edition 2024). Dual license MIT OR Apache-2.0.
- Commit after every task (steps include the commands). Commit messages are short one-liners — no body, no trailers (no `Co-Authored-By`).
- Test commands: run `cargo test` (sync) and `cargo test --all-features` (adds async paths) unless a step says otherwise.

## File Map (what exists when the plan is done)

| File | Responsibility |
|---|---|
| `Cargo.toml` | crate metadata, features, deps |
| `src/lib.rs` | crate docs, lints, module wiring, re-exports |
| `src/error.rs` | `Error<E>`, `ConfigError` |
| `src/registers/mod.rs` | addresses, register bitfield newtypes, `Fifo`, `Filter`, `PayloadSize`, `OperationMode`, `Variant` |
| `src/registers/objects.rs` | ID packing, DLC↔length, TX/RX header words |
| `src/registers/ram.rs` | `FifoLayout` const-fn RAM planner |
| `src/frame.rs` | `Frame`, `FdFrame`, `FrameFlags`, `RxFrame`, `embedded_can::Frame` impl |
| `src/config.rs` | `ClockConfig`, `NominalBitTiming`, `DataBitTiming`, presets, `Config`, `FilterMatch`, `max_spi_hz` |
| `src/bus.rs` | `Bus`/`BusAsync`: SPI opcodes + SFR/RAM transactions |
| `src/driver.rs` | `MCP251xFd`/`MCP251xFdAsync`: init, mode, layout, filters, TX/RX, events |
| `tests/driver.rs`, `tests/async_driver.rs` | mock-SPI integration tests (std allowed here; bus-layer tests live inline in `src/bus.rs` because `Bus` is crate-private) |
| `tests/compile_fail.rs` + `tests/compile_fail/*.rs` | trybuild proof of const RAM overflow check |
| `.github/workflows/ci.yml` | CI with zero-warning policy |
| `examples/rp2040/` | standalone embassy-rp crate: `enumerate`, `loopback`, `chip2chip`, `multinode` bins |
| `README.md`, `LICENSE-MIT`, `LICENSE-APACHE` | publishing collateral |

---

### Task 1: Crate scaffolding and error type

**Files:**
- Modify: `Cargo.toml` (full rewrite)
- Modify: `src/lib.rs` (replace template)
- Create: `src/error.rs`

**Interfaces:**
- Produces: `Error<E>` enum and `ConfigError` enum exactly as written below. Every later task returns `Result<_, Error<SPI::Error>>` from I/O methods and maps SPI errors with `.map_err(Error::Spi)`.

- [x] **Step 1: Rewrite `Cargo.toml`**

```toml
[package]
name = "mcp251xfd"
version = "0.1.0"
edition = "2024"
rust-version = "1.85"
description = "Driver for the Microchip MCP2517FD / MCP2518FD / MCP251863 external SPI CAN FD controllers"
license = "MIT OR Apache-2.0"
repository = "https://github.com/cojmeister/mcp251xfd"
keywords = ["can", "can-fd", "mcp2517fd", "mcp2518fd", "embedded-hal"]
categories = ["embedded", "no-std", "hardware-support"]
readme = "README.md"
# examples/rp2040 is a standalone crate (own [workspace]); it must not be
# auto-discovered as cargo example targets of this package.
autoexamples = false

[dependencies]
embedded-hal = "1.0"
embedded-can = "0.4"
embedded-hal-async = { version = "1.0", optional = true }
maybe-async-cfg = "=0.2.4"
defmt = { version = "0.3", optional = true }
log = { version = "0.4", optional = true }

[dev-dependencies]
embedded-hal-mock = { version = "0.11", features = ["eh1", "embedded-hal-async"] }
tokio = { version = "1", features = ["macros", "rt"] }
trybuild = "1"

[features]
default = []
async = ["dep:embedded-hal-async"]
defmt = ["dep:defmt"]
log = ["dep:log"]

[package.metadata.docs.rs]
all-features = true
```

Note: `repository` assumes the GitHub repo will be `cojmeister/mcp251xfd`; adjust if the actual remote differs when it is created.

- [x] **Step 2: Write the failing test inside `src/error.rs`**

Create `src/error.rs` containing only the test module first:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_is_debug_and_display() {
        let e: Error<()> = Error::TxFifoFull;
        assert_eq!(format!("{e:?}"), "TxFifoFull");
        assert_eq!(format!("{e}"), "TX FIFO is full");
        let e: Error<()> = Error::InvalidConfig(ConfigError::NominalBitTiming);
        assert!(format!("{e}").contains("nominal"));
    }
}
```

- [x] **Step 3: Replace `src/lib.rs` and run the test to verify it fails**

```rust
//! Driver for the Microchip MCP2517FD / MCP2518FD / MCP251863 external SPI
//! CAN FD controllers.
//!
//! See the crate README for a usage example. The driver is generic over
//! [`embedded_hal::spi::SpiDevice`] (and its async twin behind the `async`
//! feature) and never manages the chip-select line itself.
#![cfg_attr(not(test), no_std)]
#![deny(missing_docs)]

mod error;

pub use error::{ConfigError, Error};
```

Run: `cargo test`
Expected: FAIL — `Error` / `ConfigError` not defined.

- [x] **Step 4: Implement the error types above the test module in `src/error.rs`**

```rust
//! Error types returned by the driver.

/// Errors returned by driver operations.
///
/// `E` is the error type of the underlying [`embedded_hal::spi::SpiDevice`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error<E> {
    /// The SPI transaction itself failed.
    Spi(E),
    /// The init-time RAM echo test (write/read `0xAA55AA55`) failed.
    ///
    /// Usually wiring, a wrong CS line, or an SPI clock above the erratum
    /// limit of `0.85 * SYSCLK / 2` (see [`crate::max_spi_hz`]).
    CommunicationCheckFailed,
    /// The oscillator/PLL did not report ready within the timeout (~4 ms).
    ClockNotReady,
    /// The chip did not reach the requested operation mode within the timeout.
    ModeChangeTimeout,
    /// The operation requires Configuration mode, but the chip is not in it.
    NotInConfigMode,
    /// The target TX FIFO has no free slot. Retry after a slot frees up.
    TxFifoFull,
    /// The RX FIFO holds no message.
    RxFifoEmpty,
    /// The requested FIFO layout does not fit the 2048-byte message RAM.
    RamOverflow,
    /// A configuration value is out of range.
    InvalidConfig(ConfigError),
    /// A frame payload length is not a valid CAN (FD) length.
    InvalidPayloadLength,
    /// Waiting on the interrupt pin failed.
    IntPin,
}

/// Which part of a [`crate::config::Config`] was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// Nominal bit timing field out of range (see field docs for ranges).
    NominalBitTiming,
    /// Data bit timing field out of range (see field docs for ranges).
    DataBitTiming,
    /// Clock configuration invalid (e.g. PLL from a non-4 MHz-class input).
    Clock,
}

impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Spi(e) => write!(f, "SPI error: {e:?}"),
            Error::CommunicationCheckFailed => f.write_str("RAM echo test failed (wiring or SPI clock too fast)"),
            Error::ClockNotReady => f.write_str("oscillator/PLL not ready"),
            Error::ModeChangeTimeout => f.write_str("operation mode change timed out"),
            Error::NotInConfigMode => f.write_str("chip is not in Configuration mode"),
            Error::TxFifoFull => f.write_str("TX FIFO is full"),
            Error::RxFifoEmpty => f.write_str("RX FIFO is empty"),
            Error::RamOverflow => f.write_str("FIFO layout exceeds 2048-byte message RAM"),
            Error::InvalidConfig(ConfigError::NominalBitTiming) => f.write_str("invalid nominal bit timing"),
            Error::InvalidConfig(ConfigError::DataBitTiming) => f.write_str("invalid data bit timing"),
            Error::InvalidConfig(ConfigError::Clock) => f.write_str("invalid clock configuration"),
            Error::InvalidPayloadLength => f.write_str("invalid payload length"),
            Error::IntPin => f.write_str("interrupt pin wait failed"),
        }
    }
}

impl<E: core::fmt::Debug> core::error::Error for Error<E> {}
```

- [x] **Step 5: Run tests and lints to verify they pass**

Run: `cargo test && cargo clippy --all-features -- -D warnings && cargo fmt`
Expected: test PASSES, no warnings. (`--all-features` will pull `embedded-hal-async` and `defmt` — both must compile.)

- [x] **Step 6: Commit**

```bash
git add Cargo.toml src/lib.rs src/error.rs
git commit -m "feat: crate scaffolding, features, and error types"
```
(`Cargo.lock` is gitignored — this is a library crate.)

> **✅ Task 1 summary (done 2026-08-19, commit `b38a823`):** Crate renamed to `mcp251xfd` with the full feature/dependency set; `src/lib.rs` (`no_std` + `deny(missing_docs)`) and `src/error.rs` (`Error<E>`, `ConfigError`, Display + `core::error::Error`) landed exactly as specified, TDD order followed. `cargo test`, `--all-features`, and `clippy -D warnings` all clean. Review: spec ✅, quality approved. Known deferred minor: two intra-doc links (`max_spi_hz`, `config::Config`) are forward references that resolve when Tasks 8/14 land.

---

### Task 2: Registers core — addresses, `Fifo`/`Filter`/`PayloadSize`, `OSC`, `CiCON`, `CiINT`, `CiVEC`, `CiTREC`

**Files:**
- Create: `src/registers/mod.rs`
- Modify: `src/lib.rs` (add `pub mod registers;`)

**Interfaces:**
- Produces (used by every later task):
  - `registers::addr` constants: `C1CON: u16 = 0x000`, `C1NBTCFG = 0x004`, `C1DBTCFG = 0x008`, `C1TDC = 0x00C`, `C1TBC = 0x010`, `C1VEC = 0x018`, `C1INT = 0x01C`, `C1TREC = 0x034`, `OSC = 0xE00`, `IOCON = 0xE04`, `ECCCON = 0xE0C`, `RAM_START: u16 = 0x400`, `RAM_SIZE: usize = 2048`; functions `fifo_con(Fifo) -> u16`, `fifo_sta(Fifo) -> u16`, `fifo_ua(Fifo) -> u16`, `flt_con_byte(Filter) -> u16`, `flt_obj(Filter) -> u16`, `flt_mask(Filter) -> u16` (all `const fn`)
  - `Fifo` (newtype over `u8`, consts `F1..=F31`, `new(u8) -> Option<Self>`, `index(self) -> u8`)
  - `Filter` (newtype over `u8`, consts `F0..=F31`, `new(u8) -> Option<Self>`, `index(self) -> u8`)
  - `PayloadSize` enum `B8..B64` with `bytes() -> usize`, `plsize_code() -> u32`, `from_code(u32) -> PayloadSize`
  - `OperationMode` enum with `from_bits(u8)`/`bits()`
  - `Variant` enum { `Mcp2517Fd`, `Mcp2518Fd` }
  - Register newtypes `Osc`, `CiCon`, `CiInt`, `CiVec`, `CiTrec` — each `pub struct X(pub u32)` with the accessors listed in the code below

- [x] **Step 1: Create `src/registers/mod.rs` with the test module first**

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_addresses_match_datasheet() {
        // Reference manual: CiFIFOCONm = 0x05C + 12 * (m - 1).
        assert_eq!(addr::fifo_con(Fifo::F1), 0x05C);
        assert_eq!(addr::fifo_sta(Fifo::F1), 0x060);
        assert_eq!(addr::fifo_ua(Fifo::F1), 0x064);
        assert_eq!(addr::fifo_con(Fifo::F2), 0x068);
        assert_eq!(addr::fifo_con(Fifo::F31), 0x1C4);
    }

    #[test]
    fn filter_addresses_match_datasheet() {
        assert_eq!(addr::flt_con_byte(Filter::F0), 0x1D0);
        assert_eq!(addr::flt_con_byte(Filter::F31), 0x1EF);
        assert_eq!(addr::flt_obj(Filter::F0), 0x1F0);
        assert_eq!(addr::flt_mask(Filter::F0), 0x1F4);
        assert_eq!(addr::flt_obj(Filter::F31), 0x2E8);
    }

    #[test]
    fn osc_round_trip() {
        let osc = Osc(0).with_pll_enabled(true).with_lpmen(true);
        assert!(osc.pll_enabled() && osc.lpmen());
        assert_eq!(osc.0, (1 << 0) | (1 << 3));
        // Ready bits per datasheet: PLLRDY=8, OSCRDY=10, SCLKRDY=12.
        assert!(Osc(1 << 10).osc_ready());
        assert!(Osc(1 << 8).pll_ready());
        assert!(Osc(1 << 12).sclk_ready());
    }

    #[test]
    fn cicon_round_trip() {
        let con = CiCon(0)
            .with_iso_crc_enabled(true)
            .with_req_op_mode(OperationMode::Configuration);
        assert_eq!(con.0, (1 << 5) | (0b100 << 24));
        // OPMOD is read-only, bits 23:21.
        assert_eq!(CiCon(0b100 << 21).op_mode(), OperationMode::Configuration);
        assert_eq!(CiCon(0b000 << 21).op_mode(), OperationMode::NormalFd);
        assert_eq!(CiCon(0b110 << 21).op_mode(), OperationMode::Normal20);
    }

    #[test]
    fn ciint_flags() {
        // CiINT low half: TXIF=0, RXIF=1, MODIF=3, CERRIF=13, RXOVIF=11.
        let i = CiInt(1 << 1 | 1 << 3);
        assert!(i.rxif() && i.modif() && !i.txif());
        assert_eq!(CiInt(0).with_rxie(true).0, 1 << 17);
        assert_eq!(CiInt(0).with_txie(true).0, 1 << 16);
    }

    #[test]
    fn citrec_counters() {
        let t = CiTrec(0x0021_1503);
        assert_eq!(t.rec(), 0x03);
        assert_eq!(t.tec(), 0x15);
        assert!(t.tx_bus_off()); // TXBO = bit 21
    }

    #[test]
    fn payload_size_codes() {
        assert_eq!(PayloadSize::B8.bytes(), 8);
        assert_eq!(PayloadSize::B64.bytes(), 64);
        assert_eq!(PayloadSize::B8.plsize_code(), 0);
        assert_eq!(PayloadSize::B64.plsize_code(), 7);
        assert_eq!(PayloadSize::from_code(5), PayloadSize::B32);
    }
}
```

- [x] **Step 2: Add `pub mod registers;` to `src/lib.rs`, run tests to verify failure**

Run: `cargo test`
Expected: FAIL — nothing in the module is defined yet.

- [x] **Step 3: Implement the module above the tests**

```rust
//! Register-level definitions for the MCP251XFD family.
//!
//! Pure data — no I/O. Bit layouts follow the MCP2518FD datasheet
//! (DS20006027B) and the MCP25XXFD Family Reference Manual (DS20005678E).

/// Register and RAM addresses (12-bit SPI address space).
pub mod addr {
    use super::{Fifo, Filter};

    /// CAN control register.
    pub const C1CON: u16 = 0x000;
    /// Nominal bit timing configuration.
    pub const C1NBTCFG: u16 = 0x004;
    /// Data bit timing configuration.
    pub const C1DBTCFG: u16 = 0x008;
    /// Transmitter delay compensation.
    pub const C1TDC: u16 = 0x00C;
    /// Free-running time base counter.
    pub const C1TBC: u16 = 0x010;
    /// Interrupt vector code register.
    pub const C1VEC: u16 = 0x018;
    /// Interrupt flags (low half) and enables (high half).
    pub const C1INT: u16 = 0x01C;
    /// Transmit/receive error counters and bus state.
    pub const C1TREC: u16 = 0x034;
    /// Oscillator control (MCP2000-class SFR block).
    pub const OSC: u16 = 0xE00;
    /// I/O pin control. Byte access only (MCP2517FD erratum: multi-byte
    /// writes spanning bytes 2-3 clear LAT0/LAT1).
    pub const IOCON: u16 = 0xE04;
    /// ECC control.
    pub const ECCCON: u16 = 0xE0C;
    /// First address of the 2 KiB message RAM.
    pub const RAM_START: u16 = 0x400;
    /// Message RAM size in bytes.
    pub const RAM_SIZE: usize = 2048;

    /// Address of `CiFIFOCONm` for the given FIFO.
    pub const fn fifo_con(fifo: Fifo) -> u16 {
        0x05C + 12 * (fifo.index() as u16 - 1)
    }
    /// Address of `CiFIFOSTAm` for the given FIFO.
    pub const fn fifo_sta(fifo: Fifo) -> u16 {
        fifo_con(fifo) + 4
    }
    /// Address of `CiFIFOUAm` for the given FIFO.
    pub const fn fifo_ua(fifo: Fifo) -> u16 {
        fifo_con(fifo) + 8
    }
    /// Byte address of the filter-control byte for the given filter
    /// (one byte per filter inside `CiFLTCON0..7`).
    pub const fn flt_con_byte(filter: Filter) -> u16 {
        0x1D0 + filter.index() as u16
    }
    /// Address of `CiFLTOBJm` for the given filter.
    pub const fn flt_obj(filter: Filter) -> u16 {
        0x1F0 + 8 * filter.index() as u16
    }
    /// Address of `CiMASKm` for the given filter.
    pub const fn flt_mask(filter: Filter) -> u16 {
        flt_obj(filter) + 4
    }
}

macro_rules! index_consts {
    ($ty:ident, $($name:ident = $n:literal),* $(,)?) => {
        $(
            #[doc = concat!(stringify!($ty), " number ", stringify!($n), ".")]
            pub const $name: Self = Self($n);
        )*
    };
}

/// One of the 31 general-purpose message FIFOs (`1..=31`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Fifo(u8);

impl Fifo {
    index_consts!(Fifo,
        F1 = 1, F2 = 2, F3 = 3, F4 = 4, F5 = 5, F6 = 6, F7 = 7, F8 = 8,
        F9 = 9, F10 = 10, F11 = 11, F12 = 12, F13 = 13, F14 = 14, F15 = 15,
        F16 = 16, F17 = 17, F18 = 18, F19 = 19, F20 = 20, F21 = 21, F22 = 22,
        F23 = 23, F24 = 24, F25 = 25, F26 = 26, F27 = 27, F28 = 28, F29 = 29,
        F30 = 30, F31 = 31,
    );

    /// Creates a FIFO handle. Returns `None` unless `1 <= n <= 31`.
    pub const fn new(n: u8) -> Option<Self> {
        if n >= 1 && n <= 31 { Some(Self(n)) } else { None }
    }

    /// The FIFO number (`1..=31`).
    pub const fn index(self) -> u8 {
        self.0
    }
}

/// One of the 32 acceptance filters (`0..=31`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Filter(u8);

impl Filter {
    index_consts!(Filter,
        F0 = 0, F1 = 1, F2 = 2, F3 = 3, F4 = 4, F5 = 5, F6 = 6, F7 = 7,
        F8 = 8, F9 = 9, F10 = 10, F11 = 11, F12 = 12, F13 = 13, F14 = 14,
        F15 = 15, F16 = 16, F17 = 17, F18 = 18, F19 = 19, F20 = 20, F21 = 21,
        F22 = 22, F23 = 23, F24 = 24, F25 = 25, F26 = 26, F27 = 27, F28 = 28,
        F29 = 29, F30 = 30, F31 = 31,
    );

    /// Creates a filter handle. Returns `None` unless `n <= 31`.
    pub const fn new(n: u8) -> Option<Self> {
        if n <= 31 { Some(Self(n)) } else { None }
    }

    /// The filter number (`0..=31`).
    pub const fn index(self) -> u8 {
        self.0
    }
}

/// Payload size of a FIFO element (CAN FD DLC steps).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PayloadSize {
    /// 8 bytes.
    B8,
    /// 12 bytes.
    B12,
    /// 16 bytes.
    B16,
    /// 20 bytes.
    B20,
    /// 24 bytes.
    B24,
    /// 32 bytes.
    B32,
    /// 48 bytes.
    B48,
    /// 64 bytes.
    B64,
}

impl PayloadSize {
    /// Payload size in bytes.
    pub const fn bytes(self) -> usize {
        match self {
            Self::B8 => 8,
            Self::B12 => 12,
            Self::B16 => 16,
            Self::B20 => 20,
            Self::B24 => 24,
            Self::B32 => 32,
            Self::B48 => 48,
            Self::B64 => 64,
        }
    }

    /// The 3-bit `PLSIZE` register encoding (`0..=7`).
    pub const fn plsize_code(self) -> u32 {
        match self {
            Self::B8 => 0,
            Self::B12 => 1,
            Self::B16 => 2,
            Self::B20 => 3,
            Self::B24 => 4,
            Self::B32 => 5,
            Self::B48 => 6,
            Self::B64 => 7,
        }
    }

    /// Inverse of [`Self::plsize_code`]. Codes above 7 saturate to `B64`.
    pub const fn from_code(code: u32) -> Self {
        match code {
            0 => Self::B8,
            1 => Self::B12,
            2 => Self::B16,
            3 => Self::B20,
            4 => Self::B24,
            5 => Self::B32,
            6 => Self::B48,
            _ => Self::B64,
        }
    }
}

/// CAN controller operation modes (`CiCON.OPMOD` / `REQOP`, 3 bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OperationMode {
    /// Mixed CAN FD / classic frames (mode 0).
    NormalFd,
    /// Sleep (mode 1).
    Sleep,
    /// Internal loopback — TX internally routed to RX, nothing on the bus
    /// (mode 2).
    InternalLoopback,
    /// Listen-only (mode 3).
    ListenOnly,
    /// Configuration mode — required for layout/timing changes (mode 4).
    Configuration,
    /// External loopback (mode 5).
    ExternalLoopback,
    /// Classic CAN 2.0 only; FD frames cause errors (mode 6).
    Normal20,
    /// Restricted operation (mode 7).
    RestrictedOperation,
}

impl OperationMode {
    /// The 3-bit register encoding.
    pub const fn bits(self) -> u8 {
        match self {
            Self::NormalFd => 0,
            Self::Sleep => 1,
            Self::InternalLoopback => 2,
            Self::ListenOnly => 3,
            Self::Configuration => 4,
            Self::ExternalLoopback => 5,
            Self::Normal20 => 6,
            Self::RestrictedOperation => 7,
        }
    }

    /// Decodes the 3-bit register encoding (only the low 3 bits are used).
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0b111 {
            0 => Self::NormalFd,
            1 => Self::Sleep,
            2 => Self::InternalLoopback,
            3 => Self::ListenOnly,
            4 => Self::Configuration,
            5 => Self::ExternalLoopback,
            6 => Self::Normal20,
            _ => Self::RestrictedOperation,
        }
    }
}

/// Detected chip variant (see the spec: detection via `OSC.LPMEN`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Variant {
    /// MCP2517FD: 7-bit TX sequence numbers, Sleep mode only.
    Mcp2517Fd,
    /// MCP2518FD or MCP251863 (same die): 23-bit sequence numbers, LPM.
    Mcp2518Fd,
}

impl Variant {
    /// Mask applied to TX sequence numbers for this variant.
    pub const fn seq_mask(self) -> u32 {
        match self {
            Self::Mcp2517Fd => 0x7F,
            Self::Mcp2518Fd => 0x7F_FFFF,
        }
    }
}

macro_rules! bit {
    ($get:ident, $set:ident, $bit:literal, $doc:literal) => {
        #[doc = concat!("Reads ", $doc, " (bit ", stringify!($bit), ").")]
        pub const fn $get(self) -> bool {
            self.0 & (1 << $bit) != 0
        }
        #[doc = concat!("Sets ", $doc, " (bit ", stringify!($bit), ").")]
        pub const fn $set(self, v: bool) -> Self {
            if v { Self(self.0 | (1 << $bit)) } else { Self(self.0 & !(1 << $bit)) }
        }
    };
}

/// `OSC` — oscillator control register (0xE00). Datasheet §5.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Osc(pub u32);

impl Osc {
    bit!(pll_enabled, with_pll_enabled, 0, "`PLLEN` (10x PLL from 4 MHz-class input)");
    bit!(osc_disabled, with_osc_disabled, 2, "`OSCDIS` (clock off, sleep)");
    bit!(lpmen, with_lpmen, 3, "`LPMEN` (Low-Power Mode; writable on MCP2518FD only)");
    bit!(sclk_div2, with_sclk_div2, 4, "`SCLKDIV` (divide SYSCLK by 2)");
    bit!(pll_ready, with_pll_ready, 8, "`PLLRDY` (read-only status)");
    bit!(osc_ready, with_osc_ready, 10, "`OSCRDY` (read-only status)");
    bit!(sclk_ready, with_sclk_ready, 12, "`SCLKRDY` (read-only status)");

    /// Sets `CLKODIV` (bits 6:5): CLKO pin divider code (0=/1, 1=/2, 2=/4,
    /// 3=/10 — the power-on default).
    pub const fn with_clko_div(self, code: u32) -> Self {
        Self((self.0 & !(0b11 << 5)) | ((code & 0b11) << 5))
    }
}

/// `CiCON` — CAN control register (0x000). Datasheet §3.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiCon(pub u32);

impl CiCon {
    bit!(iso_crc_enabled, with_iso_crc_enabled, 5, "`ISOCRCEN` (ISO 11898-1:2015 CRC)");
    bit!(protocol_exception_disabled, with_protocol_exception_disabled, 6, "`PXEDIS`");
    bit!(brs_disabled, with_brs_disabled, 12, "`BRSDIS` (ignore BRS on TX)");
    bit!(restrict_retx, with_restrict_retx, 16, "`RTXAT` (honor per-FIFO retransmission attempts)");
    bit!(store_tef, with_store_tef, 19, "`STEF` (transmit event FIFO enable)");
    bit!(txq_enabled, with_txq_enabled, 20, "`TXQEN`");
    bit!(abort_all, with_abort_all, 27, "`ABAT` (request abort of all pending TX)");

    /// Current operation mode, `OPMOD` (read-only, bits 23:21).
    pub const fn op_mode(self) -> OperationMode {
        OperationMode::from_bits(((self.0 >> 21) & 0b111) as u8)
    }

    /// Sets the requested operation mode, `REQOP` (bits 26:24).
    pub const fn with_req_op_mode(self, mode: OperationMode) -> Self {
        Self((self.0 & !(0b111 << 24)) | ((mode.bits() as u32) << 24))
    }
}

/// `CiINT` — interrupt flags (bits 15:0) and enables (bits 31:16) (0x01C).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiInt(pub u32);

impl CiInt {
    bit!(txif, with_txif, 0, "`TXIF` (any TX FIFO interrupt pending)");
    bit!(rxif, with_rxif, 1, "`RXIF` (any RX FIFO interrupt pending)");
    bit!(modif, with_modif, 3, "`MODIF` (operation mode changed)");
    bit!(tefif, with_tefif, 4, "`TEFIF`");
    bit!(eccif, with_eccif, 8, "`ECCIF`");
    bit!(spicrcif, with_spicrcif, 9, "`SPICRCIF`");
    bit!(txatif, with_txatif, 10, "`TXATIF` (TX attempts exhausted)");
    bit!(rxovif, with_rxovif, 11, "`RXOVIF` (RX FIFO overflow)");
    bit!(serrif, with_serrif, 12, "`SERRIF` (system error)");
    bit!(cerrif, with_cerrif, 13, "`CERRIF` (CAN bus error)");
    bit!(ivmif, with_ivmif, 15, "`IVMIF` (invalid message)");
    bit!(txie, with_txie, 16, "`TXIE` enable");
    bit!(rxie, with_rxie, 17, "`RXIE` enable");
    bit!(modie, with_modie, 19, "`MODIE` enable");
    bit!(txatie, with_txatie, 26, "`TXATIE` enable");
    bit!(rxovie, with_rxovie, 27, "`RXOVIE` enable");
    bit!(serrie, with_serrie, 28, "`SERRIE` enable");
    bit!(cerrie, with_cerrie, 29, "`CERRIE` enable");
    bit!(ivmie, with_ivmie, 31, "`IVMIE` enable");
}

/// `CiVEC` — interrupt vector codes (0x018).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiVec(pub u32);

impl CiVec {
    /// `ICODE` (bits 6:0): highest-priority pending interrupt code.
    pub const fn icode(self) -> u8 {
        (self.0 & 0x7F) as u8
    }
    /// `RXCODE` (bits 30:24): highest-priority pending RX FIFO code.
    pub const fn rxcode(self) -> u8 {
        ((self.0 >> 24) & 0x7F) as u8
    }
    /// `TXCODE` (bits 22:16): highest-priority pending TX FIFO code.
    pub const fn txcode(self) -> u8 {
        ((self.0 >> 16) & 0x7F) as u8
    }
}

/// `CiTREC` — error counters and bus state (0x034).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiTrec(pub u32);

impl CiTrec {
    /// Receive error counter (bits 7:0).
    pub const fn rec(self) -> u8 {
        (self.0 & 0xFF) as u8
    }
    /// Transmit error counter (bits 15:8).
    pub const fn tec(self) -> u8 {
        ((self.0 >> 8) & 0xFF) as u8
    }
    bit!(error_warning, with_error_warning, 16, "`EWARN`");
    bit!(rx_warning, with_rx_warning, 17, "`RXWARN`");
    bit!(tx_warning, with_tx_warning, 18, "`TXWARN`");
    bit!(rx_error_passive, with_rx_error_passive, 19, "`RXBP`");
    bit!(tx_error_passive, with_tx_error_passive, 20, "`TXBP`");
    bit!(tx_bus_off, with_tx_bus_off, 21, "`TXBO`");
}
```

- [x] **Step 4: Run tests to verify they pass**

Run: `cargo test && cargo test --all-features`
Expected: PASS.

- [x] **Step 5: Lint and commit**

```bash
cargo clippy --all-features -- -D warnings && cargo fmt
git add src/lib.rs src/registers/mod.rs
git commit -m "feat: register addresses and core register bitfields"
```

> **✅ Task 2 summary (done 2026-08-19, commits `cce2cf0` + fix `84a264c`):** Full registers core landed: `addr` module, `Fifo`/`Filter`/`PayloadSize`/`OperationMode`/`Variant`, and `Osc`/`CiCon`/`CiInt`/`CiVec`/`CiTrec` bitfields; 8/8 tests, zero warnings. Reviewer independently verified **40+ addresses/bit positions against the MCP2518FD datasheet (DS20006027B) and Emandhal's `MCP251XFD.h` — zero mismatches** — and found the C driver itself wrong twice (its `ABAT` macro says bit 24, datasheet says bit 27; its 2517FD/2518FD SEQ-width constants are swapped): this crate is correct in both places. Fix round 1 addressed: doc comments on the `pub u32` tuple fields, `Default` derive removed from register newtypes (all-zero ≠ POR value — **later tasks: don't derive `Default` on register types**), IOCON byte-access hazard reworded as family-wide. Deferred minors in the SDD ledger (read-only-bit setters, missing `CiInt` bits incl. `WAKIE` for the future sleep task, `with_clko_div` raw code).

---

### Task 3: Registers — bit timing (`CiNBTCFG`, `CiDBTCFG`, `CiTDC`) and FIFO (`CiFIFOCON`, `CiFIFOSTA`)

**Files:**
- Modify: `src/registers/mod.rs` (append types + tests)

**Interfaces:**
- Consumes: `PayloadSize`, the `bit!` macro from Task 2.
- Produces:
  - `CiNbtCfg(pub u32)` with `new(brp: u16, tseg1: u16, tseg2: u8_as_u16, sjw)` — **takes human values, stores value − 1**
  - `CiDbtCfg(pub u32)` with `new(brp: u16, tseg1: u8, tseg2: u8, sjw: u8)`
  - `CiTdc(pub u32)` with `with_mode(TdcMode)`, `with_tdco(i8)`, `with_edge_filter(bool)`; `TdcMode` enum { `Disabled`, `Manual`, `Auto` }
  - `CiFifoCon(pub u32)` with `with_tx(bool)` (TXEN bit 7), `with_not_full_empty_ie(bool)` (bit 0), `with_rx_overflow_ie(bool)` (bit 3), `with_freset(bool)` (bit 10), `with_fifo_size(depth: u8)` (bits 28:24, stores depth − 1), `with_payload_size(PayloadSize)` (bits 31:29); byte-1 command constants `CON_BYTE1_UINC: u8 = 0x01`, `CON_BYTE1_UINC_TXREQ: u8 = 0x03`
  - `CiFifoSta(pub u32)` with `not_full_or_not_empty()` (bit 0 `TFNRFNIF`), `rx_overflow()` (bit 3), `fifo_index()` (bits 12:8 `FIFOCI`)

- [x] **Step 1: Append failing tests to the `tests` module in `src/registers/mod.rs`**

```rust
    #[test]
    fn nbtcfg_stores_minus_one() {
        // 40 MHz, 500 kbit/s, 80 TQ: brp=1, tseg1=63, tseg2=16, sjw=16.
        let r = CiNbtCfg::new(1, 63, 16, 16);
        // BRP bits 31:24, TSEG1 23:16, TSEG2 14:8, SJW 6:0 — all value-1.
        assert_eq!(r.0, (0u32 << 24) | (62 << 16) | (15 << 8) | 15);
    }

    #[test]
    fn dbtcfg_stores_minus_one() {
        // 2 Mbit/s data phase, 20 TQ: brp=1, tseg1=15, tseg2=4, sjw=4.
        let r = CiDbtCfg::new(1, 15, 4, 4);
        assert_eq!(r.0, (0u32 << 24) | (14 << 16) | (3 << 8) | 3);
    }

    #[test]
    fn tdc_auto_mode() {
        let r = CiTdc(0).with_mode(TdcMode::Auto).with_tdco(15).with_edge_filter(true);
        // TDCMOD bits 17:16 = 0b10, TDCO bits 14:8 (two's complement), EDGFLTEN bit 25.
        assert_eq!(r.0, (0b10 << 16) | (15 << 8) | (1 << 25));
        // Negative TDCO is 7-bit two's complement.
        assert_eq!(CiTdc(0).with_tdco(-1).0, 0x7F << 8);
    }

    #[test]
    fn fifocon_fields() {
        let r = CiFifoCon(0)
            .with_tx(true)
            .with_fifo_size(4)
            .with_payload_size(PayloadSize::B64)
            .with_freset(true);
        assert_eq!(r.0, (1 << 7) | (1 << 10) | (3 << 24) | (7 << 29));
        let rx = CiFifoCon(0).with_not_full_empty_ie(true).with_rx_overflow_ie(true);
        assert_eq!(rx.0, 1 | (1 << 3));
    }

    #[test]
    fn fifosta_fields() {
        assert!(CiFifoSta(1).not_full_or_not_empty());
        assert!(CiFifoSta(1 << 3).rx_overflow());
        assert_eq!(CiFifoSta(0x0500).fifo_index(), 5);
    }
```

- [x] **Step 2: Run tests to verify they fail**

Run: `cargo test`
Expected: FAIL — types not defined.

- [x] **Step 3: Append implementations to `src/registers/mod.rs`**

```rust
/// `CiNBTCFG` — nominal bit timing (0x004). All fields stored as value − 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiNbtCfg(pub u32);

impl CiNbtCfg {
    /// Builds the register from human values: `brp: 1..=256`,
    /// `tseg1: 2..=256`, `tseg2: 1..=128`, `sjw: 1..=128` (time quanta).
    /// Callers validate ranges (see `config::NominalBitTiming::validate`).
    pub const fn new(brp: u16, tseg1: u16, tseg2: u16, sjw: u16) -> Self {
        Self(
            (((brp - 1) as u32) << 24)
                | (((tseg1 - 1) as u32) << 16)
                | (((tseg2 - 1) as u32) << 8)
                | ((sjw - 1) as u32),
        )
    }
}

/// `CiDBTCFG` — data bit timing (0x008). All fields stored as value − 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiDbtCfg(pub u32);

impl CiDbtCfg {
    /// Builds the register from human values: `brp: 1..=256`,
    /// `tseg1: 1..=32`, `tseg2: 1..=16`, `sjw: 1..=16` (time quanta).
    pub const fn new(brp: u16, tseg1: u8, tseg2: u8, sjw: u8) -> Self {
        Self(
            (((brp - 1) as u32) << 24)
                | (((tseg1 - 1) as u32) << 16)
                | (((tseg2 - 1) as u32) << 8)
                | ((sjw - 1) as u32),
        )
    }
}

/// Transmitter delay compensation mode (`CiTDC.TDCMOD`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TdcMode {
    /// TDC disabled (code 0).
    Disabled,
    /// Manual: SSP = TDCV + TDCO (code 1).
    Manual,
    /// Auto: chip measures the loop delay; SSP = measured + TDCO (code 2).
    /// Recommended for data rates ≥ 1 Mbit/s.
    Auto,
}

/// `CiTDC` — transmitter delay compensation (0x00C).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiTdc(pub u32);

impl CiTdc {
    /// Sets `TDCMOD` (bits 17:16).
    pub const fn with_mode(self, mode: TdcMode) -> Self {
        let code = match mode {
            TdcMode::Disabled => 0,
            TdcMode::Manual => 1,
            TdcMode::Auto => 2,
        };
        Self((self.0 & !(0b11 << 16)) | (code << 16))
    }
    /// Sets `TDCO` (bits 14:8): SSP offset in SYSCLK cycles,
    /// two's complement `-64..=63`.
    pub const fn with_tdco(self, tdco: i8) -> Self {
        Self((self.0 & !(0x7F << 8)) | (((tdco as u32) & 0x7F) << 8))
    }
    bit!(edge_filter, with_edge_filter, 25, "`EDGFLTEN` (edge filtering, recommended for FD)");
}

/// `CiFIFOCONm` — FIFO control (0x05C + 12·(m−1)).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiFifoCon(pub u32);

impl CiFifoCon {
    /// Value for byte 1 (bits 15:8) setting `UINC` only — used to pop an
    /// RX FIFO element without touching configuration bits.
    pub const CON_BYTE1_UINC: u8 = 0x01;
    /// Value for byte 1 setting `UINC | TXREQ` — used to queue and flush a
    /// TX FIFO element.
    pub const CON_BYTE1_UINC_TXREQ: u8 = 0x03;

    bit!(not_full_empty_ie, with_not_full_empty_ie, 0, "`TFNRFNIE` (not-full/not-empty interrupt enable)");
    bit!(rx_overflow_ie, with_rx_overflow_ie, 3, "`RXOVIE`");
    bit!(rx_timestamp, with_rx_timestamp, 5, "`RXTSEN`");
    bit!(tx, with_tx, 7, "`TXEN` (1 = transmit FIFO, 0 = receive FIFO)");
    bit!(uinc, with_uinc, 8, "`UINC` (increment head/tail; write-only)");
    bit!(txreq, with_txreq, 9, "`TXREQ` (request transmission; TX FIFOs)");
    bit!(freset, with_freset, 10, "`FRESET` (FIFO reset)");

    /// Sets `FSIZE` (bits 28:24): FIFO depth in messages, `1..=32`,
    /// stored as depth − 1.
    pub const fn with_fifo_size(self, depth: u8) -> Self {
        Self((self.0 & !(0x1F << 24)) | ((((depth - 1) as u32) & 0x1F) << 24))
    }
    /// Sets `PLSIZE` (bits 31:29).
    pub const fn with_payload_size(self, p: PayloadSize) -> Self {
        Self((self.0 & !(0b111 << 29)) | (p.plsize_code() << 29))
    }
}

/// `CiFIFOSTAm` — FIFO status (0x060 + 12·(m−1)).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiFifoSta(pub u32);

impl CiFifoSta {
    bit!(not_full_or_not_empty, with_not_full_or_not_empty, 0, "`TFNRFNIF` (TX: not full / RX: not empty)");
    bit!(rx_overflow, with_rx_overflow, 3, "`RXOVIF` (RX FIFO overflowed; a message was lost)");
    bit!(tx_attempts_exhausted, with_tx_attempts_exhausted, 4, "`TXATIF`");

    /// `FIFOCI` (bits 12:8): current FIFO message index. Subject to the
    /// MCP2517FD corrupt-read erratum — do not build protocol logic on it.
    pub const fn fifo_index(self) -> u8 {
        ((self.0 >> 8) & 0x1F) as u8
    }
}
```

- [x] **Step 4: Run tests to verify they pass**

Run: `cargo test && cargo test --all-features`
Expected: PASS.

- [x] **Step 5: Lint and commit**

```bash
cargo clippy --all-features -- -D warnings && cargo fmt
git add src/registers/mod.rs
git commit -m "feat: bit timing, TDC, and FIFO control/status registers"
```

> **✅ Task 3 summary (done 2026-08-19, commits `c17dc75`/`ee486e7` + fix `dd010a9`):** `CiNbtCfg`, `CiDbtCfg`, `CiTdc` (+`TdcMode`), `CiFifoCon`, `CiFifoSta` landed; 13/13 tests, zero warnings including `clippy --all-targets`. Reviewer independently re-derived **every bit position against the datasheet, the C driver, and the Linux kernel driver — zero mismatches**; Microchip's own FRM 500k/2M worked example reproduces our exact test-vector register words. Two upstream-doc defects recorded: the datasheet's TDCO table contradicts its own "two's complement" statement (crate uses standard two's complement, matching both reference drivers — noted in `with_tdco` docs), and a third C-header bug (FSIZE comment). Fix round 1 addressed a clippy `identity_op` regression in tests + the TDCO doc note. Deferred minors in the SDD ledger (unmasked field packing pending Task 8 validation, `with_fifo_size(0)` underflow guard, `TDCV` setter for manual TDC, `CON_BYTE1_*` pinning test).

---

### Task 4: Message object codecs (`src/registers/objects.rs`)

**Files:**
- Create: `src/registers/objects.rs`
- Modify: `src/registers/mod.rs` (add `pub mod objects;` at the top)

**Interfaces:**
- Consumes: `embedded_can::{Id, StandardId, ExtendedId}`.
- Produces:
  - `dlc_to_len(dlc: u8, fdf: bool) -> usize`
  - `len_to_dlc(len: usize, fdf: bool) -> Option<u8>` (exact match only)
  - `padded_dlc_len(len: usize) -> Option<usize>` (next valid FD length ≥ `len`, `None` if > 64)
  - `pack_id(id: Id) -> u32` / `unpack_id(raw: u32, extended: bool) -> Id`
  - `TxHeader { pub id: Id, pub dlc: u8, pub rtr: bool, pub brs: bool, pub fdf: bool, pub esi: bool, pub seq: u32 }` with `to_words(&self) -> [u32; 2]`
  - `RxHeader { pub id: Id, pub dlc: u8, pub rtr: bool, pub brs: bool, pub fdf: bool, pub esi: bool, pub filhit: u8 }` with `from_words(words: [u32; 2]) -> Self`

**Bit layout being implemented** (family reference manual §4): T0/R0: `SID` bits 10:0 (for extended frames: ID bits 28:18), `EID` bits 28:11 (ID bits 17:0). T1: `DLC` 3:0, `IDE` 4, `RTR` 5, `BRS` 6, `FDF` 7, `ESI` 8, `SEQ` 31:9. R1: same flags, `FILHIT` bits 15:11.

- [ ] **Step 1: Create `src/registers/objects.rs` with failing tests**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use embedded_can::{ExtendedId, Id, StandardId};

    #[test]
    fn dlc_len_mapping() {
        for (dlc, len) in [(0, 0), (8, 8), (9, 12), (10, 16), (11, 20), (12, 24), (13, 32), (14, 48), (15, 64)] {
            assert_eq!(dlc_to_len(dlc, true), len);
            assert_eq!(len_to_dlc(len, true), Some(dlc));
        }
        // Classic CAN: DLC 9..15 all mean 8 data bytes on the wire.
        assert_eq!(dlc_to_len(12, false), 8);
        assert_eq!(len_to_dlc(9, false), None);
        assert_eq!(len_to_dlc(10, true), None); // 10 is not a valid FD length
        assert_eq!(padded_dlc_len(10), Some(12));
        assert_eq!(padded_dlc_len(64), Some(64));
        assert_eq!(padded_dlc_len(65), None);
    }

    #[test]
    fn id_packing_standard() {
        let id = Id::Standard(StandardId::new(0x123).unwrap());
        assert_eq!(pack_id(id), 0x123);
        assert_eq!(unpack_id(0x123, false), id);
    }

    #[test]
    fn id_packing_extended() {
        // 29-bit ID 0x0CFE_6E01: base (bits 28:18) -> SID field 10:0,
        // low 18 bits -> EID field 28:11.
        let raw29: u32 = 0x0CFE_6E01;
        let id = Id::Extended(ExtendedId::new(raw29).unwrap());
        let packed = pack_id(id);
        assert_eq!(packed & 0x7FF, raw29 >> 18);
        assert_eq!((packed >> 11) & 0x3_FFFF, raw29 & 0x3_FFFF);
        assert_eq!(unpack_id(packed, true), id);
    }

    #[test]
    fn tx_header_words() {
        let h = TxHeader {
            id: Id::Standard(StandardId::new(0x123).unwrap()),
            dlc: 4,
            rtr: false,
            brs: false,
            fdf: false,
            esi: false,
            seq: 1,
        };
        let [t0, t1] = h.to_words();
        assert_eq!(t0, 0x123);
        assert_eq!(t1, 4 | (1 << 9)); // DLC=4, SEQ=1
        let h_fd = TxHeader { dlc: 15, brs: true, fdf: true, seq: 0, ..h };
        let [_, t1] = h_fd.to_words();
        assert_eq!(t1, 15 | (1 << 6) | (1 << 7));
    }

    #[test]
    fn rx_header_round_trip() {
        // Extended FD frame, DLC 15, BRS, FILHIT 3.
        let raw29: u32 = 0x0CFE_6E01;
        let r0 = ((raw29 >> 18) & 0x7FF) | ((raw29 & 0x3_FFFF) << 11);
        let r1 = 15 | (1 << 4) | (1 << 6) | (1 << 7) | (3 << 11);
        let h = RxHeader::from_words([r0, r1]);
        assert_eq!(h.id, Id::Extended(ExtendedId::new(raw29).unwrap()));
        assert_eq!(h.dlc, 15);
        assert!(h.brs && h.fdf && !h.rtr && !h.esi);
        assert_eq!(h.filhit, 3);
    }
}
```

- [ ] **Step 2: Add `pub mod objects;` to `src/registers/mod.rs`, run tests to verify failure**

Run: `cargo test`
Expected: FAIL.

- [ ] **Step 3: Implement above the tests**

```rust
//! Encoding and decoding of message objects in controller RAM.
//!
//! TX objects are two header words (T0, T1) followed by the payload;
//! RX objects mirror this (R0, R1). Family reference manual §4.

use embedded_can::{ExtendedId, Id, StandardId};

/// Converts a DLC code (`0..=15`) to a payload length in bytes.
/// For classic frames (`fdf == false`), DLC values above 8 mean 8 bytes.
pub const fn dlc_to_len(dlc: u8, fdf: bool) -> usize {
    match dlc {
        0..=8 => {
            if !fdf && dlc > 8 { 8 } else { dlc as usize }
        }
        9 if fdf => 12,
        10 if fdf => 16,
        11 if fdf => 20,
        12 if fdf => 24,
        13 if fdf => 32,
        14 if fdf => 48,
        15 if fdf => 64,
        _ => 8, // classic frame with DLC 9..=15
    }
}

/// Converts an exact payload length to its DLC code; `None` if the length
/// is not directly representable (e.g. 10 bytes FD, or > 8 bytes classic).
pub const fn len_to_dlc(len: usize, fdf: bool) -> Option<u8> {
    match len {
        0..=8 => Some(len as u8),
        12 if fdf => Some(9),
        16 if fdf => Some(10),
        20 if fdf => Some(11),
        24 if fdf => Some(12),
        32 if fdf => Some(13),
        48 if fdf => Some(14),
        64 if fdf => Some(15),
        _ => None,
    }
}

/// Rounds a length up to the next valid CAN FD payload length
/// (`0..=8, 12, 16, 20, 24, 32, 48, 64`). `None` if `len > 64`.
pub const fn padded_dlc_len(len: usize) -> Option<usize> {
    match len {
        0..=8 => Some(len),
        9..=12 => Some(12),
        13..=16 => Some(16),
        17..=20 => Some(20),
        21..=24 => Some(24),
        25..=32 => Some(32),
        33..=48 => Some(48),
        49..=64 => Some(64),
        _ => None,
    }
}

/// Packs a CAN ID into the T0/R0/FLTOBJ layout: `SID` in bits 10:0 and,
/// for extended IDs, `EID` (ID bits 17:0) in bits 28:11 with the base ID
/// (bits 28:18 of the 29-bit ID) in the `SID` field.
pub fn pack_id(id: Id) -> u32 {
    match id {
        Id::Standard(sid) => sid.as_raw() as u32,
        Id::Extended(eid) => {
            let raw = eid.as_raw();
            ((raw >> 18) & 0x7FF) | ((raw & 0x3_FFFF) << 11)
        }
    }
}

/// Inverse of [`pack_id`]. `extended` selects the layout (from `IDE`).
pub fn unpack_id(raw: u32, extended: bool) -> Id {
    if extended {
        let id29 = ((raw & 0x7FF) << 18) | ((raw >> 11) & 0x3_FFFF);
        // Both operands are masked to 29 bits, so this cannot fail.
        Id::Extended(ExtendedId::new(id29).unwrap_or(ExtendedId::ZERO))
    } else {
        Id::Standard(StandardId::new((raw & 0x7FF) as u16).unwrap_or(StandardId::ZERO))
    }
}

/// The fields of a TX message object header (words T0 and T1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxHeader {
    /// CAN identifier.
    pub id: Id,
    /// DLC code (`0..=15`).
    pub dlc: u8,
    /// Remote transmission request (classic frames only).
    pub rtr: bool,
    /// Bit rate switch (FD frames only).
    pub brs: bool,
    /// FD format frame.
    pub fdf: bool,
    /// Error state indicator.
    pub esi: bool,
    /// Sequence number echoed in the TEF (`0..=0x7F` on MCP2517FD,
    /// `0..=0x7F_FFFF` on MCP2518FD). Caller masks to the variant's width.
    pub seq: u32,
}

impl TxHeader {
    /// Encodes T0 and T1.
    pub fn to_words(&self) -> [u32; 2] {
        let t0 = pack_id(self.id);
        let ide = matches!(self.id, Id::Extended(_));
        let t1 = (self.dlc as u32 & 0xF)
            | ((ide as u32) << 4)
            | ((self.rtr as u32) << 5)
            | ((self.brs as u32) << 6)
            | ((self.fdf as u32) << 7)
            | ((self.esi as u32) << 8)
            | (self.seq << 9);
        [t0, t1]
    }
}

/// The fields decoded from an RX message object header (words R0 and R1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RxHeader {
    /// CAN identifier.
    pub id: Id,
    /// DLC code (`0..=15`).
    pub dlc: u8,
    /// Remote transmission request.
    pub rtr: bool,
    /// Bit rate switch was used.
    pub brs: bool,
    /// FD format frame.
    pub fdf: bool,
    /// Error state indicator of the transmitter.
    pub esi: bool,
    /// Index of the filter that accepted this frame (`0..=31`).
    pub filhit: u8,
}

impl RxHeader {
    /// Decodes R0 and R1.
    pub fn from_words(words: [u32; 2]) -> Self {
        let [r0, r1] = words;
        let ide = r1 & (1 << 4) != 0;
        Self {
            id: unpack_id(r0, ide),
            dlc: (r1 & 0xF) as u8,
            rtr: r1 & (1 << 5) != 0,
            brs: r1 & (1 << 6) != 0,
            fdf: r1 & (1 << 7) != 0,
            esi: r1 & (1 << 8) != 0,
            filhit: ((r1 >> 11) & 0x1F) as u8,
        }
    }
}
```

Note: `StandardId::ZERO` and `ExtendedId::ZERO` exist in embedded-can 0.4. `dlc_to_len`'s first match arm has an unreachable inner branch — simplify to `0..=8 => dlc as usize` and keep the `_ => 8` classic fallback; write it that way from the start.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test`
Expected: PASS (6 new tests).

- [ ] **Step 5: Lint and commit**

```bash
cargo clippy --all-features -- -D warnings && cargo fmt
git add src/registers/mod.rs src/registers/objects.rs
git commit -m "feat: message object header codecs, ID packing, DLC mapping"
```

---

### Task 5: Frame types (`src/frame.rs`)

**Files:**
- Create: `src/frame.rs`
- Modify: `src/lib.rs` (add `pub mod frame;` and `pub use frame::{FdFrame, Frame, FrameFlags, RxFrame};`)

**Interfaces:**
- Consumes: `crate::registers::objects::{len_to_dlc, padded_dlc_len, dlc_to_len}`; `embedded_can`.
- Produces:
  - `Frame` — classic CAN frame; `new(id: impl Into<Id>, data: &[u8]) -> Option<Self>` (≤ 8 bytes), `new_remote(id, dlc: usize) -> Option<Self>`, accessors per `embedded_can::Frame`; **implements `embedded_can::Frame`**
  - `FrameFlags { pub brs: bool, pub esi: bool }` (`Default` = both false)
  - `FdFrame` — `new(id: impl Into<Id>, data: &[u8], flags: FrameFlags) -> Option<Self>` (len must be exactly a valid FD length), `new_padded(...) -> Option<Self>` (pads with zeros to next valid length), `id()`, `data() -> &[u8]`, `flags()`
  - `RxFrame { pub frame: ReceivedFrame, pub timestamp: Option<u32> }` where `ReceivedFrame` is `enum { Classic(Frame), Fd(FdFrame) }` — v0.1 always sets `timestamp: None`
  - Both frame structs store `data: [u8; 8]` / `[u8; 64]` plus a length; `data()` returns the valid slice

- [ ] **Step 1: Create `src/frame.rs` with failing tests**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use embedded_can::{Frame as _, StandardId};

    #[test]
    fn classic_frame_basics() {
        let id = StandardId::new(0x123).unwrap();
        let f = Frame::new(id, &[1, 2, 3, 4]).unwrap();
        assert_eq!(f.data(), &[1, 2, 3, 4]);
        assert_eq!(f.dlc(), 4);
        assert!(!f.is_remote_frame());
        assert!(Frame::new(id, &[0; 9]).is_none());
        let r = Frame::new_remote(id, 2).unwrap();
        assert!(r.is_remote_frame());
        assert_eq!(r.dlc(), 2);
        assert!(Frame::new_remote(id, 9).is_none());
    }

    #[test]
    fn fd_frame_lengths() {
        let id = StandardId::new(0x123).unwrap();
        assert!(FdFrame::new(id, &[0; 64], FrameFlags::default()).is_some());
        assert!(FdFrame::new(id, &[0; 10], FrameFlags::default()).is_none());
        let padded = FdFrame::new_padded(id, &[0xFF; 10], FrameFlags { brs: true, esi: false }).unwrap();
        assert_eq!(padded.data().len(), 12);
        assert_eq!(&padded.data()[..10], &[0xFF; 10]);
        assert_eq!(&padded.data()[10..], &[0, 0]);
        assert!(padded.flags().brs);
        assert!(FdFrame::new_padded(id, &[0; 65], FrameFlags::default()).is_none());
    }
}
```

- [ ] **Step 2: Wire the module in `src/lib.rs`, run tests to verify failure**

Run: `cargo test`
Expected: FAIL.

- [ ] **Step 3: Implement above the tests**

```rust
//! CAN frame types.
//!
//! [`Frame`] is a classic CAN 2.0 frame and implements
//! [`embedded_can::Frame`] for ecosystem interop. [`FdFrame`] is a CAN FD
//! frame (no ecosystem-standard trait exists for FD).

use crate::registers::objects::{len_to_dlc, padded_dlc_len};
use embedded_can::Id;

/// A classic CAN 2.0 frame (up to 8 data bytes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Frame {
    id: Id,
    dlc: u8,
    rtr: bool,
    data: [u8; 8],
}

impl Frame {
    /// The frame's CAN identifier.
    pub fn id(&self) -> Id {
        self.id
    }
    /// The data bytes (empty for remote frames).
    pub fn data(&self) -> &[u8] {
        if self.rtr { &[] } else { &self.data[..self.dlc as usize] }
    }
    /// The DLC (`0..=8`).
    pub fn dlc(&self) -> usize {
        self.dlc as usize
    }
    /// Whether this is a remote (RTR) frame.
    pub fn is_remote(&self) -> bool {
        self.rtr
    }
}

impl embedded_can::Frame for Frame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }
        let mut buf = [0u8; 8];
        buf[..data.len()].copy_from_slice(data);
        Some(Self { id: id.into(), dlc: data.len() as u8, rtr: false, data: buf })
    }

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        if dlc > 8 {
            return None;
        }
        Some(Self { id: id.into(), dlc: dlc as u8, rtr: true, data: [0; 8] })
    }

    fn is_extended(&self) -> bool {
        matches!(self.id, Id::Extended(_))
    }
    fn is_remote_frame(&self) -> bool {
        self.rtr
    }
    fn id(&self) -> Id {
        self.id
    }
    fn dlc(&self) -> usize {
        self.dlc as usize
    }
    fn data(&self) -> &[u8] {
        Frame::data(self)
    }
}

/// Per-frame CAN FD flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FrameFlags {
    /// Bit rate switch: transmit the data phase at the data bit rate.
    pub brs: bool,
    /// Error state indicator.
    pub esi: bool,
}

/// A CAN FD frame (up to 64 data bytes; length must be a valid DLC step).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FdFrame {
    id: Id,
    len: u8,
    flags: FrameFlags,
    data: [u8; 64],
}

impl FdFrame {
    /// Creates an FD frame. Returns `None` unless `data.len()` is exactly a
    /// valid CAN FD length (`0..=8, 12, 16, 20, 24, 32, 48, 64`).
    pub fn new(id: impl Into<Id>, data: &[u8], flags: FrameFlags) -> Option<Self> {
        len_to_dlc(data.len(), true)?;
        let mut buf = [0u8; 64];
        buf[..data.len()].copy_from_slice(data);
        Some(Self { id: id.into(), len: data.len() as u8, flags, data: buf })
    }

    /// Creates an FD frame, zero-padding the payload up to the next valid
    /// CAN FD length. Returns `None` if `data.len() > 64`.
    pub fn new_padded(id: impl Into<Id>, data: &[u8], flags: FrameFlags) -> Option<Self> {
        let padded = padded_dlc_len(data.len())?;
        let mut buf = [0u8; 64];
        buf[..data.len()].copy_from_slice(data);
        Some(Self { id: id.into(), len: padded as u8, flags, data: buf })
    }

    /// The frame's CAN identifier.
    pub fn id(&self) -> Id {
        self.id
    }
    /// The data bytes (padded length).
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }
    /// The FD flags.
    pub fn flags(&self) -> FrameFlags {
        self.flags
    }
}

/// A frame received from an RX FIFO.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxFrame {
    /// The received frame.
    pub frame: ReceivedFrame,
    /// RX timestamp. Always `None` in this version (timestamping is not
    /// yet configurable).
    pub timestamp: Option<u32>,
}

/// Classic or FD payload of a received frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReceivedFrame {
    /// A classic CAN 2.0 frame.
    Classic(Frame),
    /// A CAN FD frame.
    Fd(FdFrame),
}
```

Note: `Frame` has inherent methods and trait methods with the same names — the inherent ones win for direct calls; keep both so users don't need the trait in scope. If clippy objects to the duplication (`clippy::should_implement_trait` does not apply here, but if any lint fires), keep the inherent methods and silence per-lint with a scoped `#[allow]` and a comment.

Also add to this task: construct `Frame`/`FdFrame` in `RxFrame` decoding paths later (Task 12) via a crate-private constructor — add now:

```rust
impl Frame {
    pub(crate) fn from_parts(id: Id, dlc: u8, rtr: bool, data: [u8; 8]) -> Self {
        Self { id, dlc, rtr, data }
    }
}
impl FdFrame {
    pub(crate) fn from_parts(id: Id, len: u8, flags: FrameFlags, data: [u8; 64]) -> Self {
        Self { id, len, flags, data }
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test && cargo test --all-features`
Expected: PASS. (`from_parts` is unused for now — mark both `#[allow(dead_code)]` with a `// used by driver RX path (Task 12)` comment, and remove the allows in Task 12.)

- [ ] **Step 5: Lint and commit**

```bash
cargo clippy --all-features -- -D warnings && cargo fmt
git add src/lib.rs src/frame.rs
git commit -m "feat: classic and FD frame types with embedded-can interop"
```

---

### Task 6: RAM layout planner (`src/registers/ram.rs`) with compile-time overflow check

**Files:**
- Create: `src/registers/ram.rs`
- Modify: `src/registers/mod.rs` (add `pub mod ram;`)
- Create: `tests/compile_fail.rs`, `tests/compile_fail/ram_overflow.rs`

**Interfaces:**
- Consumes: `Fifo`, `PayloadSize`, `addr::{RAM_START, RAM_SIZE}`.
- Produces:
  - `FifoDirection` enum { `Transmit`, `Receive` }
  - `FifoEntry { pub direction: FifoDirection, pub payload: PayloadSize, pub depth: u8 }`
  - `LayoutError` enum { `RamOverflow`, `BadDepth`, `AlreadyConfigured` }
  - `FifoLayout` with:
    - `const fn new() -> Self`
    - `const fn try_tx_fifo(self, Fifo, PayloadSize, depth: u8) -> Result<Self, LayoutError>` / `try_rx_fifo(...)`
    - `const fn tx_fifo(self, Fifo, PayloadSize, depth: u8) -> Self` / `rx_fifo(...)` — panic on error; panics become **compile errors** when the layout is built in a `const`
    - `const fn total_bytes(&self) -> usize`
    - `fn entries(&self) -> impl Iterator<Item = (Fifo, FifoEntry)> + '_`
    - `pub(crate) fn entry(&self, Fifo) -> Option<FifoEntry>`
- Element size rule: `8 + payload.bytes()` bytes per message, times depth. FIFOs occupy RAM contiguously from `RAM_START` in FIFO-number order; addresses are chip-computed at runtime via `CiFIFOUAm`, so the planner only validates the total.

- [ ] **Step 1: Create `src/registers/ram.rs` with failing tests**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::registers::{Fifo, PayloadSize};

    #[test]
    fn element_and_total_sizes() {
        // (8 header + 64 payload) * 4 + (8 + 64) * 8 = 288 + 576 = 864.
        let l = FifoLayout::new()
            .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
            .rx_fifo(Fifo::F2, PayloadSize::B64, 8);
        assert_eq!(l.total_bytes(), 864);
        assert_eq!(l.entries().count(), 2);
        let e = l.entry(Fifo::F2).unwrap();
        assert_eq!(e.depth, 8);
        assert!(matches!(e.direction, FifoDirection::Receive));
    }

    #[test]
    fn exactly_full_is_ok() {
        // 16 bytes/element * 32 deep * 4 FIFOs = 2048 exactly.
        let l = FifoLayout::new()
            .rx_fifo(Fifo::F1, PayloadSize::B8, 32)
            .rx_fifo(Fifo::F2, PayloadSize::B8, 32)
            .rx_fifo(Fifo::F3, PayloadSize::B8, 32)
            .rx_fifo(Fifo::F4, PayloadSize::B8, 32);
        assert_eq!(l.total_bytes(), 2048);
    }

    #[test]
    fn overflow_is_err_at_runtime() {
        let l = FifoLayout::new()
            .rx_fifo(Fifo::F1, PayloadSize::B64, 28); // 72 * 28 = 2016
        assert!(matches!(
            l.try_rx_fifo(Fifo::F2, PayloadSize::B64, 1),
            Err(LayoutError::RamOverflow)
        ));
    }

    #[test]
    fn bad_depth_and_duplicates() {
        let l = FifoLayout::new().tx_fifo(Fifo::F1, PayloadSize::B8, 1);
        assert!(matches!(l.try_tx_fifo(Fifo::F1, PayloadSize::B8, 1), Err(LayoutError::AlreadyConfigured)));
        assert!(matches!(FifoLayout::new().try_tx_fifo(Fifo::F2, PayloadSize::B8, 0), Err(LayoutError::BadDepth)));
        assert!(matches!(FifoLayout::new().try_tx_fifo(Fifo::F2, PayloadSize::B8, 33), Err(LayoutError::BadDepth)));
    }

    #[test]
    fn const_layout_compiles() {
        const LAYOUT: FifoLayout = FifoLayout::new()
            .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
            .rx_fifo(Fifo::F2, PayloadSize::B64, 8);
        assert_eq!(LAYOUT.total_bytes(), 864);
    }
}
```

- [ ] **Step 2: Add `pub mod ram;` to `src/registers/mod.rs`, run tests to verify failure**

Run: `cargo test`
Expected: FAIL.

- [ ] **Step 3: Implement above the tests**

```rust
//! Message RAM layout planning.
//!
//! The chip allocates configured FIFOs contiguously from the start of the
//! 2 KiB message RAM. The planner validates that the configuration fits;
//! actual element addresses are read back from `CiFIFOUAm` at runtime.
//!
//! Build the layout in a `const` and RAM overflow becomes a compile error:
//!
//! ```
//! use mcp251xfd::registers::ram::FifoLayout;
//! use mcp251xfd::registers::{Fifo, PayloadSize};
//! const LAYOUT: FifoLayout = FifoLayout::new()
//!     .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
//!     .rx_fifo(Fifo::F2, PayloadSize::B64, 8);
//! ```

use super::{addr, Fifo, PayloadSize};

/// Transfer direction of a FIFO.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoDirection {
    /// The FIFO transmits frames.
    Transmit,
    /// The FIFO receives frames.
    Receive,
}

/// Configuration of one FIFO within a [`FifoLayout`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoEntry {
    /// Transfer direction.
    pub direction: FifoDirection,
    /// Payload size of each element.
    pub payload: PayloadSize,
    /// Number of elements (`1..=32`).
    pub depth: u8,
}

impl FifoEntry {
    /// RAM bytes used by this FIFO: `(8 + payload) * depth`.
    pub const fn bytes(&self) -> usize {
        (8 + self.payload.bytes()) * self.depth as usize
    }
}

/// Why a FIFO could not be added to a [`FifoLayout`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LayoutError {
    /// The layout would exceed the 2048-byte message RAM.
    RamOverflow,
    /// Depth must be `1..=32`.
    BadDepth,
    /// This FIFO number is already configured in the layout.
    AlreadyConfigured,
}

/// A complete message-RAM layout: which FIFOs exist, their direction,
/// payload size, and depth.
#[derive(Debug, Clone, Copy)]
pub struct FifoLayout {
    entries: [Option<FifoEntry>; 31],
    total: usize,
}

impl FifoLayout {
    /// An empty layout.
    pub const fn new() -> Self {
        Self { entries: [None; 31], total: 0 }
    }

    /// Adds a FIFO. Errors instead of panicking; use this for layouts built
    /// at runtime.
    pub const fn try_add(
        self,
        fifo: Fifo,
        entry: FifoEntry,
    ) -> Result<Self, LayoutError> {
        if entry.depth < 1 || entry.depth > 32 {
            return Err(LayoutError::BadDepth);
        }
        let slot = (fifo.index() - 1) as usize;
        if self.entries[slot].is_some() {
            return Err(LayoutError::AlreadyConfigured);
        }
        let total = self.total + entry.bytes();
        if total > addr::RAM_SIZE {
            return Err(LayoutError::RamOverflow);
        }
        let mut entries = self.entries;
        entries[slot] = Some(entry);
        Ok(Self { entries, total })
    }

    /// Adds a transmit FIFO; see [`Self::try_add`].
    pub const fn try_tx_fifo(self, fifo: Fifo, payload: PayloadSize, depth: u8) -> Result<Self, LayoutError> {
        self.try_add(fifo, FifoEntry { direction: FifoDirection::Transmit, payload, depth })
    }

    /// Adds a receive FIFO; see [`Self::try_add`].
    pub const fn try_rx_fifo(self, fifo: Fifo, payload: PayloadSize, depth: u8) -> Result<Self, LayoutError> {
        self.try_add(fifo, FifoEntry { direction: FifoDirection::Receive, payload, depth })
    }

    /// Adds a transmit FIFO.
    ///
    /// # Panics
    /// Panics if the layout would exceed RAM, the depth is out of range, or
    /// the FIFO is already configured. In a `const` context the panic is a
    /// compile error — declare layouts as `const` to get build-time checking.
    pub const fn tx_fifo(self, fifo: Fifo, payload: PayloadSize, depth: u8) -> Self {
        match self.try_tx_fifo(fifo, payload, depth) {
            Ok(l) => l,
            Err(LayoutError::RamOverflow) => panic!("FIFO layout exceeds 2048-byte message RAM"),
            Err(LayoutError::BadDepth) => panic!("FIFO depth must be 1..=32"),
            Err(LayoutError::AlreadyConfigured) => panic!("FIFO already configured"),
        }
    }

    /// Adds a receive FIFO. Panics like [`Self::tx_fifo`].
    pub const fn rx_fifo(self, fifo: Fifo, payload: PayloadSize, depth: u8) -> Self {
        match self.try_rx_fifo(fifo, payload, depth) {
            Ok(l) => l,
            Err(LayoutError::RamOverflow) => panic!("FIFO layout exceeds 2048-byte message RAM"),
            Err(LayoutError::BadDepth) => panic!("FIFO depth must be 1..=32"),
            Err(LayoutError::AlreadyConfigured) => panic!("FIFO already configured"),
        }
    }

    /// Total message-RAM bytes used (`<= 2048`).
    pub const fn total_bytes(&self) -> usize {
        self.total
    }

    /// Iterates over the configured FIFOs in FIFO-number order.
    pub fn entries(&self) -> impl Iterator<Item = (Fifo, FifoEntry)> + '_ {
        self.entries.iter().enumerate().filter_map(|(i, e)| {
            e.map(|entry| (Fifo::new(i as u8 + 1).expect("index in range"), entry))
        })
    }

    /// Looks up one FIFO's configuration.
    pub fn entry(&self, fifo: Fifo) -> Option<FifoEntry> {
        self.entries[(fifo.index() - 1) as usize]
    }
}

impl Default for FifoLayout {
    fn default() -> Self {
        Self::new()
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test`
Expected: PASS. (The doctest in the module header also runs and must pass — this requires `registers::ram` and its items to be `pub`, which they are.)

- [ ] **Step 5: Add the trybuild compile-fail proof**

Create `tests/compile_fail/ram_overflow.rs`:

```rust
use mcp251xfd::registers::ram::FifoLayout;
use mcp251xfd::registers::{Fifo, PayloadSize};

// 72 bytes/element * 29 = 2088 > 2048: must fail to compile.
const LAYOUT: FifoLayout = FifoLayout::new().rx_fifo(Fifo::F1, PayloadSize::B64, 29);

fn main() {
    let _ = LAYOUT;
}
```

Create `tests/compile_fail.rs`:

```rust
#[test]
fn const_ram_overflow_fails_to_compile() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/compile_fail/*.rs");
}
```

- [ ] **Step 6: Generate the expected stderr and verify**

Run: `TRYBUILD=overwrite cargo test --test compile_fail` then `cargo test --test compile_fail`
Expected: first run writes `tests/compile_fail/ram_overflow.stderr` (it must mention the const-eval panic "FIFO layout exceeds 2048-byte message RAM"); second run PASSES. Commit the generated `.stderr` file.

- [ ] **Step 7: Lint and commit**

```bash
cargo clippy --all-features -- -D warnings && cargo fmt
git add src/registers/mod.rs src/registers/ram.rs tests/compile_fail.rs tests/compile_fail/
git commit -m "feat: const-fn FIFO RAM layout planner with compile-time overflow check"
```

---

### Task 7: SPI bus layer (`src/bus.rs`) — sync + async from one source

**Files:**
- Create: `src/bus.rs`
- Modify: `src/lib.rs` (add `mod bus;` — crate-private)

**Interfaces:**
- Consumes: `Error` from Task 1.
- Produces (crate-private, used by `driver.rs`):
  - `Opcode` enum { `Reset = 0x0`, `Write = 0x2`, `Read = 0x3` } (+ doc-commented reserved values 0xA/0xB/0xC for the future CRC commands)
  - `cmd(op: Opcode, addr: u16) -> [u8; 2]` — big-endian `opcode<<12 | addr`
  - `Bus<SPI>` (sync, always) and `BusAsync<SPI>` (feature `async`) with methods:
    `reset()`, `read_sfr8(addr) -> u8`, `write_sfr8(addr, u8)`, `read_sfr32(addr) -> u32`, `write_sfr32(addr, u32)`, `read_ram(addr, &mut [u8])`, `write_ram(addr, &[u8])` — all returning `Result<_, Error<SPI::Error>>`. Register data is little-endian on the wire.

**maybe-async-cfg syntax** (verified against ssd1306, the reference user of this crate): `sync(keep_self)` emits an unconditional sync item under the original name; `async(feature = "async", idents(X(async = "XAsync"), ...))` emits a feature-gated copy with `Async`-suffixed name, `.await`s intact, and the listed identifier renames applied. The sync copy strips `async`/`.await`.

- [ ] **Step 1: Create `src/bus.rs` with failing unit tests**

Unit tests live inside `src/bus.rs` (the `Bus` type is crate-private, so integration tests can't reach it; dev-dependencies are available to unit tests, and the crate is `std` under test thanks to `cfg_attr(not(test), no_std)`).

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::spi::{Mock, Transaction};

    #[test]
    fn command_encoding() {
        assert_eq!(cmd(Opcode::Read, 0x000), [0x30, 0x00]);
        assert_eq!(cmd(Opcode::Write, 0xE00), [0x2E, 0x00]);
        assert_eq!(cmd(Opcode::Read, 0xBFC), [0x3B, 0xFC]);
        assert_eq!(cmd(Opcode::Reset, 0x000), [0x00, 0x00]);
    }

    #[test]
    fn read_sfr32_is_little_endian() {
        let mut spi = Mock::new(&[
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x30, 0x00]),
            Transaction::read_vec(vec![0x60, 0x04, 0x00, 0x00]),
            Transaction::transaction_end(),
        ]);
        let mut bus = Bus { spi: &mut spi };
        assert_eq!(bus.read_sfr32(0x000).unwrap(), 0x0000_0460);
        spi.done();
    }

    #[test]
    fn write_sfr32_and_sfr8() {
        let mut spi = Mock::new(&[
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x2E, 0x00]),
            Transaction::write_vec(vec![0x60, 0x00, 0x00, 0x00]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x20, 0x5D, 0x03]),
            Transaction::transaction_end(),
        ]);
        let mut bus = Bus { spi: &mut spi };
        bus.write_sfr32(0xE00, 0x0000_0060).unwrap();
        bus.write_sfr8(0x05D, 0x03).unwrap();
        spi.done();
    }

    #[test]
    fn ram_round_trip() {
        let mut spi = Mock::new(&[
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x2B, 0xFC]),
            Transaction::write_vec(vec![0x55, 0xAA, 0x55, 0xAA]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x3B, 0xFC]),
            Transaction::read_vec(vec![0x55, 0xAA, 0x55, 0xAA]),
            Transaction::transaction_end(),
        ]);
        let mut bus = Bus { spi: &mut spi };
        bus.write_ram(0xBFC, &0xAA55_AA55u32.to_le_bytes()).unwrap();
        let mut buf = [0u8; 4];
        bus.read_ram(0xBFC, &mut buf).unwrap();
        assert_eq!(u32::from_le_bytes(buf), 0xAA55_AA55);
        spi.done();
    }
}

#[cfg(all(test, feature = "async"))]
mod async_tests {
    use super::*;
    use embedded_hal_mock::eh1::spi::{Mock, Transaction};

    #[tokio::test]
    async fn async_read_sfr32() {
        let mut spi = Mock::new(&[
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x30, 0x00]),
            Transaction::read_vec(vec![0x60, 0x04, 0x00, 0x00]),
            Transaction::transaction_end(),
        ]);
        let mut bus = BusAsync { spi: &mut spi };
        assert_eq!(bus.read_sfr32(0x000).await.unwrap(), 0x0000_0460);
        spi.done();
    }
}
```

Note: the mock `Mock` type implements both the sync and async `SpiDevice` traits (the async impl needs the `embedded-hal-async` mock feature, already enabled in Task 1's dev-dependency). `&mut Mock` also implements `SpiDevice`, which is why `Bus { spi: &mut spi }` works while the test retains `spi` to call `.done()`.

- [ ] **Step 2: Add `mod bus;` to `src/lib.rs`, run tests to verify failure**

Run: `cargo test`
Expected: FAIL — module contents missing.

- [ ] **Step 3: Implement `src/bus.rs` above the tests**

```rust
//! SPI transaction layer.
//!
//! Command format: 16-bit big-endian word = 4-bit opcode | 12-bit address,
//! followed by data bytes (registers little-endian). RAM accesses
//! (0x400..0xC00) must be 32-bit aligned and sized in 32-bit multiples.
//!
//! Everything runs inside `SpiDevice::transaction` so chip-select framing
//! is correct on shared buses; the driver never touches CS itself.

use crate::error::Error;
use embedded_hal::spi::{Operation, SpiDevice};
#[cfg(feature = "async")]
use embedded_hal_async::spi::SpiDevice as SpiDeviceAsync;

/// SPI instruction opcodes (high nibble of the command word).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Opcode {
    /// Reset the chip to Configuration mode with default registers.
    Reset = 0x0,
    /// Write bytes starting at an address.
    Write = 0x2,
    /// Read bytes starting at an address.
    Read = 0x3,
    // Reserved for future CRC support (spec §"deferred"):
    // WriteCrc = 0xA, ReadCrc = 0xB, SafeWrite = 0xC.
}

/// Encodes the 2-byte command word.
pub(crate) const fn cmd(op: Opcode, addr: u16) -> [u8; 2] {
    (((op as u16) << 12) | (addr & 0x0FFF)).to_be_bytes()
}

/// Low-level register/RAM access over a shared-bus-capable SPI device.
#[maybe_async_cfg::maybe(sync(keep_self), async(feature = "async"))]
pub(crate) struct Bus<SPI> {
    pub(crate) spi: SPI,
}

#[maybe_async_cfg::maybe(
    sync(keep_self),
    async(feature = "async", idents(SpiDevice(async = "SpiDeviceAsync")))
)]
impl<SPI: SpiDevice> Bus<SPI> {
    /// Sends the RESET instruction.
    pub(crate) async fn reset(&mut self) -> Result<(), Error<SPI::Error>> {
        self.spi.write(&cmd(Opcode::Reset, 0)).await.map_err(Error::Spi)
    }

    /// Reads one byte from an SFR address.
    pub(crate) async fn read_sfr8(&mut self, addr: u16) -> Result<u8, Error<SPI::Error>> {
        let c = cmd(Opcode::Read, addr);
        let mut buf = [0u8; 1];
        self.spi
            .transaction(&mut [Operation::Write(&c), Operation::Read(&mut buf)])
            .await
            .map_err(Error::Spi)?;
        Ok(buf[0])
    }

    /// Writes one byte to an SFR address (also the only safe way to touch
    /// IOCON, per the MCP2517FD erratum).
    pub(crate) async fn write_sfr8(&mut self, addr: u16, value: u8) -> Result<(), Error<SPI::Error>> {
        let c = cmd(Opcode::Write, addr);
        self.spi
            .transaction(&mut [Operation::Write(&c), Operation::Write(&[value])])
            .await
            .map_err(Error::Spi)
    }

    /// Reads a 32-bit register (little-endian on the wire).
    pub(crate) async fn read_sfr32(&mut self, addr: u16) -> Result<u32, Error<SPI::Error>> {
        let c = cmd(Opcode::Read, addr);
        let mut buf = [0u8; 4];
        self.spi
            .transaction(&mut [Operation::Write(&c), Operation::Read(&mut buf)])
            .await
            .map_err(Error::Spi)?;
        Ok(u32::from_le_bytes(buf))
    }

    /// Writes a 32-bit register (little-endian on the wire).
    pub(crate) async fn write_sfr32(&mut self, addr: u16, value: u32) -> Result<(), Error<SPI::Error>> {
        let c = cmd(Opcode::Write, addr);
        self.spi
            .transaction(&mut [Operation::Write(&c), Operation::Write(&value.to_le_bytes())])
            .await
            .map_err(Error::Spi)
    }

    /// Reads from message RAM. `addr` must be 32-bit aligned and `buf.len()`
    /// a multiple of 4 (hardware requirement).
    pub(crate) async fn read_ram(&mut self, addr: u16, buf: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        debug_assert!(addr % 4 == 0 && buf.len() % 4 == 0);
        debug_assert!((0x400..0xC00).contains(&addr));
        let c = cmd(Opcode::Read, addr);
        self.spi
            .transaction(&mut [Operation::Write(&c), Operation::Read(buf)])
            .await
            .map_err(Error::Spi)
    }

    /// Writes to message RAM. Same alignment rules as [`Self::read_ram`].
    pub(crate) async fn write_ram(&mut self, addr: u16, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        debug_assert!(addr % 4 == 0 && data.len() % 4 == 0);
        debug_assert!((0x400..0xC00).contains(&addr));
        let c = cmd(Opcode::Write, addr);
        self.spi
            .transaction(&mut [Operation::Write(&c), Operation::Write(data)])
            .await
            .map_err(Error::Spi)
    }
}
```

Fallback note (only if compilation of the attribute fails): the syntax above is copied from `ssd1306`'s production usage. If a `maybe-async-cfg` version mismatch rejects it, check `https://docs.rs/maybe-async-cfg` and `ssd1306` `src/lib.rs` for the current canonical form; required outcome: unconditional `Bus` (sync, `SpiDevice`), feature-gated `BusAsync` (async, `SpiDeviceAsync`), one source.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test && cargo test --all-features`
Expected: PASS, including the `async_tests` module under `--all-features`.

- [ ] **Step 5: Lint and commit**

```bash
cargo clippy --all-features -- -D warnings && cargo fmt
git add src/lib.rs src/bus.rs
git commit -m "feat: SPI bus layer with sync/async variants via maybe-async-cfg"
```

---

### Task 8: Configuration types and presets (`src/config.rs`)

**Files:**
- Create: `src/config.rs`
- Modify: `src/lib.rs` (add `pub mod config;`, `pub use config::{max_spi_hz, ClockConfig, Config, DataBitTiming, FilterMatch, NominalBitTiming};` and re-export `pub use registers::{Fifo, Filter, OperationMode, PayloadSize, Variant};` plus `pub use registers::ram::FifoLayout;`)

**Interfaces:**
- Consumes: `CiNbtCfg`, `CiDbtCfg`, `ConfigError`, `objects::pack_id`, `embedded_can::Id`.
- Produces:
  - `max_spi_hz(sysclk_hz: u32) -> u32` — erratum cap `0.85 * sysclk / 2`
  - `ClockConfig { pub xtal_hz: u32, pub pll: bool, pub sclk_div2: bool }` + `const MHZ40 / MHZ20 / MHZ4_PLL`, `sysclk_hz()`, `validate()`
  - `NominalBitTiming { pub brp: u16, pub tseg1: u16, pub tseg2: u16, pub sjw: u16 }` + `validate() -> Result<(), ConfigError>`, `to_reg() -> CiNbtCfg`, presets `KBPS125_40MHZ`, `KBPS250_40MHZ`, `KBPS500_40MHZ`, `MBPS1_40MHZ`
  - `DataBitTiming { pub brp: u16, pub tseg1: u8, pub tseg2: u8, pub sjw: u8 }` + `validate()`, `to_reg() -> CiDbtCfg`, `tdco() -> i8` (= `min(brp * tseg1, 63)`), presets `MBPS2_40MHZ`, `MBPS5_40MHZ`, `MBPS8_40MHZ`
  - `Config { pub clock: ClockConfig, pub nominal: NominalBitTiming, pub data: Option<DataBitTiming> }` + `validate()`
  - `FilterMatch { pub fltobj: u32, pub mask: u32 }` + `exact(id: Id) -> Self`, `accept_all() -> Self`, `with_mask(id: Id, id_mask: u32) -> Self`

- [ ] **Step 1: Create `src/config.rs` with failing tests**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use embedded_can::{ExtendedId, Id, StandardId};

    #[test]
    fn spi_cap() {
        assert_eq!(max_spi_hz(40_000_000), 17_000_000);
        assert_eq!(max_spi_hz(20_000_000), 8_500_000);
    }

    #[test]
    fn clock_sysclk() {
        assert_eq!(ClockConfig::MHZ40.sysclk_hz(), 40_000_000);
        assert_eq!(ClockConfig::MHZ4_PLL.sysclk_hz(), 40_000_000);
        let div = ClockConfig { xtal_hz: 40_000_000, pll: false, sclk_div2: true };
        assert_eq!(div.sysclk_hz(), 20_000_000);
        // PLL only valid from a 4 MHz-class input.
        assert!(ClockConfig { xtal_hz: 40_000_000, pll: true, sclk_div2: false }.validate().is_err());
    }

    #[test]
    fn preset_bit_counts() {
        // Preset invariant: sysclk / (brp * (1 + tseg1 + tseg2)) = bit rate.
        for (p, rate) in [
            (NominalBitTiming::KBPS125_40MHZ, 125_000u32),
            (NominalBitTiming::KBPS250_40MHZ, 250_000),
            (NominalBitTiming::KBPS500_40MHZ, 500_000),
            (NominalBitTiming::MBPS1_40MHZ, 1_000_000),
        ] {
            let tq = 1 + p.tseg1 as u32 + p.tseg2 as u32;
            assert_eq!(40_000_000 / (p.brp as u32 * tq), rate);
            assert!(p.validate().is_ok());
            assert!(p.sjw <= p.tseg2);
        }
        for (p, rate) in [
            (DataBitTiming::MBPS2_40MHZ, 2_000_000u32),
            (DataBitTiming::MBPS5_40MHZ, 5_000_000),
            (DataBitTiming::MBPS8_40MHZ, 8_000_000),
        ] {
            let tq = 1 + p.tseg1 as u32 + p.tseg2 as u32;
            assert_eq!(40_000_000 / (p.brp as u32 * tq), rate);
            assert!(p.validate().is_ok());
        }
    }

    #[test]
    fn timing_validation() {
        let mut t = NominalBitTiming::KBPS500_40MHZ;
        t.brp = 0;
        assert!(t.validate().is_err());
        t = NominalBitTiming::KBPS500_40MHZ;
        t.sjw = t.tseg2 + 1;
        assert!(t.validate().is_err());
        let mut d = DataBitTiming::MBPS2_40MHZ;
        d.tseg1 = 33;
        assert!(d.validate().is_err());
    }

    #[test]
    fn tdco_follows_recipe() {
        // TDCO = DBRP * DTSEG1, clamped to 63 (7-bit signed max).
        assert_eq!(DataBitTiming::MBPS2_40MHZ.tdco(), 15);
        let big = DataBitTiming { brp: 16, tseg1: 16, tseg2: 4, sjw: 4 };
        assert_eq!(big.tdco(), 63);
    }

    #[test]
    fn filter_match() {
        let id = Id::Standard(StandardId::new(0x123).unwrap());
        let m = FilterMatch::exact(id);
        assert_eq!(m.fltobj, 0x123);
        // Mask: all 11 SID bits + MIDE (bit 30).
        assert_eq!(m.mask, 0x7FF | (1 << 30));
        let eid = Id::Extended(ExtendedId::new(0x0CFE_6E01).unwrap());
        let me = FilterMatch::exact(eid);
        assert_eq!(me.fltobj & (1 << 30), 1 << 30); // EXIDE set
        assert_eq!(me.mask, 0x1FFF_FFFF | (1 << 30));
        assert_eq!(FilterMatch::accept_all().mask, 0);
    }
}
```

- [ ] **Step 2: Wire the module and re-exports in `src/lib.rs`, run tests to verify failure**

Run: `cargo test`
Expected: FAIL.

- [ ] **Step 3: Implement above the tests**

```rust
//! Driver configuration: clocks, bit timing, filters.

use crate::error::ConfigError;
use crate::registers::objects::pack_id;
use crate::registers::{CiDbtCfg, CiNbtCfg};
use embedded_can::Id;

/// The maximum safe SPI clock for a given SYSCLK, per the MCP2517FD
/// "fast SPI corrupts RAM reads" erratum: `0.85 * SYSCLK / 2`.
///
/// The driver cannot observe the actual SPI clock through `SpiDevice` —
/// configure your bus at or below this and the init-time RAM echo test
/// will confirm it.
pub const fn max_spi_hz(sysclk_hz: u32) -> u32 {
    (sysclk_hz / 2 / 100) * 85
}

/// Oscillator configuration.
///
/// SYSCLK = `xtal_hz * 10` (if `pll`) `/ 2` (if `sclk_div2`); SYSCLK must
/// land in `2..=40 MHz`, and the PLL input must be a 4 MHz-class crystal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ClockConfig {
    /// Crystal / external clock frequency in Hz (`4..=40 MHz` crystal,
    /// `2..=40 MHz` external clock).
    pub xtal_hz: u32,
    /// Enable the 10x PLL (only from a 4 MHz-class input).
    pub pll: bool,
    /// Divide SYSCLK by 2 (`OSC.SCLKDIV`).
    pub sclk_div2: bool,
}

impl ClockConfig {
    /// 40 MHz crystal, no PLL — the recommended configuration.
    pub const MHZ40: Self = Self { xtal_hz: 40_000_000, pll: false, sclk_div2: false };
    /// 20 MHz crystal, no PLL.
    pub const MHZ20: Self = Self { xtal_hz: 20_000_000, pll: false, sclk_div2: false };
    /// 4 MHz crystal with 10x PLL -> 40 MHz SYSCLK (PLL adds lock time).
    pub const MHZ4_PLL: Self = Self { xtal_hz: 4_000_000, pll: true, sclk_div2: false };

    /// The resulting SYSCLK in Hz.
    pub const fn sysclk_hz(&self) -> u32 {
        let base = if self.pll { self.xtal_hz * 10 } else { self.xtal_hz };
        if self.sclk_div2 { base / 2 } else { base }
    }

    /// Checks PLL-input and SYSCLK range constraints.
    pub const fn validate(&self) -> Result<(), ConfigError> {
        if self.pll && self.xtal_hz > 5_000_000 {
            return Err(ConfigError::Clock);
        }
        let sysclk = self.sysclk_hz();
        if sysclk < 2_000_000 || sysclk > 40_000_000 {
            return Err(ConfigError::Clock);
        }
        Ok(())
    }
}

/// Nominal (arbitration-phase) bit timing in time quanta (TQ = BRP/SYSCLK).
///
/// Bit time = `(1 + tseg1 + tseg2) * TQ`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NominalBitTiming {
    /// Baud rate prescaler (`1..=256`).
    pub brp: u16,
    /// Time segment 1: propagation + phase 1 (`2..=256` TQ).
    pub tseg1: u16,
    /// Time segment 2: phase 2 (`1..=128` TQ).
    pub tseg2: u16,
    /// Synchronization jump width (`1..=128` TQ, `<= tseg2`).
    pub sjw: u16,
}

impl NominalBitTiming {
    /// 125 kbit/s at 40 MHz SYSCLK (160 TQ, 80% sample point).
    pub const KBPS125_40MHZ: Self = Self { brp: 2, tseg1: 127, tseg2: 32, sjw: 32 };
    /// 250 kbit/s at 40 MHz SYSCLK (160 TQ, 80% sample point).
    pub const KBPS250_40MHZ: Self = Self { brp: 1, tseg1: 127, tseg2: 32, sjw: 32 };
    /// 500 kbit/s at 40 MHz SYSCLK (80 TQ, 80% sample point).
    pub const KBPS500_40MHZ: Self = Self { brp: 1, tseg1: 63, tseg2: 16, sjw: 16 };
    /// 1 Mbit/s at 40 MHz SYSCLK (40 TQ, 80% sample point).
    pub const MBPS1_40MHZ: Self = Self { brp: 1, tseg1: 31, tseg2: 8, sjw: 8 };

    /// Range-checks all fields.
    pub const fn validate(&self) -> Result<(), ConfigError> {
        if self.brp < 1 || self.brp > 256
            || self.tseg1 < 2 || self.tseg1 > 256
            || self.tseg2 < 1 || self.tseg2 > 128
            || self.sjw < 1 || self.sjw > 128
            || self.sjw > self.tseg2
        {
            return Err(ConfigError::NominalBitTiming);
        }
        Ok(())
    }

    /// Encodes into `CiNBTCFG`. Call [`Self::validate`] first.
    pub const fn to_reg(&self) -> CiNbtCfg {
        CiNbtCfg::new(self.brp, self.tseg1, self.tseg2, self.sjw)
    }
}

/// Data-phase bit timing for CAN FD (BRS), in time quanta.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DataBitTiming {
    /// Baud rate prescaler (`1..=256`; keep equal to the nominal BRP,
    /// ideally 1, to minimize quantization error).
    pub brp: u16,
    /// Time segment 1 (`1..=32` TQ).
    pub tseg1: u8,
    /// Time segment 2 (`1..=16` TQ).
    pub tseg2: u8,
    /// Synchronization jump width (`1..=16` TQ, `<= tseg2`).
    pub sjw: u8,
}

impl DataBitTiming {
    /// 2 Mbit/s at 40 MHz SYSCLK (20 TQ, 80% sample point).
    pub const MBPS2_40MHZ: Self = Self { brp: 1, tseg1: 15, tseg2: 4, sjw: 4 };
    /// 5 Mbit/s at 40 MHz SYSCLK (8 TQ, 75% sample point).
    pub const MBPS5_40MHZ: Self = Self { brp: 1, tseg1: 5, tseg2: 2, sjw: 2 };
    /// 8 Mbit/s at 40 MHz SYSCLK (5 TQ, 80% sample point).
    pub const MBPS8_40MHZ: Self = Self { brp: 1, tseg1: 3, tseg2: 1, sjw: 1 };

    /// Range-checks all fields.
    pub const fn validate(&self) -> Result<(), ConfigError> {
        if self.brp < 1 || self.brp > 256
            || self.tseg1 < 1 || self.tseg1 > 32
            || self.tseg2 < 1 || self.tseg2 > 16
            || self.sjw < 1 || self.sjw > 16
            || self.sjw > self.tseg2
        {
            return Err(ConfigError::DataBitTiming);
        }
        Ok(())
    }

    /// Encodes into `CiDBTCFG`. Call [`Self::validate`] first.
    pub const fn to_reg(&self) -> CiDbtCfg {
        CiDbtCfg::new(self.brp, self.tseg1, self.tseg2, self.sjw)
    }

    /// Transmitter delay compensation offset for auto TDC mode:
    /// `DBRP * DTSEG1`, clamped to the 7-bit signed maximum of 63
    /// (SYSCLK cycles). Standard recipe, also used by the Linux driver.
    pub const fn tdco(&self) -> i8 {
        let v = self.brp as u32 * self.tseg1 as u32;
        if v > 63 { 63 } else { v as i8 }
    }
}

/// Complete controller configuration for [`init`](crate::MCP251xFd::init).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Oscillator configuration.
    pub clock: ClockConfig,
    /// Nominal bit timing.
    pub nominal: NominalBitTiming,
    /// Data-phase bit timing; `None` disables CAN FD bit rate switching.
    pub data: Option<DataBitTiming>,
}

impl Config {
    /// Validates every part of the configuration.
    pub const fn validate(&self) -> Result<(), ConfigError> {
        if let Err(e) = self.clock.validate() {
            return Err(e);
        }
        if let Err(e) = self.nominal.validate() {
            return Err(e);
        }
        if let Some(d) = self.data {
            if let Err(e) = d.validate() {
                return Err(e);
            }
        }
        Ok(())
    }
}

/// An acceptance filter: `CiFLTOBJ` value plus `CiMASK` value.
///
/// A received frame matches when `(frame_id ^ fltobj) & mask == 0`
/// (per-bit: mask 1 = must match, 0 = don't care). Bit 30 (`MIDE`) makes
/// the standard/extended distinction part of the match.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FilterMatch {
    /// Raw `CiFLTOBJm` value (packed ID + `EXIDE` bit 30).
    pub fltobj: u32,
    /// Raw `CiMASKm` value (packed ID mask + `MIDE` bit 30).
    pub mask: u32,
}

impl FilterMatch {
    const IDE_BIT: u32 = 1 << 30;

    /// Matches exactly one identifier (and its standard/extended kind).
    pub fn exact(id: Id) -> Self {
        let (obj, id_mask) = match id {
            Id::Standard(_) => (pack_id(id), 0x7FF),
            Id::Extended(_) => (pack_id(id) | Self::IDE_BIT, 0x1FFF_FFFF),
        };
        Self { fltobj: obj, mask: id_mask | Self::IDE_BIT }
    }

    /// Matches every frame, standard and extended.
    pub fn accept_all() -> Self {
        Self { fltobj: 0, mask: 0 }
    }

    /// Matches `id` under a custom mask over the *natural* identifier bits
    /// (11-bit for standard, 29-bit for extended); the mask is packed into
    /// the register layout for you. The standard/extended kind always
    /// participates in the match (`MIDE` set).
    pub fn with_mask(id: Id, id_mask: u32) -> Self {
        let packed_mask = match id {
            Id::Standard(_) => id_mask & 0x7FF,
            Id::Extended(_) => {
                ((id_mask >> 18) & 0x7FF) | ((id_mask & 0x3_FFFF) << 11)
            }
        };
        let obj = match id {
            Id::Standard(_) => pack_id(id),
            Id::Extended(_) => pack_id(id) | Self::IDE_BIT,
        };
        Self { fltobj: obj, mask: packed_mask | Self::IDE_BIT }
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test && cargo test --all-features`
Expected: PASS.

- [ ] **Step 5: Lint and commit**

```bash
cargo clippy --all-features -- -D warnings && cargo fmt
git add src/lib.rs src/config.rs
git commit -m "feat: clock and bit-timing config with 40 MHz presets, filter matches"
```

---

### Task 9: Driver struct and init sequence (`src/driver.rs`)

**Files:**
- Create: `src/driver.rs`
- Modify: `src/lib.rs` (add `mod driver;`, `pub use driver::MCP251xFd;`, and `#[cfg(feature = "async")] pub use driver::MCP251xFdAsync;`)
- Create: `tests/driver.rs`

**Interfaces:**
- Consumes: `Bus`/`BusAsync`, all register types, `Config`, `Error`, `Variant`.
- Produces:
  - `MCP251xFd<SPI: embedded_hal::spi::SpiDevice>` / `MCP251xFdAsync<SPI: embedded_hal_async::spi::SpiDevice>` with:
    - `new(spi: SPI) -> Self`
    - `release(self) -> SPI`
    - `async fn reset(&mut self) -> Result<(), Error<SPI::Error>>` (async only on the Async variant — same for all methods below)
    - `async fn init<D: DelayNs>(&mut self, config: &Config, delay: &mut D) -> Result<Variant, Error<SPI::Error>>`
  - Init sequence (exact SPI operations, in order): RESET → 700 µs delay → RAM echo test at 0xBFC → write `OSC` → poll `OSC` ready (≤ 40 tries, 100 µs apart) → write `OSC|LPMEN`, read back (variant), rewrite `OSC` → 32 × 64-byte RAM zero writes (0x400..0xC00) → `C1NBTCFG` → [`C1DBTCFG` if FD] → `C1TDC` → `C1CON` (ISO CRC, REQOP=Configuration) → `C1INT = 0`.

- [ ] **Step 1: Create `tests/driver.rs` with failing tests**

```rust
//! Byte-exact integration tests for the sync driver against a mock SPI bus.

use embedded_hal_mock::eh1::delay::NoopDelay;
use embedded_hal_mock::eh1::spi::{Mock, Transaction};
use mcp251xfd::{ClockConfig, Config, DataBitTiming, Error, MCP251xFd, NominalBitTiming, Variant};

/// One WRITE-register transaction: command word + 4 LE data bytes.
fn w32(addr: u16, val: u32) -> Vec<Transaction> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x20 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::write_vec(val.to_le_bytes().to_vec()),
        Transaction::transaction_end(),
    ]
}

/// One READ-register transaction returning `val`.
fn r32(addr: u16, val: u32) -> Vec<Transaction> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x30 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::read_vec(val.to_le_bytes().to_vec()),
        Transaction::transaction_end(),
    ]
}

fn wram(addr: u16, data: &[u8]) -> Vec<Transaction> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x20 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::write_vec(data.to_vec()),
        Transaction::transaction_end(),
    ]
}

fn rram(addr: u16, data: &[u8]) -> Vec<Transaction> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x30 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::read_vec(data.to_vec()),
        Transaction::transaction_end(),
    ]
}

fn reset_txn() -> Vec<Transaction> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x00, 0x00]),
        Transaction::transaction_end(),
    ]
}

pub const TEST_CONFIG: Config = Config {
    clock: ClockConfig::MHZ40,
    nominal: NominalBitTiming::KBPS500_40MHZ,
    data: Some(DataBitTiming::MBPS2_40MHZ),
};

/// The full expected init sequence for `TEST_CONFIG` on an MCP2517FD
/// (LPMEN does not stick), with the oscillator ready on the first poll.
fn init_expectations() -> Vec<Transaction> {
    let mut e = Vec::new();
    e.extend(reset_txn());
    e.extend(wram(0xBFC, &0xAA55_AA55u32.to_le_bytes()));
    e.extend(rram(0xBFC, &0xAA55_AA55u32.to_le_bytes()));
    e.extend(w32(0xE00, 0x0000_0060)); // OSC: CLKODIV=0b11 (default), no PLL
    e.extend(r32(0xE00, 0x0000_0460)); // OSCRDY set
    e.extend(w32(0xE00, 0x0000_0068)); // probe LPMEN
    e.extend(r32(0xE00, 0x0000_0460)); // LPMEN did not stick -> MCP2517FD
    e.extend(w32(0xE00, 0x0000_0060)); // clear LPMEN
    for i in 0..32u16 {
        e.extend(wram(0x400 + i * 64, &[0u8; 64]));
    }
    e.extend(w32(0x004, 0x003E_0F0F)); // NBTCFG: brp1, tseg1 63, tseg2 16, sjw 16
    e.extend(w32(0x008, 0x000E_0303)); // DBTCFG: brp1, tseg1 15, tseg2 4, sjw 4
    e.extend(w32(0x00C, 0x0202_0F00)); // TDC: auto, TDCO=15, edge filter
    e.extend(w32(0x000, 0x0400_0020)); // CiCON: ISOCRCEN, REQOP=Configuration
    e.extend(w32(0x01C, 0x0000_0000)); // CiINT cleared, all disabled
    e
}

#[test]
fn init_full_sequence_detects_2517fd() {
    let mut spi = Mock::new(&init_expectations());
    let mut can = MCP251xFd::new(&mut spi);
    let variant = can.init(&TEST_CONFIG, &mut NoopDelay).unwrap();
    assert_eq!(variant, Variant::Mcp2517Fd);
    spi.done();
}

#[test]
fn init_detects_2518fd_when_lpmen_sticks() {
    let mut e = Vec::new();
    e.extend(reset_txn());
    e.extend(wram(0xBFC, &0xAA55_AA55u32.to_le_bytes()));
    e.extend(rram(0xBFC, &0xAA55_AA55u32.to_le_bytes()));
    e.extend(w32(0xE00, 0x0000_0060));
    e.extend(r32(0xE00, 0x0000_0460));
    e.extend(w32(0xE00, 0x0000_0068));
    e.extend(r32(0xE00, 0x0000_0468)); // LPMEN stuck -> MCP2518FD
    e.extend(w32(0xE00, 0x0000_0060));
    for i in 0..32u16 {
        e.extend(wram(0x400 + i * 64, &[0u8; 64]));
    }
    e.extend(w32(0x004, 0x003E_0F0F));
    e.extend(w32(0x008, 0x000E_0303));
    e.extend(w32(0x00C, 0x0202_0F00));
    e.extend(w32(0x000, 0x0400_0020));
    e.extend(w32(0x01C, 0x0000_0000));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert_eq!(can.init(&TEST_CONFIG, &mut NoopDelay).unwrap(), Variant::Mcp2518Fd);
    spi.done();
}

#[test]
fn init_fails_on_bad_echo() {
    let mut e = Vec::new();
    e.extend(reset_txn());
    e.extend(wram(0xBFC, &0xAA55_AA55u32.to_le_bytes()));
    e.extend(rram(0xBFC, &[0x00, 0x00, 0x00, 0x00])); // dead bus
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(matches!(
        can.init(&TEST_CONFIG, &mut NoopDelay),
        Err(Error::CommunicationCheckFailed)
    ));
    spi.done();
}
```

Note: the mock helpers (`w32`, `r32`, `wram`, `rram`, `reset_txn`, `TEST_CONFIG`) are reused by Tasks 10-12 — they stay in this file, and later tests are added to the same `tests/driver.rs`.

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test`
Expected: FAIL — `MCP251xFd` does not exist.

- [ ] **Step 3: Create `src/driver.rs`, wire `src/lib.rs`**

```rust
//! The MCP251XFD driver.

use crate::bus::Bus;
#[cfg(feature = "async")]
use crate::bus::BusAsync;
use crate::config::Config;
use crate::error::Error;
use crate::registers::{addr, CiCon, CiTdc, OperationMode, Osc, TdcMode, Variant};
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::SpiDevice;
#[cfg(feature = "async")]
use embedded_hal_async::delay::DelayNs as DelayNsAsync;
#[cfg(feature = "async")]
use embedded_hal_async::spi::SpiDevice as SpiDeviceAsync;

/// Driver for one MCP2517FD / MCP2518FD / MCP251863 chip.
///
/// Generic over [`embedded_hal::spi::SpiDevice`]; chip-select framing is the
/// SPI device's job. Keep the SPI clock at or below
/// [`max_spi_hz`](crate::max_spi_hz)`(sysclk)` (silicon erratum).
///
/// The `async` feature additionally provides `MCP251xFdAsync` with the same
/// API over [`embedded_hal_async::spi::SpiDevice`].
#[maybe_async_cfg::maybe(sync(keep_self), async(feature = "async", idents(Bus(async = "BusAsync"))))]
pub struct MCP251xFd<SPI> {
    bus: Bus<SPI>,
    seq_mask: u32,
}

#[maybe_async_cfg::maybe(
    sync(keep_self),
    async(
        feature = "async",
        idents(
            Bus(async = "BusAsync"),
            SpiDevice(async = "SpiDeviceAsync"),
            DelayNs(async = "DelayNsAsync"),
        )
    )
)]
impl<SPI: SpiDevice> MCP251xFd<SPI> {
    /// Creates a driver over an SPI device. Call [`Self::init`] next.
    pub fn new(spi: SPI) -> Self {
        Self { bus: Bus { spi }, seq_mask: Variant::Mcp2517Fd.seq_mask() }
    }

    /// Consumes the driver and returns the SPI device.
    pub fn release(self) -> SPI {
        self.bus.spi
    }

    /// Sends the RESET instruction. The chip returns to Configuration mode
    /// with default registers.
    pub async fn reset(&mut self) -> Result<(), Error<SPI::Error>> {
        self.bus.reset().await
    }

    /// Resets and fully initializes the chip; returns the detected variant.
    ///
    /// Sequence (see spec §3.5): reset, RAM echo check, oscillator setup and
    /// ready-poll, variant detection via `OSC.LPMEN`, message-RAM zero fill,
    /// bit timing, `CiCON` (ISO CRC, stays in Configuration mode), interrupts
    /// cleared. Configure FIFOs/filters afterwards, then switch modes.
    pub async fn init<D: DelayNs>(
        &mut self,
        config: &Config,
        delay: &mut D,
    ) -> Result<Variant, Error<SPI::Error>> {
        config.validate().map_err(Error::InvalidConfig)?;
        self.bus.reset().await?;
        delay.delay_us(700).await;

        // SPI sanity check: catches wiring and over-spec SPI clocks early.
        const ECHO: u32 = 0xAA55_AA55;
        const ECHO_ADDR: u16 = 0xBFC; // last RAM word
        self.bus.write_ram(ECHO_ADDR, &ECHO.to_le_bytes()).await?;
        let mut echo = [0u8; 4];
        self.bus.read_ram(ECHO_ADDR, &mut echo).await?;
        if u32::from_le_bytes(echo) != ECHO {
            return Err(Error::CommunicationCheckFailed);
        }

        // Oscillator. CLKODIV keeps its power-on default (divide by 10).
        let osc = Osc(0)
            .with_pll_enabled(config.clock.pll)
            .with_sclk_div2(config.clock.sclk_div2)
            .with_clko_div(0b11);
        self.bus.write_sfr32(addr::OSC, osc.0).await?;
        let mut ready = false;
        for _ in 0..40 {
            let st = Osc(self.bus.read_sfr32(addr::OSC).await?);
            if st.osc_ready() && (!config.clock.pll || st.pll_ready()) {
                ready = true;
                break;
            }
            delay.delay_us(100).await;
        }
        if !ready {
            return Err(Error::ClockNotReady);
        }

        // Variant detection: LPMEN is implemented on MCP2518FD only.
        self.bus.write_sfr32(addr::OSC, osc.with_lpmen(true).0).await?;
        let variant = if Osc(self.bus.read_sfr32(addr::OSC).await?).lpmen() {
            Variant::Mcp2518Fd
        } else {
            Variant::Mcp2517Fd
        };
        self.bus.write_sfr32(addr::OSC, osc.0).await?;
        self.seq_mask = variant.seq_mask();

        // Zero the message RAM (ECC stays disabled in this version).
        let zeros = [0u8; 64];
        let mut a = addr::RAM_START;
        while (a as usize) < addr::RAM_START as usize + addr::RAM_SIZE {
            self.bus.write_ram(a, &zeros).await?;
            a += 64;
        }

        // Bit timing.
        self.bus.write_sfr32(addr::C1NBTCFG, config.nominal.to_reg().0).await?;
        let mut tdc = CiTdc(0);
        if let Some(data) = config.data {
            self.bus.write_sfr32(addr::C1DBTCFG, data.to_reg().0).await?;
            tdc = tdc
                .with_mode(TdcMode::Auto)
                .with_tdco(data.tdco())
                .with_edge_filter(true);
        }
        self.bus.write_sfr32(addr::C1TDC, tdc.0).await?;

        // CiCON: ISO CRC on, remain in Configuration mode.
        let con = CiCon(0)
            .with_iso_crc_enabled(true)
            .with_req_op_mode(OperationMode::Configuration);
        self.bus.write_sfr32(addr::C1CON, con.0).await?;

        // All interrupt flags cleared, all enables off.
        self.bus.write_sfr32(addr::C1INT, 0).await?;

        Ok(variant)
    }
}
```

In `src/lib.rs` add:

```rust
mod driver;

pub use driver::MCP251xFd;
#[cfg(feature = "async")]
pub use driver::MCP251xFdAsync;
```

Zero-warning note: `seq_mask` is written here but only read from Task 11's `transmit` — until then, annotate the field with `#[allow(dead_code)] // read by transmit (Task 11)` and remove the allow in Task 11.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test && cargo test --all-features`
Expected: PASS — the three driver tests plus all earlier tests. If the mock reports a transaction mismatch, the failure message names the first differing byte sequence: compare against the init sequence order in this task's Interfaces block before touching register values (order bugs are more likely than value bugs).

- [ ] **Step 5: Lint and commit**

```bash
cargo clippy --all-features -- -D warnings && cargo fmt
git add src/lib.rs src/driver.rs tests/driver.rs
git commit -m "feat: driver init sequence with variant detection and RAM echo check"
```

---

### Task 10: Mode changes, FIFO layout application, filters

**Files:**
- Modify: `src/driver.rs` (methods added to the existing `maybe`-annotated impl block)
- Modify: `tests/driver.rs` (append tests)

**Interfaces:**
- Consumes: `FifoLayout`/`FifoDirection` (Task 6), `FilterMatch` (Task 8), `CiFifoCon` (Task 3), test helpers from Task 9.
- Produces:
  - `async fn set_mode<D: DelayNs>(&mut self, mode: OperationMode, delay: &mut D) -> Result<(), Error<SPI::Error>>` — read-modify-write `CiCON.REQOP`, poll `OPMOD` (≤ 40 tries, 100 µs apart), `ModeChangeTimeout` on failure
  - `async fn apply_layout(&mut self, layout: &FifoLayout) -> Result<(), Error<SPI::Error>>` — requires Configuration mode (`NotInConfigMode` otherwise); RX FIFOs get `TFNRFNIE | RXOVIE`, TX FIFOs get `TXEN`; all get `FRESET`, depth, payload size
  - `async fn set_filter(&mut self, filter: Filter, matcher: FilterMatch, target: Fifo) -> Result<(), Error<SPI::Error>>` — disable byte, `FLTOBJ`, `MASK`, enable byte `0x80 | fifo`
  - `async fn disable_filter(&mut self, filter: Filter) -> Result<(), Error<SPI::Error>>`

- [ ] **Step 1: Append failing tests to `tests/driver.rs`**

Add a byte-wise write helper next to the existing ones (the bus sends the command and the value as two write operations):

```rust
fn w8(addr: u16, val: u8) -> Vec<Transaction> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x20 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::write_vec(vec![val]),
        Transaction::transaction_end(),
    ]
}
```

```rust
use mcp251xfd::{Fifo, FifoLayout, Filter, FilterMatch, OperationMode, PayloadSize};
use embedded_can::{Id, StandardId};

#[test]
fn set_mode_normal_fd() {
    let mut e = Vec::new();
    e.extend(r32(0x000, 0x0480_0020)); // OPMOD=Config, REQOP=Config, ISOCRC
    e.extend(w32(0x000, 0x0080_0020)); // REQOP := NormalFd (0)
    e.extend(r32(0x000, 0x0000_0020)); // OPMOD now NormalFd
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.set_mode(OperationMode::NormalFd, &mut NoopDelay).unwrap();
    spi.done();
}

#[test]
fn apply_layout_writes_fifo_configs() {
    const LAYOUT: FifoLayout = FifoLayout::new()
        .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
        .rx_fifo(Fifo::F2, PayloadSize::B64, 8);
    let mut e = Vec::new();
    e.extend(r32(0x000, 0x0080_0000)); // OPMOD=Config
    // TX: PLSIZE=7<<29 | FSIZE=3<<24 | FRESET | TXEN.
    e.extend(w32(0x05C, 0xE300_0480));
    // RX: PLSIZE=7<<29 | FSIZE=7<<24 | FRESET | RXOVIE | TFNRFNIE.
    e.extend(w32(0x068, 0xE700_0409));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.apply_layout(&LAYOUT).unwrap();
    spi.done();
}

#[test]
fn apply_layout_requires_config_mode() {
    let mut e = Vec::new();
    e.extend(r32(0x000, 0x0000_0000)); // OPMOD=NormalFd
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    let layout = FifoLayout::new().rx_fifo(Fifo::F1, PayloadSize::B8, 1);
    assert!(matches!(can.apply_layout(&layout), Err(Error::NotInConfigMode)));
    spi.done();
}

#[test]
fn set_filter_exact_standard_id() {
    let id = Id::Standard(StandardId::new(0x123).unwrap());
    let mut e = Vec::new();
    e.extend(w8(0x1D0, 0x00)); // disable filter 0 while editing
    e.extend(w32(0x1F0, 0x0000_0123)); // FLTOBJ
    e.extend(w32(0x1F4, 0x4000_07FF)); // MASK: SID bits + MIDE
    e.extend(w8(0x1D0, 0x82)); // enable, point to FIFO 2
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.set_filter(Filter::F0, FilterMatch::exact(id), Fifo::F2).unwrap();
    spi.done();
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --test driver`
Expected: FAIL — methods missing.

- [ ] **Step 3: Implement in `src/driver.rs` (inside the existing `maybe` impl block)**

Add imports: `use crate::config::FilterMatch;`, `use crate::registers::ram::{FifoDirection, FifoLayout};`, `use crate::registers::{CiFifoCon, Fifo, Filter};`.

```rust
    /// Requests an operation mode and waits (≤ ~4 ms) until the chip
    /// reports it. Preserves the rest of `CiCON`.
    pub async fn set_mode<D: DelayNs>(
        &mut self,
        mode: OperationMode,
        delay: &mut D,
    ) -> Result<(), Error<SPI::Error>> {
        let con = CiCon(self.bus.read_sfr32(addr::C1CON).await?);
        self.bus
            .write_sfr32(addr::C1CON, con.with_req_op_mode(mode).0)
            .await?;
        for _ in 0..40 {
            let now = CiCon(self.bus.read_sfr32(addr::C1CON).await?);
            if now.op_mode() == mode {
                return Ok(());
            }
            delay.delay_us(100).await;
        }
        Err(Error::ModeChangeTimeout)
    }

    /// Writes the FIFO configuration registers for a validated layout.
    ///
    /// The chip allocates RAM for the FIFOs itself; the layout only needs
    /// to fit (guaranteed by [`FifoLayout`]'s construction). Requires
    /// Configuration mode. RX FIFOs are configured with not-empty and
    /// overflow interrupts enabled at the FIFO level; whether they reach
    /// the INT pin is controlled by `configure_interrupts` (Task 12; make
    /// this an intra-doc link once that method exists).
    pub async fn apply_layout(&mut self, layout: &FifoLayout) -> Result<(), Error<SPI::Error>> {
        let con = CiCon(self.bus.read_sfr32(addr::C1CON).await?);
        if con.op_mode() != OperationMode::Configuration {
            return Err(Error::NotInConfigMode);
        }
        for (fifo, entry) in layout.entries() {
            let mut reg = CiFifoCon(0)
                .with_fifo_size(entry.depth)
                .with_payload_size(entry.payload)
                .with_freset(true);
            reg = match entry.direction {
                FifoDirection::Transmit => reg.with_tx(true),
                FifoDirection::Receive => {
                    reg.with_not_full_empty_ie(true).with_rx_overflow_ie(true)
                }
            };
            self.bus.write_sfr32(addr::fifo_con(fifo), reg.0).await?;
        }
        Ok(())
    }

    /// Configures and enables an acceptance filter routing matches into
    /// `target`. The filter is disabled while its registers are updated
    /// (hardware requirement).
    pub async fn set_filter(
        &mut self,
        filter: Filter,
        matcher: FilterMatch,
        target: Fifo,
    ) -> Result<(), Error<SPI::Error>> {
        self.bus.write_sfr8(addr::flt_con_byte(filter), 0x00).await?;
        self.bus.write_sfr32(addr::flt_obj(filter), matcher.fltobj).await?;
        self.bus.write_sfr32(addr::flt_mask(filter), matcher.mask).await?;
        self.bus
            .write_sfr8(addr::flt_con_byte(filter), 0x80 | target.index())
            .await
    }

    /// Disables an acceptance filter.
    pub async fn disable_filter(&mut self, filter: Filter) -> Result<(), Error<SPI::Error>> {
        self.bus.write_sfr8(addr::flt_con_byte(filter), 0x00).await
    }
```

(`configure_interrupts` is referenced in a doc comment but lands in Task 12 — use plain text, not an intra-doc link, until Task 12 makes it resolvable, otherwise `cargo doc -D warnings` fails.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test && cargo test --all-features`
Expected: PASS.

- [ ] **Step 5: Lint and commit**

```bash
cargo clippy --all-features -- -D warnings && cargo fmt
git add src/driver.rs tests/driver.rs
git commit -m "feat: operation modes, FIFO layout application, acceptance filters"
```

---

### Task 11: Transmit path

**Files:**
- Modify: `src/driver.rs` (add `seq` field; add `transmit`, `transmit_fd`, private `transmit_raw`)
- Modify: `tests/driver.rs` (append tests)

**Interfaces:**
- Consumes: `TxHeader`, `len_to_dlc` (Task 4), `Frame`/`FdFrame` (Task 5), `CiFifoSta`, `CiFifoCon::CON_BYTE1_UINC_TXREQ`.
- Produces:
  - `async fn transmit(&mut self, fifo: Fifo, frame: &Frame) -> Result<(), Error<SPI::Error>>` — classic frames (incl. RTR)
  - `async fn transmit_fd(&mut self, fifo: Fifo, frame: &FdFrame) -> Result<(), Error<SPI::Error>>` — FDF always set, BRS/ESI from flags
  - Both non-blocking: `Err(Error::TxFifoFull)` when the FIFO has no free slot. Sequence numbers auto-increment per driver instance, masked to the detected variant's width.
  - Struct change: add `seq: u32` (init 0 in `new`); **remove the `#[allow(dead_code)]` from `seq_mask`**.

- [ ] **Step 1: Append failing tests to `tests/driver.rs`**

```rust
use mcp251xfd::{FdFrame, Frame, FrameFlags};
use embedded_can::Frame as _;

#[test]
fn transmit_classic_frame() {
    let frame = Frame::new(StandardId::new(0x123).unwrap(), &[1, 2, 3, 4]).unwrap();
    let mut e = Vec::new();
    // First transmit: seq 0.
    e.extend(r32(0x060, 0x0000_0001)); // CiFIFOSTA1: not full
    e.extend(r32(0x064, 0x0000_0000)); // CiFIFOUA1: offset 0
    e.extend(wram(0x400, &[
        0x23, 0x01, 0x00, 0x00, // T0: SID 0x123
        0x04, 0x00, 0x00, 0x00, // T1: DLC 4, SEQ 0
        0x01, 0x02, 0x03, 0x04, // payload
    ]));
    e.extend(w8(0x05D, 0x03)); // CiFIFOCON1 byte1: UINC | TXREQ
    // Second transmit: seq increments, chip UA advanced to 0x10.
    e.extend(r32(0x060, 0x0000_0001));
    e.extend(r32(0x064, 0x0000_0010));
    e.extend(wram(0x410, &[
        0x23, 0x01, 0x00, 0x00,
        0x04, 0x02, 0x00, 0x00, // T1: DLC 4, SEQ 1 (1 << 9)
        0x01, 0x02, 0x03, 0x04,
    ]));
    e.extend(w8(0x05D, 0x03));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.transmit(Fifo::F1, &frame).unwrap();
    can.transmit(Fifo::F1, &frame).unwrap();
    spi.done();
}

#[test]
fn transmit_full_fifo_errors() {
    let frame = Frame::new(StandardId::new(0x123).unwrap(), &[]).unwrap();
    let mut e = Vec::new();
    e.extend(r32(0x060, 0x0000_0000)); // full: TFNRFNIF clear
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(matches!(can.transmit(Fifo::F1, &frame), Err(Error::TxFifoFull)));
    spi.done();
}

#[test]
fn transmit_fd_frame_with_brs() {
    // 12-byte payload -> DLC 9; FDF | BRS set; payload already word-aligned.
    let frame = FdFrame::new(
        StandardId::new(0x7F).unwrap(),
        &[0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB],
        FrameFlags { brs: true, esi: false },
    )
    .unwrap();
    let mut e = Vec::new();
    e.extend(r32(0x060, 0x0000_0001));
    e.extend(r32(0x064, 0x0000_0000));
    // T1 = DLC 9 | BRS(1<<6) | FDF(1<<7) | SEQ 0 = 0xC9.
    let mut obj = vec![0x7F, 0x00, 0x00, 0x00, 0xC9, 0x00, 0x00, 0x00];
    obj.extend_from_slice(frame.data());
    e.extend(wram(0x400, &obj));
    e.extend(w8(0x05D, 0x03));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.transmit_fd(Fifo::F1, &frame).unwrap();
    spi.done();
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --test driver`
Expected: FAIL.

- [ ] **Step 3: Implement in `src/driver.rs`**

Struct becomes (both fields documented — they're private, but keep the comments):

```rust
pub struct MCP251xFd<SPI> {
    bus: Bus<SPI>,
    // Next TX sequence number (echoed in the TEF); masked per variant.
    seq: u32,
    seq_mask: u32,
}
```

`new` initializes `seq: 0`. Add imports: `use crate::frame::{FdFrame, Frame};`, `use crate::registers::objects::{len_to_dlc, TxHeader};`, `use crate::registers::CiFifoSta;`.

```rust
    /// Queues a classic CAN 2.0 frame on a transmit FIFO and requests
    /// transmission. Non-blocking: [`Error::TxFifoFull`] when no slot is
    /// free (wait for a TX interrupt or retry).
    pub async fn transmit(&mut self, fifo: Fifo, frame: &Frame) -> Result<(), Error<SPI::Error>> {
        let header = TxHeader {
            id: frame.id(),
            dlc: frame.dlc() as u8,
            rtr: frame.is_remote(),
            brs: false,
            fdf: false,
            esi: false,
            seq: 0,
        };
        self.transmit_raw(fifo, header, frame.data()).await
    }

    /// Queues a CAN FD frame. Same contract as [`Self::transmit`].
    pub async fn transmit_fd(&mut self, fifo: Fifo, frame: &FdFrame) -> Result<(), Error<SPI::Error>> {
        let dlc = match len_to_dlc(frame.data().len(), true) {
            Some(d) => d,
            None => return Err(Error::InvalidPayloadLength),
        };
        let header = TxHeader {
            id: frame.id(),
            dlc,
            rtr: false,
            brs: frame.flags().brs,
            fdf: true,
            esi: frame.flags().esi,
            seq: 0,
        };
        self.transmit_raw(fifo, header, frame.data()).await
    }

    async fn transmit_raw(
        &mut self,
        fifo: Fifo,
        mut header: TxHeader,
        payload: &[u8],
    ) -> Result<(), Error<SPI::Error>> {
        let sta = CiFifoSta(self.bus.read_sfr32(addr::fifo_sta(fifo)).await?);
        if !sta.not_full_or_not_empty() {
            return Err(Error::TxFifoFull);
        }
        let ua = self.bus.read_sfr32(addr::fifo_ua(fifo)).await? & 0xFFF;

        header.seq = self.seq & self.seq_mask;
        self.seq = self.seq.wrapping_add(1);
        let [t0, t1] = header.to_words();

        // 8 header bytes + payload zero-padded to a 32-bit boundary.
        let mut obj = [0u8; 72];
        obj[0..4].copy_from_slice(&t0.to_le_bytes());
        obj[4..8].copy_from_slice(&t1.to_le_bytes());
        obj[8..8 + payload.len()].copy_from_slice(payload);
        let len = 8 + payload.len().div_ceil(4) * 4;

        self.bus.write_ram(addr::RAM_START + ua as u16, &obj[..len]).await?;
        self.bus
            .write_sfr8(addr::fifo_con(fifo) + 1, CiFifoCon::CON_BYTE1_UINC_TXREQ)
            .await
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test && cargo test --all-features`
Expected: PASS.

- [ ] **Step 5: Lint and commit**

```bash
cargo clippy --all-features -- -D warnings && cargo fmt
git add src/driver.rs tests/driver.rs
git commit -m "feat: classic and FD transmit with per-instance sequence numbers"
```

---

### Task 12: Receive path, FIFO status, interrupts, events, error counters

**Files:**
- Modify: `src/driver.rs`
- Modify: `src/frame.rs` (remove the two `#[allow(dead_code)]` on `from_parts`)
- Modify: `src/lib.rs` (add `pub use frame::ReceivedFrame;` and `pub use driver::Event;`)
- Modify: `tests/driver.rs` (append tests)

**Interfaces:**
- Consumes: `RxHeader::from_words`, `dlc_to_len` (Task 4), `Frame::from_parts`/`FdFrame::from_parts` (Task 5), `CiInt`, `CiVec`, `CiTrec`, `CiFifoCon::CON_BYTE1_UINC`.
- Produces:
  - `async fn receive(&mut self, fifo: Fifo) -> Result<RxFrame, Error<SPI::Error>>` — non-blocking, `Err(RxFifoEmpty)` when empty
  - `async fn fifo_status(&mut self, fifo: Fifo) -> Result<CiFifoSta, Error<SPI::Error>>`
  - `async fn clear_rx_overflow(&mut self, fifo: Fifo) -> Result<(), Error<SPI::Error>>` (byte-write 0 to `CiFIFOSTA` byte 0)
  - `async fn interrupt_flags(&mut self) -> Result<CiInt, Error<SPI::Error>>`
  - `async fn clear_interrupts(&mut self, flags: CiInt) -> Result<(), Error<SPI::Error>>` — byte-writes to `C1INT` bytes 0-1, writing 0 for each flag set in `flags` and 1 elsewhere (write-0-to-clear semantics)
  - `async fn configure_interrupts(&mut self, enables: CiInt) -> Result<(), Error<SPI::Error>>` — byte-writes bytes 2-3 (the enable half); build the value with `CiInt(0).with_rxie(true)...`
  - `async fn pending_event(&mut self) -> Result<Event, Error<SPI::Error>>` — decodes `C1VEC.ICODE`
  - `async fn error_counters(&mut self) -> Result<CiTrec, Error<SPI::Error>>`
  - `Event` enum (in `driver.rs`): `None` (0x40), `Fifo(Fifo)` (codes 1..=31), `TxQueue` (0), `Error` (0x41), `WakeUp` (0x42), `ReceiveOverflow` (0x43), `AddressError` (0x44), `SystemError` (0x45), `TimeBaseOverflow` (0x46), `ModeChange` (0x47), `InvalidMessage` (0x48), `TransmitEvent` (0x49), `TransmitAttempt` (0x4A), `Unknown(u8)` — `#[non_exhaustive]`, derives Debug/Clone/Copy/PartialEq/Eq + optional defmt

- [ ] **Step 1: Append failing tests to `tests/driver.rs`**

```rust
use mcp251xfd::{Event, ReceivedFrame};

#[test]
fn receive_classic_frame() {
    let mut e = Vec::new();
    e.extend(r32(0x06C, 0x0000_0001)); // CiFIFOSTA2: not empty
    e.extend(r32(0x070, 0x0000_00A0)); // CiFIFOUA2: offset 0xA0
    e.extend(rram(0x4A0, &[0x23, 0x01, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00])); // R0, R1
    e.extend(rram(0x4A8, &[0x01, 0x02, 0x03, 0x04])); // payload
    e.extend(w8(0x069, 0x01)); // CiFIFOCON2 byte1: UINC
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    let rx = can.receive(Fifo::F2).unwrap();
    match rx.frame {
        ReceivedFrame::Classic(f) => {
            assert_eq!(f.id(), Id::Standard(StandardId::new(0x123).unwrap()));
            assert_eq!(f.data(), &[1, 2, 3, 4]);
        }
        ReceivedFrame::Fd(_) => panic!("expected classic frame"),
    }
    assert_eq!(rx.timestamp, None);
    spi.done();
}

#[test]
fn receive_fd_frame() {
    let mut e = Vec::new();
    e.extend(r32(0x06C, 0x0000_0001));
    e.extend(r32(0x070, 0x0000_0000));
    // R1 = DLC 9 | BRS(1<<6) | FDF(1<<7) = 0xC9 -> 12 payload bytes.
    e.extend(rram(0x400, &[0x7F, 0x00, 0x00, 0x00, 0xC9, 0x00, 0x00, 0x00]));
    e.extend(rram(0x408, &[0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB]));
    e.extend(w8(0x069, 0x01));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    let rx = can.receive(Fifo::F2).unwrap();
    match rx.frame {
        ReceivedFrame::Fd(f) => {
            assert_eq!(f.data().len(), 12);
            assert_eq!(f.data()[11], 0xAB);
            assert!(f.flags().brs);
        }
        ReceivedFrame::Classic(_) => panic!("expected FD frame"),
    }
    spi.done();
}

#[test]
fn receive_empty_fifo_errors() {
    let mut e = Vec::new();
    e.extend(r32(0x06C, 0x0000_0000));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(matches!(can.receive(Fifo::F2), Err(Error::RxFifoEmpty)));
    spi.done();
}

#[test]
fn interrupts_and_events() {
    let mut e = Vec::new();
    e.extend(r32(0x01C, 0x0000_0002)); // RXIF
    e.extend(w8(0x01C, 0xFD)); // clear RXIF: write 0 to bit 1, 1 elsewhere
    e.extend(w8(0x01D, 0xFF));
    e.extend(w8(0x01E, 0x02)); // enables: RXIE (bit 17 -> byte2 bit 1)
    e.extend(w8(0x01F, 0x00));
    e.extend(r32(0x018, 0x0000_0002)); // ICODE = 2 -> FIFO 2
    e.extend(r32(0x018, 0x0000_0040)); // ICODE = 0x40 -> none
    e.extend(r32(0x034, 0x0021_1503)); // TREC
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    let flags = can.interrupt_flags().unwrap();
    assert!(flags.rxif());
    can.clear_interrupts(flags).unwrap();
    can.configure_interrupts(mcp251xfd::registers::CiInt(0).with_rxie(true)).unwrap();
    assert_eq!(can.pending_event().unwrap(), Event::Fifo(Fifo::F2));
    assert_eq!(can.pending_event().unwrap(), Event::None);
    let trec = can.error_counters().unwrap();
    assert_eq!(trec.tec(), 0x15);
    assert!(trec.tx_bus_off());
    spi.done();
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --test driver`
Expected: FAIL.

- [ ] **Step 3: Implement in `src/driver.rs`**

Add imports: `use crate::frame::{FrameFlags, ReceivedFrame, RxFrame};`, `use crate::registers::objects::{dlc_to_len, RxHeader};`, `use crate::registers::{CiInt, CiTrec, CiVec};`. Define `Event` at module level (outside the `maybe` impl — it is shared by both variants):

```rust
/// Decoded `CiVEC.ICODE`: the highest-priority pending interrupt source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Event {
    /// No interrupt pending (code 0x40).
    None,
    /// A FIFO interrupt (codes 1..=31).
    Fifo(Fifo),
    /// TXQ interrupt (code 0; the TXQ is not used by this driver version).
    TxQueue,
    /// CAN bus error (code 0x41).
    Error,
    /// Wake-up (code 0x42).
    WakeUp,
    /// RX FIFO overflow (code 0x43).
    ReceiveOverflow,
    /// Illegal register/RAM address access (code 0x44).
    AddressError,
    /// System error, e.g. SPI underrun per erratum — re-request the
    /// operation mode to recover (code 0x45).
    SystemError,
    /// Time base counter overflow (code 0x46).
    TimeBaseOverflow,
    /// Operation mode changed (code 0x47).
    ModeChange,
    /// Invalid message received (code 0x48).
    InvalidMessage,
    /// Transmit event FIFO (code 0x49).
    TransmitEvent,
    /// Transmit attempts exhausted (code 0x4A).
    TransmitAttempt,
    /// A code this driver version does not know.
    Unknown(u8),
}

impl Event {
    /// Decodes an `ICODE` value.
    pub const fn from_icode(code: u8) -> Self {
        match code {
            0 => Self::TxQueue,
            1..=31 => match Fifo::new(code) {
                Some(f) => Self::Fifo(f),
                None => Self::Unknown(code),
            },
            0x40 => Self::None,
            0x41 => Self::Error,
            0x42 => Self::WakeUp,
            0x43 => Self::ReceiveOverflow,
            0x44 => Self::AddressError,
            0x45 => Self::SystemError,
            0x46 => Self::TimeBaseOverflow,
            0x47 => Self::ModeChange,
            0x48 => Self::InvalidMessage,
            0x49 => Self::TransmitEvent,
            0x4A => Self::TransmitAttempt,
            other => Self::Unknown(other),
        }
    }
}
```

Methods inside the `maybe` impl block:

```rust
    /// Pops one frame from a receive FIFO. Non-blocking:
    /// [`Error::RxFifoEmpty`] when there is nothing to read.
    pub async fn receive(&mut self, fifo: Fifo) -> Result<RxFrame, Error<SPI::Error>> {
        let sta = CiFifoSta(self.bus.read_sfr32(addr::fifo_sta(fifo)).await?);
        if !sta.not_full_or_not_empty() {
            return Err(Error::RxFifoEmpty);
        }
        let ua = self.bus.read_sfr32(addr::fifo_ua(fifo)).await? & 0xFFF;
        let base = addr::RAM_START + ua as u16;

        let mut hdr = [0u8; 8];
        self.bus.read_ram(base, &mut hdr).await?;
        let r0 = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]);
        let r1 = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
        let header = RxHeader::from_words([r0, r1]);

        let len = dlc_to_len(header.dlc, header.fdf);
        let padded = len.div_ceil(4) * 4;
        let mut data = [0u8; 64];
        if padded > 0 {
            self.bus.read_ram(base + 8, &mut data[..padded]).await?;
        }
        self.bus
            .write_sfr8(addr::fifo_con(fifo) + 1, CiFifoCon::CON_BYTE1_UINC)
            .await?;

        let frame = if header.fdf {
            ReceivedFrame::Fd(FdFrame::from_parts(
                header.id,
                len as u8,
                FrameFlags { brs: header.brs, esi: header.esi },
                data,
            ))
        } else {
            let mut d8 = [0u8; 8];
            d8.copy_from_slice(&data[..8]);
            ReceivedFrame::Classic(Frame::from_parts(header.id, len as u8, header.rtr, d8))
        };
        Ok(RxFrame { frame, timestamp: None })
    }

    /// Reads a FIFO's status register.
    pub async fn fifo_status(&mut self, fifo: Fifo) -> Result<CiFifoSta, Error<SPI::Error>> {
        Ok(CiFifoSta(self.bus.read_sfr32(addr::fifo_sta(fifo)).await?))
    }

    /// Clears a FIFO's overflow (and attempt-exhausted) flags.
    pub async fn clear_rx_overflow(&mut self, fifo: Fifo) -> Result<(), Error<SPI::Error>> {
        self.bus.write_sfr8(addr::fifo_sta(fifo), 0x00).await
    }

    /// Reads the global interrupt flags (and enable bits).
    pub async fn interrupt_flags(&mut self) -> Result<CiInt, Error<SPI::Error>> {
        Ok(CiInt(self.bus.read_sfr32(addr::C1INT).await?))
    }

    /// Clears the interrupt flags set in `flags` (write-0-to-clear; only
    /// the flag half is touched).
    pub async fn clear_interrupts(&mut self, flags: CiInt) -> Result<(), Error<SPI::Error>> {
        self.bus.write_sfr8(addr::C1INT, !(flags.0 as u8)).await?;
        self.bus.write_sfr8(addr::C1INT + 1, !((flags.0 >> 8) as u8)).await
    }

    /// Writes the interrupt enable half of `C1INT`. Build the value with
    /// the `with_*ie` methods, e.g. `CiInt(0).with_rxie(true)`.
    pub async fn configure_interrupts(&mut self, enables: CiInt) -> Result<(), Error<SPI::Error>> {
        self.bus.write_sfr8(addr::C1INT + 2, (enables.0 >> 16) as u8).await?;
        self.bus.write_sfr8(addr::C1INT + 3, (enables.0 >> 24) as u8).await
    }

    /// Reads and decodes the highest-priority pending interrupt.
    pub async fn pending_event(&mut self) -> Result<Event, Error<SPI::Error>> {
        Ok(Event::from_icode(CiVec(self.bus.read_sfr32(addr::C1VEC).await?).icode()))
    }

    /// Reads the error counters and bus state (`CiTREC`).
    pub async fn error_counters(&mut self) -> Result<CiTrec, Error<SPI::Error>> {
        Ok(CiTrec(self.bus.read_sfr32(addr::C1TREC).await?))
    }
```

Classic-frame `len` note: `dlc_to_len(dlc, false)` caps at 8, so `Frame::from_parts(.., len as u8, ..)` stores a DLC of at most 8 even if a nonconforming node sent DLC 9..15 — document this on `receive`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test && cargo test --all-features`
Expected: PASS.

- [ ] **Step 5: Lint and commit**

```bash
cargo clippy --all-features -- -D warnings && cargo fmt
git add src/driver.rs src/frame.rs src/lib.rs tests/driver.rs
git commit -m "feat: receive path, interrupt flags/enables, event decode, error counters"
```

---

### Task 13: Async-only `wait_rx` on an interrupt pin

**Files:**
- Modify: `src/driver.rs` (new `#[cfg(feature = "async")]` impl block — written by hand, NOT inside the `maybe` macro, because it exists only for the async variant)
- Create: `tests/async_driver.rs`

**Interfaces:**
- Consumes: `receive` (Task 12), `embedded_hal_async::digital::Wait`.
- Produces: on `MCP251xFdAsync` only:
  - `async fn wait_rx<P: Wait>(&mut self, fifo: Fifo, int_pin: &mut P) -> Result<RxFrame, Error<SPI::Error>>`

- [ ] **Step 1: Create `tests/async_driver.rs` with failing tests**

```rust
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
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> { Ok(()) }
    async fn wait_for_low(&mut self) -> Result<(), Self::Error> { Ok(()) }
    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> { Ok(()) }
    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> { Ok(()) }
    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> { Ok(()) }
}

fn r32(addr: u16, val: u32) -> Vec<Transaction> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x30 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::read_vec(val.to_le_bytes().to_vec()),
        Transaction::transaction_end(),
    ]
}

fn rram(addr: u16, data: &[u8]) -> Vec<Transaction> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x30 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::read_vec(data.to_vec()),
        Transaction::transaction_end(),
    ]
}

fn w8(addr: u16, val: u8) -> Vec<Transaction> {
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
    e.extend(rram(0x400, &[0x23, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00]));
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
    assert!(matches!(can.receive(Fifo::F2).await, Err(Error::RxFifoEmpty)));
    spi.done();
}
```

Add to `Cargo.toml` `[dev-dependencies]`: nothing new (embedded-hal is already a main dependency; `embedded-hal-async` comes in via `--all-features`).

- [ ] **Step 2: Run to verify failure**

Run: `cargo test --all-features --test async_driver`
Expected: FAIL — `wait_rx` missing.

- [ ] **Step 3: Implement in `src/driver.rs`**

```rust
/// Async-only conveniences built on the interrupt pin.
#[cfg(feature = "async")]
impl<SPI: SpiDeviceAsync> MCP251xFdAsync<SPI> {
    /// Waits until a frame arrives on `fifo` and returns it.
    ///
    /// Level-triggered and race-free: the FIFO is checked *before* waiting,
    /// and the chip's open-drain nINT stays asserted (low) while any enabled
    /// interrupt is pending. Requirements: the FIFO was configured by
    /// [`Self::apply_layout`] (which sets its not-empty interrupt) and RXIE
    /// is enabled via [`Self::configure_interrupts`]; `int_pin` is the MCU
    /// input wired to nINT (any [`embedded_hal_async::digital::Wait`]
    /// implementation — e.g. an embassy `Input`/`ExtiInput`).
    pub async fn wait_rx<P: embedded_hal_async::digital::Wait>(
        &mut self,
        fifo: Fifo,
        int_pin: &mut P,
    ) -> Result<crate::frame::RxFrame, Error<SPI::Error>> {
        loop {
            match self.receive(fifo).await {
                Err(Error::RxFifoEmpty) => {
                    int_pin.wait_for_low().await.map_err(|_| Error::IntPin)?;
                }
                other => return other,
            }
        }
    }
}
```

Note: `maybe-async-cfg` renames doc-comment intra-links too only within its own items — this block is hand-written, so `Self::apply_layout` resolves against `MCP251xFdAsync` (generated with the same method names). If `cargo doc` cannot resolve the links (generated items sometimes lose doc anchors), downgrade them to plain code spans — zero-warning docs win over link niceties.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --all-features`
Expected: PASS, including both async test files.

- [ ] **Step 5: Lint and commit**

```bash
cargo clippy --all-features -- -D warnings && cargo fmt
git add src/driver.rs tests/async_driver.rs
git commit -m "feat: async wait_rx on the nINT pin"
```

---

### Task 14: Publishing collateral — crate docs, README, licenses

**Files:**
- Modify: `src/lib.rs` (expand crate-level docs)
- Create: `README.md`, `LICENSE-MIT`, `LICENSE-APACHE`

**Interfaces:** none produced; this task is documentation only, but it is gated like any other (docs build warning-free).

- [ ] **Step 1: Expand the crate-level docs in `src/lib.rs`**

Replace the doc header with:

```rust
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
```

- [ ] **Step 2: Write `README.md`**

Content requirements (write it out, keep it under ~120 lines): title + one-line description; badges omitted until CI exists on GitHub; feature bullet list mirroring the crate docs; supported chips table (MCP2517FD / MCP2518FD / MCP251863, auto-detected); the sync usage example from Step 1 plus a short async/embassy snippet:

````markdown
```rust,ignore
// embassy: share one SPI bus between many chips
let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(spi);
let device = SpiDevice::new(&spi_bus, cs_pin); // embassy-embedded-hal
let mut can = MCP251xFdAsync::new(device);
// ... same API as the sync driver, plus:
let frame = can.wait_rx(Fifo::F2, &mut int_pin).await?;
```
````

…then sections: **SPI clock limit** (the erratum, `max_spi_hz`, 17 MHz @ 40 MHz), **Feature flags** (`async`, `defmt`, `log`), **Status** (v0.1 scope + deferred list from the spec), **Hardware examples** (pointer to `examples/rp2040`), **License** (MIT OR Apache-2.0 dual-license boilerplate), **References** (datasheet DS20006027B, family reference manual DS20005678E, errata DS80000792D, the Emandhal C driver).

- [ ] **Step 3: Add license texts**

```bash
curl -sSf -o LICENSE-APACHE https://www.apache.org/licenses/LICENSE-2.0.txt
```

Create `LICENSE-MIT` with the standard MIT text and the line `Copyright (c) 2026 Lucas Cohen`.

- [ ] **Step 4: Verify docs and packaging**

Run: `RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps && cargo package --list --allow-dirty`
Expected: docs build with zero warnings; the package listing contains `src/`, `README.md`, both licenses, `Cargo.toml` — and does **not** need `examples/rp2040` (it may appear; that is acceptable, it is small text).

- [ ] **Step 5: Commit**

```bash
git add src/lib.rs README.md LICENSE-MIT LICENSE-APACHE
git commit -m "docs: crate documentation, README, dual license"
```

---

### Task 15: CI workflow

**Files:**
- Create: `.github/workflows/ci.yml`

- [ ] **Step 1: Write the workflow**

```yaml
name: CI

on:
  push:
    branches: [main]
  pull_request:

env:
  CARGO_TERM_COLOR: always
  RUSTFLAGS: "-D warnings"

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@stable
      - run: cargo test
      - run: cargo test --all-features

  build-no-std:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@stable
        with:
          targets: thumbv6m-none-eabi
      - run: cargo build --target thumbv6m-none-eabi --no-default-features
      - run: cargo build --target thumbv6m-none-eabi --all-features

  lint:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@stable
        with:
          components: clippy, rustfmt
      - run: cargo clippy --all-features -- -D warnings
      - run: cargo fmt --check

  docs:
    runs-on: ubuntu-latest
    env:
      RUSTDOCFLAGS: "-D warnings"
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@stable
      - run: cargo doc --all-features --no-deps

  examples:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@stable
        with:
          targets: thumbv6m-none-eabi
      - run: cargo build --release
        working-directory: examples/rp2040
```

Note: the `examples` job will fail until Task 16 creates `examples/rp2040` — if CI is pushed before Task 16 lands, temporarily gate the job with `if: ${{ hashFiles('examples/rp2040/Cargo.toml') != '' }}`; remove the gate in Task 16. If the repo has no GitHub remote yet, this file simply travels with the initial push.

- [ ] **Step 2: Sanity-check locally (CI's exact commands)**

Run: `RUSTFLAGS="-D warnings" cargo test --all-features && cargo clippy --all-features -- -D warnings && cargo fmt --check && RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps`
Expected: all pass. Also run `rustup target add thumbv6m-none-eabi` once, then `RUSTFLAGS="-D warnings" cargo build --target thumbv6m-none-eabi --all-features` — this is the proof the crate is really `no_std` (any accidental `std` use fails here).

- [ ] **Step 3: Commit**

```bash
git add .github/workflows/ci.yml
git commit -m "ci: tests, no_std build, clippy/fmt/docs with zero-warning policy"
```

---

### Task 16: Hardware examples crate — scaffold, `enumerate`, `loopback`

**Files:**
- Create: `examples/rp2040/Cargo.toml`, `examples/rp2040/.cargo/config.toml`, `examples/rp2040/memory.x`, `examples/rp2040/build.rs`, `examples/rp2040/src/common.rs` (shared via `#[path]` include), `examples/rp2040/src/bin/enumerate.rs`, `examples/rp2040/src/bin/loopback.rs`

**Board facts (from the project owner):** RP2040; SPI1 with SCK=GPIO10, MOSI=GPIO11, MISO=GPIO12; ten MCP2517FD chips with CS on GPIOs 3, 4, 5, 6, 7, 8, 9, 13, 14, 15. Crystal assumed 40 MHz (`ClockConfig::MHZ40`) — **verify on the board silkscreen/schematic before flashing; a 20 MHz crystal needs `ClockConfig::MHZ20` and a 8.5 MHz SPI cap.** INT pins are not known — examples poll instead of using `wait_rx`.

**Interfaces:** none consumed by later crate tasks; the examples consume the whole public API and are the hardware acceptance tests. These binaries cannot run in CI — the verification step is `cargo build --release` for `thumbv6m-none-eabi`; flashing/running is done by the project owner with `probe-rs`.

- [ ] **Step 1: Scaffold the crate**

`examples/rp2040/Cargo.toml` — versions below were current at plan time (embassy releases move fast; if `cargo build` reports resolution failures, bump the embassy crates to the latest set together, they version-lock each other):

```toml
[package]
name = "mcp251xfd-examples-rp2040"
version = "0.0.0"
edition = "2021"
publish = false

# Standalone workspace: keeps embassy/cortex-m deps out of the library's tree.
[workspace]

[dependencies]
mcp251xfd = { path = "../..", features = ["async", "defmt"] }
embedded-can = "0.4"

embassy-executor = { version = "0.7", features = ["arch-cortex-m", "executor-thread", "defmt"] }
embassy-rp = { version = "0.4", features = ["defmt", "time-driver", "critical-section-impl", "rp2040"] }
embassy-time = { version = "0.4", features = ["defmt"] }
embassy-sync = "0.6"
embassy-embedded-hal = "0.3"

cortex-m = "0.7"
cortex-m-rt = "0.7"
defmt = "0.3"
defmt-rtt = "0.4"
panic-probe = { version = "0.3", features = ["print-defmt"] }
static-cell = "2"

[profile.release]
debug = 2
lto = true
opt-level = "z"
```

`examples/rp2040/.cargo/config.toml`:

```toml
[build]
target = "thumbv6m-none-eabi"

[target.thumbv6m-none-eabi]
runner = "probe-rs run --chip RP2040"
rustflags = ["-C", "link-arg=-Tlink.x", "-C", "link-arg=-Tdefmt.x"]
```

`examples/rp2040/memory.x` (standard RP2040 with boot2):

```text
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}
```

`examples/rp2040/build.rs` (standard cortex-m-rt memory.x copier):

```rust
use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
}
```

- [ ] **Step 2: Shared board setup, `examples/rp2040/src/common.rs`**

Included from each binary with `#[path = "../common.rs"] mod common;` (bin crates cannot share a lib without more structure; this keeps it simple).

```rust
//! Shared board setup for the 10-chip MCP2517FD test board.

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_rp::gpio::{AnyPin, Level, Output, Pin};
use embassy_rp::peripherals::SPI1;
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_rp::Peripherals;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use mcp251xfd::{ClockConfig, Config, DataBitTiming, NominalBitTiming};
use static_cell::StaticCell;

pub type Bus = Mutex<NoopRawMutex, Spi<'static, SPI1, Async>>;
pub type Device = SpiDevice<'static, NoopRawMutex, Spi<'static, SPI1, Async>, Output<'static>>;

static SPI_BUS: StaticCell<Bus> = StaticCell::new();

/// 500 kbit/s nominal, 2 Mbit/s data, 40 MHz crystal. Adjust `clock` if the
/// board's crystal is not 40 MHz.
pub const CAN_CONFIG: Config = Config {
    clock: ClockConfig::MHZ40,
    nominal: NominalBitTiming::KBPS500_40MHZ,
    data: Some(DataBitTiming::MBPS2_40MHZ),
};

/// Sets up SPI1 (SCK=GP10, MOSI=GP11, MISO=GP12) at the erratum-safe
/// 17 MHz and returns one `SpiDevice` per chip-select pin
/// (GP3..GP9, GP13, GP14, GP15).
pub fn setup(p: Peripherals) -> [Device; 10] {
    let mut cfg = SpiConfig::default();
    cfg.frequency = 17_000_000; // = mcp251xfd::max_spi_hz(40 MHz SYSCLK)
    let spi = Spi::new(p.SPI1, p.PIN_10, p.PIN_11, p.PIN_12, p.DMA_CH0, p.DMA_CH1, cfg);
    let bus: &'static Bus = SPI_BUS.init(Mutex::new(spi));
    let cs: [AnyPin; 10] = [
        p.PIN_3.degrade(), p.PIN_4.degrade(), p.PIN_5.degrade(),
        p.PIN_6.degrade(), p.PIN_7.degrade(), p.PIN_8.degrade(),
        p.PIN_9.degrade(), p.PIN_13.degrade(), p.PIN_14.degrade(),
        p.PIN_15.degrade(),
    ];
    cs.map(|pin| SpiDevice::new(bus, Output::new(pin, Level::High)))
}
```

- [ ] **Step 3: `enumerate` binary**

`examples/rp2040/src/bin/enumerate.rs`:

```rust
//! Initializes all 10 chips on the shared bus and reports each one's
//! detected variant — the first thing to run on new hardware.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Delay;
use mcp251xfd::MCP251xFdAsync;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let devices = common::setup(p);
    let mut ok = 0;
    for (i, dev) in devices.into_iter().enumerate() {
        let mut can = MCP251xFdAsync::new(dev);
        match can.init(&common::CAN_CONFIG, &mut Delay).await {
            Ok(variant) => {
                info!("chip {}: init OK, variant {}", i, variant);
                ok += 1;
            }
            Err(_) => error!("chip {}: init FAILED (wiring/CS/SPI clock?)", i),
        }
    }
    info!("{}/10 chips initialized", ok);
}
```

- [ ] **Step 4: `loopback` binary**

`examples/rp2040/src/bin/loopback.rs` — full per-chip driver-stack check with no transceivers/bus wiring needed:

```rust
//! Internal-loopback smoke test: every chip transmits to itself, classic
//! and FD. Verifies the entire driver stack per chip in isolation.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use embedded_can::{Frame as _, StandardId};
use mcp251xfd::{
    FdFrame, Fifo, FifoLayout, Filter, FilterMatch, Frame, FrameFlags,
    MCP251xFdAsync, OperationMode, PayloadSize, ReceivedFrame,
};
use panic_probe as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B64, 8);

/// Polls an RX FIFO for up to ~100 ms.
async fn recv_timeout(
    can: &mut MCP251xFdAsync<common::Device>,
    fifo: Fifo,
) -> Option<mcp251xfd::RxFrame> {
    for _ in 0..100 {
        match can.receive(fifo).await {
            Ok(rx) => return Some(rx),
            Err(_) => Timer::after_millis(1).await,
        }
    }
    None
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let devices = common::setup(p);
    for (i, dev) in devices.into_iter().enumerate() {
        let mut can = MCP251xFdAsync::new(dev);
        if can.init(&common::CAN_CONFIG, &mut Delay).await.is_err() {
            error!("chip {}: init failed", i);
            continue;
        }
        can.apply_layout(&LAYOUT).await.unwrap();
        can.set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2).await.unwrap();
        can.set_mode(OperationMode::InternalLoopback, &mut Delay).await.unwrap();

        // Classic frame.
        let tx = Frame::new(StandardId::new(0x123).unwrap(), &[1, 2, 3, 4]).unwrap();
        can.transmit(Fifo::F1, &tx).await.unwrap();
        match recv_timeout(&mut can, Fifo::F2).await {
            Some(rx) => match rx.frame {
                ReceivedFrame::Classic(f) if f.data() == [1, 2, 3, 4] => {
                    info!("chip {}: classic loopback OK", i)
                }
                _ => error!("chip {}: classic loopback WRONG FRAME", i),
            },
            None => error!("chip {}: classic loopback TIMEOUT", i),
        }

        // FD frame with bit-rate switch.
        let mut payload = [0u8; 64];
        for (n, b) in payload.iter_mut().enumerate() {
            *b = n as u8;
        }
        let tx = FdFrame::new(
            StandardId::new(0x456).unwrap(),
            &payload,
            FrameFlags { brs: true, esi: false },
        )
        .unwrap();
        can.transmit_fd(Fifo::F1, &tx).await.unwrap();
        match recv_timeout(&mut can, Fifo::F2).await {
            Some(rx) => match rx.frame {
                ReceivedFrame::Fd(f) if f.data() == payload => {
                    info!("chip {}: FD-64 loopback OK", i)
                }
                _ => error!("chip {}: FD loopback WRONG FRAME", i),
            },
            None => error!("chip {}: FD loopback TIMEOUT", i),
        }
    }
    info!("loopback test complete");
}
```

- [ ] **Step 5: Build for the target and verify**

Run (from `examples/rp2040/`): `rustup target add thumbv6m-none-eabi && cargo build --release`
Expected: both binaries build with zero warnings. If embassy versions fail to resolve, bump them together (see Step 1 note). If `Peripherals`/pin API names differ in the resolved embassy-rp version, fix against its docs — the board wiring in this task is the source of truth.

Remove the temporary `if:` gate from the CI `examples` job if Task 15 added it.

- [ ] **Step 6: Commit**

```bash
git add examples/rp2040 .github/workflows/ci.yml
git commit -m "feat: RP2040 hardware examples - enumerate and internal loopback"
```

- [ ] **Step 7 (owner, on hardware): flash and check**

Run: `cd examples/rp2040 && cargo run --release --bin enumerate` (needs a debug probe + probe-rs), expect `10/10 chips initialized`, each reporting `Mcp2517Fd`. Then `cargo run --release --bin loopback`, expect `classic loopback OK` and `FD-64 loopback OK` for every chip. Init failures point at CS wiring or crystal assumptions, not driver logic — the mock tests already pin the byte protocol.

---

### Task 17: Hardware examples — `chip2chip` and `multinode`

**Files:**
- Create: `examples/rp2040/src/bin/chip2chip.rs`, `examples/rp2040/src/bin/multinode.rs`

**Precondition:** these two binaries need the chips wired to a **common CAN bus through transceivers** (the loopback test does not). If the board lacks that, the binaries still must build; running them is deferred.

**Interfaces:**
- Consumes: everything from Task 16's `common.rs` plus `recv_timeout` — move `recv_timeout` from `loopback.rs` into `common.rs` in this task (make it `pub`) so all three bus binaries share it, and update `loopback.rs` to call `common::recv_timeout`.

- [ ] **Step 1: `chip2chip` binary**

`examples/rp2040/src/bin/chip2chip.rs`:

```rust
//! Two chips on the shared CAN bus: chip 0 transmits, chip 1 receives.
//! Classic at the nominal rate, then FD with bit-rate switch.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Delay;
use embedded_can::{Frame as _, StandardId};
use mcp251xfd::{
    FdFrame, Fifo, FifoLayout, Filter, FilterMatch, Frame, FrameFlags,
    MCP251xFdAsync, OperationMode, PayloadSize, ReceivedFrame,
};
use panic_probe as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B64, 8);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut devices = common::setup(p).into_iter();
    let dev_a = devices.next().unwrap();
    let dev_b = devices.next().unwrap();

    let mut a = MCP251xFdAsync::new(dev_a);
    let mut b = MCP251xFdAsync::new(dev_b);
    for (name, can) in [("A", &mut a), ("B", &mut b)] {
        can.init(&common::CAN_CONFIG, &mut Delay).await.expect(name);
        can.apply_layout(&LAYOUT).await.unwrap();
        can.set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2).await.unwrap();
        can.set_mode(OperationMode::NormalFd, &mut Delay).await.unwrap();
    }

    // Classic 500 kbit/s.
    let tx = Frame::new(StandardId::new(0x100).unwrap(), &[0xDE, 0xAD]).unwrap();
    a.transmit(Fifo::F1, &tx).await.unwrap();
    match common::recv_timeout(&mut b, Fifo::F2).await {
        Some(rx) => match rx.frame {
            ReceivedFrame::Classic(f) if f.data() == [0xDE, 0xAD] => info!("classic A->B OK"),
            _ => error!("classic A->B wrong frame"),
        },
        None => error!("classic A->B TIMEOUT (transceivers? termination?)"),
    }

    // FD 500k/2M with BRS.
    let payload: [u8; 48] = core::array::from_fn(|i| i as u8);
    let tx = FdFrame::new(StandardId::new(0x200).unwrap(), &payload, FrameFlags { brs: true, esi: false }).unwrap();
    a.transmit_fd(Fifo::F1, &tx).await.unwrap();
    match common::recv_timeout(&mut b, Fifo::F2).await {
        Some(rx) => match rx.frame {
            ReceivedFrame::Fd(f) if f.data() == payload => info!("FD-48 BRS A->B OK"),
            _ => error!("FD A->B wrong frame"),
        },
        None => error!("FD A->B TIMEOUT"),
    }
    info!("chip2chip complete");
}
```

- [ ] **Step 2: `multinode` binary**

`examples/rp2040/src/bin/multinode.rs` — three or more nodes exercising broadcast, selective delivery, and arbitration (spec §6):

```rust
//! Multi-node bus test with chips 0 (A), 1 (B), 2 (C):
//! 1. Broadcast: A transmits once; B and C (filters wide open) both receive.
//! 2. Selective: B accepts only 0x0B0, C only 0x0C0 (+ a broadcast ID all
//!    accept); A sends all three; each node sees exactly what its filters
//!    admit.
//! 3. Arbitration: A (ID 0x010) and B (ID 0x700) queue frames back-to-back;
//!    C must receive both intact, lower ID typically first. Repeated 10x,
//!    all 20 frames must arrive.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Delay;
use embedded_can::{Frame as _, Id, StandardId};
use mcp251xfd::{
    Fifo, FifoLayout, Filter, FilterMatch, Frame, MCP251xFdAsync,
    OperationMode, PayloadSize, ReceivedFrame,
};
use panic_probe as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B8, 8)
    .rx_fifo(Fifo::F2, PayloadSize::B8, 16);

const BROADCAST: u16 = 0x7DF;

fn sid(raw: u16) -> Id {
    Id::Standard(StandardId::new(raw).unwrap())
}

async fn drain_ids(
    can: &mut MCP251xFdAsync<common::Device>,
    got: &mut [Option<Id>],
) -> usize {
    let mut n = 0;
    while n < got.len() {
        match common::recv_timeout(can, Fifo::F2).await {
            Some(rx) => {
                let id = match rx.frame {
                    ReceivedFrame::Classic(f) => f.id(),
                    ReceivedFrame::Fd(f) => f.id(),
                };
                got[n] = Some(id);
                n += 1;
            }
            None => break,
        }
    }
    n
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut devices = common::setup(p).into_iter();
    let mut a = MCP251xFdAsync::new(devices.next().unwrap());
    let mut b = MCP251xFdAsync::new(devices.next().unwrap());
    let mut c = MCP251xFdAsync::new(devices.next().unwrap());

    for (name, can) in [("A", &mut a), ("B", &mut b), ("C", &mut c)] {
        can.init(&common::CAN_CONFIG, &mut Delay).await.expect(name);
        can.apply_layout(&LAYOUT).await.unwrap();
        can.set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2).await.unwrap();
        can.set_mode(OperationMode::Normal20, &mut Delay).await.unwrap();
    }

    // --- 1. Broadcast: one TX, every other node receives it. ---
    a.transmit(Fifo::F1, &Frame::new(sid(0x123), &[0x42]).unwrap()).await.unwrap();
    let mut got_b = [None; 1];
    let mut got_c = [None; 1];
    let nb = drain_ids(&mut b, &mut got_b).await;
    let nc = drain_ids(&mut c, &mut got_c).await;
    if nb == 1 && nc == 1 && got_b[0] == Some(sid(0x123)) && got_c[0] == Some(sid(0x123)) {
        info!("broadcast OK: B and C both received");
    } else {
        error!("broadcast FAILED: B got {}, C got {}", nb, nc);
    }

    // --- 2. Selective delivery via filters. ---
    for (name, can, own) in [("B", &mut b, 0x0B0u16), ("C", &mut c, 0x0C0)] {
        can.set_mode(OperationMode::Configuration, &mut Delay).await.expect(name);
        can.apply_layout(&LAYOUT).await.unwrap(); // FRESET drains stale frames
        can.set_filter(Filter::F0, FilterMatch::exact(sid(own)), Fifo::F2).await.unwrap();
        can.set_filter(Filter::F1, FilterMatch::exact(sid(BROADCAST)), Fifo::F2).await.unwrap();
        can.set_mode(OperationMode::Normal20, &mut Delay).await.unwrap();
    }
    for id in [0x0B0, 0x0C0, BROADCAST] {
        a.transmit(Fifo::F1, &Frame::new(sid(id), &[id as u8]).unwrap()).await.unwrap();
    }
    let mut got_b = [None; 3];
    let mut got_c = [None; 3];
    let nb = drain_ids(&mut b, &mut got_b).await;
    let nc = drain_ids(&mut c, &mut got_c).await;
    let b_ok = nb == 2 && got_b[..2].contains(&Some(sid(0x0B0))) && got_b[..2].contains(&Some(sid(BROADCAST)));
    let c_ok = nc == 2 && got_c[..2].contains(&Some(sid(0x0C0))) && got_c[..2].contains(&Some(sid(BROADCAST)));
    if b_ok && c_ok {
        info!("selective delivery OK: each node saw its ID + broadcast only");
    } else {
        error!("selective delivery FAILED: B {} frames, C {} frames", nb, nc);
    }

    // --- 3. Arbitration: A (high prio 0x010) vs B (low prio 0x700). ---
    // Filters on C are still wide open from setup; re-open B's TX role.
    let mut received = 0usize;
    let mut high_first = 0usize;
    for round in 0..10u8 {
        b.transmit(Fifo::F1, &Frame::new(sid(0x700), &[round]).unwrap()).await.unwrap();
        a.transmit(Fifo::F1, &Frame::new(sid(0x010), &[round]).unwrap()).await.unwrap();
        let mut got = [None; 2];
        let n = drain_ids(&mut c, &mut got).await;
        received += n;
        if n == 2 && got[0] == Some(sid(0x010)) {
            high_first += 1;
        }
    }
    if received == 20 {
        info!("arbitration OK: all 20 frames arrived; high-priority first in {}/10 rounds", high_first);
    } else {
        error!("arbitration FAILED: only {}/20 frames arrived", received);
    }
    info!("multinode complete");
}
```

Note on the arbitration check: true simultaneous start-of-frame cannot be forced over one shared SPI bus, so "high priority first" is reported as a statistic, not asserted — the hard assertion is losslessness (all 20 frames arrive intact), which is what CAN arbitration guarantees. Note `Normal20` mode is used here (classic frames only) so a scope/analyzer on the bus shows plain CAN 2.0.

- [ ] **Step 3: Move `recv_timeout` into `common.rs`**

Make it `pub async fn recv_timeout(can: &mut MCP251xFdAsync<Device>, fifo: Fifo) -> Option<RxFrame>` in `common.rs` (imports: `embassy_time::Timer`, `mcp251xfd::{Fifo, MCP251xFdAsync, RxFrame}`), delete the copy in `loopback.rs`, and call `common::recv_timeout` there. Each binary only compiles the `common` items it uses — add `#![allow(dead_code)]`? No: `common.rs` is included per-binary and unused items warn. Instead annotate `common.rs` items that not all binaries use with `#[allow(dead_code)]` — here that is unnecessary if every binary uses `setup`, `CAN_CONFIG`, `Device`, and `recv_timeout`; `enumerate` does not use `recv_timeout`, so put `#[allow(dead_code)]` on `recv_timeout` with a comment `// not used by every binary that includes common.rs`.

- [ ] **Step 4: Build and commit**

Run (from `examples/rp2040/`): `cargo build --release`
Expected: all four binaries, zero warnings.

```bash
git add examples/rp2040/src
git commit -m "feat: chip-to-chip and multi-node (broadcast/filter/arbitration) examples"
```

- [ ] **Step 5 (owner, on hardware): run when the CAN bus wiring exists**

`cargo run --release --bin chip2chip`, then `cargo run --release --bin multinode`. Expected defmt output: `classic A->B OK`, `FD-48 BRS A->B OK`; `broadcast OK`, `selective delivery OK`, `arbitration OK: all 20 frames arrived`.

---

## Plan Self-Review (performed while writing)

- **Spec coverage:** init recipe + variant detection (T9), bit timing incl. TDC auto (T3/T8/T9), FIFO layout with const RAM check (T6/T10) + trybuild proof (T6), filters (T8/T10), classic+FD TX/RX (T11/T12), interrupts/CiVEC events/error counters (T12), async `wait_rx` via `Wait` pin (T13), `embedded_can::Frame` interop (T5), `max_spi_hz` + erratum handling (T7/T8/T9 docs), doc policy + zero warnings (global constraints, every task), CI incl. `thumbv6m` (T15), examples: enumerate/loopback/chip2chip/multinode (T16/T17). Deferred spec items (CRC SPI, TEF, sleep, GPIO, ECC, bitrate solver, buffered runner) are intentionally absent, matching spec v0.1 scope.
- **Known deviations from spec text (deliberate, small):** the TX object scratch buffer lives on the stack in `transmit_raw` (72 B) instead of inside the bus struct — simpler ownership, same no-alloc guarantee. Bus-layer unit tests live inline in `src/bus.rs` rather than `tests/` because `Bus` is crate-private (file map updated).
- **Type consistency check:** `Error`/`ConfigError` (T1) used by all; `Fifo`/`Filter`/`PayloadSize`/`OperationMode`/`Variant` (T2) match usages in T6-T17; `CiFifoCon::CON_BYTE1_*` constants (T3) used in T11/T12; `TxHeader`/`RxHeader` field lists identical at definition (T4) and use (T11/T12); `FifoLayout` builder names (`tx_fifo`/`rx_fifo`/`try_*`) consistent across T6/T10/T16/T17; `MCP251xFdAsync::new` + `receive`/`transmit` signatures identical between sync tests (T9-T12) and async tests (T13).

