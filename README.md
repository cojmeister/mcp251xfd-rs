# mcp251xfd

A `no_std` Rust driver for the Microchip MCP2517FD / MCP2518FD / MCP251863 external SPI CAN FD controllers.

## Features

- Generic over `embedded_hal::spi::SpiDevice` — works on shared SPI buses; the driver never touches chip select.
- The `async` feature adds `MCP251xFdAsync` over `embedded_hal_async::spi::SpiDevice` (embassy-compatible), generated from the same source. Sync and async coexist in one binary.
- Classic CAN 2.0 (`Frame`, with `embedded_can::Frame` interop) and CAN FD up to 64 bytes (`FdFrame`).
- Compile-time message-RAM budgeting: build a `FifoLayout` in a `const` and overflowing the 2 KiB RAM is a compile error.

## Supported chips

| Chip | Notes |
|---|---|
| MCP2517FD | 7-bit TX sequence numbers, Sleep mode only |
| MCP2518FD | 23-bit TX sequence numbers, Low Power Mode |
| MCP251863 | Same die as the MCP2518FD (integrated transceiver) |

The variant is auto-detected during `init` (probed via `OSC.LPMEN`) and returned to the caller — no configuration needed up front.

## Usage

```rust,ignore
use mcp251xfd::{
    ClockConfig, Config, DataBitTiming, Fifo, FifoLayout, Filter,
    FilterMatch, Frame, MCP251xFd, NominalBitTiming, OperationMode,
    PayloadSize,
};
use embedded_can::{Frame as _, StandardId};

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B64, 8);

let mut can = MCP251xFd::new(spi_device);
let variant = can.init(
    &Config {
        clock: ClockConfig::MHZ40,
        nominal: NominalBitTiming::KBPS500_40MHZ,
        data: Some(DataBitTiming::MBPS2_40MHZ),
    },
    &mut delay,
)?;
can.apply_layout(&LAYOUT)?;
can.set_filter(Filter::F0, FilterMatch::accept_all(), Fifo::F2)?;
can.set_mode(OperationMode::NormalFd, &mut delay)?;

let frame = Frame::new(StandardId::new(0x123).unwrap(), &[1, 2, 3, 4]).unwrap();
can.transmit(Fifo::F1, &frame)?;
```

The async API is identical, plus `.await`:

```rust,ignore
// embassy: share one SPI bus between many chips
let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(spi);
let device = SpiDevice::new(&spi_bus, cs_pin); // embassy-embedded-hal
let mut can = MCP251xFdAsync::new(device);
// ... same API as the sync driver, plus:
let frame = can.wait_rx(Fifo::F2, &mut int_pin).await?;
```

`wait_rx` has mock coverage only — it has never been run against a real nINT
line. See [Status](#status).

## SPI clock limit (silicon erratum)

RAM reads corrupt above `0.85 * SYSCLK / 2` — 17 MHz at the recommended 40 MHz SYSCLK. The driver cannot observe your bus clock; size it with `max_spi_hz`. `MCP251xFd::init` verifies communication with a RAM echo test and fails with `Error::CommunicationCheckFailed` on an over-clocked bus.

## Blocking or async

Both APIs are generated from one source, so they are feature-identical.

Use **async** when the core issuing SPI also runs other work. Use **blocking**
when the core is dedicated to CAN — and in particular on any target where DMA
completion interrupts are serviced on a *different* core than the one that
issued the transfer.

The RP2040 under `embassy-rp` is that case: `DMA_IRQ_0` is enabled in
whichever core calls `embassy_rp::init` (core 0), `DMA_IRQ_1` is never used,
so core 1's SPI DMA completions are serviced on core 0 at an unpredictable
phase. Blocking removes the interrupt entirely and frees two DMA channels, and
for the 3–18 byte transfers this driver issues, DMA setup overhead dominates
anyway. See the crate docs for the full explanation, and
`examples/rp2040/src/bin/blocking_core1.rs` for a worked setup.

## Known hardware anomalies

### MCP2517FD: transmit stalls under a receive-heavy load

On the **MCP2517FD only** (DS80000792D item 1 — the MCP2518FD/MCP251863
errata have no equivalent), a long enough gap between SPI bytes, or between
the last byte and nCS rising, during an SPI **READ that touches message RAM**
causes a TX MAB underflow. The budget is tight: 5 nominal bit times, so 10 µs
at 500 kbit/s.

| Where | What you see |
|---|---|
| `CiINT` | `SERRIF` latched, usually with `MODIF` and `IVMIF` |
| `CiCON.OPMOD` | Restricted Operation, or Listen Only if `SERR2LOM` is set |
| TX FIFO | reports full and stops draining — both modes ignore `TXREQ` |
| `CiTREC` | completely clean: `TEC` 0, `REC` 0, not bus-off, not error-passive |

It looks nothing like a bus fault, and clearing the interrupt flags never
fixes it — the operation mode is what changed. Recover with
`recover_system_error`, which clears the flags and re-requests Normal mode;
the chip then retransmits the offending message itself and no reset is needed.

Only `receive` issues RAM reads, so transmit-only workloads do not trigger it.

## Feature flags

| Flag | Effect |
|---|---|
| `async` | Adds `MCP251xFdAsync` over `embedded_hal_async::spi::SpiDevice` |
| `defmt` | Implements `defmt::Format` on public error, config, and frame types |

## Minimum supported Rust version

1.85 (Rust 2024 edition), any feature combination. Verified in CI with `cargo check` on 1.85.

## Status

v0.1: reset/init, oscillator setup with variant detection, bit timing, FIFO layout, acceptance filters, classic + FD transmit, receive, interrupt flags/events, error counters, and an async `wait_rx` helper on the nINT pin.

Validated on hardware — a board carrying ten MCP2517FDs on one shared SPI bus — via the [hardware examples](#hardware-examples): init and variant detection on all ten chips, the measured on-wire bit rate, classic and FD-64 internal loopback, real-bus traffic between two and three nodes, 29-bit identifiers, masked acceptance filters, remote frames, multi-FIFO layouts, and a 43,000-frame soak with no corruption and no bus errors.

**Implemented but never exercised on hardware** — mock coverage only, so treat as unproven:

- `wait_rx` and the interrupt API (`configure_interrupts`, `clear_interrupts`, `interrupt_flags`, `pending_event`): the nINT line was never wired on the test board
- Error recovery — bus-off and error-passive entry and exit (`error_counters` never left zero during the soak)
- `Sleep`/wake mode transitions
- `Variant::Mcp2518Fd` and the MCP251863 (the test board is all MCP2517FD)
- Any oscillator configuration using the PLL, and any SYSCLK other than 20 MHz
- Data-phase rates other than 2 Mbit/s
- Gapped FIFO layouts (see `FifoLayout` for why they are not validated against the chip's address generation)

Two defects that hardware caught, for calibration on what mock tests can and cannot prove: a bit-timing preset paired with the wrong crystal (every rate silently halved), and an SPI clock above what the chip tolerates (intermittently corrupted register and message-RAM reads). Both were invisible to the mock suite *and* to internal loopback, because loopback shares the oscillator at both ends. The `bitrate` example exists specifically to close the first gap.

Not yet implemented:

- CRC-protected SPI transfers (the safe-write/CRC opcodes)
- The Transmit Event FIFO (TEF) and the dedicated TX Queue (TXQ)
- Helpers for Listen-Only / Restricted Operation beyond the generic `set_mode`
- Sleep/wake conveniences beyond `set_mode` (fire-and-forget for Sleep — see its docs)
- GPIO pin (`IOCON`) and CLKO divider control
- Interrupt sources other than RX (TEF, error, wake-up, mode-change, etc.)
- RX timestamping (`RxFrame::timestamp` is always `None`)
- Retransmission policy (`CiCON.RTXAT` stays 0 = unlimited retransmission, so the per-FIFO `TXAT` field is inert)

## Hardware examples

`examples/rp2040` is a standalone embassy crate with ten runnable RP2040 binaries, used as the driver's hardware acceptance tests. Eight need SPI wiring only — nothing touches the CAN pins: `enumerate` (every chip resets, initializes, and reports its variant), `bitrate` (measures the actual on-wire bit rate), `loopback` (layout, filters, classic + FD-64 TX/RX through internal loopback), `extended` (29-bit identifiers), `filters` (masked acceptance filtering), `remote` (RTR frames), `layouts` (multi-FIFO RAM budgeting and routing), and `soak` (sustained traffic with a corruption-rate report). `chip2chip` (classic, then FD-48 with bit-rate switch, between two chips) and `multinode` (broadcast delivery, per-node acceptance filters, back-to-back delivery across three nodes) additionally need transceivers and a terminated CAN bus.

Logs leave over the RP2040's own USB port as CDC-ACM serial, so **no debug probe is needed** — any serial terminal reads them.

See [`examples/rp2040/README.md`](examples/rp2040/README.md) for the board wiring, the crystal/SPI-clock assumptions, and how to build and flash them.

## Bit timing and the clock it assumes

Bit-timing presets are named for the SYSCLK they are computed against
(`KBPS500_40MHZ`, `KBPS500_20MHZ`, …). **A preset is only correct on the clock
in its name.** `Config::validate` range-checks the fields but cannot tell that a
40 MHz preset was paired with a 20 MHz crystal: every register value is
individually legal, so the bus just runs at half the intended rate.

Internal loopback cannot catch it either — both ends of the link share the same
oscillator, so loopback passes at the wrong bit rate. Check the pairing
explicitly:

```rust
let sysclk = config.clock.sysclk_hz();
assert_eq!(config.nominal.bit_rate_hz(sysclk), 500_000);
assert_eq!(config.nominal.sample_point_permille(), 800);
```

This is not hypothetical: it is how a board ran at 250 kbit/s while every
loopback test reported success. `examples/rp2040`'s `bitrate` binary measures
the real on-wire rate and reports the mismatch.

The SPI clock is bound to the same assumption — `max_spi_hz(sysclk)` is the
erratum-safe cap of 0.85 × SYSCLK/2, so a wrong crystal also over-clocks the
bus, which corrupts register and message-RAM reads.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

## References

- MCP2518FD datasheet (Microchip DS20006027B)
- MCP25XXFD Family Reference Manual (Microchip DS20005678E)
- MCP2517FD silicon errata (Microchip DS80000792)
- MCP2518FD silicon errata (Microchip DS80000789)
- [Emandhal/MCP251XFD](https://github.com/Emandhal/MCP251XFD) — a C driver for the same family, used as a cross-check during development
