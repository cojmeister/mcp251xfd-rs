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

## SPI clock limit (silicon erratum)

RAM reads corrupt above `0.85 * SYSCLK / 2` — 17 MHz at the recommended 40 MHz SYSCLK. The driver cannot observe your bus clock; size it with `max_spi_hz`. `MCP251xFd::init` verifies communication with a RAM echo test and fails with `Error::CommunicationCheckFailed` on an over-clocked bus.

## Feature flags

| Flag | Effect |
|---|---|
| `async` | Adds `MCP251xFdAsync` over `embedded_hal_async::spi::SpiDevice` |
| `defmt` | Implements `defmt::Format` on public error, config, and frame types |
| `log` | Depends on the `log` crate (reserved for future diagnostics) |

## Status

This is v0.1: reset/init, oscillator setup with variant detection, bit timing, FIFO layout, acceptance filters, classic + FD transmit, receive, interrupt flags/events, error counters, and an async `wait_rx` helper on the nINT pin.

Not yet implemented:

- CRC-protected SPI transfers (the safe-write/CRC opcodes)
- The Transmit Event FIFO (TEF) and the dedicated TX Queue (TXQ)
- Helpers for Listen-Only / Restricted Operation beyond the generic `set_mode`
- Sleep/wake conveniences beyond `set_mode` (fire-and-forget for Sleep — see its docs)
- GPIO pin (`IOCON`) and CLKO divider control
- Interrupt sources other than RX (TEF, error, wake-up, mode-change, etc.)

## Hardware examples

Runnable examples for the RP2040 (via embassy) are planned under `examples/rp2040`.

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
