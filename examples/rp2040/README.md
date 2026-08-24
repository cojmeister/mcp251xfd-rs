# RP2040 hardware examples

Hardware acceptance tests for the [`mcp251xfd`](../..) driver, targeting a
test board with an RP2040 and **ten MCP2517FD controllers** on one shared SPI
bus. This crate is a standalone workspace — it is not built by the library's
`cargo test`/`cargo build` and keeps the embassy dependency tree out of the
library.

## Board wiring

| Signal | RP2040 pin |
|---|---|
| SPI1 SCK | GPIO 10 |
| SPI1 MOSI | GPIO 11 |
| SPI1 MISO | GPIO 12 |
| Chip selects (chips 0–9) | GPIO 3, 4, 5, 6, 7, 8, 9, 13, 14, 15 |

Assumptions baked into `src/common.rs`:

- **40 MHz crystal** on every MCP2517FD (`ClockConfig::MHZ40`). Verify on the
  board silkscreen/schematic before flashing — a 20 MHz board needs
  `ClockConfig::MHZ20`, hand-built bit timings (the library ships no 20 MHz
  presets), and an 8.5 MHz SPI clock.
- SPI runs at **17 MHz** = `mcp251xfd::max_spi_hz(40 MHz)`, the erratum-safe
  cap of 0.85 × SYSCLK/2.
- CAN bit rates: 500 kbit/s nominal, 2 Mbit/s data (FD with bit-rate switch).

## Binaries

| Binary | Needs | What it proves |
|---|---|---|
| `enumerate` | SPI wiring only | Every chip resets, initializes, and reports its variant — run this first on new hardware. |
| `loopback` | SPI wiring only | Full per-chip driver stack (layout, filters, classic + FD-64 TX/RX) via internal loopback — nothing touches the CAN pins. |
| `chip2chip` | Transceivers + common CAN bus | Chip 0 → chip 1 over the real bus: classic at the nominal rate, then FD-48 with bit-rate switch. |
| `multinode` | Transceivers + common CAN bus | Three nodes: broadcast delivery, per-node acceptance filters, and arbitration losslessness (20/20 frames over 10 contended rounds). |

## Building

```sh
rustup target add thumbv6m-none-eabi
cargo build --release
```

Builds all four binaries with zero warnings. The `.cargo/config.toml` sets the
target and the linker scripts (`link.x`, `link-rp.x`, `defmt.x`); `memory.x`
is the standard RP2040 layout with the 256-byte boot2 region.

## Running

Needs a debug probe and [probe-rs](https://probe.rs) (the configured runner):

```sh
cargo run --release --bin enumerate   # expect: 10/10 chips initialized
cargo run --release --bin loopback    # expect: classic + FD-64 loopback OK per chip
cargo run --release --bin chip2chip   # expect: classic A->B OK, FD-48 BRS A->B OK
cargo run --release --bin multinode   # expect: broadcast/selective/arbitration OK
```

Logs arrive over RTT via `defmt`. `enumerate`/`loopback` failures point at CS
wiring, SPI clock, or the crystal assumption — not driver logic (the library's
mock tests pin the byte protocol). `chip2chip`/`multinode` timeouts usually
mean missing transceivers or bus termination.

## Dependency notes

The embassy crates version-lock each other — bump them together if resolution
fails. `Cargo.lock` is committed so builds are reproducible.
