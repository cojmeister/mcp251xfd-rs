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

Assumptions baked into `src/common.rs`. Both were **measured on this board**,
not taken from the silkscreen:

- **20 MHz crystal** (`ClockConfig::MHZ20`) with hand-built bit timings, since
  the library ships no `*_20MHZ` presets for 500 kbit/s + 2 Mbit/s. The board
  was originally configured for 40 MHz and ran the bus at half rate — 250
  kbit/s — while every loopback test passed, because internal loopback shares
  the same oscillator at both ends. `bitrate` is what caught it; re-run it if
  the board is revised.
- SPI runs at `mcp251xfd::max_spi_hz(CAN_CONFIG.clock.sysclk_hz())` = **8.5 MHz**
  at a 20 MHz SYSCLK (the erratum-safe cap of 0.85 × SYSCLK/2), which embassy-rp
  quantizes to **7.8125 MHz**. Derived from `CAN_CONFIG` so the two cannot drift
  apart. At the 15.625 MHz the 40 MHz assumption permitted, SPI reads corrupted
  intermittently: message-object headers came back as zeros, `CiFIFOUA` pointed
  into blank RAM, and `C1CON` returned a bogus `OPMOD` — surfacing as random
  `FDF` loss and `NotInConfigMode`. A sweep found the chips clean at every rate
  up to 12.5 MHz and broken at 15.625 MHz.
- CAN bit rates: 500 kbit/s nominal, 2 Mbit/s data (FD with bit-rate switch).

## Binaries

| Binary | Needs | What it proves |
|---|---|---|
| `enumerate` | SPI wiring only | Every chip resets, initializes, and reports its variant — run this first on new hardware. |
| `bitrate` | SPI wiring only | Measures the **actual on-wire bit rate** and checks it against `CAN_CONFIG`. The only test here that can catch a wrong crystal — run it second. |
| `loopback` | SPI wiring only | Full per-chip driver stack (layout, filters, classic + FD-64 TX/RX) via internal loopback — nothing touches the CAN pins. |
| `extended` | SPI wiring only | 29-bit extended identifiers: the SID/EID split and the standard-vs-extended distinction (`EXIDE`/`MIDE`). |
| `filters` | SPI wiring only | `FilterMatch::with_mask` — masked acceptance filtering, including masks covering only one half of the extended-ID split. |
| `remote` | SPI wiring only | Classic remote (RTR) frames: the RTR bit survives, no data is delivered, and no stale payload leaks out of the reused RAM slot. |
| `layouts` | SPI wiring only | Multi-FIFO layouts: several RX FIFOs of differing `PayloadSize`, each fed by its own filter, checking the chip's RAM address generation and routing. |
| `soak` | SPI wiring only | Sustained traffic at the production SPI clock, reporting corruption rate in ppm plus `TxFifoFull`, RX overflow, and TEC/REC. Runs until stopped. |
| `chip2chip` | Transceivers + common CAN bus | Chip 0 → chip 1 over the real bus: classic at the nominal rate, then FD-48 with bit-rate switch. |
| `multinode` | Transceivers + common CAN bus | Three nodes: broadcast delivery, per-node acceptance filters, and back-to-back delivery from two transmitters (20/20 frames over 10 rounds). |
| `regdump` | SPI wiring only | Dumps every configuration register the driver writes for all ten chips, and diffs the bit-timing registers against the values `CAN_CONFIG` implies. **Not yet verified on hardware.** |
| `stall` | Transceivers + common CAN bus | Reproduces the MCP2517FD TX MAB underflow stall (DS80000792D item 1) with a receive-then-echo load at 500 Hz on `Normal20`, reports the fault signature, and times four recovery ladders. **Not yet verified on hardware.** |
| `blocking_core1` | Transceivers + common CAN bus | The blocking driver run on core 1 under the same `Normal20` load as `stall`, while core 0 measures its own scheduling jitter -- run the two back to back to see whether the cross-core DMA interrupt is what causes the stall. **Not yet verified on hardware.** |
| `batch` | SPI wiring only | Times `transmit` in a loop against `transmit_batch` for ten chips, three frames each, at 500 Hz, and probes the partial-fill path against a two-deep FIFO. **Not yet verified on hardware.** |

Each binary **repeats its sweep every 5 seconds** rather than reporting once,
so output is still arriving whenever you open the serial port. Every pass
returns its chips to Configuration mode first, because `MCP251xFd::init`
documents that its opening RESET is only reliable from that mode.

Driver failures are logged with the error discriminant and the sweep moves on
to the next chip — one dead chip does not cost the diagnostics for the other
nine. `ClockNotReady` means the crystal is not what `CAN_CONFIG` claims,
`CommunicationCheckFailed` means CS wiring or an over-spec SPI clock, `Spi(_)`
means the RP2040 peripheral itself.

## Logging: USB serial, no probe required

Log output leaves over the RP2040's **own USB port** as a CDC-ACM serial device
(`log` + `embassy-usb-logger`), so these tests need no debug probe and no
`defmt` tooling — any serial terminal reads them as plain text. Baud rate is
irrelevant for USB CDC; pick anything.

The logger writes into a non-blocking 1 KiB pipe, so lines produced while no
terminal has the port open are dropped. That is why every binary loops.

## Building

```sh
rustup target add thumbv6m-none-eabi
cargo build --release
```

Builds all fourteen binaries with zero warnings. The `.cargo/config.toml` sets the
target and the linker scripts (`link.x`, `link-rp.x`); `memory.x` is the
standard RP2040 layout (2 MB flash, 256-byte boot2 region) and relies on
embassy-rp's default `BOOT_LOADER_W25Q080` second stage — a board with a
different flash part needs the matching `boot2-*` feature.

## Running

Host-side requirement is one tool:

```sh
cargo install elf2uf2-rs --locked
```

Hold **BOOTSEL** while plugging the board in (or while tapping RESET) so it
mounts as the `RPI-RP2` drive, then:

```sh
cargo run --release --bin enumerate    # expect: 10/10 chips initialized
cargo run --release --bin bitrate      # expect: measured ~500000 bit/s -- OK
cargo run --release --bin loopback     # expect: classic + FD-64 loopback OK per chip
cargo run --release --bin extended     # expect: extended: all 12 checks OK
cargo run --release --bin filters      # expect: filters: all 27 checks OK
cargo run --release --bin remote       # expect: remote: all 12 checks OK
cargo run --release --bin layouts      # expect: layouts: all 68 checks OK
cargo run --release --bin soak         # expect: cycle N: ... ALL OK (runs until stopped)
cargo run --release --bin chip2chip    # expect: classic A->B OK, FD-48 BRS A->B OK
cargo run --release --bin multinode    # expect: broadcast/selective/back-to-back OK
cargo run --release --bin regdump      # expect: per-chip register dump, no mismatch lines
cargo run --release --bin stall        # expect: fault signature + ladder timings (not yet verified)
cargo run --release --bin blocking_core1 # expect: core0/core1 jitter + stall counts (not yet verified)
cargo run --release --bin batch        # expect: transmit vs transmit_batch timings within noise of each other
```

The board reboots into the firmware and appears as a serial port (`COMn` on
Windows, `/dev/ttyACM0` on Linux). Open it in any terminal to watch the sweep.

To skip the separate terminal, change the runner in `.cargo/config.toml` to
`elf2uf2-rs -d -s` and `cargo run` attaches to the port itself. It holds the
port exclusively, so use one or the other.

Run them in that order: `enumerate` proves the SPI wiring, `bitrate` proves the
crystal assumption, and only then does `loopback`'s verdict mean anything.
`extended` and `filters` extend the SPI-only set to the identifier and
acceptance-filter paths.

Both of those lean on acceptance filters rather than plain round trips, and
deliberately so: `pack_id` writes the transmit object and `unpack_id` reads the
receive object, so a swapped SID/EID split would cancel out and a round trip
would look correct. The chip compares its own canonical `R0` against
`CiFLTOBJ`, which makes the filter an independent check — the same reason
`bitrate` exists rather than trusting loopback.

`soak` is the one to leave running. Everything else sends a handful of frames
per pass, which is exactly why an intermittent SPI fault stayed hidden for so
long: a 50%-per-frame corruption was invisible to `enumerate`, and a
1-in-10,000 fault would be invisible to all of the single-shot tests. `soak`
reports the corruption rate in parts per million, and only volume reaches
`SEQ` wraparound (every 128 transmits on the MCP2517FD), FIFO index
wraparound, and the `TxFifoFull` / RX-overflow paths.
A `bitrate` mismatch of almost exactly 2× means the crystal is not what
`CAN_CONFIG` claims, and it reports which direction.

`enumerate`/`loopback` failures point at CS wiring, SPI clock, or the crystal
assumption — not driver logic (the library's mock tests pin the byte
protocol). Intermittent, chip-varying corruption — `FDF` lost at random,
`NotInConfigMode` after a successful `init` — is the signature of an
over-clocked SPI bus, not of driver logic. `chip2chip`/`multinode` timeouts usually mean missing transceivers
or bus termination, and the failure path dumps `CiTREC` (TEC/REC, bus-off,
error-passive) to confirm it.

A panic — which the test bodies avoid by design — halts the core and the USB
serial port disappears from the host. That is the symptom to look for.

## Dependency notes

The embassy crates version-lock each other — bump them together if resolution
fails. `Cargo.lock` is committed so builds are reproducible, and CI builds this crate
with `--locked` so drift fails loudly instead of being silently rewritten.

`embassy-usb-logger` is pinned to the 0.4 line on purpose: 0.6 needs
`embassy-usb-driver` 0.2, and `embassy-rp` 0.4 provides 0.1.
