# mcp251xfd — Rust driver for the Microchip MCP251XFD CAN FD controller family

**Date:** 2026-08-19
**Status:** Approved design, pending implementation plan
**Crate name:** `mcp251xfd` (unclaimed on crates.io as of this date)

## 1. Goal and scope

A `#![no_std]` Rust driver crate for the Microchip MCP2517FD, MCP2518FD, and
MCP251863 external SPI-to-CAN-FD controllers, published on crates.io. One
driver supports all three variants (they are register-compatible; the driver
detects the variant at init and gates the few differences).

The driver must work in both blocking and async contexts from a single
codebase, and integrate with embassy-rs purely through standard
`embedded-hal` / `embedded-hal-async` 1.0 traits — no embassy dependency in
the driver itself.

### v0.1 scope (approved)

- Reset, oscillator/clock configuration, variant detection, init sequence
- Bit timing (nominal + data) via explicit values and const presets
- FIFO configuration with compile-time RAM layout checking
- Transmit and receive for classic CAN 2.0 and CAN FD (up to 64-byte payloads)
- Acceptance filters (mask/match, routed to a FIFO)
- Operation mode control (Configuration, Normal FD, Normal 2.0, Internal
  Loopback, Listen-Only)
- Interrupt flag read/clear, CiVEC event decode, async `wait_rx` on an INT pin
- Error counters / bus state (CiTREC)

### Deferred (designed-for, not in v0.1 — each has a reserved home)

- CRC-protected SPI commands (READ_CRC 0xB / WRITE_CRC 0xA / SAFE_WRITE 0xC):
  reserved opcode entries in the bus layer
- TEF (transmit event FIFO), RX timestamping (field already present on
  `RxFrame`, always `None` in v0.1)
- Sleep / Low-Power Mode, chip GPIO pins (IOCON), ECC enablement
- Bit-timing solver from raw bitrates (v0.1 ships presets + explicit values)
- Buffered embassy-style runner (background task + TX/RX handles) as an
  optional layer on top

## 2. Reference hardware

Primary test platform: RP2040 with **10× MCP2517FD** on one shared SPI bus.

- SPI1: GPIO 10 = SCK, GPIO 11 = MOSI, GPIO 12 = MISO
- Chip selects: GPIOs 3–9, 13, 14, 15 (one per chip)

Topology consequence: the driver is generic over `SpiDevice` (which owns CS
framing); users construct ten `SpiDevice`s over one shared bus
(`embassy-embedded-hal` shared_bus or `embedded-hal-bus`) and ten driver
instances.

## 3. Architecture: three layers

```
src/
├── lib.rs           # no_std, deny(missing_docs), re-exports
├── registers/
│   ├── mod.rs       # register types: addresses + bitfield accessors, no I/O
│   ├── objects.rs   # TX/RX message object encode/decode (T0/T1, R0/R1)
│   └── ram.rs       # const-fn FIFO RAM layout planner (2 KB budget)
├── frame.rs         # Frame (classic), FdFrame, RxFrame; embedded_can interop
├── config.rs        # ClockConfig, BitTiming (+presets), FifoLayout, FilterConfig
├── bus.rs           # Layer 1: SPI command/transaction layer (maybe-async-cfg)
├── driver.rs        # Layer 2: MCP251xFd / MCP251xFdAsync
└── error.rs         # Error<SpiErr>
```

Each layer is independently testable: `registers/` is pure data (host unit
tests), `bus.rs`/`driver.rs` test against `embedded-hal-mock` with byte-exact
SPI expectations.

### 3.1 Layer 0 — `registers/`

Hand-rolled newtypes over `u32` with `const fn` getters/setters; no
proc-macro bitfield crate (small register count, reviewable against the
datasheet, minimal dependency tree, everything const-testable).

```rust
pub struct CiCon(pub u32);
impl CiCon {
    pub const ADDR: u16 = 0x000;
    pub const fn with_req_op_mode(self, m: OperationMode) -> Self { /* bits 26:24 */ }
    pub const fn op_mode(self) -> OperationMode { /* bits 23:21 */ }
}
```

Registers in v0.1: `CiCON` (0x000), `CiNBTCFG` (0x004), `CiDBTCFG` (0x008),
`CiTDC` (0x00C), `CiINT` (0x01C), `CiVEC` (0x018), `CiTREC` (0x034),
`CiFIFOCONm/CiFIFOSTAm/CiFIFOUAm` (computed: `0x05C + 12*(m-1)`, m = 1..=31),
`CiFLTCONm` (0x1D0 + m, byte-granular), `CiFLTOBJm`/`CiMASKm`
(0x1F0/0x1F4 + 8*m), `OSC` (0xE00), `IOCON` (0xE04, byte access only),
`ECCCON` (0xE0C, touched only to confirm ECC disabled).

`objects.rs`: pure functions

- `encode_tx(&frame, seq) -> TxHeader([u32; 2])` — SID/EID packing into
  T0 (which differs from the natural 29-bit ID layout), T1 flags
  (DLC, IDE, RTR, BRS, FDF, ESI, SEQ)
- `decode_rx([u32; 2]) -> RxHeader` — R0/R1 including FILHIT
- DLC ↔ byte-length mapping (0–8, 12, 16, 20, 24, 32, 48, 64)
- payload padding to 32-bit word multiples

`ram.rs`: the RAM planner. **Fully `const fn`** so layouts declared as
`const` are validated at compile time (const-eval panic on overflow =
compile error); the same code path returns `Err(RamOverflow)` when built at
runtime. Element size = 8 B header + padded payload; FIFOs allocate
contiguously from 0x400 in the chip's fixed order; total budget 2048 bytes.

### 3.2 Frame types — `frame.rs`

```rust
pub struct Frame   { id: embedded_can::Id, dlc: u8, rtr: bool, data: [u8; 8] }
pub struct FdFrame { id: embedded_can::Id, len: u8, flags: FrameFlags, data: [u8; 64] }
pub enum RxFrame   { Classic(Frame), Fd(FdFrame) }  // + timestamp: Option<u32> (None in v0.1)
```

- `Frame` implements `embedded_can::Frame` (v0.4) for ecosystem interop;
  `embedded_can::Id`/`StandardId`/`ExtendedId` are used everywhere (no own
  ID types).
- `FdFrame` constructors reject invalid lengths; `new_padded` rounds up to
  the next valid DLC (documented). `FrameFlags` carries BRS/ESI. No RTR in
  FD, enforced by the type.
- No ecosystem-standard FD frame trait exists (`embedded-can` is classic
  only); this is the accepted pattern (embassy-stm32, mcp2517, mcp2518fd all
  roll their own).

### 3.3 Layer 1 — `bus.rs`

SPI protocol: 16-bit big-endian command word = 4-bit opcode + 12-bit address.
v0.1 opcodes: `RESET (0x0)`, `WRITE (0x2)`, `READ (0x3)`. CRC opcodes
(0xA/0xB/0xC) are reserved enum entries for later.

All I/O goes through `SpiDevice::transaction` with multi-`Operation`
sequences (never manual CS):

- write: `[Write(cmd[0..2]), Write(data)]`
- read: `[Write(cmd[0..2]), Read(buf)]`

Register data is LSB-first on the wire (little-endian u32). RAM accesses
(0x400–0xBFF) are word-aligned and word-multiple, enforced here. One
internal scratch buffer (78 bytes: 2 cmd + 8 header + 64 payload + 4
timestamp) — no alloc.

API: `read_sfr8/read_sfr32`, `write_sfr8/write_sfr32`, `read_ram`,
`write_ram`, `reset`.

### 3.4 Sync/async strategy

Written once in async style; **`maybe-async-cfg`** (version-pinned, the
ssd1306 pattern) generates:

- `MCP251xFd<SPI: embedded_hal::spi::SpiDevice>` — always available
- `MCP251xFdAsync<SPI: embedded_hal_async::spi::SpiDevice>` — behind the
  `async` feature

Features are additive; both variants coexist in one binary. `DelayNs` (sync
or async twin) is passed into `init()` only, not stored. Async-only extras
(e.g. `wait_rx`) additionally bound on `embedded_hal_async::digital::Wait`.

### 3.5 Layer 2 — `driver.rs`

**Init sequence** (mirrors the Emandhal C driver's recipe):

1. `reset()` → chip in Configuration mode
2. SPI sanity check: write/read `0xAA55AA55` at RAM top →
   `Error::CommunicationCheckFailed` on mismatch (catches wiring and
   too-fast SPI clock immediately)
3. Program `OSC` from `ClockConfig` (xtal Hz, PLL ×10 on/off, SCLKDIV);
   poll OSCRDY/PLLRDY/SCLKRDY, 4 ms timeout, ready bits required stable
4. **Variant detection:** set `OSC.LPMEN`, read back — sticks ⇒
   MCP2518FD/MCP251863, else MCP2517FD; then clear it. (DEVID is useless:
   returns the same value on all variants.) Stored variant gates SEQ width
   (7 vs 23 bits).
5. Zero-fill message RAM (2 KB)
6. Write `CiCON` (ISO CRC on, edge filter on when FD configured, RTXAT per
   config), `CiNBTCFG`/`CiDBTCFG`, and `CiTDC` (auto mode,
   `TDCO = DBRP × DTSEG1`) when a data bitrate is configured

**Bit timing:** explicit `NominalBitTiming { brp, tseg1, tseg2, sjw }` (+
optional `DataBitTiming`), with `const` presets for 40 MHz SYSCLK ×
{125k, 250k, 500k, 1M} nominal × {2M, 5M, 8M} data. Solver-from-bitrate is a
fast-follow.

**FIFO configuration:** whole-layout model.

```rust
const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 4)   // depth 4
    .rx_fifo(Fifo::F2, PayloadSize::B64, 8);  // compile error here if > 2048 B

can.apply_layout(&LAYOUT).await?;             // requires Configuration mode
can.set_filter(Filter::F0, Match::exact(id), Fifo::F2).await?;
can.set_mode(OperationMode::NormalFd).await?;
```

`apply_layout` outside Configuration mode ⇒ `Error::NotInConfigMode`. No
typestate in v0.1 (can be layered later without breaking the base API).

**TX:** `transmit(fifo, &frame)` — read `CiFIFOSTAm`; full ⇒
`Err(TxFifoFull)` (non-blocking contract); else read `CiFIFOUAm`, write
header+payload at `0x400 + UA`, set `UINC | TXREQ`. SEQ auto-increments per
instance, masked to the variant's width.

**RX:** `receive(fifo)` — read `CiFIFOSTAm`; empty ⇒ `Err(RxFifoEmpty)`;
else read UA, read 2 header words, decode DLC, read payload (rounded to
words), set `UINC`. RXOVIF surfaced via `fifo_status()`.

`transmit`/`receive` never busy-wait; awaits are SPI I/O only. Waiting
composes on top: async `wait_rx(fifo, &mut int_pin)` loops
{ check FIFO → if empty `int_pin.wait_for_low().await` } — race-free because
nINT is level-active open-drain. Sync users poll.

**Events:** `interrupt_flags() -> CiIntFlags`, `clear_interrupts(flags)`,
`pending_event() -> Event` (decoded `CiVEC`), `error_counters()` (CiTREC:
TEC/REC/bus-off).

### 3.6 Errors — `error.rs`

```rust
#[non_exhaustive]
pub enum Error<SpiErr> {
    Spi(SpiErr),
    CommunicationCheckFailed,
    ClockNotReady,
    ModeChangeTimeout,
    NotInConfigMode,
    TxFifoFull,
    RxFifoEmpty,
    RamOverflow,
    InvalidConfig(ConfigError),
    InvalidPayloadLength,
}
```

`Debug` always; `defmt::Format` / `log` behind features;
`core::error::Error` implemented (no_std-stable since Rust 1.81).

## 4. Errata handling (MCP2517FD, DS80000792D; most shared with 2518FD)

| Erratum | Handling |
|---|---|
| Fast SPI corrupts RAM reads — F_SCK ≤ 0.85 × SYSCLK/2 (17 MHz @ 40 MHz) | Not enforceable through `SpiDevice` (bus clock invisible). Provide `const fn max_spi_hz(sysclk_hz) -> u32` + prominent docs; init's RAM echo test catches violations in practice. |
| IOCON multi-byte write clears LAT0/LAT1 | IOCON accessed byte-wise only (`write_sfr8`); whole-register IOCON writes are not exposed. |
| SFR address rollover broken (0x3FF→0x400, 0xFFF→0x000) | Bus layer bounds-checks; never relies on wraparound. |
| TX-MAB underflow on long inter-byte gaps | Non-issue on MCU hardware SPI (back-to-back bytes); docs note for Linux-SPI users. On SERRIF+MODIF, re-request Normal mode (documented recovery). |
| READ_CRC wrong CRC on live registers | Deferred with CRC feature; design note: retry on mismatch, avoid FIFOs 7/15/23/31 for CRC-heavy use. |
| Wake-up OSCDIS failure | Deferred with sleep support (≥50 T_SYSCLK delay + double OSCRDY read). |

## 5. Dependencies and crate metadata

- `embedded-hal = "1.0"` (always), `embedded-hal-async = "1.0"` (feature
  `async`), `embedded-can = "0.4"` (always; tiny), `maybe-async-cfg`
  (pinned), optional `defmt`, optional `log`
- Features: `default = []`; `async`; `defmt`; `log` — all additive
- `#![no_std]`, `#![deny(missing_docs)]`, MSRV pinned (edition 2024 ⇒ 1.85+),
  dual license MIT OR Apache-2.0, docs.rs `all-features = true`

## 6. Testing

### Host tests (CI, no hardware)

- `registers/`: bitfield round-trips; TX/RX object encode/decode checked
  against byte sequences lifted from the C reference driver and datasheet
  examples (ID packing and DLC mapping are the highest-risk code)
- `ram.rs`: planner boundaries (exactly 2048, off-by-one-word), const +
  runtime paths; `trybuild` compile-fail test proving the const overflow
  check fires at build time
- `bus.rs` / `driver.rs`: `embedded-hal-mock` `SpiDevice` with **byte-exact
  transaction expectations** for reset, init, transmit, receive (e.g.
  transmit on FIFO1 ⇒ READ CiFIFOSTA1, READ CiFIFOUA1, WRITE 0x400+UA…,
  WRITE CiFIFOCON1 with UINC|TXREQ). Async paths via embedded-hal-mock's
  async support under `tokio::test`.

### Hardware verification (`examples/` directory)

`examples/rp2040/` is a separate, non-published workspace member (embassy-rp
based; embedded examples can't build for the host). Binaries:

1. **enumerate** — init all 10 chips over the shared bus; report per-chip
   variant detection + RAM echo result
2. **loopback** — Internal Loopback mode: transmit → receive on the same
   chip; verifies the whole stack per chip with no transceiver/bus wiring
3. **chip-to-chip** — classic 500k and FD 500k/2M between two chips (if the
   board wires them to a common CAN bus)

### CI (GitHub Actions)

- `cargo test` (host), `cargo build --no-default-features`,
  `cargo build --all-features`
- `cargo build --target thumbv6m-none-eabi` (RP2040; proves no_std, no
  accidental float/64-bit-div bloat)
- **Zero-warning policy:** `RUSTFLAGS="-D warnings"` on all builds and
  `cargo clippy --all-features -- -D warnings`
- `cargo fmt --check`; `cargo doc` with `RUSTDOCFLAGS="-D warnings"`

## 7. References

- Emandhal/MCP251XFD C driver — https://github.com/Emandhal/MCP251XFD
  (architecture and init-sequence reference; register bitfields
  cross-checkable against `MCP251XFD.h`)
- MCP2518FD datasheet DS20006027B; MCP25XXFD Family Reference Manual
  DS20005678E; MCP2517FD errata DS80000792D; AN4808 migration guide
- Prior art: `mcp2517` crate (active, sync-only), `adom-inc/mcp2518fd`
  (unpublished, async XOR sync via maybe-async), Linux `mcp251xfd` kernel
  driver (errata workarounds)
