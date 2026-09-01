# Field-feedback gaps: design

Date: 2026-09-01
Driver rev under test by the reporter: `f33acb0`, `features = ["async"]`

Source: field feedback from a downstream project running **ten MCP2517FD**
controllers on one shared SPI bus from an RP2040 (Cortex-M0+, `no_std`,
embassy, no allocator), classic CAN at 500 kbit/s, three frames per
controller every 2 ms (500 Hz, hard deadline). 20 MHz crystal, so
`max_spi_hz(20_000_000)` = 8.5 MHz requested, quantised by the RP2040 clock
divider to 7.5 MHz actual.

Two consumer constraints shape the whole design: nothing may block
unboundedly or allocate, and **chip-select transaction count, not clocked
bytes, is the bottleneck** at ten controllers.

## Sources

Every register address, bit position and behavioural claim below was checked
against a primary document. Nothing here rests on recollection.

- **DS20006027B** — MCP2518FD data sheet (SPI framing, SFR access rules)
- **DS20005678E** — MCP25XXFD Family Reference Manual (modes, FIFO reset,
  transmission sequence)
- **DS80000792D** — MCP2517FD Silicon Errata
- **DS80000789F** — MCP2518FD Silicon Errata
- Linux mainline `drivers/net/can/spi/mcp251xfd/` — independent cross-check
  of register bit positions and of the errata-1 symptom pairing

## 1. Root cause of the transmit stall (report item 3)

### Finding

The stall is **MCP2517FD Silicon Errata DS80000792D item 1**, "TX MAB
underflow/RX MAB overflow due to long delays between SPI bytes". Quoted:

> The SPI interface may block the CAN FD Controller module from accessing RAM
> in-between SPI bytes and between the last byte and the rising edge of the
> nCS line during an SPI READ or SPI READ_CRC instruction while accessing
> RAM. If the CAN FD Controller module is blocked for more than TSPIMAXDLY, a
> TX MAB underflow or an RX MAB overflow may occur.
>
> In case of a TX MAB underflow, the device will notify the application by
> setting SERRIF and MODIF and by transitioning to Restricted Operation or
> Listen Only mode (depending on CiCON.SERR2LOM). After the application
> requests Normal mode, the CAN FD Controller module will automatically
> attempt to retransmit the message that caused the TX MAB underflow. It is
> not necessary to reset the device.

### How it accounts for each reported observation

| Reported observation | Explanation |
| --- | --- |
| `CiINT` latches `SERRIF` (12) and `IVMIF` (15) | `SERRIF` per the erratum. `IVMIF` arises from the aborted CAN frame; Linux mainline documents exactly this pairing ("there are Bus Errors due to the aborted CAN frame, so a IVMIF will be seen as well"). |
| TX FIFO reports full and stops draining | Restricted Operation and Listen Only both **ignore `TXREQ`**. Queued frames are never handed to the bus, so the FIFO fills. |
| `CiTREC` clean: `TEC 0`, `REC 0`, `TXBO false`, `TXBP false` | The device performed a **mode transition**, not a bus-error escalation. Error counters are not involved. |
| Stage 1 (`clear_interrupts` on latched flags) never sufficient | Clearing an interrupt flag does not change `CiCON.OPMOD`. The device stays in Restricted/Listen Only. |
| Stage 2 (mode cycle) sufficient 700/700 | The ladder contains `set_mode(Normal)`, which is the documented recovery. |
| Stage 3 (full `reset()` + RAM zero-fill) never needed | The erratum states verbatim: "It is not necessary to reset the device." |
| Board self-heals once load stops | Load is what supplies the RAM-read contention that trips TSPIMAXDLY. |

### Why a transmit-only load could not reproduce it

The erratum fires during an SPI **READ** instruction *while accessing RAM*.
`transmit()` issues `write_ram`; only `receive()` issues `read_ram`, and it
does so twice per frame (8-byte header, then payload). A transmit-only load
never touches the trigger, which is why 86,901 sustained transmits were
clean while the receive-then-echo path faults at ~5.4/s.

### The consumer's timing budget

DS80000792D Table 1, worst-case scenarios:

| Scenario | Frame format | TSPIMAXDLY |
| --- | --- | --- |
| 1 | CAN Base Frame | 5 NBT |
| 2 | CAN FD Control Field | 3 NBT + 5 DBT |
| 3 | CAN FD Data Phase | 32 DBT |

Classic CAN at 500 kbit/s gives NBT = 2 us, so scenario 1 allows **10 us**.
At 7.5 MHz SCK one SPI byte is ~1.07 us. Ten byte-times of stall inside a RAM
read, or between its last byte and nCS rising, is enough.

### Items 3 and 4 are the same defect

The report's item 4 establishes that on `embassy-rp` 0.4.0 every SPI DMA
completion raised by core 1 is serviced in core 0's NVIC, at arbitrary phase
relative to core 0's own real-time cycle. A delayed DMA completion delays the
end of the `SpiDevice` transaction and therefore **the nCS rising edge** —
which is precisely the window DS80000792D item 1 names. The blocking driver
on core 1 removes that interrupt entirely.

**Prediction to be tested on hardware:** running the blocking driver on core 1
eliminates or greatly reduces the stall rate.
`examples/rp2040/src/bin/stall.rs` and `blocking_core1.rs` are designed to
test this.

### Three corrections to the report

1. **`apply_layout`'s `FRESET` is not the load-bearing step.** FRM section
   4.14: "A FIFO can be reset by: Setting `CiFIFOCONm.FRESET` **or** Placing
   the module into Configuration Mode (OPMOD = 100)." The reporter's
   `set_mode(Configuration)` already reset every FIFO. The `FRESET` inside
   `apply_layout` was redundant in that ladder; `set_mode(Normal)` did the
   work. The production dependency on the undocumented side effect is not
   needed.
2. **The 22,000 us ladder is replaceable by a bounded one.** FRM Figure 2-1
   shows `System Error` as a transition edge from the Normal modes into
   Restricted Operation / Listen Only, whose exit edge is `REQOP = "Normal"`
   — a direct edge, with no Configuration-mode round trip. Recovery is a
   `CiCON` read, a `REQOP` write, and a flag clear, plus bus integration
   (11 consecutive recessive bits, ~22 us at 500 kbit/s).
3. **This anomaly is MCP2517FD-only.** DS80000789F (MCP2518FD / MCP251863)
   contains no TX MAB underflow item. Linux gates the behaviour behind a
   per-device quirk. Relevant if the fleet ever mixes parts.

## 2. Verified register facts

All confirmed against DS20006027B / DS20005678E and cross-checked against
Linux `mcp251xfd.h`. Every bit position the driver already uses was verified
correct; the following are bits the driver does **not** yet expose.

| Register | Bit | Name | Use |
| --- | --- | --- | --- |
| `CiCON` | 18 | `SERR2LOM` | selects Restricted vs Listen Only on system error |
| `CiCON` | 11 | `BUSY` | module is transmitting/receiving |
| `CiFIFOSTA` | 1 | `TFHRFHIF` | TX half empty / RX half full |
| `CiFIFOSTA` | 2 | `TFERFFIF` | **TX empty / RX full** — the bit the reporter hand-rolled |
| `CiFIFOSTA` | 5 | `TXERR` | bus error during transmission |
| `CiFIFOSTA` | 6 | `TXLARB` | lost arbitration |
| `CiFIFOSTA` | 7 | `TXABT` | message aborted |

`CiFIFOCON.FRESET` is bit 10, i.e. **byte 1, mask `0x04`** — writable with the
driver's existing single-byte SFR write.

FRM section 4.14, governing `reset_fifo`:

> Resetting the FIFO will reset the Head and Tail Pointers, and the
> `CiFIFOSTAm` register. The settings in the `CiFIFOCONm` register will not
> change. Before resetting a TX FIFO using FRESET, ensure no transmissions
> are pending.

DS20006027B section 4.1, governing the combined status+address read:

> The SFR access is byte-oriented. Any number of data bytes can be read or
> written with one instruction. The address is incremented by one
> automatically after every data byte. The address rolls over from 0xFFF to
> 0x000.

`CiFIFOSTA` sits at `0x05C + 12(m-1) + 4` and `CiFIFOUA` at `+ 8`, i.e. they
are adjacent and inside the SFR space with no rollover boundary between them.
One 8-byte READ at the `CiFIFOSTA` address returns both.

## 3. API additions

All new driver methods live inside the existing `maybe_async_cfg::maybe`
impl block, so blocking and async variants are generated from one source and
parity remains automatic.

### `src/registers/mod.rs`

- `CiCon`: add `serr2lom` (18), `busy` (11).
- `CiFifoSta`: add `half_full` (1), `tx_empty_or_rx_full` (2), `tx_err` (5),
  `tx_lost_arbitration` (6), `tx_aborted` (7).
- `CiFifoCon`: add `CON_BYTE1_FRESET: u8 = 0x04`.

### `src/driver.rs`

| Method | Transactions | Report item |
| --- | --- | --- |
| `read_register_raw(addr: u16) -> u32` | 1 | 1 |
| `write_register_raw(addr: u16, value: u32)` | 1 | 1 |
| `control_register() -> CiCon` | 1 | 1 |
| `fifo_config(fifo) -> CiFifoCon` | 1 | 1, 2 |
| `fifo_user_address(fifo) -> u32` | 1 | 1 |
| `read_back_config() -> ChipConfig` | 2 | 1 |
| `reset_fifo(fifo)` | 1 | 3 |
| `recover_system_error(mode, delay) -> bool` | ~3 + mode poll | 3 |
| `transmit_batch(fifo, &[Frame]) -> u8` | 3N | 5 |

Raw accessors are **not** feature-gated: a bench operator needs them on the
build already flashed, and reflashing may cost a physical button press on an
inaccessible board. Both carry doc warnings that they bypass the driver's
state tracking and that writing configuration registers through them can
desynchronise the driver from the chip.

`ChipConfig` is a plain struct of `CiCon`, `CiNbtCfg`, `CiDbtCfg`, `CiTdc` so
"what I asked for" can be diffed against "what the chip has". `C1CON`/
`C1NBTCFG` and `C1DBTCFG`/`C1TDC` are two adjacent pairs, so `read_back_config`
costs two transactions rather than four. It is defined
in `driver.rs` alongside `Event` and re-exported from `lib.rs`. `CiNbtCfg`,
`CiDbtCfg` and `CiTdc` are already public in `registers`, but are not
currently re-exported at the crate root; this change adds them, since a
`ChipConfig` whose field types cannot be named is not usable.

`recover_system_error<D: DelayNs>(mode: OperationMode, delay: &mut D) ->
Result<bool, Error<SPI::Error>>` reads `CiCON`. If `OPMOD` is neither
`RestrictedOperation` nor `ListenOnly` it returns `Ok(false)` having issued
no writes. Otherwise, in this order:

1. clear `SERRIF | MODIF | IVMIF` via the existing `clear_interrupts` path,
2. request `mode` via the existing `set_mode`, which polls `OPMOD`,

then return `Ok(true)`. Flags are cleared *before* the mode request so that a
second underflow occurring during recovery latches a fresh `SERRIF` rather
than being masked by the stale one.

The target mode is an explicit parameter rather than remembered state, so the
method stays honest about the driver holding no mode record. Passing a mode
the FSM cannot reach directly surfaces as the existing
`Error::ModeChangeTimeout`; the caller passes whichever Normal mode it was
running in.

`transmit_batch` performs one readiness check and then queues frames,
returning the count accepted so partial success stays visible. It stops at
the first refusal rather than reordering.

### Hot-path change

`transmit_raw` and `receive` currently issue `read_sfr32(fifo_sta)` followed
by `read_sfr32(fifo_ua)`. These fold into a single 8-byte read at
`fifo_sta`, giving **4 to 3 chip-select transactions per frame** for every
existing caller with no API change. For the reporter's workload that is 120
to 90 transactions per 2 ms cycle.

This is unaffected by the `FIFOCI` corruption erratum (DS80000792D item 7 /
DS80000789F item 6), because only bit 0 of `CiFIFOSTA` is consumed.

### Explicitly not doing

- **No `transmit_is_progressing` probe.** A health check that queues a real
  frame injects traffic onto a live bus, and on a wedged bus adds a frame
  that may never leave. `fifo_config().txreq()` plus `CiFIFOSTA.TFERFFIF`
  gives the caller the same signal with no side effect and no driver-owned
  timing policy.
- **No per-FIFO `PLSIZE`/depth/base tracking.** Predicting `CiFIFOUA` would
  require mirroring the chip's RAM allocator and would desynchronise silently
  whenever `write_register_raw` or a gapped layout is used. The `CiFIFOUA`
  read stays.
- **The report's estimate of ~50 transactions per cycle is not reachable.**
  The `CiFIFOUA` read and the `UINC|TXREQ` write are both mandatory per
  frame, and DS20006027B section 4.1's framing permits exactly one command
  word per chip-select assertion, so they cannot be merged. 90 is the floor
  without address tracking.

## 4. Documentation

- `apply_layout`: document that it asserts `FRESET` on every FIFO it
  configures, state that this is intentional, and point at `reset_fifo` as
  the primitive for resetting a FIFO without rewriting configuration.
- `transmit`, `transmit_fd`, `fifo_status`,
  `CiFifoSta::not_full_or_not_empty`: warn that FIFO room is **not** a health
  signal, and cross-reference the known-anomaly section.
- `max_spi_hz`: name the erratum items precisely — DS80000792D item 5 is
  corrupted **reads** on the MCP2517FD, DS80000789F item 4 is corrupted
  **writes** on the MCP2518FD — so a reader knows the ceiling is a
  correctness limit, not a conservative guess. Note that host HALs quantise
  **downward** (8.5 MHz requested becomes 7.5 MHz on an RP2040 at 120 MHz
  `clk_peri`, 7.8125 MHz at 125 MHz), so a scope reading below the cap is not
  a misconfiguration. Record the reporter's measurement: clean to 12.5 MHz,
  failing at 15.625 MHz.
- `lib.rs` gains three sections, mirrored in `README.md`:
  - **SPI mode**: mode (0,0) is a chip requirement. It happens to match
    `SpiConfig::default()` on `embassy-rp`, so it will silently work until
    someone uses a HAL whose default differs and then fail like a wiring
    fault.
  - **Choosing blocking vs async**: async suits a core shared with other
    work; blocking suits a dedicated core, and specifically any target where
    DMA completion interrupts are serviced on a different core than the one
    issuing them. Name the RP2040 multicore case with the `embassy-rp`
    `src/dma.rs` mechanism, and note that for 3-18 byte transfers DMA setup
    overhead dominates.
  - **Known hardware anomalies**: the DS80000792D item 1 signature table,
    the recovery, and the MCP2517FD-only scope.

## 5. Examples

Four new binaries in `examples/rp2040/src/bin/`, built on the existing
verified `common.rs`. `common.rs` gains one **additive** function for the
blocking case; no existing function is modified.

| Binary | Purpose |
| --- | --- |
| `regdump.rs` | Dumps configuration registers across all ten chips using the new raw and typed accessors; diffs `read_back_config()` against the `Config` that was applied. |
| `stall.rs` | Drives the receive-then-echo load that reproduces the fault, detects the `OPMOD` transition into Restricted/Listen Only, and times four recovery ladders head-to-head: clear-flags-only, `recover_system_error`, `reset_fifo`, and the full mode cycle. Validates the root cause on hardware. **Needs the CAN bus wired**: it must run `Normal20`, because FRM Figure 2-1 shows the System Error transition leaving the *Normal* modes and internal loopback is a *Debug* mode — a loopback run would prove nothing either way. |
| `blocking_core1.rs` | Blocking driver on core 1 via `embassy-rp` multicore with a shared-bus `SpiDevice`, measuring cycle jitter on core 0 and counting stalls the same way `stall.rs` does. Tests the prediction that removing the cross-core DMA interrupt removes the stall. **Needs the CAN bus wired**, and runs `Normal20` so its count is directly comparable with `stall.rs`. |
| `batch.rs` | Measures `transmit` versus `transmit_batch` transaction count and cycle cost. |

## 6. Testing

- Every new driver method gets `embedded-hal-mock` coverage in
  `tests/driver.rs` and `tests/async_driver.rs`, asserting the exact SPI
  transaction sequence.
- The combined `CiFIFOSTA`+`CiFIFOUA` read changes the expected transaction
  sequence of the existing transmit and receive tests. Updating those
  expectations is itself the proof that the transaction count dropped.
- `recover_system_error` is tested for both branches: a chip reporting
  Restricted Operation (recovers, returns `true`) and one reporting Normal
  (no writes issued, returns `false`).
- Hardware validation is the four example binaries. Nothing in this change is
  claimed hardware-verified until those are run on the ten-chip board.

## 7. Packaging

`Cargo.toml` currently has `exclude = [".github/"]`, with a comment saying
internal design artefacts are not part of the crate — but `docs/` is not
listed, so this spec would ship inside the published package. Commit
`1412d62` ("docs: remove internal design documents") shows that is not the
intent. `docs/` joins the exclude list.

## 8. Out of scope

- CRC-protected SPI transfers (`READ_CRC` / `WRITE_CRC` / `WRITE_SAFE`),
  which DS80000789F item 1 recommends for the MCP2518FD. Larger change,
  independent of this feedback.
- TXQ and TEF support.
- Any change to `init`'s sequencing.
