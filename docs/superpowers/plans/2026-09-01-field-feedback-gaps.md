# Field-Feedback Gaps Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close six gaps reported from a ten-controller MCP2517FD deployment: add raw/typed register read-back, a bounded system-error recovery primitive, a single-FIFO reset, a batched transmit, a 25% cut in per-frame SPI transactions, and the documentation and hardware examples that make the reported transmit stall diagnosable in an afternoon.

**Architecture:** All new driver methods go inside the existing `maybe_async_cfg::maybe` impl block in `src/driver.rs`, so blocking and async variants are generated from one source and parity stays automatic. New register bits go in `src/registers/mod.rs` as pure data. One new transaction primitive (`read_sfr32_pair`) goes in `src/bus.rs` and is used to fold the per-frame status and user-address reads into a single chip-select assertion. Four new hardware example binaries go in `examples/rp2040/src/bin/`, built on the existing verified `common.rs`.

**Tech Stack:** Rust 2024 edition, MSRV 1.85, `no_std`. `embedded-hal` 1.0 / `embedded-hal-async` 1.0, `embedded-can` 0.4, `maybe-async-cfg` 0.2.4. Tests use `embedded-hal-mock` 0.11 + `tokio`. Examples use `embassy-rp` 0.4, `embassy-executor` 0.7, `embassy-embedded-hal` 0.3.

**Spec:** `docs/superpowers/specs/2026-09-01-field-feedback-gaps-design.md` — read it first. It carries the errata quotations and the root-cause argument that these tasks encode.

## Global Constraints

- **`no_std`, no allocator, no unbounded blocking.** No `std`, no `alloc`, no `Vec` in `src/`. Test files are `std` and may use `Vec`.
- **MSRV 1.85, edition 2024.** No newer language features.
- **`#![deny(missing_docs)]` is set in `src/lib.rs`.** Every new public item — including every public struct field and enum variant — needs a doc comment, or the build fails.
- **Parity is automatic; do not hand-write async variants.** Any method added inside the `#[maybe_async_cfg::maybe(...)]` impl block gets both a sync and an async form. Never add a method to only one.
- **Every register address and bit position must match the spec's verified table.** Do not invent bit positions. The spec §2 lists each one with its source document.
- **Verification commands** (these are what CI runs — `--all-features` does *not* link for tests because `defmt` needs a global logger):
  - `cargo test` and `cargo test --features async`
  - `cargo clippy --features async --all-targets -- -D warnings`
  - `cargo fmt --check`
  - `cargo build --target thumbv6m-none-eabi --all-features`
- **Baseline at plan start:** 71 tests passing, fmt clean, clippy clean. Never commit with fewer passing tests than the task before.
- **Commit style:** Conventional Commits (`feat:`, `fix:`, `docs:`, `test:`, `perf:`). End every commit message body with:
  `Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>`
- **Branch:** `field-feedback-gaps`, already created, spec already committed.
- **Examples are a separate workspace.** `examples/rp2040` has its own `[workspace]` and `Cargo.lock`. Build them with `cd examples/rp2040 && cargo build --release`, never from the repo root.
- **Do not modify any existing function in `examples/rp2040/src/common.rs`.** It is hardware-verified. Additions only.

---

### Task 1: New register bits

Pure data, no I/O. Adds the `CiCON`, `CiFIFOSTA` and `CiFIFOCON` bits the driver does not yet expose but which later tasks and the diagnostic examples need.

**Files:**
- Modify: `src/registers/mod.rs` (`CiCon` impl ~line 399, `CiFifoCon` impl ~line 621, `CiFifoSta` impl ~line 681, tests module at end)

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `CiCon::busy() -> bool`, `CiCon::with_busy(bool) -> CiCon`
  - `CiCon::serr2lom() -> bool`, `CiCon::with_serr2lom(bool) -> CiCon`
  - `CiFifoSta::half_full() -> bool`, `with_half_full`
  - `CiFifoSta::tx_empty_or_rx_full() -> bool`, `with_tx_empty_or_rx_full`
  - `CiFifoSta::tx_err() -> bool`, `with_tx_err`
  - `CiFifoSta::tx_lost_arbitration() -> bool`, `with_tx_lost_arbitration`
  - `CiFifoSta::tx_aborted() -> bool`, `with_tx_aborted`
  - `CiFifoCon::CON_BYTE1_FRESET: u8` (value `0x04`)

The existing `bit!(get, with_get, bit, "doc")` macro at `src/registers/mod.rs:331` generates both accessors and their docs. Use it; do not hand-write these.

- [ ] **Step 1: Write the failing test**

Append to the `mod tests` block at the end of `src/registers/mod.rs`:

```rust
    #[test]
    fn new_con_bits_match_datasheet() {
        // DS20006027B Register 3-1 / Linux mcp251xfd.h.
        assert!(CiCon(1 << 11).busy());
        assert!(CiCon(1 << 18).serr2lom());
        assert_eq!(CiCon(0).with_busy(true).0, 1 << 11);
        assert_eq!(CiCon(0).with_serr2lom(true).0, 1 << 18);
        // Must not disturb the mode fields the driver already relies on.
        let c = CiCon(0).with_req_op_mode(OperationMode::Normal20).with_serr2lom(true);
        assert_eq!(c.0 >> 24 & 0b111, OperationMode::Normal20.bits() as u32);
    }

    #[test]
    fn new_fifo_status_bits_match_datasheet() {
        // DS20006027B Register 3-23.
        assert!(CiFifoSta(1 << 1).half_full());
        assert!(CiFifoSta(1 << 2).tx_empty_or_rx_full());
        assert!(CiFifoSta(1 << 5).tx_err());
        assert!(CiFifoSta(1 << 6).tx_lost_arbitration());
        assert!(CiFifoSta(1 << 7).tx_aborted());
    }

    /// Decodes the two `CiFIFOSTA` values captured on the reporter's board
    /// either side of the transmit stall (see the design doc, section 1).
    #[test]
    fn reported_stall_fifo_status_values_decode() {
        let healthy = CiFifoSta(0x0000_0A03);
        assert!(healthy.not_full_or_not_empty(), "TX FIFO had room");
        assert!(healthy.half_full());
        assert_eq!(healthy.fifo_index(), 10);

        let faulted = CiFifoSta(0x0000_0800);
        assert!(!faulted.not_full_or_not_empty(), "TX FIFO was full");
        assert!(!faulted.half_full());
        assert!(!faulted.tx_empty_or_rx_full());
        assert_eq!(faulted.fifo_index(), 8);
        // The stall is not a bus-error condition: none of the TX error bits
        // are set, matching the clean CiTREC that was reported alongside it.
        assert!(!faulted.tx_err());
        assert!(!faulted.tx_lost_arbitration());
        assert!(!faulted.tx_aborted());
    }

    #[test]
    fn freset_byte1_mask_matches_bit_10() {
        // FRESET is CiFIFOCON bit 10, i.e. bit 2 of byte 1.
        assert_eq!(CiFifoCon::CON_BYTE1_FRESET, 1 << (10 - 8));
        assert_eq!(CiFifoCon(0).with_freset(true).0 >> 8, CiFifoCon::CON_BYTE1_FRESET as u32);
    }
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test --lib registers`
Expected: FAIL — `no method named 'busy' found for struct 'CiCon'`, and similar for the other new accessors.

- [ ] **Step 3: Add the bits**

In `impl CiCon`, insert `busy` between the `protocol_exception_disabled` (bit 6) and `brs_disabled` (bit 12) entries, keeping the existing ascending-bit order:

```rust
    bit!(
        busy,
        with_busy,
        11,
        "`BUSY` (module is transmitting or receiving)"
    );
```

and insert `serr2lom` between `restrict_retx` (bit 16) and `store_tef` (bit 19):

```rust
    bit!(
        serr2lom,
        with_serr2lom,
        18,
        "`SERR2LOM` (on a system error, fall back to Listen Only instead of \
         Restricted Operation)"
    );
```

In `impl CiFifoSta`, insert `half_full` and `tx_empty_or_rx_full` after the existing bit-0 entry, and the three TX diagnostics after the bit-4 entry, preserving ascending order:

```rust
    bit!(
        half_full,
        with_half_full,
        1,
        "`TFHRFHIF` (TX: FIFO half or less full / RX: FIFO half or more full)"
    );
    bit!(
        tx_empty_or_rx_full,
        with_tx_empty_or_rx_full,
        2,
        "`TFERFFIF` (TX: FIFO empty / RX: FIFO full)"
    );
```

```rust
    bit!(
        tx_err,
        with_tx_err,
        5,
        "`TXERR` (a bus error occurred while transmitting)"
    );
    bit!(
        tx_lost_arbitration,
        with_tx_lost_arbitration,
        6,
        "`TXLARB` (arbitration was lost while transmitting)"
    );
    bit!(
        tx_aborted,
        with_tx_aborted,
        7,
        "`TXABT` (the transmission was aborted)"
    );
```

In `impl CiFifoCon`, add alongside the existing `CON_BYTE1_*` constants:

```rust
    /// Value for byte 1 setting `FRESET` only — resets one FIFO's pointers
    /// and status without touching its configuration bits.
    pub const CON_BYTE1_FRESET: u8 = 0x04;
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test --lib registers`
Expected: PASS.

- [ ] **Step 5: Full verification**

Run: `cargo test && cargo test --features async && cargo clippy --features async --all-targets -- -D warnings && cargo fmt --check`
Expected: all pass, test count 75 (71 baseline + 4 new).

- [ ] **Step 6: Commit**

```bash
git add src/registers/mod.rs
git commit -m "feat(registers): expose SERR2LOM, BUSY and the TX FIFO status bits

Adds CiCON.BUSY (11) and CiCON.SERR2LOM (18), CiFIFOSTA TFHRFHIF (1),
TFERFFIF (2), TXERR (5), TXLARB (6) and TXABT (7), and a CON_BYTE1_FRESET
constant for single-byte FRESET writes.

TFERFFIF is the TX-FIFO-empty bit a downstream consumer had to hand-roll
against a raw register read. Bit positions verified against DS20006027B
and cross-checked against Linux mcp251xfd.h.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: Paired SFR read primitive

Adds the bus-layer primitive that lets two adjacent 32-bit registers be fetched in one chip-select assertion. Used by Task 3 to cut the per-frame transaction count.

**Files:**
- Modify: `src/bus.rs` (add method after `read_sfr32`, add tests to both test modules)

**Interfaces:**
- Consumes: nothing.
- Produces: `Bus::read_sfr32_pair(&mut self, addr: u16) -> Result<(u32, u32), Error<SPI::Error>>` — returns the register at `addr` and the one at `addr + 4`. `pub(crate)`, and the async twin `BusAsync::read_sfr32_pair` is generated automatically.

- [ ] **Step 1: Write the failing test**

Add to the `mod tests` block in `src/bus.rs`:

```rust
    #[test]
    fn read_sfr32_pair_returns_both_registers_in_one_transaction() {
        // One command word, then eight data bytes: the SFR address
        // auto-increments after every byte (DS20006027B section 4.1), so this
        // returns CiFIFOSTA1 (0x060) followed by CiFIFOUA1 (0x064).
        let mut spi = Mock::new(&[
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x30, 0x60]),
            Transaction::read_vec(vec![
                0x03, 0x0A, 0x00, 0x00, // 0x060 -> 0x00000A03
                0x10, 0x00, 0x00, 0x00, // 0x064 -> 0x00000010
            ]),
            Transaction::transaction_end(),
        ]);
        let mut bus = Bus { spi: &mut spi };
        assert_eq!(
            bus.read_sfr32_pair(0x060).unwrap(),
            (0x0000_0A03, 0x0000_0010)
        );
        spi.done();
    }
```

And to the `mod async_tests` block:

```rust
    #[tokio::test]
    async fn async_read_sfr32_pair() {
        let mut spi = Mock::new(&[
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x30, 0x60]),
            Transaction::read_vec(vec![0x01, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00]),
            Transaction::transaction_end(),
        ]);
        let mut bus = BusAsync { spi: &mut spi };
        assert_eq!(
            bus.read_sfr32_pair(0x060).await.unwrap(),
            (0x0000_0001, 0x0000_0020)
        );
        spi.done();
    }
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test --features async --lib bus`
Expected: FAIL — `no method named 'read_sfr32_pair'`.

- [ ] **Step 3: Implement**

Insert into the `impl<SPI: SpiDevice> Bus<SPI>` block in `src/bus.rs`, directly after `read_sfr32`:

```rust
    /// Reads two consecutive 32-bit registers in a single transaction.
    ///
    /// Per DS20006027B section 4.1 the SFR address auto-increments after
    /// every data byte, so one 8-byte READ at `addr` returns the register at
    /// `addr` followed by the one at `addr + 4`. Both must lie inside the SFR
    /// space with no 0xFFF rollover between them — the rollover itself is
    /// broken silicon (DS80000792D item 4 / DS80000789F item 3), so the
    /// caller must not straddle it.
    ///
    /// Halves the chip-select count wherever the driver needs a status
    /// register and the user address that follows it.
    pub(crate) async fn read_sfr32_pair(
        &mut self,
        addr: u16,
    ) -> Result<(u32, u32), Error<SPI::Error>> {
        debug_assert!(addr % 4 == 0, "SFR pair reads must be 32-bit aligned");
        debug_assert!(addr < 0xFF8, "SFR pair read would straddle the address rollover");
        let c = cmd(Opcode::Read, addr);
        let mut buf = [0u8; 8];
        self.spi
            .transaction(&mut [Operation::Write(&c), Operation::Read(&mut buf)])
            .await
            .map_err(Error::Spi)?;
        Ok((
            u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
        ))
    }
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test --features async --lib bus`
Expected: PASS.

- [ ] **Step 5: Full verification**

Run: `cargo test && cargo test --features async && cargo clippy --features async --all-targets -- -D warnings && cargo fmt --check`
Expected: all pass, 77 tests.

- [ ] **Step 6: Commit**

```bash
git add src/bus.rs
git commit -m "feat(bus): add read_sfr32_pair for adjacent register reads

One 8-byte READ returns two consecutive 32-bit registers, since the SFR
address auto-increments after every data byte (DS20006027B section 4.1).

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: Fold the per-frame status and user-address reads

Cuts `transmit` and `receive` from four chip-select transactions per frame to three, for every existing caller, with no API change. For the reporting consumer that is 120 to 90 transactions per 2 ms cycle.

**Files:**
- Modify: `src/driver.rs` (`transmit_raw` ~line 485, `receive` ~line 554)
- Modify: `tests/driver.rs` (add `r32_pair` helper; update every transmit and receive test)
- Modify: `tests/async_driver.rs` (add `r32_pair` helper; update both tests)

**Interfaces:**
- Consumes: `Bus::read_sfr32_pair` from Task 2.
- Produces: no API change. `addr::fifo_ua(f) == addr::fifo_sta(f) + 4` holds by construction (`src/registers/mod.rs:46-52`), which is what makes the fold valid.

- [ ] **Step 1: Add the test helper and update the expectations**

Add to `tests/driver.rs` beside the other helpers:

```rust
/// One READ transaction returning two consecutive 32-bit registers.
fn r32_pair(addr: u16, lo: u32, hi: u32) -> Vec<Transaction<u8>> {
    let mut data = lo.to_le_bytes().to_vec();
    data.extend_from_slice(&hi.to_le_bytes());
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x30 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::read_vec(data),
        Transaction::transaction_end(),
    ]
}
```

Add the identical helper to `tests/async_driver.rs`.

Now find every site that reads a FIFO status register immediately followed by that FIFO's user address, and collapse the pair. Locate them with:

```bash
grep -n "r32(0x060\|r32(0x064\|r32(0x06C\|r32(0x070" tests/driver.rs tests/async_driver.rs
```

For each adjacent pair, replace

```rust
    e.extend(r32(0x060, STATUS));
    e.extend(r32(0x064, USER_ADDR));
```

with

```rust
    e.extend(r32_pair(0x060, STATUS, USER_ADDR));
```

**The full-FIFO and empty-FIFO bail-out paths also change**, because the driver now fetches both registers before it inspects the status. In `transmit_full_fifo_errors`, replace

```rust
    e.extend(r32(0x060, 0x0000_0000)); // full: TFNRFNIF clear
```

with

```rust
    // The user address is fetched in the same transaction and then discarded.
    e.extend(r32_pair(0x060, 0x0000_0000, 0x0000_0000)); // full: TFNRFNIF clear
```

Apply the same change to `async_receive_empty_errors` (`0x06C`) and to the first, empty poll inside `wait_rx_polls_until_frame_arrives` (`0x06C`), and to any other test whose FIFO read is a bail-out. Do not change `fifo_status` tests: `fifo_status` still issues a plain 4-byte read.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test --features async`
Expected: FAIL. The mock reports unexpected transactions — the driver still issues two separate reads while the expectations now describe one.

- [ ] **Step 3: Implement**

In `transmit_raw`, replace the two reads:

```rust
        let sta = CiFifoSta(self.bus.read_sfr32(addr::fifo_sta(fifo)).await?);
        if !sta.not_full_or_not_empty() {
            return Err(Error::TxFifoFull);
        }
        // ... comment ...
        let ua = self.bus.read_sfr32(addr::fifo_ua(fifo)).await?;
```

with a single paired read. Keep the existing comment about validating the raw value before narrowing:

```rust
        // `CiFIFOUA` sits directly above `CiFIFOSTA` (0x05C + 12(m-1) + 4 and
        // + 8), so one 8-byte READ fetches both and the readiness check costs
        // no chip-select assertion of its own.
        let (sta_raw, ua) = self.bus.read_sfr32_pair(addr::fifo_sta(fifo)).await?;
        if !CiFifoSta(sta_raw).not_full_or_not_empty() {
            return Err(Error::TxFifoFull);
        }
```

Leave the `ua` validation that follows exactly as it is.

Apply the same change in `receive`:

```rust
        let (sta_raw, ua) = self.bus.read_sfr32_pair(addr::fifo_sta(fifo)).await?;
        if !CiFifoSta(sta_raw).not_full_or_not_empty() {
            return Err(Error::RxFifoEmpty);
        }
```

again leaving the `ua` validation below untouched.

If `CiFifoSta` becomes unused in `driver.rs` after this, it will not — `fifo_status` still returns it. Do not remove the import.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test && cargo test --features async`
Expected: PASS, 77 tests.

- [ ] **Step 5: Full verification**

Run: `cargo clippy --features async --all-targets -- -D warnings && cargo fmt --check`
Expected: pass.

- [ ] **Step 6: Commit**

```bash
git add src/driver.rs tests/driver.rs tests/async_driver.rs
git commit -m "perf(driver): fetch FIFO status and user address in one transaction

CiFIFOSTA and CiFIFOUA are adjacent and the SFR address auto-increments
after every data byte, so one 8-byte READ replaces the two 4-byte READs
that transmit and receive each issued.

Four chip-select transactions per frame become three. At ten controllers
sending three frames per 2 ms cycle that is 120 transactions per cycle
down to 90; transaction count, not clocked bytes, is the binding
constraint at that fan-out.

Unaffected by the FIFOCI corruption erratum (DS80000792D item 7), which
touches bits the driver does not read.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: Raw and typed register read-back

The highest-value gap in the report: a consumer had to pull `embedded-hal-async` into its own dependency list and re-implement the driver's SPI framing to read a configuration register.

**Files:**
- Modify: `src/driver.rs` (add `ChipConfig` near `Event`; add methods before `fifo_status`)
- Modify: `src/lib.rs` (re-exports)
- Modify: `tests/driver.rs` (new tests)

**Interfaces:**
- Consumes: `Bus::read_sfr32_pair` (Task 2).
- Produces:
  - `MCP251xFd::read_register_raw(&mut self, address: u16) -> Result<u32, Error<SPI::Error>>`
  - `MCP251xFd::write_register_raw(&mut self, address: u16, value: u32) -> Result<(), Error<SPI::Error>>`
  - `MCP251xFd::control_register(&mut self) -> Result<CiCon, Error<SPI::Error>>`
  - `MCP251xFd::fifo_config(&mut self, fifo: Fifo) -> Result<CiFifoCon, Error<SPI::Error>>`
  - `MCP251xFd::fifo_user_address(&mut self, fifo: Fifo) -> Result<u32, Error<SPI::Error>>`
  - `MCP251xFd::read_back_config(&mut self) -> Result<ChipConfig, Error<SPI::Error>>`
  - `pub struct ChipConfig { pub con: CiCon, pub nominal: CiNbtCfg, pub data: CiDbtCfg, pub tdc: CiTdc }`
  - crate-root re-exports of `ChipConfig`, `CiCon`, `CiFifoCon`, `CiNbtCfg`, `CiDbtCfg`, `CiTdc`

- [ ] **Step 1: Write the failing tests**

Add to `tests/driver.rs`:

```rust
#[test]
fn read_register_raw_reads_any_address() {
    let mut e = Vec::new();
    e.extend(r32(0x000, 0x0480_0020));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert_eq!(can.read_register_raw(0x000).unwrap(), 0x0480_0020);
    spi.done();
}

#[test]
fn write_register_raw_writes_any_address() {
    let mut e = Vec::new();
    e.extend(w32(0x1F0, 0xDEAD_BEEF));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.write_register_raw(0x1F0, 0xDEAD_BEEF).unwrap();
    spi.done();
}

#[test]
fn control_register_decodes_op_mode() {
    let mut e = Vec::new();
    // OPMOD = 7 (Restricted Operation) in bits 23:21.
    e.extend(r32(0x000, 0x00E0_0000));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert_eq!(
        can.control_register().unwrap().op_mode(),
        OperationMode::RestrictedOperation
    );
    spi.done();
}

#[test]
fn fifo_config_reads_back_txreq() {
    let mut e = Vec::new();
    // CiFIFOCON1 at 0x05C: TXEN | TXREQ still pending.
    e.extend(r32(0x05C, (1 << 7) | (1 << 9)));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    let con = can.fifo_config(Fifo::F1).unwrap();
    assert!(con.tx());
    assert!(con.txreq(), "a TX FIFO with frames still queued");
    spi.done();
}

#[test]
fn fifo_user_address_reads_the_ua_register() {
    let mut e = Vec::new();
    e.extend(r32(0x064, 0x0000_0030));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert_eq!(can.fifo_user_address(Fifo::F1).unwrap(), 0x30);
    spi.done();
}

#[test]
fn read_back_config_fetches_all_four_timing_registers() {
    // C1CON/C1NBTCFG and C1DBTCFG/C1TDC are two adjacent pairs, so this
    // costs two transactions rather than four.
    let mut e = Vec::new();
    e.extend(r32_pair(0x000, 0x0480_0020, 0x003E_0F0F));
    e.extend(r32_pair(0x008, 0x000E_0303, 0x0202_0F00));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    let cfg = can.read_back_config().unwrap();
    assert_eq!(cfg.con.op_mode(), OperationMode::Configuration);
    assert!(cfg.con.iso_crc_enabled());
    assert_eq!(cfg.nominal.0, 0x003E_0F0F);
    assert_eq!(cfg.data.0, 0x000E_0303);
    assert_eq!(cfg.tdc.0, 0x0202_0F00);
    spi.done();
}
```

Extend the `use mcp251xfd::{...}` list at the top of `tests/driver.rs` with `ChipConfig` is not needed (the value is only used through its fields), but you **do** need nothing new — `OperationMode` and `Fifo` are already imported. Verify by compiling.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test --test driver`
Expected: FAIL — `no method named 'read_register_raw'`.

- [ ] **Step 3: Implement**

Add `CiNbtCfg`, `CiDbtCfg` to the `use crate::registers::{...}` list at the top of `src/driver.rs` (`CiCon`, `CiFifoCon`, `CiTdc` are already there).

Add the struct immediately after the `Event` enum's `impl` block in `src/driver.rs`:

```rust
/// A snapshot of the chip's configuration registers, for diffing what the
/// driver asked for against what the chip actually holds.
///
/// Returned by [`MCP251xFd::read_back_config`]. Since [`MCP251xFd::init`]
/// builds `CiCON` and the bit-timing registers from its own [`Config`],
/// this is the only way to confirm the chip agrees with that intent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ChipConfig {
    /// `CiCON` — operation mode, ISO CRC, retransmission and TEF policy.
    pub con: CiCon,
    /// `CiNBTCFG` — nominal bit timing.
    pub nominal: CiNbtCfg,
    /// `CiDBTCFG` — data bit timing (meaningful only in CAN FD modes).
    pub data: CiDbtCfg,
    /// `CiTDC` — transmitter delay compensation.
    pub tdc: CiTdc,
}
```

Add the methods inside the `maybe_async_cfg` impl block, immediately before `fifo_status`:

```rust
    /// Reads one 32-bit register straight off the chip.
    ///
    /// Diagnostic escape hatch. The driver does not interpret the result and
    /// keeps no record of it. `address` is a 12-bit SPI address; the named
    /// constants live in [`registers::addr`](crate::registers::addr).
    ///
    /// This is deliberately not behind a feature flag: a bench operator needs
    /// it on the build that is already flashed.
    pub async fn read_register_raw(&mut self, address: u16) -> Result<u32, Error<SPI::Error>> {
        self.bus.read_sfr32(address).await
    }

    /// Writes one 32-bit register straight to the chip.
    ///
    /// Diagnostic escape hatch, and a sharp one.
    ///
    /// **Writing a configuration register through this can desynchronise the
    /// driver from the chip.** The driver tracks the TX sequence counter and
    /// the variant's sequence mask internally, and assumes it is the only
    /// writer of `CiCON`, the FIFO control registers and the filter
    /// registers. Changing those behind its back is not detected.
    ///
    /// Two addresses are actively unsafe to write this way:
    ///
    /// - `IOCON` (0xE04) must be written one byte at a time. A multi-byte
    ///   write spanning bytes 2-3 clears `LAT0`/`LAT1`
    ///   (DS80000792D item 6 / DS80000789F item 5). This method always writes
    ///   four bytes.
    /// - `CiFIFOCON` byte 1 carries the write-only `UINC`, `TXREQ` and
    ///   `FRESET` strobes. Use [`Self::transmit`] and [`Self::reset_fifo`]
    ///   instead of assembling them by hand.
    pub async fn write_register_raw(
        &mut self,
        address: u16,
        value: u32,
    ) -> Result<(), Error<SPI::Error>> {
        self.bus.write_sfr32(address, value).await
    }

    /// Reads `CiCON`, whose `OPMOD` field is the chip's current operation
    /// mode.
    ///
    /// The driver keeps no record of the mode it last requested, and the chip
    /// can leave a mode on its own — a system error drops it into Restricted
    /// Operation or Listen Only (see [`Self::recover_system_error`]). This is
    /// how to find out where it actually is.
    pub async fn control_register(&mut self) -> Result<CiCon, Error<SPI::Error>> {
        Ok(CiCon(self.bus.read_sfr32(addr::C1CON).await?))
    }

    /// Reads a FIFO's control register, i.e. the configuration
    /// [`Self::apply_layout`] wrote plus the live `TXREQ` strobe.
    ///
    /// `TXREQ` is the useful bit at runtime: the chip sets it when frames are
    /// queued and clears it once the FIFO drains, so it distinguishes "frames
    /// are still pending" from "the FIFO is idle" — which
    /// [`Self::fifo_status`]'s not-full flag does not.
    pub async fn fifo_config(&mut self, fifo: Fifo) -> Result<CiFifoCon, Error<SPI::Error>> {
        Ok(CiFifoCon(self.bus.read_sfr32(addr::fifo_con(fifo)).await?))
    }

    /// Reads a FIFO's user address (`CiFIFOUA`): the message RAM offset of
    /// the next element the host should write or read.
    ///
    /// Not meaningful in Configuration mode (DS20006027B Register 3-31
    /// Note 1).
    pub async fn fifo_user_address(&mut self, fifo: Fifo) -> Result<u32, Error<SPI::Error>> {
        self.bus.read_sfr32(addr::fifo_ua(fifo)).await
    }

    /// Reads back the configuration registers [`Self::init`] wrote, so the
    /// [`Config`] that was asked for can be diffed against what the chip
    /// holds.
    ///
    /// `C1CON`/`C1NBTCFG` and `C1DBTCFG`/`C1TDC` are adjacent pairs, so this
    /// costs two SPI transactions, not four.
    pub async fn read_back_config(&mut self) -> Result<ChipConfig, Error<SPI::Error>> {
        let (con, nominal) = self.bus.read_sfr32_pair(addr::C1CON).await?;
        let (data, tdc) = self.bus.read_sfr32_pair(addr::C1DBTCFG).await?;
        Ok(ChipConfig {
            con: CiCon(con),
            nominal: CiNbtCfg(nominal),
            data: CiDbtCfg(data),
            tdc: CiTdc(tdc),
        })
    }
```

In `src/lib.rs`, extend the re-exports:

```rust
pub use driver::ChipConfig;
pub use driver::Event;
pub use driver::MCP251xFd;
```

and extend the registers re-export line so `ChipConfig`'s field types can be named by consumers:

```rust
pub use registers::{CiCon, CiDbtCfg, CiFifoCon, CiFifoSta, CiInt, CiNbtCfg, CiTdc, CiTrec};
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test --test driver`
Expected: PASS.

- [ ] **Step 5: Full verification**

Run: `cargo test && cargo test --features async && cargo clippy --features async --all-targets -- -D warnings && cargo fmt --check && cargo doc --features async --no-deps`
Expected: all pass, 83 tests. `cargo doc` must emit no `missing_docs` errors for the new `ChipConfig` fields.

- [ ] **Step 6: Commit**

```bash
git add src/driver.rs src/lib.rs tests/driver.rs
git commit -m "feat(driver): add raw and typed register read-back

Adds read_register_raw/write_register_raw as documented diagnostic escape
hatches, plus typed accessors for CiCON, CiFIFOCON and CiFIFOUA, and
read_back_config returning a ChipConfig snapshot of the four registers
init writes.

Without these a consumer cannot check whether the chip agrees with the
driver's intent, and one downstream project ended up re-implementing the
driver's SPI framing to do it. Deliberately not feature-gated: a bench
operator needs them on the build already flashed.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: `reset_fifo`

The primitive the reported stall recovery actually needs: assert `FRESET` on one FIFO without the Configuration-mode cycle and without rewriting layout and filters.

**Files:**
- Modify: `src/driver.rs` (add after `clear_rx_overflow`)
- Modify: `tests/driver.rs`

**Interfaces:**
- Consumes: `CiFifoCon::CON_BYTE1_FRESET` (Task 1).
- Produces: `MCP251xFd::reset_fifo(&mut self, fifo: Fifo) -> Result<(), Error<SPI::Error>>`

- [ ] **Step 1: Write the failing test**

Add to `tests/driver.rs`:

```rust
#[test]
fn reset_fifo_writes_only_the_freset_strobe() {
    let mut e = Vec::new();
    // CiFIFOCON1 byte 1 (0x05D) = FRESET, leaving UINC and TXREQ clear and
    // every configuration byte untouched.
    e.extend(w8(0x05D, 0x04));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.reset_fifo(Fifo::F1).unwrap();
    spi.done();
}

#[test]
fn reset_fifo_addresses_the_right_fifo() {
    let mut e = Vec::new();
    e.extend(w8(0x069, 0x04)); // CiFIFOCON2 byte 1
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.reset_fifo(Fifo::F2).unwrap();
    spi.done();
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test --test driver reset_fifo`
Expected: FAIL — `no method named 'reset_fifo'`.

- [ ] **Step 3: Implement**

Add to `src/driver.rs` immediately after `clear_rx_overflow`:

```rust
    /// Resets one FIFO by asserting `FRESET`: its head and tail pointers and
    /// its `CiFIFOSTA` register are cleared, discarding whatever was queued.
    ///
    /// Per DS20005678E section 4.14 the `CiFIFOCONm` configuration bits are
    /// left unchanged and the strobe self-clears when the reset completes, so
    /// this is a single SPI transaction and does **not** require
    /// Configuration mode. That makes it the cheap way to clear one wedged
    /// FIFO — [`Self::apply_layout`] also asserts `FRESET`, but only as a
    /// side effect of rewriting every FIFO's configuration, and it requires
    /// Configuration mode.
    ///
    /// The same section requires that no transmissions are pending when a TX
    /// FIFO is reset this way. Frames already handed to the chip are lost:
    /// check [`Self::fifo_config`]'s `txreq` first if that matters, or abort
    /// them deliberately. After a system error the chip has stopped
    /// transmitting anyway — see [`Self::recover_system_error`], which is the
    /// right tool for that case and does not discard queued frames.
    pub async fn reset_fifo(&mut self, fifo: Fifo) -> Result<(), Error<SPI::Error>> {
        self.bus
            .write_sfr8(addr::fifo_con(fifo) + 1, CiFifoCon::CON_BYTE1_FRESET)
            .await
    }
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test --test driver reset_fifo`
Expected: PASS.

- [ ] **Step 5: Full verification**

Run: `cargo test && cargo test --features async && cargo clippy --features async --all-targets -- -D warnings && cargo fmt --check`
Expected: all pass, 85 tests.

- [ ] **Step 6: Commit**

```bash
git add src/driver.rs tests/driver.rs
git commit -m "feat(driver): add reset_fifo for single-FIFO FRESET

One SPI transaction, no Configuration mode, no rewrite of layout or
filters. Per DS20005678E section 4.14 FRESET clears the FIFO pointers and
status while leaving CiFIFOCON's configuration bits alone.

A downstream project was routing this through apply_layout, relying on an
undocumented FRESET side effect to recover a wedged TX FIFO.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 6: `recover_system_error`

The bounded recovery for the reported stall. Replaces a ladder the reporter measured at 22,000 us — eleven times their 2 ms budget — with a handful of transactions.

**Files:**
- Modify: `src/driver.rs` (add after `error_counters`)
- Modify: `tests/driver.rs`

**Interfaces:**
- Consumes: `CiCon::op_mode` (existing), `Self::clear_interrupts` (existing), `Self::set_mode` (existing).
- Produces: `MCP251xFd::recover_system_error<D: DelayNs>(&mut self, mode: OperationMode, delay: &mut D) -> Result<bool, Error<SPI::Error>>`

- [ ] **Step 1: Write the failing tests**

Add to `tests/driver.rs`:

```rust
#[test]
fn recover_system_error_restores_normal_mode_from_restricted() {
    let mut e = Vec::new();
    // CiCON reports OPMOD = 7 (Restricted Operation) after a TX MAB underflow.
    e.extend(r32(0x000, 0x00E0_0020));
    // Clear SERRIF (12), MODIF (3) and IVMIF (15), write-0-to-clear, flag
    // half only: byte 0 = !0x08 = 0xF7, byte 1 = !0x90 = 0x6F.
    e.extend(w8(0x01C, 0xF7));
    e.extend(w8(0x01D, 0x6F));
    // set_mode: read-modify-write REQOP, then poll OPMOD.
    e.extend(r32(0x000, 0x00E0_0020));
    e.extend(w32(0x000, 0x0600_0020)); // REQOP = 6 (Normal20)
    e.extend(r32(0x000, 0x06C0_0020)); // OPMOD = 6, reached
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(
        can.recover_system_error(OperationMode::Normal20, &mut NoopDelay)
            .unwrap()
    );
    spi.done();
}

#[test]
fn recover_system_error_is_a_no_op_when_the_mode_is_healthy() {
    let mut e = Vec::new();
    // OPMOD = 6 (Normal20): nothing to recover, and nothing must be written.
    e.extend(r32(0x000, 0x00C0_0020));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(
        !can.recover_system_error(OperationMode::Normal20, &mut NoopDelay)
            .unwrap()
    );
    spi.done();
}

#[test]
fn recover_system_error_also_handles_the_listen_only_fallback() {
    // With CiCON.SERR2LOM set the chip drops into Listen Only instead.
    let mut e = Vec::new();
    e.extend(r32(0x000, 0x0060_0020)); // OPMOD = 3 (ListenOnly)
    e.extend(w8(0x01C, 0xF7));
    e.extend(w8(0x01D, 0x6F));
    e.extend(r32(0x000, 0x0060_0020));
    e.extend(w32(0x000, 0x0600_0020));
    e.extend(r32(0x000, 0x06C0_0020));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(
        can.recover_system_error(OperationMode::Normal20, &mut NoopDelay)
            .unwrap()
    );
    spi.done();
}
```

`NoopDelay` is already imported in `tests/driver.rs`. Before running, confirm the exact byte sequence `set_mode` emits by reading `src/driver.rs`'s `set_mode` and the existing `set_mode_normal_fd` test — if `set_mode` reads `CiCON` before writing, the expectations above already match; if the observed order differs, correct the expectations to match the real implementation rather than changing `set_mode`.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test --test driver recover_system_error`
Expected: FAIL — `no method named 'recover_system_error'`.

- [ ] **Step 3: Implement**

Add to `src/driver.rs` immediately after `error_counters`:

```rust
    /// Recovers from a system error that parked the chip in Restricted
    /// Operation or Listen Only mode, returning whether it needed to.
    ///
    /// Reads `CiCON`. If `OPMOD` is neither
    /// [`OperationMode::RestrictedOperation`] nor
    /// [`OperationMode::ListenOnly`], returns `Ok(false)` having issued no
    /// writes. Otherwise it clears `SERRIF`, `MODIF` and `IVMIF`, requests
    /// `mode`, and returns `Ok(true)`.
    ///
    /// Flags are cleared *before* the mode request so a second system error
    /// during recovery latches a fresh `SERRIF` instead of hiding behind the
    /// stale one.
    ///
    /// # Why this exists
    ///
    /// On the MCP2517FD a TX MAB underflow (DS80000792D item 1) parks the
    /// chip here. Both modes ignore `TXREQ`, so the TX FIFO fills and stops
    /// draining while `CiTREC` stays completely clean — no `TEC`, no `REC`,
    /// no bus-off — which makes it look nothing like a bus fault. Clearing
    /// the interrupt flags does not help: the flags are not what is wrong,
    /// the operation mode is. The erratum's own recovery is to request Normal
    /// mode, after which the chip retransmits the offending message by
    /// itself, and it states that resetting the device is not necessary.
    ///
    /// `mode` is explicit rather than remembered, because the driver keeps no
    /// record of the mode it last requested. Pass the Normal mode the
    /// application was running in. Per DS20005678E Figure 2-1 the transition
    /// out of Restricted Operation and Listen Only is a direct edge to the
    /// Normal modes, so no Configuration-mode round trip is needed and the
    /// cost is a few SPI transactions plus bus re-integration (11 consecutive
    /// recessive bits).
    ///
    /// Queued frames are preserved. Use [`Self::reset_fifo`] instead only
    /// when the queue contents should be discarded.
    pub async fn recover_system_error<D: DelayNs>(
        &mut self,
        mode: OperationMode,
        delay: &mut D,
    ) -> Result<bool, Error<SPI::Error>> {
        let con = CiCon(self.bus.read_sfr32(addr::C1CON).await?);
        if !matches!(
            con.op_mode(),
            OperationMode::RestrictedOperation | OperationMode::ListenOnly
        ) {
            return Ok(false);
        }
        self.clear_interrupts(
            CiInt(0)
                .with_serrif(true)
                .with_modif(true)
                .with_ivmif(true),
        )
        .await?;
        self.set_mode(mode, delay).await?;
        Ok(true)
    }
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test --test driver recover_system_error`
Expected: PASS.

- [ ] **Step 5: Full verification**

Run: `cargo test && cargo test --features async && cargo clippy --features async --all-targets -- -D warnings && cargo fmt --check`
Expected: all pass, 88 tests.

- [ ] **Step 6: Commit**

```bash
git add src/driver.rs tests/driver.rs
git commit -m "feat(driver): add recover_system_error for the TX MAB underflow stall

A TX MAB underflow (MCP2517FD errata DS80000792D item 1) parks the chip in
Restricted Operation or Listen Only, where TXREQ is ignored: the TX FIFO
fills and stops draining while CiTREC stays clean. Clearing the interrupt
flags cannot fix it because the operation mode is what changed.

recover_system_error reads CiCON, and if the mode is one of those two,
clears SERRIF/MODIF/IVMIF and re-requests the caller's Normal mode. Per
the erratum the chip then retransmits the offending message itself and no
reset is needed. Figure 2-1 makes this a direct edge, so there is no
Configuration-mode round trip.

A downstream project was recovering this with a full mode cycle plus
layout and filter rewrite, measured at 22,000 us against a 2,000 us
budget, and had to keep it out of production as a result.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 7: `transmit_batch`

**Files:**
- Modify: `src/driver.rs` (add after `transmit`)
- Modify: `tests/driver.rs`

**Interfaces:**
- Consumes: `Self::transmit` (existing).
- Produces: `MCP251xFd::transmit_batch(&mut self, fifo: Fifo, frames: &[Frame]) -> Result<u8, Error<SPI::Error>>`

Be honest in the docs about what this does and does not save. After Task 3 the readiness check shares a transaction with the user-address read, so a batch costs exactly the same SPI transactions as the same number of `transmit` calls. Its value is the explicit accepted-count contract, not a transaction saving. Do not claim otherwise.

- [ ] **Step 1: Write the failing tests**

Add to `tests/driver.rs`:

```rust
#[test]
fn transmit_batch_queues_every_frame_and_counts_them() {
    let frame = Frame::new(StandardId::new(0x123).unwrap(), &[1, 2, 3, 4]).unwrap();
    let body = [
        0x23, 0x01, 0x00, 0x00, // T0: SID 0x123
        0x04, 0x00, 0x00, 0x00, // T1: DLC 4, SEQ 0
        0x01, 0x02, 0x03, 0x04,
    ];
    let mut e = Vec::new();
    e.extend(r32_pair(0x060, 0x0000_0001, 0x0000_0000));
    e.extend(wram(0x400, &body));
    e.extend(w8(0x05D, 0x03));
    e.extend(r32_pair(0x060, 0x0000_0001, 0x0000_0010));
    let mut second = body;
    second[5] = 0x02; // SEQ 1
    e.extend(wram(0x410, &second));
    e.extend(w8(0x05D, 0x03));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert_eq!(
        can.transmit_batch(Fifo::F1, &[frame, frame]).unwrap(),
        2
    );
    spi.done();
}

#[test]
fn transmit_batch_reports_the_accepted_prefix_when_the_fifo_fills() {
    let frame = Frame::new(StandardId::new(0x123).unwrap(), &[]).unwrap();
    let body = [0x23, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
    let mut e = Vec::new();
    e.extend(r32_pair(0x060, 0x0000_0001, 0x0000_0000));
    e.extend(wram(0x400, &body));
    e.extend(w8(0x05D, 0x03));
    // Second attempt: FIFO now full, so the batch stops here.
    e.extend(r32_pair(0x060, 0x0000_0000, 0x0000_0010));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert_eq!(
        can.transmit_batch(Fifo::F1, &[frame, frame, frame]).unwrap(),
        1,
        "only the first frame was accepted"
    );
    spi.done();
}

#[test]
fn transmit_batch_of_nothing_is_a_no_op() {
    let mut spi = Mock::new(&[]);
    let mut can = MCP251xFd::new(&mut spi);
    assert_eq!(can.transmit_batch(Fifo::F1, &[]).unwrap(), 0);
    spi.done();
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test --test driver transmit_batch`
Expected: FAIL — `no method named 'transmit_batch'`.

- [ ] **Step 3: Implement**

Add to `src/driver.rs` immediately after `transmit`:

```rust
    /// Queues several classic frames on a transmit FIFO, returning how many
    /// were accepted.
    ///
    /// Frames are queued in order and the first refusal ends the batch, so
    /// the return value is the length of the accepted prefix — frames are
    /// never reordered or skipped. A result below `frames.len()` means the
    /// FIFO filled; the caller retries the remainder once a slot frees up.
    /// Any error other than a full FIFO propagates and the count is lost, so
    /// the frames already queued in that call have still been handed to the
    /// chip.
    ///
    /// This costs exactly the same SPI transactions as calling
    /// [`Self::transmit`] in a loop: the readiness check already shares a
    /// transaction with the user-address read, so there is nothing further to
    /// fold. What it adds is the accepted-count contract, which makes partial
    /// success explicit instead of leaving the caller to match on
    /// [`Error::TxFifoFull`] mid-loop and work out how far it got.
    ///
    /// At most 255 frames are queued; any beyond that are ignored, which no
    /// FIFO can hold anyway (the deepest is 32 elements).
    pub async fn transmit_batch(
        &mut self,
        fifo: Fifo,
        frames: &[Frame],
    ) -> Result<u8, Error<SPI::Error>> {
        let mut accepted: u8 = 0;
        for frame in frames.iter().take(u8::MAX as usize) {
            match self.transmit(fifo, frame).await {
                Ok(()) => accepted += 1,
                Err(Error::TxFifoFull) => return Ok(accepted),
                Err(e) => return Err(e),
            }
        }
        Ok(accepted)
    }
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test --test driver transmit_batch`
Expected: PASS.

- [ ] **Step 5: Full verification**

Run: `cargo test && cargo test --features async && cargo clippy --features async --all-targets -- -D warnings && cargo fmt --check`
Expected: all pass, 91 tests.

- [ ] **Step 6: Commit**

```bash
git add src/driver.rs tests/driver.rs
git commit -m "feat(driver): add transmit_batch with an accepted-frame count

Queues frames in order, stopping at the first refusal, and returns the
length of the accepted prefix so partial success is explicit.

This costs the same transactions as a transmit loop -- after the paired
status/user-address read there is nothing left to fold -- so the docs say
so rather than claiming a saving.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 8: Documentation

No behaviour change. This is where the report's items 2, 4 and 6 land, plus the `apply_layout` side effect from item 3.

**Files:**
- Modify: `src/config.rs` (`max_spi_hz` doc, ~line 8)
- Modify: `src/driver.rs` (`apply_layout`, `transmit`, `transmit_fd`, `fifo_status` docs)
- Modify: `src/registers/mod.rs` (`CiFifoSta::not_full_or_not_empty` doc)
- Modify: `src/lib.rs` (three new crate-level sections)
- Modify: `README.md`
- Modify: `Cargo.toml` (exclude `docs/`)

**Interfaces:**
- Consumes: everything from Tasks 1-7 — the docs cross-reference `reset_fifo`, `recover_system_error`, `fifo_config` and `ChipConfig` by name. Those names must already exist or `cargo doc` intra-doc links fail.
- Produces: no code.

- [ ] **Step 1: Update `max_spi_hz`**

In `src/config.rs`, replace the doc comment on `max_spi_hz` with:

```rust
/// The maximum safe SPI clock for a given SYSCLK: `0.85 * SYSCLK / 2`.
///
/// This is a **correctness limit from silicon errata**, not a conservative
/// guess. Every supported variant carries the anomaly, in mirrored forms:
///
/// - MCP2517FD, DS80000792D item 5: "The SPI may read corrupted data from
///   the RAM at fast SPI speeds."
/// - MCP2518FD, DS80000789F item 4: "The SPI may write corrupted data to the
///   RAM at fast SPI speeds." The MCP251863 uses the same die.
///
/// Both name the same fix: keep FSCK at or below `0.85 * (FSYSCLK/2)`. Both
/// require simultaneous CAN bus activity to trigger, which is why the
/// init-time RAM echo test (Configuration mode, no bus traffic) proves wiring
/// and byte order but cannot confirm erratum compliance at your clock.
///
/// # Host HALs quantise downward
///
/// This returns a ceiling; your HAL picks the nearest achievable rate at or
/// below it, so a scope on SCK will usually read *lower* than this number and
/// that is correct. On an RP2040 the same 8.5 MHz ceiling (a 20 MHz SYSCLK)
/// becomes 7.5 MHz at a 120 MHz `clk_peri` and 7.8125 MHz at 125 MHz.
///
/// The ceiling is real: on a ten-chip MCP2517FD board it was measured clean
/// at every rate up to 12.5 MHz and corrupting at 15.625 MHz.
///
/// The driver cannot observe your actual SPI clock through `SpiDevice`, so
/// sizing the bus is yours to do. Derive it from the same [`ClockConfig`] the
/// chip uses and the two cannot drift:
///
/// ```
/// # use mcp251xfd::{ClockConfig, max_spi_hz};
/// let sysclk = ClockConfig::MHZ20.sysclk_hz();
/// assert_eq!(max_spi_hz(sysclk), 8_500_000);
/// ```
pub const fn max_spi_hz(sysclk_hz: u32) -> u32 {
```

- [ ] **Step 2: Document `apply_layout`'s `FRESET` side effect**

In `src/driver.rs`, append to `apply_layout`'s doc comment, before `pub async fn`:

```rust
    /// # `FRESET` side effect
    ///
    /// Every FIFO this writes is configured with `FRESET` asserted, so its
    /// head and tail pointers and its `CiFIFOSTA` register are reset and
    /// anything queued in it is discarded. **This is intentional and may be
    /// relied on**: applying a layout to a FIFO that is mid-stream and
    /// leaving its pointers where they were would leave the chip's idea of
    /// the FIFO and the driver's disagreeing.
    ///
    /// It is, however, a blunt instrument for resetting a FIFO: it requires
    /// Configuration mode and rewrites every FIFO's configuration.
    /// [`Self::reset_fifo`] asserts `FRESET` on one FIFO in a single
    /// transaction, in any mode, without touching configuration — prefer it
    /// when a reset is all you want.
```

- [ ] **Step 3: Warn that FIFO room is not a health signal**

Append to `transmit`'s doc comment in `src/driver.rs`:

```rust
    /// # A free slot is not a health signal
    ///
    /// [`Error::TxFifoFull`] means the FIFO is full; its absence means only
    /// that a slot was free, **not** that anything is reaching the bus. A
    /// controller can sit with free FIFO space and drain nothing — see the
    /// crate-level "Known hardware anomalies" section, where exactly that
    /// costs a stalled MCP2517FD its entire transmit path while `CiTREC`
    /// still reads clean.
    ///
    /// To tell queued-and-moving from queued-and-wedged, read
    /// [`Self::fifo_config`]'s `txreq` — the chip clears it when the FIFO
    /// drains — or [`Self::fifo_status`]'s `tx_empty_or_rx_full`. Neither
    /// costs a frame on the bus.
```

Append a shorter cross-reference to `transmit_fd`:

```rust
    /// A free slot is not a health signal; see [`Self::transmit`].
```

Replace `fifo_status`'s doc comment with:

```rust
    /// Reads a FIFO's status register.
    ///
    /// For a TX FIFO, `not_full_or_not_empty` reports only that a slot is
    /// free. It does **not** report that frames are reaching the bus: a
    /// stalled controller shows free space while draining nothing. Use
    /// `tx_empty_or_rx_full` (the FIFO actually drained) or
    /// [`Self::fifo_config`]'s `txreq` (frames still pending) for that, and
    /// see the crate-level "Known hardware anomalies" section.
```

In `src/registers/mod.rs`, the `bit!` macro generates `not_full_or_not_empty`'s doc from its string argument, so extend that argument:

```rust
    bit!(
        not_full_or_not_empty,
        with_not_full_or_not_empty,
        0,
        "`TFNRFNIF` (TX: not full / RX: not empty). For a TX FIFO this is a \
         capacity flag, not a health flag: a stalled controller reports free \
         space while draining nothing"
    );
```

- [ ] **Step 4: Add the three crate-level sections**

In `src/lib.rs`, insert after the existing "SPI clock limit (silicon erratum)" section and before the "Example" section:

```rust
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
//! For a dedicated core the blocking driver is simply better: it removes the
//! cross-core interrupt, frees two DMA channels, and for the 3-18 byte
//! transfers this driver issues, DMA setup overhead dominates the transfer
//! anyway. If the core has slack against its deadline, busy-waiting on the
//! SPI FIFO costs it nothing it needs.
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
```

- [ ] **Step 5: Mirror the essentials in `README.md`**

Insert both sections directly after the existing `## SPI clock limit (silicon erratum)` section. The README is an index, not a duplicate — these are deliberately shorter than the rustdoc and point at it.

````markdown
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
````

- [ ] **Step 6: Exclude `docs/` from the published crate**

In `Cargo.toml`:

```toml
exclude = [".github/", "docs/"]
```

The adjacent comment already says internal design artefacts are not part of the crate; `docs/` was simply never listed.

- [ ] **Step 7: Verify**

Run: `cargo test && cargo test --features async && cargo doc --features async --no-deps && cargo clippy --features async --all-targets -- -D warnings && cargo fmt --check`
Expected: all pass. `cargo doc` must report no broken intra-doc links — every `[`Self::...`]` used above refers to a method added in Tasks 4-7.

Also run `cargo package --list --allow-dirty | grep -c docs/` and expect `0`.

- [ ] **Step 8: Commit**

```bash
git add src/lib.rs src/config.rs src/driver.rs src/registers/mod.rs README.md Cargo.toml
git commit -m "docs: erratum ceiling, blocking-vs-async, and the TX MAB stall

- max_spi_hz names the errata items it implements (DS80000792D item 5 for
  corrupted reads, DS80000789F item 4 for corrupted writes) and warns that
  host HALs quantise downward, so a scope reading below the cap is correct.
- apply_layout documents that it asserts FRESET, that this is intentional,
  and that reset_fifo is the cheaper primitive.
- transmit and fifo_status warn that a free FIFO slot is a capacity signal,
  not a health signal.
- New crate-level sections: the mode (0,0) requirement, when to prefer the
  blocking driver (naming the RP2040 cross-core DMA case), and the
  MCP2517FD TX MAB underflow signature with its recovery.
- docs/ joins the package exclude list; only .github/ was listed.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 9: Blocking board setup and the register-dump example

**Files:**
- Modify: `examples/rp2040/src/common.rs` (additions only)
- Create: `examples/rp2040/src/bin/regdump.rs`

**Interfaces:**
- Consumes: `read_back_config`, `control_register`, `fifo_config`, `fifo_user_address`, `read_register_raw` (Task 4).
- Produces, in `common.rs`:
  - `pub type BlockingBus = blocking_mutex::Mutex<CriticalSectionRawMutex, RefCell<Spi<'static, SPI1, Blocking>>>`
  - `pub type BlockingDevice = BlockingSpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI1, Blocking>, Output<'static>>`
  - `pub type BlockingCanError = mcp251xfd::Error<SpiDeviceError<embassy_rp::spi::Error, core::convert::Infallible>>`
  - `pub fn init_board_blocking() -> ([BlockingDevice; 10], UsbDriver, embassy_rp::peripherals::CORE1)`

`CriticalSectionRawMutex`, not `NoopRawMutex`: Task 11 moves the devices to core 1, which requires `Send`, and `NoopRawMutex` is deliberately not `Sync`. The `critical-section-impl` feature is already enabled on `embassy-rp`.

- [ ] **Step 1: Add the blocking board setup**

Append to `examples/rp2040/src/common.rs` — **add only, change nothing that is already there**:

```rust
use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice as BlockingSpiDevice;
use embassy_rp::peripherals::CORE1;
use embassy_rp::spi::Blocking;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;

/// The blocking counterpart of [`Bus`].
///
/// Guarded by a critical-section mutex rather than [`NoopRawMutex`] because
/// `blocking_core1` moves the devices to the second core, which requires
/// them to be `Send`.
pub type BlockingBus = BlockingMutex<CriticalSectionRawMutex, RefCell<Spi<'static, SPI1, Blocking>>>;

/// The blocking counterpart of [`Device`].
pub type BlockingDevice =
    BlockingSpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI1, Blocking>, Output<'static>>;

/// The concrete error type a driver call on a [`BlockingDevice`] returns.
#[allow(dead_code)]
pub type BlockingCanError =
    mcp251xfd::Error<SpiDeviceError<embassy_rp::spi::Error, core::convert::Infallible>>;

static BLOCKING_SPI_BUS: StaticCell<BlockingBus> = StaticCell::new();

/// Brings up the board with a **blocking** SPI1, same pins and same clock as
/// [`init_board`], and hands back `CORE1` so the caller can start the second
/// core.
///
/// `Spi::new_blocking` takes no DMA channels, so this also frees DMA_CH0 and
/// DMA_CH1 and raises no DMA completion interrupt at all — which is the whole
/// point on a dedicated core. See the driver's "Choosing blocking or async"
/// docs.
///
/// Call this *or* [`init_board`], never both: each calls `embassy_rp::init`.
#[allow(dead_code)]
pub fn init_board_blocking() -> ([BlockingDevice; 10], UsbDriver, CORE1) {
    let p = embassy_rp::init(Default::default());

    let mut cfg = SpiConfig::default();
    cfg.frequency = mcp251xfd::max_spi_hz(CAN_CONFIG.clock.sysclk_hz());
    cfg.phase = Phase::CaptureOnFirstTransition;
    cfg.polarity = Polarity::IdleLow;

    let spi = Spi::new_blocking(p.SPI1, p.PIN_10, p.PIN_11, p.PIN_12, cfg);
    let bus: &'static BlockingBus = BLOCKING_SPI_BUS.init(BlockingMutex::new(RefCell::new(spi)));
    let cs: [AnyPin; 10] = [
        p.PIN_3.degrade(),
        p.PIN_4.degrade(),
        p.PIN_5.degrade(),
        p.PIN_6.degrade(),
        p.PIN_7.degrade(),
        p.PIN_8.degrade(),
        p.PIN_9.degrade(),
        p.PIN_13.degrade(),
        p.PIN_14.degrade(),
        p.PIN_15.degrade(),
    ];
    let devices = cs.map(|pin| BlockingSpiDevice::new(bus, Output::new(pin, Level::High)));

    (devices, Driver::new(p.USB, Irqs), p.CORE1)
}
```

If the existing `use` block already imports any of these names, merge rather than duplicating.

- [ ] **Step 2: Verify it compiles before writing the binary**

Run: `cd examples/rp2040 && cargo build --release 2>&1 | tail -20`
Expected: builds. Fix any import or type mismatch now, while the surface is small.

- [ ] **Step 3: Write the register dump binary**

Create `examples/rp2040/src/bin/regdump.rs`:

```rust
//! Dumps every configuration register the driver writes, for all ten chips.
//!
//! The status registers were always readable (`fifo_status`,
//! `interrupt_flags`, `error_counters`); the *configuration* registers were
//! not, so there was no way to check whether a chip agreed with what `init`
//! believed it had written. This dumps both, and diffs the bit-timing
//! registers against the values `CAN_CONFIG` implies.
//!
//! Needs SPI wiring only -- nothing here touches the CAN bus.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use log::{error, info};
use mcp251xfd::registers::addr;
use mcp251xfd::{Fifo, FifoLayout, MCP251xFdAsync, PayloadSize};
use panic_halt as _;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
    .rx_fifo(Fifo::F2, PayloadSize::B64, 8);

type Can = MCP251xFdAsync<common::Device>;

async fn dump(index: usize, can: &mut Can) -> Result<(), common::CanError> {
    let variant = can.init(&common::CAN_CONFIG, &mut Delay).await?;
    can.apply_layout(&LAYOUT).await?;

    let cfg = can.read_back_config().await?;
    info!(
        "chip {index}: {variant:?} CiCON={:#010X} mode={:?} isocrc={} rtxat={}",
        cfg.con.0,
        cfg.con.op_mode(),
        cfg.con.iso_crc_enabled(),
        cfg.con.restrict_retx(),
    );
    info!(
        "chip {index}: NBTCFG={:#010X} DBTCFG={:#010X} TDC={:#010X}",
        cfg.nominal.0, cfg.data.0, cfg.tdc.0
    );

    // What init should have written, derived from the same config the driver
    // was handed -- so a mismatch means the chip disagrees, not that these
    // literals drifted.
    let want_nbt = common::CAN_CONFIG.nominal.to_register().0;
    if cfg.nominal.0 != want_nbt {
        error!(
            "chip {index}: NBTCFG mismatch: chip has {:#010X}, config implies {:#010X}",
            cfg.nominal.0, want_nbt
        );
    }

    for fifo in [Fifo::F1, Fifo::F2] {
        let con = can.fifo_config(fifo).await?;
        let sta = can.fifo_status(fifo).await?;
        let ua = can.fifo_user_address(fifo).await?;
        info!(
            "chip {index} {fifo:?}: CON={:#010X} tx={} txreq={} | STA={:#010X} ready={} empty_or_full={} | UA={:#06X}",
            con.0,
            con.tx(),
            con.txreq(),
            sta.0,
            sta.not_full_or_not_empty(),
            sta.tx_empty_or_rx_full(),
            ua,
        );
    }

    // The raw escape hatch, on a register the typed API does not cover.
    let iocon = can.read_register_raw(addr::IOCON).await?;
    info!("chip {index}: IOCON={iocon:#010X}");

    Ok(())
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut chips: [Can; 10] = devices.map(MCP251xFdAsync::new);
    loop {
        info!("--- register dump ---");
        for (i, can) in chips.iter_mut().enumerate() {
            common::ensure_configuration(can).await;
            if let Err(e) = dump(i, can).await {
                error!("chip {i}: {e:?}");
            }
        }
        Timer::after_secs(5).await;
    }
}
```

**Check before running:** the `to_register()` call assumes `NominalBitTiming` exposes such a method. Confirm with `grep -n "fn to_register\|fn register\|CiNbtCfg::new" src/config.rs`. If the conversion has a different name, use it; if the type offers no public conversion, drop the mismatch check and the `want_nbt` binding rather than adding a method to the library in this task.

- [ ] **Step 4: Build**

Run: `cd examples/rp2040 && cargo build --release --bin regdump 2>&1 | tail -20`
Expected: builds clean.

- [ ] **Step 5: Lint**

Run: `cd examples/rp2040 && cargo clippy --bins -- -D warnings && cargo fmt --check`
Expected: pass.

- [ ] **Step 6: Commit**

```bash
git add examples/rp2040/src/common.rs examples/rp2040/src/bin/regdump.rs
git commit -m "examples: add regdump and blocking board setup

regdump dumps the configuration registers for all ten chips using the new
read-back accessors and diffs the bit timing against what CAN_CONFIG
implies.

common.rs gains init_board_blocking alongside the existing async setup;
no existing function is modified. It uses a critical-section mutex because
blocking_core1 moves the devices across cores.

Not yet run on hardware.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 10: The stall reproduction and recovery-ladder example

The binary that turns the root-cause argument into a measurement. It reproduces the fault, confirms the mode transition the erratum predicts, and times four recovery ladders against each other.

**Files:**
- Create: `examples/rp2040/src/bin/stall.rs`

**Interfaces:**
- Consumes: `control_register`, `fifo_config`, `recover_system_error`, `reset_fifo`, `interrupt_flags`, `clear_interrupts`, `error_counters`.
- Produces: nothing other code depends on.

- [ ] **Step 1: Write the binary**

Create `examples/rp2040/src/bin/stall.rs`:

```rust
//! Reproduces the MCP2517FD transmit stall and times four recovery ladders.
//!
//! # What this is testing
//!
//! DS80000792D item 1: during an SPI READ that accesses message RAM, the SPI
//! interface can block the CAN FSM from reaching RAM -- in the gaps between
//! bytes and between the last byte and nCS rising. Held off longer than
//! T_SPIMAXDLY (5 nominal bit times for a classic base frame, so 10 us at
//! 500 kbit/s), the chip suffers a TX MAB underflow: it sets SERRIF and MODIF
//! and drops into Restricted Operation or Listen Only, where TXREQ is
//! ignored. The TX FIFO fills, nothing drains, and CiTREC stays perfectly
//! clean -- so it looks nothing like a bus fault.
//!
//! Only `receive` issues RAM reads, which is why a transmit-only load does not
//! reproduce this and a receive-then-echo load does.
//!
//! # What it reports
//!
//! Per fault: the latched CiINT flags, CiCON.OPMOD, the TX FIFO's CiFIFOSTA
//! and TXREQ, and CiTREC -- i.e. the full signature, so it can be compared
//! against the table in the driver docs.
//!
//! Then it recovers, rotating through four ladders and timing each:
//!
//! 1. clear the latched flags only        -- expected to never work
//! 2. `recover_system_error`              -- expected to always work, cheaply
//! 3. `reset_fifo` then re-request Normal -- works, but discards queued frames
//! 4. full Configuration-mode cycle       -- works, and is the expensive one
//!
//! Ladder 1 failing while 2 succeeds is the evidence that the operation mode,
//! not the interrupt flags, is what is wrong. Ladders 2 and 4 differing by
//! roughly two orders of magnitude in microseconds is the argument for
//! putting recovery in a production path.
//!
//! # Wiring
//!
//! **This one needs the CAN bus wired**, unlike most binaries here —
//! transceivers and termination, exactly as `multinode` needs. It cannot use
//! internal loopback: DS20005678E Figure 2-1 shows the System Error
//! transition leaving the *Normal* modes, and internal loopback is a *Debug*
//! mode. A loopback run might never reproduce the fault, and would prove
//! nothing either way.
//!
//! `Normal20` also matches the conditions the fault was reported under:
//! classic CAN at 500 kbit/s, where T_SPIMAXDLY is at its tightest (5 nominal
//! bit times, 10 us).
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Instant, Timer};
use embedded_can::{Frame as _, StandardId};
use log::{error, info, warn};
use mcp251xfd::{
    CiInt, Fifo, FifoLayout, Filter, FilterMatch, Frame, MCP251xFdAsync, OperationMode,
    PayloadSize,
};
use panic_halt as _;

const TX: Fifo = Fifo::F1;
const RX: Fifo = Fifo::F2;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(TX, PayloadSize::B8, 8)
    .rx_fifo(RX, PayloadSize::B8, 8);

/// Classic CAN on the real bus. Must be a Normal mode — see the wiring note.
const MODE: OperationMode = OperationMode::Normal20;

/// Where recovery returns to. Figure 2-1 makes Restricted Operation and
/// Listen Only exit directly to the Normal modes, so this needs no
/// Configuration-mode round trip.
const RECOVER_TO: OperationMode = MODE;

/// The two modes a system error parks the chip in (`CiCON.SERR2LOM` picks
/// which). Neither is reachable any other way in this binary, so seeing
/// either one *is* the fault.
fn is_stalled(mode: OperationMode) -> bool {
    matches!(
        mode,
        OperationMode::RestrictedOperation | OperationMode::ListenOnly
    )
}

type Can = MCP251xFdAsync<common::Device>;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Ladder {
    ClearFlagsOnly,
    RecoverSystemError,
    ResetFifoThenMode,
    FullConfigCycle,
}

impl Ladder {
    const ALL: [Ladder; 4] = [
        Ladder::ClearFlagsOnly,
        Ladder::RecoverSystemError,
        Ladder::ResetFifoThenMode,
        Ladder::FullConfigCycle,
    ];

    fn name(self) -> &'static str {
        match self {
            Ladder::ClearFlagsOnly => "clear-flags-only",
            Ladder::RecoverSystemError => "recover_system_error",
            Ladder::ResetFifoThenMode => "reset_fifo+mode",
            Ladder::FullConfigCycle => "full-config-cycle",
        }
    }
}

async fn run_ladder(can: &mut Can, ladder: Ladder) -> Result<(), common::CanError> {
    match ladder {
        Ladder::ClearFlagsOnly => {
            let flags = can.interrupt_flags().await?;
            can.clear_interrupts(flags).await?;
        }
        Ladder::RecoverSystemError => {
            can.recover_system_error(RECOVER_TO, &mut Delay).await?;
        }
        Ladder::ResetFifoThenMode => {
            can.reset_fifo(TX).await?;
            can.recover_system_error(RECOVER_TO, &mut Delay).await?;
        }
        Ladder::FullConfigCycle => {
            can.set_mode(OperationMode::Configuration, &mut Delay).await?;
            can.apply_layout(&LAYOUT).await?;
            can.set_filter(Filter::F0, FilterMatch::accept_all(), RX).await?;
            can.set_mode(MODE, &mut Delay).await?;
        }
    }
    Ok(())
}

/// True once the chip is transmitting again: out of the stalled modes, with
/// the TX FIFO showing room.
async fn is_recovered(can: &mut Can) -> Result<bool, common::CanError> {
    if is_stalled(can.control_register().await?.op_mode()) {
        return Ok(false);
    }
    Ok(can.fifo_status(TX).await?.not_full_or_not_empty())
}

async fn report_signature(can: &mut Can, faults: u32) -> Result<(), common::CanError> {
    let int = can.interrupt_flags().await?;
    let con = can.control_register().await?;
    let sta = can.fifo_status(TX).await?;
    let fcon = can.fifo_config(TX).await?;
    let trec = can.error_counters().await?;
    warn!(
        "fault {faults}: CiINT={:#010X} serrif={} modif={} ivmif={} | OPMOD={:?}",
        int.0,
        int.serrif(),
        int.modif(),
        int.ivmif(),
        con.op_mode(),
    );
    warn!(
        "fault {faults}: FIFOSTA={:#010X} room={} txreq={} | TEC={} REC={} bo={} bp={}",
        sta.0,
        sta.not_full_or_not_empty(),
        fcon.txreq(),
        trec.tec(),
        trec.rec(),
        trec.tx_bus_off(),
        trec.tx_error_passive(),
    );
    Ok(())
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut chips: [Can; 10] = devices.map(MCP251xFdAsync::new);

    for (i, can) in chips.iter_mut().enumerate() {
        common::ensure_configuration(can).await;
        if let Err(e) = setup(can).await {
            error!("chip {i}: setup failed: {e:?}");
        }
    }

    let frame = Frame::new(StandardId::new(0x100).unwrap(), &[0xAA; 8]).unwrap();
    let mut cycles: u32 = 0;
    let mut faults: u32 = 0;
    let mut ladder_index = 0usize;

    loop {
        cycles += 1;
        // The receive-then-echo path: transmit, then read the frame back out
        // of RAM. The read is the half that trips the erratum.
        for can in chips.iter_mut() {
            let _ = can.transmit(TX, &frame).await;
        }
        for can in chips.iter_mut() {
            // Drain whatever arrived. Each `receive` is two RAM READs.
            while can.receive(RX).await.is_ok() {}
        }

        for (i, can) in chips.iter_mut().enumerate() {
            let stalled = match can.control_register().await {
                Ok(con) => is_stalled(con.op_mode()),
                Err(e) => {
                    error!("chip {i}: mode read failed: {e:?}");
                    continue;
                }
            };
            if !stalled {
                continue;
            }

            faults += 1;
            if let Err(e) = report_signature(can, faults).await {
                error!("chip {i}: signature read failed: {e:?}");
            }

            let ladder = Ladder::ALL[ladder_index % Ladder::ALL.len()];
            ladder_index += 1;

            let t0 = Instant::now();
            let outcome = run_ladder(can, ladder).await;
            let elapsed = t0.elapsed().as_micros();

            match outcome {
                Ok(()) => match is_recovered(can).await {
                    Ok(true) => info!(
                        "fault {faults} chip {i}: ladder {} RECOVERED in {elapsed} us",
                        ladder.name()
                    ),
                    Ok(false) => {
                        warn!(
                            "fault {faults} chip {i}: ladder {} DID NOT RECOVER ({elapsed} us)",
                            ladder.name()
                        );
                        // Fall back to the ladder known to work so the sweep
                        // can continue.
                        let _ = run_ladder(can, Ladder::FullConfigCycle).await;
                    }
                    Err(e) => error!("chip {i}: recovery check failed: {e:?}"),
                },
                Err(e) => error!("chip {i}: ladder {} errored: {e:?}", ladder.name()),
            }
        }

        if cycles % 500 == 0 {
            info!("{cycles} cycles, {faults} faults");
        }
        // 500 Hz, matching the load that reproduced this in the field.
        Timer::after_micros(2000).await;
    }
}

async fn setup(can: &mut Can) -> Result<(), common::CanError> {
    can.init(&common::CAN_CONFIG, &mut Delay).await?;
    can.apply_layout(&LAYOUT).await?;
    can.set_filter(Filter::F0, FilterMatch::accept_all(), RX).await?;
    can.configure_interrupts(CiInt(0).with_serrie(true).with_ivmie(true).with_modie(true))
        .await?;
    can.set_mode(MODE, &mut Delay).await?;
    Ok(())
}
```

**Check before building:** `CiTrec`'s bus-state accessors are used above as `tx_bus_off()` and `tx_error_passive()`. Confirm the real names with `grep -n "impl CiTrec" -A 25 src/registers/mod.rs` and use whatever is actually there.

- [ ] **Step 2: Build**

Run: `cd examples/rp2040 && cargo build --release --bin stall 2>&1 | tail -20`
Expected: builds clean. Fix accessor-name mismatches against the library rather than adding methods.

- [ ] **Step 3: Lint**

Run: `cd examples/rp2040 && cargo clippy --bins -- -D warnings && cargo fmt --check`
Expected: pass.

- [ ] **Step 4: Commit**

```bash
git add examples/rp2040/src/bin/stall.rs
git commit -m "examples: add stall reproduction and recovery-ladder timing

Drives the receive-then-echo load at 500 Hz, reports the full DS80000792D
item 1 signature when a chip drops into Restricted Operation or Listen
Only, and rotates through four recovery ladders timing each one.

Clear-flags-only failing while recover_system_error succeeds is the
evidence that the operation mode rather than the interrupt flags is what
is wrong.

Not yet run on hardware.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 11: The blocking-on-core-1 example

**Files:**
- Create: `examples/rp2040/src/bin/blocking_core1.rs`

**Interfaces:**
- Consumes: `common::init_board_blocking`, `common::BlockingDevice`, `common::BlockingCanError` (Task 9); the blocking `MCP251xFd`.
- Produces: nothing other code depends on.

Note this binary uses `#[cortex_m_rt::entry]` and builds two executors by hand. `#[embassy_executor::main]` assumes a single executor on the calling core and cannot express the split.

- [ ] **Step 1: Write the binary**

Create `examples/rp2040/src/bin/blocking_core1.rs`:

```rust
//! The blocking driver on core 1, with core 0 measuring its own jitter.
//!
//! # Why blocking on a dedicated core
//!
//! `embassy_rp::init` calls `dma::init`, which enables `DMA_IRQ_0` in the
//! calling core's NVIC -- and `init` runs on core 0. The handler loops over
//! all twelve DMA channels on every completion. `embassy-rp` never uses
//! `DMA_IRQ_1`, so there is no second line to give core 1. Every SPI DMA
//! completion raised by core 1 is therefore serviced on core 0, at arbitrary
//! phase relative to core 0's own real-time cycle.
//!
//! `Spi::new_blocking` takes no DMA channels and raises no completion
//! interrupt, so core 0 is left alone and two DMA channels come back. For the
//! 3-18 byte transfers this driver issues, DMA setup overhead dominates the
//! transfer anyway.
//!
//! There may also be a correctness gain. DS80000792D item 1 is triggered by
//! delays between SPI bytes and between the last byte and nCS rising; a DMA
//! completion serviced late on another core is one way to produce exactly
//! that. If `stall` faults on the async driver and this binary does not under
//! the same load, that is the cross-core interrupt being the mechanism.
//!
//! # What it reports
//!
//! Core 0 runs a fixed 2 ms cycle and counts how many starts land late, plus
//! the worst overshoot. Core 1 runs the CAN load and counts its own cycles.
//! Compare the late count here against the same measurement with the async
//! driver.
//!
//! # Wiring
//!
//! **Needs the CAN bus wired**, same as `stall` and for the same reason: it
//! runs `Normal20` so its fault rate is directly comparable with `stall`'s.
//! Run the two back to back and compare — that comparison is the experiment.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Executor;
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_time::{Delay, Instant, Ticker, Timer};
use embedded_can::{Frame as _, StandardId};
use log::{error, info};
use mcp251xfd::{
    Fifo, FifoLayout, Filter, FilterMatch, Frame, MCP251xFd, OperationMode, PayloadSize,
};
use panic_halt as _;
use static_cell::StaticCell;

const TX: Fifo = Fifo::F1;
const RX: Fifo = Fifo::F2;

const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(TX, PayloadSize::B8, 8)
    .rx_fifo(RX, PayloadSize::B8, 8);

/// Classic CAN on the real bus, matching `stall` so the two runs compare.
const MODE: OperationMode = OperationMode::Normal20;

/// Core 0's cycle period, and core 1's.
const CYCLE_US: u64 = 2000;

static CORE1_CYCLES: AtomicU32 = AtomicU32::new(0);
static CORE1_ERRORS: AtomicU32 = AtomicU32::new(0);
/// Times core 1 found a chip parked in Restricted Operation or Listen Only —
/// the DS80000792D item 1 signature. If this stays at zero under a load that
/// makes `stall` fault, the cross-core DMA interrupt was the mechanism.
static CORE1_STALLS: AtomicU32 = AtomicU32::new(0);

static mut CORE1_STACK: Stack<8192> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

type Can = MCP251xFd<common::BlockingDevice>;

#[cortex_m_rt::entry]
fn main() -> ! {
    let (devices, usb, core1) = common::init_board_blocking();

    spawn_core1(
        core1,
        // The stack is only ever touched by core 1 after this point.
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| spawner.must_spawn(can_task(devices)));
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.must_spawn(common::logger_task(usb));
        spawner.must_spawn(jitter_task());
    });
}

/// Core 1: the entire CAN workload, all of it blocking SPI.
#[embassy_executor::task]
async fn can_task(devices: [common::BlockingDevice; 10]) {
    let mut chips: [Can; 10] = devices.map(MCP251xFd::new);

    for can in chips.iter_mut() {
        let _ = can.set_mode(OperationMode::Configuration, &mut Delay);
        if let Err(e) = setup(can) {
            CORE1_ERRORS.fetch_add(1, Ordering::Relaxed);
            error!("core1 setup: {e:?}");
        }
    }

    let frame = Frame::new(StandardId::new(0x100).unwrap(), &[0xAA; 8]).unwrap();
    let mut ticker = Ticker::every(embassy_time::Duration::from_micros(CYCLE_US));

    loop {
        for can in chips.iter_mut() {
            if can.transmit(TX, &frame).is_err() {
                CORE1_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
        }
        for can in chips.iter_mut() {
            while can.receive(RX).is_ok() {}
        }
        // Same fault check `stall` makes, so the two binaries' counts mean
        // the same thing. Recover in place and keep going.
        for can in chips.iter_mut() {
            if let Ok(con) = can.control_register() {
                if matches!(
                    con.op_mode(),
                    OperationMode::RestrictedOperation | OperationMode::ListenOnly
                ) {
                    CORE1_STALLS.fetch_add(1, Ordering::Relaxed);
                    let _ = can.recover_system_error(MODE, &mut Delay);
                }
            }
        }
        CORE1_CYCLES.fetch_add(1, Ordering::Relaxed);
        ticker.next().await;
    }
}

fn setup(can: &mut Can) -> Result<(), common::BlockingCanError> {
    can.init(&common::CAN_CONFIG, &mut Delay)?;
    can.apply_layout(&LAYOUT)?;
    can.set_filter(Filter::F0, FilterMatch::accept_all(), RX)?;
    can.set_mode(MODE, &mut Delay)?;
    Ok(())
}

/// Core 0: a fixed cycle that does nothing but notice when it starts late.
#[embassy_executor::task]
async fn jitter_task() {
    common::wait_for_host().await;
    info!("blocking driver on core 1; core 0 measuring its own jitter");

    let period = embassy_time::Duration::from_micros(CYCLE_US);
    let mut ticker = Ticker::every(period);
    let mut expected = Instant::now() + period;
    let mut late: u32 = 0;
    let mut worst_us: u64 = 0;
    let mut cycles: u32 = 0;

    loop {
        ticker.next().await;
        let now = Instant::now();
        if now > expected {
            let over = (now - expected).as_micros();
            // One tick of slack: the timer itself has finite resolution.
            if over > 100 {
                late += 1;
                worst_us = worst_us.max(over);
            }
        }
        expected += period;
        cycles += 1;

        // Every ten minutes at 500 Hz.
        if cycles % 300_000 == 0 {
            info!(
                "core0: {late} late starts in {cycles} cycles, worst {worst_us} us | core1: {} cycles, {} errors, {} stalls",
                CORE1_CYCLES.load(Ordering::Relaxed),
                CORE1_ERRORS.load(Ordering::Relaxed),
                CORE1_STALLS.load(Ordering::Relaxed),
            );
        }
        // Keep the log alive on a shorter cadence too.
        if cycles % 15_000 == 0 {
            Timer::after_micros(0).await;
        }
    }
}
```

**Checks before building:**
- `Ticker::every` takes an `embassy_time::Duration`; confirm the import path used above compiles or add `use embassy_time::Duration`.
- `spawn_core1`'s closure must be `FnOnce() -> !`. `Executor::run` returns `!`, so the closure body above satisfies it — confirm the build agrees.
- `common::wait_for_host` and `common::logger_task` already exist and are reused unchanged.

- [ ] **Step 2: Build**

Run: `cd examples/rp2040 && cargo build --release --bin blocking_core1 2>&1 | tail -30`
Expected: builds clean. The most likely failure is a `Send` bound on the closure moved into `spawn_core1` — if `BlockingDevice` is not `Send`, confirm Task 9 used `CriticalSectionRawMutex` and not `NoopRawMutex`.

- [ ] **Step 3: Lint**

Run: `cd examples/rp2040 && cargo clippy --bins -- -D warnings && cargo fmt --check`
Expected: pass. `static mut CORE1_STACK` is accessed through `addr_of_mut!` specifically to avoid the `static_mut_refs` lint.

- [ ] **Step 4: Commit**

```bash
git add examples/rp2040/src/bin/blocking_core1.rs
git commit -m "examples: add blocking driver on core 1 with core 0 jitter metering

Runs the whole CAN workload on core 1 through the blocking driver and has
core 0 measure how many of its own 2 ms cycle starts land late.

The async path exports every SPI DMA completion raised by core 1 to core
0's NVIC, because embassy-rp enables DMA_IRQ_0 on the core that calls
init and never uses DMA_IRQ_1. Blocking removes that interrupt and frees
two DMA channels.

Not yet run on hardware.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

### Task 12: The batch-transmit measurement example, and the final sweep

**Files:**
- Create: `examples/rp2040/src/bin/batch.rs`
- Modify: `examples/rp2040/README.md`

**Interfaces:**
- Consumes: `transmit_batch` (Task 7).
- Produces: nothing.

- [ ] **Step 1: Write the binary**

Create `examples/rp2040/src/bin/batch.rs`:

```rust
//! Times `transmit` in a loop against `transmit_batch`, ten chips, three
//! frames each, at the 500 Hz cycle rate that motivated the API.
//!
//! Both should come out the same: after the paired status/user-address read,
//! the readiness check shares a transaction with the user-address fetch, so
//! there is nothing further for a batch to fold. Three chip-select
//! transactions per frame is the floor without the driver mirroring the
//! chip's RAM allocator.
//!
//! What `transmit_batch` actually buys is the accepted-count contract. The
//! partial-fill probe below is the part worth reading: it fills the FIFO
//! deliberately and checks the returned count is the accepted prefix.
//!
//! If the two timings differ by more than noise, that is worth investigating
//! -- it would mean the transaction accounting in the driver docs is wrong.
//!
//! Runs on internal loopback: SPI wiring only.
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;

use embassy_executor::Spawner;
use embassy_time::{Delay, Instant, Timer};
use embedded_can::{Frame as _, StandardId};
use log::{error, info};
use mcp251xfd::{
    Fifo, FifoLayout, Filter, FilterMatch, Frame, MCP251xFdAsync, OperationMode, PayloadSize,
};
use panic_halt as _;

const TX: Fifo = Fifo::F1;
const RX: Fifo = Fifo::F2;

/// Depth 16, as in the field deployment: three frames per cycle cannot fill
/// a FIFO that drains within the cycle.
const LAYOUT: FifoLayout = FifoLayout::new()
    .tx_fifo(TX, PayloadSize::B8, 16)
    .rx_fifo(RX, PayloadSize::B8, 8);

/// A short TX FIFO used only by the partial-fill probe.
const SHALLOW: FifoLayout = FifoLayout::new()
    .tx_fifo(TX, PayloadSize::B8, 2)
    .rx_fifo(RX, PayloadSize::B8, 8);

const MODE: OperationMode = OperationMode::InternalLoopback;
const CYCLES: u32 = 500;

type Can = MCP251xFdAsync<common::Device>;

async fn setup(can: &mut Can, layout: &FifoLayout) -> Result<(), common::CanError> {
    can.set_mode(OperationMode::Configuration, &mut Delay).await?;
    can.apply_layout(layout).await?;
    can.set_filter(Filter::F0, FilterMatch::accept_all(), RX).await?;
    can.set_mode(MODE, &mut Delay).await?;
    Ok(())
}

/// Fills a two-deep FIFO with a four-frame batch and checks the count.
async fn partial_fill_probe(can: &mut Can, frames: &[Frame; 4]) {
    if let Err(e) = setup(can, &SHALLOW).await {
        error!("partial-fill setup: {e:?}");
        return;
    }
    match can.transmit_batch(TX, frames).await {
        Ok(n) if (n as usize) < frames.len() => info!(
            "partial fill: {n} of {} accepted, remainder correctly refused",
            frames.len()
        ),
        Ok(n) => info!("partial fill: all {n} accepted (the FIFO drained mid-batch)"),
        Err(e) => error!("partial fill: {e:?}"),
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (devices, usb, _bus) = common::init_board();
    spawner.must_spawn(common::logger_task(usb));
    common::wait_for_host().await;

    let mut chips: [Can; 10] = devices.map(MCP251xFdAsync::new);
    for (i, can) in chips.iter_mut().enumerate() {
        common::ensure_configuration(can).await;
        if let Err(e) = can.init(&common::CAN_CONFIG, &mut Delay).await {
            error!("chip {i}: init: {e:?}");
        }
        if let Err(e) = setup(can, &LAYOUT).await {
            error!("chip {i}: setup: {e:?}");
        }
    }

    let three = [
        Frame::new(StandardId::new(0x101).unwrap(), &[1; 8]).unwrap(),
        Frame::new(StandardId::new(0x102).unwrap(), &[2; 8]).unwrap(),
        Frame::new(StandardId::new(0x103).unwrap(), &[3; 8]).unwrap(),
    ];
    let four = [
        Frame::new(StandardId::new(0x201).unwrap(), &[1; 8]).unwrap(),
        Frame::new(StandardId::new(0x202).unwrap(), &[2; 8]).unwrap(),
        Frame::new(StandardId::new(0x203).unwrap(), &[3; 8]).unwrap(),
        Frame::new(StandardId::new(0x204).unwrap(), &[4; 8]).unwrap(),
    ];

    loop {
        // Individual transmits.
        let t0 = Instant::now();
        for _ in 0..CYCLES {
            for can in chips.iter_mut() {
                for f in &three {
                    let _ = can.transmit(TX, f).await;
                }
                while can.receive(RX).await.is_ok() {}
            }
        }
        let individual = t0.elapsed().as_micros();

        // Batched.
        let t1 = Instant::now();
        for _ in 0..CYCLES {
            for can in chips.iter_mut() {
                let _ = can.transmit_batch(TX, &three).await;
                while can.receive(RX).await.is_ok() {}
            }
        }
        let batched = t1.elapsed().as_micros();

        info!(
            "{CYCLES} cycles x 10 chips x 3 frames: transmit {individual} us ({} us/cycle), transmit_batch {batched} us ({} us/cycle)",
            individual / CYCLES as u64,
            batched / CYCLES as u64,
        );

        partial_fill_probe(&mut chips[0], &four).await;
        if let Err(e) = setup(&mut chips[0], &LAYOUT).await {
            error!("restore chip 0: {e:?}");
        }

        Timer::after_secs(5).await;
    }
}
```

- [ ] **Step 2: Build and lint**

Run: `cd examples/rp2040 && cargo build --release --bin batch && cargo clippy --bins -- -D warnings && cargo fmt --check`
Expected: pass.

- [ ] **Step 3: Document the new binaries**

Add a row per new binary to the table in `examples/rp2040/README.md`, matching the existing format. For each, state what wiring it needs (all four need SPI only) and what it reports. Mark all four as not yet hardware-verified.

- [ ] **Step 4: Final full sweep**

Run every check the CI runs, from the repo root:

```bash
cargo test
cargo test --features async
cargo clippy --features async --all-targets -- -D warnings
cargo fmt --check
cargo doc --features async --no-deps
cargo build --target thumbv6m-none-eabi --no-default-features
cargo build --target thumbv6m-none-eabi --all-features
cargo package --list --allow-dirty | grep -c 'docs/'   # expect 0
```

then from `examples/rp2040`:

```bash
cargo build --release
cargo clippy --bins -- -D warnings
cargo fmt --check
```

Expected: everything passes, 91 library tests, all eleven example binaries build.

Record the final test count and note explicitly in the handoff that **no example has been run on hardware**.

- [ ] **Step 5: Commit**

```bash
git add examples/rp2040/src/bin/batch.rs examples/rp2040/README.md
git commit -m "examples: add batch-transmit timing and document the new binaries

batch times transmit against transmit_batch at the 500 Hz, ten-chip,
three-frame shape that motivated the API, and probes the partial-fill
path against a two-deep FIFO.

The two should time the same; the docs claim no transaction saving, and
this is what would catch that claim being wrong.

Not yet run on hardware.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

## Notes for the executor

- **Tasks 1-8 are library work and fully verifiable here.** Tasks 9-12 build but cannot be run without the ten-chip board; never describe them as verified.
- **If a library accessor name in an example does not exist**, fix the example to match the library. Do not add methods to the library from an example task — if something is genuinely missing, stop and report it.
- **Task 3 is the one most likely to surprise.** Updating the existing test expectations is not busywork; if a test's expectations do not collapse cleanly into a single paired read, that is a site where status and user address were *not* adjacent, and it deserves a second look before being forced.
- **Do not weaken a test to make it pass.** If an expectation and the implementation disagree, work out which is wrong first.
