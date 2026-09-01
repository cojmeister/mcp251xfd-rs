# Hardware results — 2026-09-01

Bench: the ten-MCP2517FD RP2040 board, driven from a Raspberry Pi that holds
the board's USB CDC serial (`/dev/ttyACM0`) and a PCAN-USB adapter on `can0` at
500 kbit/s. Only the chip on **GP15 — index 9** has its CAN connector wired to
the adapter; the other nine are SPI-only on this bench.

Every number below came off this board.

## 1. Register read-back — `regdump`

45 sweeps × 10 chips = **450 chip dumps, zero errors**. Every value identical
across all ten chips and all 45 passes.

| Register | Read from silicon | Expected from `CAN_CONFIG` | |
|---|---|---|---|
| `NBTCFG` | `0x001E0707` | BRP 1, TSEG1 31, TSEG2 8, SJW 8 | match |
| `DBTCFG` | `0x00060101` | BRP 1, TSEG1 7, TSEG2 2, SJW 2 | match |
| `TDC` | `0x02020700` | TDCO = BRP·DTSEG1 = 7, auto | match |
| `CiCON` | `0x04800020` | Configuration, ISO CRC on, RTXAT clear | match |
| TX `FIFOCON` | `0xE3000480` | PLSIZE B64, depth 4 | match |
| RX `FIFOCON` | `0xE7000409` | PLSIZE B64, depth 8, RXOVIE + TFNRFNIE | match |
| RX `FIFOUA` | `0x0120` | 288 = 4 × (8 + 64) | match |

`read_back_config` and `NominalBitTiming::to_reg` agree bit for bit, so the
built-in mismatch check never fired. The RX FIFO's user address landing exactly
at 288 confirms the chip's own RAM allocator lands where the layout math says
it will.

TX `FIFOSTA` read `0x00000007` — bits 0, 1 and 2, which is correct for an empty
TX FIFO and validates `CiFifoSta::tx_empty_or_rx_full` (bit 2) on silicon.

One benign anomaly: chip 2 reports `IOCON=0x03020003` where the other nine
report `0x03000003`. The difference is bit 17, the GPIO1/INT1 **pin level**, not
configuration.

## 2. The transmit stall, reproduced

### Why the first attempt found nothing

`bench_async` runs the configuration the stall appears under — async driver on
core 1, so its SPI DMA completions are raised there and serviced on core 0 —
and produced **zero faults** in 75,968 cycles and 77,190 received frames.

The reason is that core 0 was otherwise idle, so it serviced `DMA_IRQ_0` the
instant it fired. A core 0 busy with its own real-time work does not. That
missing variable is the entire mechanism, and it is controllable.

### The instrument

`bench_interference` has core 0 hold its own interrupts off with
`cortex_m::interrupt::free` for `d` microseconds at a time. `DMA_IRQ_0` cannot
be serviced while masked, so the nCS rising edge for whatever transfer core 1
has in flight is delayed by up to `d`. DS80000792D item 1 puts T_SPIMAXDLY at
5 nominal bit times — 10 µs at 500 kbit/s — so sweeping `d` across that should
make faults appear.

### The signature

First fault caught, at `d = 8 µs`:

```
CiINT   = 0x90089008   serrif=true modif=true ivmif=true
OPMOD   = RestrictedOperation
FIFOSTA = 0x00000323   room=true  empty=false  txreq=true
CiTREC  : TEC=0 REC=0 bus_off=false error_passive=false
```

Every element the erratum predicts:

| Predicted | Observed |
|---|---|
| sets `SERRIF` and `MODIF` | `CiINT` low half `0x9008` = MODIF(3) + SERRIF(12) + IVMIF(15) |
| transitions to Restricted Operation | `OPMOD = RestrictedOperation` |
| `TXREQ` ignored, FIFO stops draining | `txreq=true`, `empty=false`, `TXERR` set |
| not a bus-error condition | `TEC=0 REC=0`, not bus-off, not error-passive |

The `IVMIF` accompanying `SERRIF`/`MODIF` is the pairing Linux mainline
documents for the aborted CAN frame.

`FIFOSTA=0x00000323` decodes to `room=true` while the controller was wedged and
draining nothing. That is the "FIFO has space, so it must be healthy" trap,
caught with the chip in the act: free capacity is not a health signal.

### Dose–response

Pooled over both arms of `bench_d10`, 12 s per cell:

| `d` (µs) | 6 | 8 | 9 | 10 | 11 | 12 | 14 |
|---|---|---|---|---|---|---|---|
| stalls | 2 | 4 | 14 | 18 | 34 | 68 | 72 |

Monotonic, with onset around 8–9 µs. Faults below the erratum's 10 µs are
expected: the effective delay is the mask plus executor latency, not the mask
alone. **Treat 8–12 µs as a bracket, not a measurement.**

### The control that closes it

`bench_interference_blocking` applies the identical sweep to the identical
workload with the **blocking** driver, which uses no DMA and so raises no
completion for core 0 to be late with:

| core-0 mask (µs) | 0 | 2 | 4 | 8 | 10 | 12 | 16 | 24 | 40 | total |
|---|---|---|---|---|---|---|---|---|---|---|
| async on core 1 | 0 | 0 | 0 | 7 | 0 | 18 | 32 | 75 | 162 | **294** |
| blocking on core 1 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | **0** |

The two runs are matched to within four cycles over ~110 s (54,971 vs 54,969
cycles; 95,898 vs 95,892 frames). At `d = 40 µs` the blocking run still logged
**11,375 late cycle starts on core 0**, so the interference was applied and
biting — it simply had nothing to delay.

**Conclusion: the cross-core DMA completion is the mechanism.** The stall and
the cross-core jitter are one defect, and moving the CAN workload to the blocking
driver on a dedicated core is a fix for the stall, not only for jitter.

## 3. Recovery

`recover_system_error` across every run: **506 faults, 506 recovered, zero
failures.**

| Run | n | min | median | max |
|---|---|---|---|---|
| `bench_interference` | 294 | 131 µs | 246 µs | 564 µs |
| `bench_d10` | 212 | 119 µs | 137 µs | 229 µs |

For comparison, the obvious recovery — a full Configuration-mode cycle
re-applying layout and filters — measures **22,000 µs** on this board, which
rules it out of any millisecond-scale control cycle. A median of 137–246 µs
fits comfortably.

## 4. A correction to our own method

The first sweep held each `d` for one 12 s phase in **ascending order**, and
produced a conspicuous zero at `d = 10 µs` between 7 at 8 µs and 18 at 12 µs.

The hypothesis was cadence resonance: with a 60 µs gap and ~10 µs of loop
overhead, `d = 10` gives an ~80 µs period, and 80 divides core 1's 2000 µs cycle
exactly 25 times.

`bench_d10` tested that with two arms — fixed gap and jittered gap — and
**refuted it**. The fixed-gap arm used the identical 60 µs gap and produced 8
stalls at `d = 10`, not 0. Both arms agree with each other.

What actually fixed it was **round-robin interleaving** of the `d` values.
A single ascending pass confounds `d` with elapsed time, and the per-point
numbers from that first sweep are not reliable — its `d = 8` gave 7 where the
interleaved run gives ~1. Only the shape and the async/blocking contrast
survive from it, and the interleaved run reproduces both.

## Binaries

| Binary | Wiring | Purpose |
|---|---|---|
| `regdump` | SPI only | Dump and diff configuration registers on all ten chips |
| `bench_async` | CAN bus | Async driver on core 1 — the configuration the stall appears under |
| `bench_blocking` | CAN bus | Same workload, blocking driver — the control |
| `bench_interference` | CAN bus | Sweeps core-0 interrupt-mask time against the stall rate |
| `bench_interference_blocking` | CAN bus | The same sweep with no DMA — the control that closes the argument |
| `bench_d10` | CAN bus | Two-arm, round-robin re-test of the `d = 10 µs` notch |

Feed the bussed chip from the host so `receive` runs — only `receive` issues the
RAM reads the erratum needs:

```
cangen can0 -g 1 -I 321 -L 8 -D r
```

## Still not exercised on hardware

`write_register_raw` (every run only reads), `reset_fifo`, and `transmit_batch`.
