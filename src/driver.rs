//! The MCP251XFD driver.

use crate::bus::Bus;
#[cfg(feature = "async")]
use crate::bus::BusAsync;
use crate::config::Config;
use crate::config::FilterMatch;
use crate::error::Error;
use crate::frame::{FdFrame, Frame, FrameFlags, ReceivedFrame, RxFrame};
use crate::registers::objects::{RxHeader, TxHeader, dlc_to_len, len_to_dlc};
use crate::registers::ram::{FifoDirection, FifoLayout};
use crate::registers::{
    CiCon, CiDbtCfg, CiFifoCon, CiFifoSta, CiInt, CiNbtCfg, CiTdc, CiTrec, CiVec, Fifo, Filter,
    OperationMode, Osc, TdcMode, Variant, addr,
};
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
#[maybe_async_cfg::maybe(
    sync(keep_self),
    async(feature = "async", idents(Bus(async = "BusAsync")))
)]
pub struct MCP251xFd<SPI> {
    bus: Bus<SPI>,
    // Next TX sequence number (echoed in the TEF); masked per variant.
    seq: u32,
    // Written by `init`; read by `transmit`.
    seq_mask: u32,
}

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
        Self {
            bus: Bus { spi },
            seq: 0,
            seq_mask: Variant::Mcp2517Fd.seq_mask(),
        }
    }

    /// Consumes the driver and returns the SPI device.
    pub fn release(self) -> SPI {
        self.bus.spi
    }

    /// Sends the RESET instruction. The chip returns to Configuration mode
    /// with default registers.
    ///
    /// Per DS20006027B §4.1.1: should only be issued while the device is in
    /// Configuration mode, and it does not change Message Memory (RAM).
    pub async fn reset(&mut self) -> Result<(), Error<SPI::Error>> {
        self.bus.reset().await
    }

    /// Resets and fully initializes the chip; returns the detected variant.
    ///
    /// Sequence (per DS20006027B §4.1.1 and the oscillator/RAM init flow):
    /// reset, RAM echo check, oscillator setup and ready-poll, variant
    /// detection via `OSC.LPMEN`, message-RAM zero fill, bit timing, `CiCON`
    /// (ISO CRC, stays in Configuration mode), interrupts cleared. Configure
    /// FIFOs/filters afterwards, then switch modes.
    ///
    /// The initial RESET assumes the chip is already in Configuration mode
    /// (DS20006027B §4.1.1) — true right after power-on. On a warm restart
    /// where the chip was left in another mode, RESET is not guaranteed to
    /// take effect; power-cycle the chip or otherwise ensure Configuration
    /// mode before calling this.
    pub async fn init<D: DelayNs>(
        &mut self,
        config: &Config,
        delay: &mut D,
    ) -> Result<Variant, Error<SPI::Error>> {
        config.validate().map_err(Error::InvalidConfig)?;
        self.bus.reset().await?;
        // TOSCSTAB: oscillator stabilization time (DS20006027B Table 7-3).
        delay.delay_us(3000).await;

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
        self.bus
            .write_sfr32(addr::OSC, osc.with_lpmen(true).0)
            .await?;
        let variant = if Osc(self.bus.read_sfr32(addr::OSC).await?).lpmen() {
            Variant::Mcp2518Fd
        } else {
            Variant::Mcp2517Fd
        };
        self.bus.write_sfr32(addr::OSC, osc.0).await?;
        self.seq_mask = variant.seq_mask();

        // Zero the message RAM (ECC stays disabled in this version, but the
        // zero-fill also seeds valid ECC parity so a later ECC enable
        // doesn't see uninitialized words — the reason the Linux driver
        // does this too).
        let zeros = [0u8; 64];
        let mut a = addr::RAM_START;
        while (a as usize) < addr::RAM_START as usize + addr::RAM_SIZE {
            self.bus.write_ram(a, &zeros).await?;
            a += 64;
        }

        // Bit timing.
        self.bus
            .write_sfr32(addr::C1NBTCFG, config.nominal.to_reg().0)
            .await?;
        let mut tdc = CiTdc(0);
        if let Some(data) = config.data {
            self.bus
                .write_sfr32(addr::C1DBTCFG, data.to_reg().0)
                .await?;
            tdc = tdc
                .with_mode(TdcMode::Auto)
                .with_tdco(data.tdco())
                .with_edge_filter(true);
        }
        self.bus.write_sfr32(addr::C1TDC, tdc.0).await?;

        // CiCON: ISO CRC on, remain in Configuration mode. `BRSDIS` is set
        // when no data-phase timing was configured, so a caller that leaves
        // `Config::data` at `None` and then transmits a frame with
        // `FrameFlags::brs` cannot switch the chip into an unconfigured data
        // phase with no secondary sample point.
        let con = CiCon(0)
            .with_iso_crc_enabled(true)
            .with_brs_disabled(config.data.is_none())
            .with_req_op_mode(OperationMode::Configuration);
        self.bus.write_sfr32(addr::C1CON, con.0).await?;

        // Confirm the chip really is in Configuration mode rather than
        // assuming the request took. If the opening RESET was ignored -- which
        // DS20006027B only guarantees from Configuration mode -- returning
        // `Ok` here would hand back a chip in its previous mode, and the next
        // `apply_layout` would fail with `NotInConfigMode` instead of the
        // error belonging to this call.
        let mut in_config = false;
        for _ in 0..80 {
            if CiCon(self.bus.read_sfr32(addr::C1CON).await?).op_mode()
                == OperationMode::Configuration
            {
                in_config = true;
                break;
            }
            delay.delay_us(100).await;
        }
        if !in_config {
            return Err(Error::ModeChangeTimeout);
        }

        // All interrupt flags cleared, all enables off.
        self.bus.write_sfr32(addr::C1INT, 0).await?;

        Ok(variant)
    }

    /// Requests an operation mode and waits (≤ ~8 ms) until the chip
    /// reports it. Preserves the rest of `CiCON`.
    ///
    /// Requesting Sleep is fire-and-forget: per DS20006027B Register 3-7
    /// Note 2, `OPMOD` reads Configuration while the chip is asleep, so
    /// completion cannot be polled — and on MCP2518FD/MCP251863 Low-Power
    /// Mode, the SPI chip-select assertion a poll would perform wakes the
    /// device right back up. This method returns `Ok(())` right after
    /// writing `REQOP` for that mode, without polling.
    ///
    /// Mode changes are otherwise not always instantaneous: the chip
    /// finishes any bus activity in progress first (FRM §2.1/§2.1.3), and
    /// the FSM does not support switching directly between the two Normal
    /// modes or between the two Debug (loopback) modes (FRM §2.1.1/§2.1.2)
    /// — such a request never completes and this method times out with
    /// [`Error::ModeChangeTimeout`]. Go through Configuration mode first
    /// when switching between those pairs.
    pub async fn set_mode<D: DelayNs>(
        &mut self,
        mode: OperationMode,
        delay: &mut D,
    ) -> Result<(), Error<SPI::Error>> {
        let con = CiCon(self.bus.read_sfr32(addr::C1CON).await?);
        self.bus
            .write_sfr32(addr::C1CON, con.with_req_op_mode(mode).0)
            .await?;
        if mode == OperationMode::Sleep {
            return Ok(());
        }
        // Worst case: the bus is mid-frame when the mode change is
        // requested. A maximum-length CAN FD frame is ~736 bits; at the
        // slowest supported nominal bit rate (125 kbit/s) that's ~5.9 ms,
        // so 80 tries * 100 us (~8 ms) covers it with margin (Linux's
        // mcp251xfd driver sizes its wait the same way; Emandhal waits 7 ms).
        for _ in 0..80 {
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
    /// to fit, which [`FifoLayout`]'s construction guarantees — provided
    /// FIFO numbers are allocated contiguously from [`Fifo::F1`]. See
    /// [`FifoLayout`] for why gapped layouts are not validated against the
    /// chip's address generation.
    ///
    /// Requires Configuration mode. RX FIFOs are configured with not-empty
    /// and overflow interrupts enabled at the FIFO level; whether they
    /// reach the INT pin is controlled by `configure_interrupts`.
    ///
    /// `TXAT` (retransmission attempts, bits 22:21) is left at 0 for TX
    /// FIFOs; that field is inert while `CiCON.RTXAT` stays clear, which is
    /// how [`Self::init`] leaves it, so this has no effect yet — the
    /// transmit path makes the retransmission-attempts choice explicit.
    ///
    /// This assumes a freshly reset/initialized chip. Calling it again with
    /// a different layout does not clear FIFOs the previous layout
    /// configured: their register contents are left as they were, and they
    /// still participate in the chip's RAM address generation alongside
    /// the new layout's FIFOs.
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
        self.bus
            .write_sfr8(addr::flt_con_byte(filter), 0x00)
            .await?;
        self.bus
            .write_sfr32(addr::flt_obj(filter), matcher.fltobj)
            .await?;
        self.bus
            .write_sfr32(addr::flt_mask(filter), matcher.mask)
            .await?;
        self.bus
            .write_sfr8(addr::flt_con_byte(filter), 0x80 | target.index())
            .await
    }

    /// Disables an acceptance filter.
    pub async fn disable_filter(&mut self, filter: Filter) -> Result<(), Error<SPI::Error>> {
        self.bus.write_sfr8(addr::flt_con_byte(filter), 0x00).await
    }

    /// Queues a classic CAN 2.0 frame on a transmit FIFO and requests
    /// transmission. Non-blocking: [`Error::TxFifoFull`] when no slot is
    /// free (wait for a TX interrupt or retry).
    ///
    /// Retransmission is unlimited: [`Self::init`] leaves `CiCON.RTXAT`
    /// clear, so the per-FIFO `TXAT` field is inert and the chip keeps
    /// retrying a frame that loses arbitration or errors out until it wins
    /// the bus.
    ///
    /// This FIFO's `PLSIZE` (set via [`Self::apply_layout`]) must be at
    /// least as large as the longest frame handed to it: the TX message
    /// object only reserves `8 + PLSIZE` bytes, and a longer frame is
    /// written straight over the neighbouring elements of this or the next
    /// FIFO (DS20006027B Register 3-22 bit 31 `PLSIZE`; Table 3-5, Transmit
    /// Message Object). This driver keeps no per-FIFO `PLSIZE` record and
    /// cannot detect the mismatch — it only refuses the write when it would
    /// leave the 2048-byte message RAM entirely
    /// ([`Error::CommunicationCheckFailed`]). Size FIFOs for the frames
    /// they carry. Classic frames never exceed 8 bytes, so a `PLSIZE` of
    /// `PayloadSize::B8` or larger is always safe here.
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
    ///
    /// `frame.flags().brs` works as expected (`CiCON.BRSDIS` stays clear).
    /// `frame.flags().esi`, however, is inert: `T1.ESI` only drives the
    /// wire ESI bit in CAN-to-CAN gateway mode (`CiCON.ESIGM` = 1, DS20006027B
    /// Register 3-7 bit 17 and the `T1.ESI` note), and [`Self::init`] never
    /// sets `ESIGM` — so the transmitted ESI bit reflects the controller's
    /// own error-passive state instead of this field.
    ///
    /// This FIFO's `PLSIZE` (set via [`Self::apply_layout`]) must be at
    /// least as large as the longest frame handed to it: the TX message
    /// object only reserves `8 + PLSIZE` bytes, and a longer frame is
    /// written straight over the neighbouring elements of this or the next
    /// FIFO (DS20006027B Register 3-22 bit 31 `PLSIZE`; Table 3-5, Transmit
    /// Message Object). This driver keeps no per-FIFO `PLSIZE` record and
    /// cannot detect the mismatch — it only refuses the write when it would
    /// leave the 2048-byte message RAM entirely
    /// ([`Error::CommunicationCheckFailed`]). A 64-byte FD frame therefore
    /// needs `PayloadSize::B64`; size FIFOs for the frames they carry.
    pub async fn transmit_fd(
        &mut self,
        fifo: Fifo,
        frame: &FdFrame,
    ) -> Result<(), Error<SPI::Error>> {
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

    /// Writes a TX message object and requests transmission.
    ///
    /// Checks `CiFIFOSTA.TFNRFNIF` (not-full flag) first and bails out with
    /// [`Error::TxFifoFull`] if clear, then reads `CiFIFOUA` for the next
    /// free slot's RAM offset, writes the header and payload there, and
    /// sets `UINC | TXREQ` in `CiFIFOCON` byte 1 to hand the slot to the
    /// chip and request transmission.
    ///
    /// `CiFIFOUA` is not guaranteed to read back a valid offset while the
    /// chip is still in Configuration mode (DS20006027B Register 3-31 Note
    /// 1); a value at or past the end of the 2048-byte message RAM would
    /// otherwise turn into a corrupted SPI address, so it is rejected here
    /// as [`Error::CommunicationCheckFailed`] before any RAM access.
    ///
    /// The object's end is bounded the same way: a write of `8 + payload`
    /// bytes starting at `UA` must still land inside the 2048-byte window.
    /// It would not if the FIFO's `PLSIZE` is smaller than this frame (the
    /// chip then spaces its elements more tightly than the driver writes
    /// them) — near the top of RAM that becomes an SPI write past the last
    /// message-RAM address, which the address-rollover erratum
    /// (DS80000792 / DS80000789) says must not be relied on. Such a write
    /// is refused with [`Error::CommunicationCheckFailed`]; an overrun that
    /// stays inside the window cannot be detected here, since the driver
    /// keeps no per-FIFO `PLSIZE` record (see [`Self::transmit_fd`]).
    async fn transmit_raw(
        &mut self,
        fifo: Fifo,
        mut header: TxHeader,
        payload: &[u8],
    ) -> Result<(), Error<SPI::Error>> {
        // `CiFIFOUA` sits directly above `CiFIFOSTA` (0x05C + 12(m-1) + 4 and
        // + 8), so one 8-byte READ fetches both and the readiness check costs
        // no chip-select assertion of its own.
        let (sta_raw, ua) = self.bus.read_sfr32_pair(addr::fifo_sta(fifo)).await?;
        if !CiFifoSta(sta_raw).not_full_or_not_empty() {
            return Err(Error::TxFifoFull);
        }
        // Validate the *raw* read before narrowing to `UA` (bits 11:0):
        // masking first would fold a corrupt value like 0x0000_1000 into a
        // plausible 0 and write the wrong message object. Message objects are
        // 32-bit aligned, so an unaligned `UA` is corrupt too -- and would
        // trip the alignment `debug_assert` in `bus::write_ram`, or issue a
        // RAM access the chip does not support in a release build.
        if ua >= addr::RAM_SIZE as u32 || ua % 4 != 0 {
            return Err(Error::CommunicationCheckFailed);
        }
        // 8 header bytes + payload zero-padded to a 32-bit boundary.
        let len = 8 + payload.len().div_ceil(4) * 4;
        // End-bound the write as well as its start: see the note above.
        if ua as usize + len > addr::RAM_SIZE {
            return Err(Error::CommunicationCheckFailed);
        }

        header.seq = self.seq & self.seq_mask;
        self.seq = self.seq.wrapping_add(1);
        let [t0, t1] = header.to_words();

        let mut obj = [0u8; 72];
        obj[0..4].copy_from_slice(&t0.to_le_bytes());
        obj[4..8].copy_from_slice(&t1.to_le_bytes());
        obj[8..8 + payload.len()].copy_from_slice(payload);

        self.bus
            .write_ram(addr::RAM_START + ua as u16, &obj[..len])
            .await?;
        self.bus
            .write_sfr8(addr::fifo_con(fifo) + 1, CiFifoCon::CON_BYTE1_UINC_TXREQ)
            .await
    }

    /// Pops one frame from a receive FIFO. Non-blocking:
    /// [`Error::RxFifoEmpty`] when there is nothing to read. Also fails with
    /// [`Error::CommunicationCheckFailed`] if `CiFIFOUA` reads back too
    /// close to the end of the 2048-byte message RAM to hold even a message
    /// object's 8-byte header (same implausible-value check as
    /// [`Self::transmit`]'s underlying `transmit_raw`); no RAM access is
    /// attempted in that case.
    ///
    /// A classic frame whose sender used a nonconforming DLC of 9..=15 is
    /// stored with `Frame::dlc()` capped at 8: `dlc_to_len` maps every
    /// classic DLC above 8 to 8 payload bytes, since only that many are
    /// ever present in the message object (DS20006027B Table 3-6, Receive
    /// Message Object, bits R1.3-0 `DLC`).
    ///
    /// This FIFO's `PLSIZE` (set via [`Self::apply_layout`]) must be at
    /// least as large as the longest frame its filters can accept: the RX
    /// message object only reserves `8 + PLSIZE` bytes, and a frame whose
    /// DLC decodes to more payload than that overruns into the next FIFO
    /// slot (`DLCMM`, DS20006027B Register 3-22 bit 31; Table 3-6 Note 1).
    /// This driver keeps no per-FIFO `PLSIZE` record and does not guard
    /// against it here, so bytes beyond the configured `PLSIZE` are
    /// undefined in that case — size filters and FIFOs consistently to
    /// avoid it. The one case that *is* caught is a read that would leave
    /// the 2048-byte message RAM entirely: it fails with
    /// [`Error::CommunicationCheckFailed`] instead.
    ///
    /// `fifo` must be a receive FIFO. The driver keeps no per-FIFO
    /// direction record either, so passing a transmit FIFO's handle decodes
    /// that FIFO's TX message object as if it were an RX one — arbitrary
    /// identifier, DLC and payload — *and* still writes `UINC`, advancing
    /// the transmit FIFO's tail past an element the chip has not sent.
    pub async fn receive(&mut self, fifo: Fifo) -> Result<RxFrame, Error<SPI::Error>> {
        let (sta_raw, ua) = self.bus.read_sfr32_pair(addr::fifo_sta(fifo)).await?;
        if !CiFifoSta(sta_raw).not_full_or_not_empty() {
            return Err(Error::RxFifoEmpty);
        }
        // Validated raw and for 32-bit alignment before narrowing, as in
        // `transmit_raw`. Every RX object starts with an 8-byte header, so a
        // `UA` above `RAM_SIZE - 8` is implausible and would make the header
        // read below run past the end of message RAM on its own.
        if ua % 4 != 0 || ua as usize + 8 > addr::RAM_SIZE {
            return Err(Error::CommunicationCheckFailed);
        }
        let base = addr::RAM_START + ua as u16;

        let mut hdr = [0u8; 8];
        self.bus.read_ram(base, &mut hdr).await?;
        let r0 = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]);
        let r1 = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
        let header = RxHeader::from_words([r0, r1]);

        let len = dlc_to_len(header.dlc, header.fdf);
        let padded = len.div_ceil(4) * 4;
        // End-bound the read the same way `transmit_raw` bounds its write.
        if ua as usize + 8 + padded > addr::RAM_SIZE {
            return Err(Error::CommunicationCheckFailed);
        }
        let mut data = [0u8; 64];
        // A remote frame carries no data bytes on the wire (classic CAN
        // only — CAN FD has no remote frames), so the object's payload slot
        // holds whatever occupied this RAM element before. Skip the read and
        // leave `data` all-zero, matching [`Frame::new_remote`].
        let remote = header.rtr && !header.fdf;
        if padded > 0 && !remote {
            self.bus.read_ram(base + 8, &mut data[..padded]).await?;
            // Only `len` bytes are payload; the rest of the last word holds
            // whatever occupied this RAM slot before. Zero it so the frame's
            // derived `PartialEq`/`Debug` can't see the previous occupant.
            data[len..padded].fill(0);
        }
        self.bus
            .write_sfr8(addr::fifo_con(fifo) + 1, CiFifoCon::CON_BYTE1_UINC)
            .await?;

        let frame = if header.fdf {
            ReceivedFrame::Fd(FdFrame::from_parts(
                header.id,
                len as u8,
                FrameFlags {
                    brs: header.brs,
                    esi: header.esi,
                },
                data,
            ))
        } else {
            let mut d8 = [0u8; 8];
            d8.copy_from_slice(&data[..8]);
            ReceivedFrame::Classic(Frame::from_parts(header.id, len as u8, header.rtr, d8))
        };
        Ok(RxFrame {
            frame,
            timestamp: None,
        })
    }

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

    /// Reads a FIFO's status register.
    pub async fn fifo_status(&mut self, fifo: Fifo) -> Result<CiFifoSta, Error<SPI::Error>> {
        Ok(CiFifoSta(self.bus.read_sfr32(addr::fifo_sta(fifo)).await?))
    }

    /// Clears a FIFO's overflow (and attempt-exhausted) flags.
    pub async fn clear_rx_overflow(&mut self, fifo: Fifo) -> Result<(), Error<SPI::Error>> {
        self.bus.write_sfr8(addr::fifo_sta(fifo), 0x00).await
    }

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

    /// Reads the global interrupt flags (and enable bits).
    pub async fn interrupt_flags(&mut self) -> Result<CiInt, Error<SPI::Error>> {
        Ok(CiInt(self.bus.read_sfr32(addr::C1INT).await?))
    }

    /// Clears the software-clearable interrupt flags set in `flags`
    /// (write-0-to-clear; only the flag half, `C1INT` bytes 0-1, is
    /// touched).
    ///
    /// Per DS20006027B Register 3-14's attribute rows, only `IVMIF`,
    /// `WAKIF`, `CERRIF`, `SERRIF`, `MODIF`, and `TBCIF` are actually
    /// software-clearable (`HS/C`) this way. `TXIF`, `RXIF`, `TEFIF`,
    /// `RXOVIF`, `TXATIF`, `SPICRCIF`, and `ECCIF` are read-only mirrors of
    /// FIFO- or module-level state and are cleared at their source instead
    /// — e.g. `RXOVIF`/`TXATIF` via [`Self::clear_rx_overflow`], `RXIF` by
    /// draining the FIFO with [`Self::receive`] — so writing 0 to those
    /// bits here is ignored/harmless.
    pub async fn clear_interrupts(&mut self, flags: CiInt) -> Result<(), Error<SPI::Error>> {
        self.bus.write_sfr8(addr::C1INT, !(flags.0 as u8)).await?;
        self.bus
            .write_sfr8(addr::C1INT + 1, !((flags.0 >> 8) as u8))
            .await
    }

    /// Writes the interrupt enable half of `C1INT`. Build the value with
    /// the `with_*ie` methods, e.g. `CiInt(0).with_rxie(true)`.
    pub async fn configure_interrupts(&mut self, enables: CiInt) -> Result<(), Error<SPI::Error>> {
        self.bus
            .write_sfr8(addr::C1INT + 2, (enables.0 >> 16) as u8)
            .await?;
        self.bus
            .write_sfr8(addr::C1INT + 3, (enables.0 >> 24) as u8)
            .await
    }

    /// Reads and decodes the highest-priority pending interrupt.
    pub async fn pending_event(&mut self) -> Result<Event, Error<SPI::Error>> {
        Ok(Event::from_icode(
            CiVec(self.bus.read_sfr32(addr::C1VEC).await?).icode(),
        ))
    }

    /// Reads the error counters and bus state (`CiTREC`).
    pub async fn error_counters(&mut self) -> Result<CiTrec, Error<SPI::Error>> {
        Ok(CiTrec(self.bus.read_sfr32(addr::C1TREC).await?))
    }
}

/// Async-only conveniences built on the interrupt pin.
#[cfg(feature = "async")]
impl<SPI: SpiDeviceAsync> MCP251xFdAsync<SPI> {
    /// Waits until a frame arrives on `fifo` and returns it.
    ///
    /// Level-triggered and race-free: the FIFO is checked *before* waiting
    /// on the pin, so a frame that arrives between calls is never missed.
    /// Per DS20006027B Register 3-2 (`IOCON`, 0xE04) and §6.0.1, the INT
    /// pins are active-low and default to push-pull (open-drain is
    /// selectable via `IOCON.INTOD`, which this driver never sets); either
    /// way, nINT stays asserted low as long as any enabled interrupt is
    /// pending. Requirements: the FIFO was configured by
    /// [`Self::apply_layout`] (which sets its not-empty interrupt) and RXIE
    /// is enabled via [`Self::configure_interrupts`]; `int_pin` is the MCU
    /// input wired to nINT (any [`embedded_hal_async::digital::Wait`]
    /// implementation — e.g. an embassy `Input`/`ExtiInput`).
    ///
    /// # Caveats
    ///
    /// nINT is a single, global line: per §6.0.1 it is asserted whenever
    /// *any* enabled interrupt source (`xIF & xIE`) is pending, not just
    /// `fifo`'s. This method assumes the only enabled sources are ones that
    /// draining `fifo` via [`Self::receive`] actually clears. If some other
    /// enabled source keeps nINT low, `int_pin.wait_for_low()` returns
    /// immediately every iteration and the loop busy-spins over SPI with no
    /// executor yield, starving other tasks on a single-priority executor.
    /// Reachable ways to trigger this: another RX FIFO with its not-empty
    /// interrupt (RXIE) enabled and a frame sitting in it; `RXOVIE` enabled,
    /// since a latched `RXOVIF` only clears via [`Self::clear_rx_overflow`],
    /// not by draining `fifo`; or `TXIE` with a TX FIFO whose not-full
    /// interrupt is enabled. Clearing `IOCON.PM1` (Register 3-2 bit 25; POR
    /// default 1 = GPIO1) configures INT1 as a dedicated RX interrupt pin
    /// (asserted when `CiINT.RXIF` and `RXIE` are set), which narrows but
    /// does not eliminate this: INT1 is still shared across all RX FIFOs,
    /// so a busy other RX FIFO can still hold it low.
    pub async fn wait_rx<P: embedded_hal_async::digital::Wait>(
        &mut self,
        fifo: Fifo,
        int_pin: &mut P,
    ) -> Result<RxFrame, Error<SPI::Error>> {
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
