//! The MCP251XFD driver.

use crate::bus::Bus;
#[cfg(feature = "async")]
use crate::bus::BusAsync;
use crate::config::Config;
use crate::config::FilterMatch;
use crate::error::Error;
use crate::registers::ram::{FifoDirection, FifoLayout};
use crate::registers::{
    CiCon, CiFifoCon, CiTdc, Fifo, Filter, OperationMode, Osc, TdcMode, Variant, addr,
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
    // Written by `init`; read by `transmit` (Task 11).
    #[allow(dead_code)] // read by transmit (Task 11)
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
        Self {
            bus: Bus { spi },
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

        // CiCON: ISO CRC on, remain in Configuration mode.
        let con = CiCon(0)
            .with_iso_crc_enabled(true)
            .with_req_op_mode(OperationMode::Configuration);
        self.bus.write_sfr32(addr::C1CON, con.0).await?;

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
    /// to fit (guaranteed by [`FifoLayout`]'s construction). Requires
    /// Configuration mode. RX FIFOs are configured with not-empty and
    /// overflow interrupts enabled at the FIFO level; whether they reach
    /// the INT pin is controlled by `configure_interrupts`.
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
}
