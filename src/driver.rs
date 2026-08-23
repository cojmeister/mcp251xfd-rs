//! The MCP251XFD driver.

use crate::bus::Bus;
#[cfg(feature = "async")]
use crate::bus::BusAsync;
use crate::config::Config;
use crate::error::Error;
use crate::registers::{CiCon, CiTdc, OperationMode, Osc, TdcMode, Variant, addr};
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
    pub async fn reset(&mut self) -> Result<(), Error<SPI::Error>> {
        self.bus.reset().await
    }

    /// Resets and fully initializes the chip; returns the detected variant.
    ///
    /// Sequence (see spec §3.5): reset, RAM echo check, oscillator setup and
    /// ready-poll, variant detection via `OSC.LPMEN`, message-RAM zero fill,
    /// bit timing, `CiCON` (ISO CRC, stays in Configuration mode), interrupts
    /// cleared. Configure FIFOs/filters afterwards, then switch modes.
    pub async fn init<D: DelayNs>(
        &mut self,
        config: &Config,
        delay: &mut D,
    ) -> Result<Variant, Error<SPI::Error>> {
        config.validate().map_err(Error::InvalidConfig)?;
        self.bus.reset().await?;
        delay.delay_us(700).await;

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

        // Zero the message RAM (ECC stays disabled in this version).
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
}
