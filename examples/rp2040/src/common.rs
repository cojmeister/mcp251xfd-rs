//! Shared board setup for the 10-chip MCP2517FD test board.

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_rp::gpio::{AnyPin, Level, Output, Pin};
use embassy_rp::peripherals::SPI1;
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_rp::Peripherals;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use mcp251xfd::{ClockConfig, Config, DataBitTiming, NominalBitTiming};
use static_cell::StaticCell;

pub type Bus = Mutex<NoopRawMutex, Spi<'static, SPI1, Async>>;
pub type Device = SpiDevice<'static, NoopRawMutex, Spi<'static, SPI1, Async>, Output<'static>>;

static SPI_BUS: StaticCell<Bus> = StaticCell::new();

/// 500 kbit/s nominal, 2 Mbit/s data, 40 MHz crystal. If the board's crystal
/// is 20 MHz instead: set `clock` to `ClockConfig::MHZ20`, hand-build
/// `nominal`/`data` bit timings (the library ships no `*_20MHZ` presets, only
/// `*_40MHZ`), and lower `setup`'s SPI frequency to 8_500_000 (the
/// erratum-safe cap of 0.85 * SYSCLK / 2 at a 20 MHz SYSCLK).
pub const CAN_CONFIG: Config = Config {
    clock: ClockConfig::MHZ40,
    nominal: NominalBitTiming::KBPS500_40MHZ,
    data: Some(DataBitTiming::MBPS2_40MHZ),
};

/// Sets up SPI1 (SCK=GP10, MOSI=GP11, MISO=GP12) at the erratum-safe
/// 17 MHz and returns one `SpiDevice` per chip-select pin
/// (GP3..GP9, GP13, GP14, GP15).
pub fn setup(p: Peripherals) -> [Device; 10] {
    let mut cfg = SpiConfig::default();
    cfg.frequency = 17_000_000; // = mcp251xfd::max_spi_hz(40 MHz SYSCLK)
    let spi = Spi::new(
        p.SPI1, p.PIN_10, p.PIN_11, p.PIN_12, p.DMA_CH0, p.DMA_CH1, cfg,
    );
    let bus: &'static Bus = SPI_BUS.init(Mutex::new(spi));
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
    cs.map(|pin| SpiDevice::new(bus, Output::new(pin, Level::High)))
}
