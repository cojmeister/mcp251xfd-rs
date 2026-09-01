//! Shared board setup for the 10-chip MCP2517FD test board.

use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice as BlockingSpiDevice;
use embassy_embedded_hal::shared_bus::SpiDeviceError;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{AnyPin, Level, Output, Pin};
use embassy_rp::peripherals::{CORE1, SPI1, USB};
use embassy_rp::spi::{Async, Blocking, Config as SpiConfig, Phase, Polarity, Spi};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Timer};
use mcp251xfd::{
    ClockConfig, Config, DataBitTiming, Fifo, FifoLayout, Filter, FilterMatch, MCP251xFdAsync,
    NominalBitTiming, OperationMode, RxFrame,
};
use static_cell::StaticCell;

pub type Bus = Mutex<NoopRawMutex, Spi<'static, SPI1, Async>>;
pub type Device = SpiDevice<'static, NoopRawMutex, Spi<'static, SPI1, Async>, Output<'static>>;

/// The concrete error type every driver call on a [`Device`] can return.
///
/// Spelled out so the test bodies can propagate with `?` and log the
/// discriminant instead of `unwrap()`-ing: without a debug probe a panic is
/// an invisible hang, and on a 10-chip sweep one bad chip must not cost the
/// diagnostics for the other nine.
#[allow(dead_code)] // not used by every binary that includes common.rs
pub type CanError =
    mcp251xfd::Error<SpiDeviceError<embassy_rp::spi::Error, core::convert::Infallible>>;

pub type UsbDriver = Driver<'static, USB>;

static SPI_BUS: StaticCell<Bus> = StaticCell::new();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

/// 500 kbit/s nominal, 2 Mbit/s data on a **20 MHz** crystal.
///
/// The crystal here was measured, not assumed. `bitrate` times a frame's
/// loopback round trip at two payload sizes; the difference cancels the fixed
/// SPI/driver overhead and yields the on-wire bit period directly. With the
/// library's 40 MHz presets this board reported 246 kbit/s -- exactly half of
/// the configured 500 -- so SYSCLK is 20 MHz. These timings measure
/// 480 kbit/s (the residual is `bitrate`'s stuff-bit estimate, not a timing
/// error).
///
/// Internal loopback structurally cannot catch a wrong crystal: both ends of
/// the link share the same wrong clock, so every loopback test passes at the
/// wrong bit rate. `bitrate` is the only test here that validates this
/// constant, and it is the one to re-run if the board is revised.
///
/// The library ships no `*_20MHZ` presets, hence the literals:
/// - nominal: 20 MHz / 500 kHz = 40 TQ per bit at BRP=1, split 1 + 31 + 8 for
///   an 80% sample point. SJW = 8 = min(TSEG1, TSEG2).
/// - data: 20 MHz / 2 MHz = 10 TQ per bit, split 1 + 7 + 2 (80%), SJW = 2,
///   so the driver derives TDCO = BRP * DTSEG1 = 7.
pub const CAN_CONFIG: Config = Config {
    clock: ClockConfig::MHZ20,
    nominal: NominalBitTiming {
        brp: 1,
        tseg1: 31,
        tseg2: 8,
        sjw: 8,
    },
    data: Some(DataBitTiming {
        brp: 1,
        tseg1: 7,
        tseg2: 2,
        sjw: 2,
    }),
};
/// Brings up the board: SPI1 (SCK=GP10, MOSI=GP11, MISO=GP12) with one
/// `SpiDevice` per chip-select pin (GP3..GP9, GP13, GP14, GP15), plus the USB
/// peripheral the log output leaves through and the shared bus handle (which
/// diagnostics use to re-clock the bus at runtime via `Spi::set_config`).
pub fn init_board() -> ([Device; 10], UsbDriver, &'static Bus) {
    let p = embassy_rp::init(Default::default());

    let mut cfg = SpiConfig::default();
    // Derived from `CAN_CONFIG` rather than hardcoded so the SPI clock cannot
    // drift away from the crystal: the erratum-safe cap is 0.85 * SYSCLK / 2,
    // i.e. 8.5 MHz at this board's 20 MHz SYSCLK. embassy-rp quantizes that
    // down to 125 MHz / 2 / 8 = 7.8125 MHz, so that is what a scope on SCK
    // shows.
    //
    // This matters more than it looks. The board was originally run at
    // 15.625 MHz (the cap for a 40 MHz crystal it does not have) and SPI reads
    // corrupted intermittently: message-object headers read back as zeros,
    // `CiFIFOUA` pointed into blank RAM, and `C1CON` returned a bogus OPMOD --
    // surfacing as random `FDF` loss and `NotInConfigMode`. A sweep found the
    // chips clean at every rate up to 12.5 MHz and broken at 15.625 MHz.
    cfg.frequency = mcp251xfd::max_spi_hz(CAN_CONFIG.clock.sysclk_hz());
    // The MCP251xFD requires SPI mode (0,0). These are already
    // `SpiConfig::default()`, but a hard chip requirement should not rest on
    // an upstream default staying put.
    cfg.phase = Phase::CaptureOnFirstTransition;
    cfg.polarity = Polarity::IdleLow;

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
    let devices = cs.map(|pin| SpiDevice::new(bus, Output::new(pin, Level::High)));

    (devices, Driver::new(p.USB, Irqs), bus)
}

/// Runs the USB CDC-ACM serial device that carries every `log` line.
///
/// Must be spawned before the first log call. The logger writes into a
/// non-blocking 1 KiB pipe, so output produced while no terminal has the port
/// open is dropped rather than stalling the test -- which is why each binary
/// repeats its sweep instead of reporting once.
#[embassy_executor::task]
pub async fn logger_task(driver: UsbDriver) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

/// Gives the host a moment to enumerate the CDC device before the first log
/// line, so a terminal already open across a re-flash catches the first pass.
pub async fn wait_for_host() {
    Timer::after_secs(2).await;
}

/// Returns a chip to Configuration mode so the RESET at the start of
/// [`MCP251xFdAsync::init`] is guaranteed to take effect.
///
/// `init` documents that RESET is only reliable from Configuration mode.
/// These binaries loop, and they leave their chips in Loopback/Normal mode,
/// so without this every pass after the first would re-init from a mode where
/// RESET may be ignored. An error is deliberately discarded: a chip that is
/// absent, or already in Configuration mode straight out of power-on, is not
/// a failure worth reporting here.
#[allow(dead_code)] // not used by every binary that includes common.rs
pub async fn ensure_configuration(can: &mut MCP251xFdAsync<Device>) {
    let _ = can.set_mode(OperationMode::Configuration, &mut Delay).await;
}

/// Polls an RX FIFO for up to ~100 ms.
///
/// Only an empty FIFO is retried. Any other error -- an SPI fault, a bad
/// `CiFIFOUA` read-back -- is logged once and aborts the wait, so a dead bus
/// is not reported as a CAN timeout that sends the operator looking at
/// transceivers and termination.
#[allow(dead_code)] // not used by every binary that includes common.rs
pub async fn recv_timeout(can: &mut MCP251xFdAsync<Device>, fifo: Fifo) -> Option<RxFrame> {
    for _ in 0..100 {
        match can.receive(fifo).await {
            Ok(rx) => return Some(rx),
            Err(mcp251xfd::Error::RxFifoEmpty) => Timer::after_millis(1).await,
            Err(e) => {
                log::error!("recv on {fifo:?}: {e:?}");
                return None;
            }
        }
    }
    None
}

/// Applies `layout` and arms filter `F0` with `filter`, routing matches into
/// `Fifo::F2`, then returns the chip to internal loopback.
///
/// Goes through Configuration mode because [`MCP251xFdAsync::apply_layout`]
/// requires it, and re-applying the layout is also what drains the RX FIFO
/// (`FRESET`) so each filter case starts from empty.
#[allow(dead_code)] // not used by every binary that includes common.rs
pub async fn arm_filter(
    can: &mut MCP251xFdAsync<Device>,
    layout: &FifoLayout,
    filter: FilterMatch,
) -> Result<(), CanError> {
    can.set_mode(OperationMode::Configuration, &mut Delay)
        .await?;
    can.apply_layout(layout).await?;
    can.set_filter(Filter::F0, filter, Fifo::F2).await?;
    can.set_mode(OperationMode::InternalLoopback, &mut Delay)
        .await?;
    Ok(())
}

/// The blocking counterpart of [`Bus`].
///
/// Guarded by a critical-section mutex rather than [`NoopRawMutex`] because
/// `blocking_core1` moves the devices to the second core, which requires
/// them to be `Send`.
pub type BlockingBus =
    BlockingMutex<CriticalSectionRawMutex, RefCell<Spi<'static, SPI1, Blocking>>>;

/// The blocking counterpart of [`Device`].
pub type BlockingDevice = BlockingSpiDevice<
    'static,
    CriticalSectionRawMutex,
    Spi<'static, SPI1, Blocking>,
    Output<'static>,
>;

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
/// DMA_CH1 and raises no DMA completion interrupt at all -- which is the whole
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

/// Async bus guarded by a critical-section mutex.
///
/// [`Bus`] uses [`NoopRawMutex`], which is deliberately not `Sync`, so devices
/// built on it cannot be moved to the second core. The bench binaries that run
/// the **async** driver on core 1 -- the configuration the stall appears
/// under -- need `Send` devices, hence this parallel set.
pub type AsyncCsBus = Mutex<CriticalSectionRawMutex, Spi<'static, SPI1, Async>>;

/// The cross-core-capable counterpart of [`Device`].
pub type AsyncCsDevice =
    SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI1, Async>, Output<'static>>;

static ASYNC_CS_SPI_BUS: StaticCell<AsyncCsBus> = StaticCell::new();

/// Brings up the board with an **async** SPI1 whose devices are `Send`, and
/// hands back `CORE1` so the caller can start the second core.
///
/// Identical pins, clock and SPI mode to [`init_board`]; the only differences
/// are the mutex kind and the returned `CORE1`. Keeping DMA_CH0/DMA_CH1 wired
/// up is the point: this is the configuration whose DMA completions are
/// serviced on core 0 no matter which core issued the transfer.
///
/// Call this *or* [`init_board`] *or* [`init_board_blocking`], never more than
/// one: each calls `embassy_rp::init`.
#[allow(dead_code)]
pub fn init_board_async_cs() -> ([AsyncCsDevice; 10], UsbDriver, CORE1) {
    let p = embassy_rp::init(Default::default());

    let mut cfg = SpiConfig::default();
    cfg.frequency = mcp251xfd::max_spi_hz(CAN_CONFIG.clock.sysclk_hz());
    cfg.phase = Phase::CaptureOnFirstTransition;
    cfg.polarity = Polarity::IdleLow;

    let spi = Spi::new(
        p.SPI1, p.PIN_10, p.PIN_11, p.PIN_12, p.DMA_CH0, p.DMA_CH1, cfg,
    );
    let bus: &'static AsyncCsBus = ASYNC_CS_SPI_BUS.init(Mutex::new(spi));
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
    let devices = cs.map(|pin| SpiDevice::new(bus, Output::new(pin, Level::High)));

    (devices, Driver::new(p.USB, Irqs), p.CORE1)
}
