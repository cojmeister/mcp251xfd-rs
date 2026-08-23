//! Byte-exact integration tests for the sync driver against a mock SPI bus.

use embedded_can::{Id, StandardId};
use embedded_hal_mock::eh1::delay::NoopDelay;
use embedded_hal_mock::eh1::spi::{Mock, Transaction};
use mcp251xfd::{
    ClockConfig, Config, DataBitTiming, Error, Fifo, FifoLayout, Filter, FilterMatch, MCP251xFd,
    NominalBitTiming, OperationMode, PayloadSize, Variant,
};

/// One WRITE-register transaction: command word + 4 LE data bytes.
fn w32(addr: u16, val: u32) -> Vec<Transaction<u8>> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x20 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::write_vec(val.to_le_bytes().to_vec()),
        Transaction::transaction_end(),
    ]
}

/// One READ-register transaction returning `val`.
fn r32(addr: u16, val: u32) -> Vec<Transaction<u8>> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x30 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::read_vec(val.to_le_bytes().to_vec()),
        Transaction::transaction_end(),
    ]
}

fn wram(addr: u16, data: &[u8]) -> Vec<Transaction<u8>> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x20 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::write_vec(data.to_vec()),
        Transaction::transaction_end(),
    ]
}

fn rram(addr: u16, data: &[u8]) -> Vec<Transaction<u8>> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x30 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::read_vec(data.to_vec()),
        Transaction::transaction_end(),
    ]
}

/// One byte-wise WRITE transaction: command word + 1 data byte.
fn w8(addr: u16, val: u8) -> Vec<Transaction<u8>> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x20 | (addr >> 8) as u8, (addr & 0xFF) as u8]),
        Transaction::write_vec(vec![val]),
        Transaction::transaction_end(),
    ]
}

fn reset_txn() -> Vec<Transaction<u8>> {
    vec![
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0x00, 0x00]),
        Transaction::transaction_end(),
    ]
}

pub const TEST_CONFIG: Config = Config {
    clock: ClockConfig::MHZ40,
    nominal: NominalBitTiming::KBPS500_40MHZ,
    data: Some(DataBitTiming::MBPS2_40MHZ),
};

/// The full expected init sequence for `TEST_CONFIG` on an MCP2517FD
/// (LPMEN does not stick), with the oscillator ready on the first poll.
fn init_expectations() -> Vec<Transaction<u8>> {
    let mut e = Vec::new();
    e.extend(reset_txn());
    e.extend(wram(0xBFC, &0xAA55_AA55u32.to_le_bytes()));
    e.extend(rram(0xBFC, &0xAA55_AA55u32.to_le_bytes()));
    e.extend(w32(0xE00, 0x0000_0060)); // OSC: CLKODIV=0b11 (default), no PLL
    e.extend(r32(0xE00, 0x0000_0460)); // OSCRDY set
    e.extend(w32(0xE00, 0x0000_0068)); // probe LPMEN
    e.extend(r32(0xE00, 0x0000_0460)); // LPMEN did not stick -> MCP2517FD
    e.extend(w32(0xE00, 0x0000_0060)); // clear LPMEN
    for i in 0..32u16 {
        e.extend(wram(0x400 + i * 64, &[0u8; 64]));
    }
    e.extend(w32(0x004, 0x003E_0F0F)); // NBTCFG: brp1, tseg1 63, tseg2 16, sjw 16
    e.extend(w32(0x008, 0x000E_0303)); // DBTCFG: brp1, tseg1 15, tseg2 4, sjw 4
    e.extend(w32(0x00C, 0x0202_0F00)); // TDC: auto, TDCO=15, edge filter
    e.extend(w32(0x000, 0x0400_0020)); // CiCON: ISOCRCEN, REQOP=Configuration
    e.extend(w32(0x01C, 0x0000_0000)); // CiINT cleared, all disabled
    e
}

#[test]
fn init_full_sequence_detects_2517fd() {
    let mut spi = Mock::new(&init_expectations());
    let mut can = MCP251xFd::new(&mut spi);
    let variant = can.init(&TEST_CONFIG, &mut NoopDelay).unwrap();
    assert_eq!(variant, Variant::Mcp2517Fd);
    spi.done();
}

#[test]
fn init_detects_2518fd_when_lpmen_sticks() {
    let mut e = Vec::new();
    e.extend(reset_txn());
    e.extend(wram(0xBFC, &0xAA55_AA55u32.to_le_bytes()));
    e.extend(rram(0xBFC, &0xAA55_AA55u32.to_le_bytes()));
    e.extend(w32(0xE00, 0x0000_0060));
    e.extend(r32(0xE00, 0x0000_0460));
    e.extend(w32(0xE00, 0x0000_0068));
    e.extend(r32(0xE00, 0x0000_0468)); // LPMEN stuck -> MCP2518FD
    e.extend(w32(0xE00, 0x0000_0060));
    for i in 0..32u16 {
        e.extend(wram(0x400 + i * 64, &[0u8; 64]));
    }
    e.extend(w32(0x004, 0x003E_0F0F));
    e.extend(w32(0x008, 0x000E_0303));
    e.extend(w32(0x00C, 0x0202_0F00));
    e.extend(w32(0x000, 0x0400_0020));
    e.extend(w32(0x01C, 0x0000_0000));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert_eq!(
        can.init(&TEST_CONFIG, &mut NoopDelay).unwrap(),
        Variant::Mcp2518Fd
    );
    spi.done();
}

#[test]
fn init_fails_on_bad_echo() {
    let mut e = Vec::new();
    e.extend(reset_txn());
    e.extend(wram(0xBFC, &0xAA55_AA55u32.to_le_bytes()));
    e.extend(rram(0xBFC, &[0x00, 0x00, 0x00, 0x00])); // dead bus
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(matches!(
        can.init(&TEST_CONFIG, &mut NoopDelay),
        Err(Error::CommunicationCheckFailed)
    ));
    spi.done();
}

#[test]
fn set_mode_normal_fd() {
    let mut e = Vec::new();
    e.extend(r32(0x000, 0x0480_0020)); // OPMOD=Config, REQOP=Config, ISOCRC
    e.extend(w32(0x000, 0x0080_0020)); // REQOP := NormalFd (0)
    e.extend(r32(0x000, 0x0000_0020)); // OPMOD now NormalFd
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.set_mode(OperationMode::NormalFd, &mut NoopDelay)
        .unwrap();
    spi.done();
}

#[test]
fn apply_layout_writes_fifo_configs() {
    const LAYOUT: FifoLayout = FifoLayout::new()
        .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
        .rx_fifo(Fifo::F2, PayloadSize::B64, 8);
    let mut e = Vec::new();
    e.extend(r32(0x000, 0x0080_0000)); // OPMOD=Config
    // TX: PLSIZE=7<<29 | FSIZE=3<<24 | FRESET | TXEN.
    e.extend(w32(0x05C, 0xE300_0480));
    // RX: PLSIZE=7<<29 | FSIZE=7<<24 | FRESET | RXOVIE | TFNRFNIE.
    e.extend(w32(0x068, 0xE700_0409));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.apply_layout(&LAYOUT).unwrap();
    spi.done();
}

#[test]
fn apply_layout_requires_config_mode() {
    let mut e = Vec::new();
    e.extend(r32(0x000, 0x0000_0000)); // OPMOD=NormalFd
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    let layout = FifoLayout::new().rx_fifo(Fifo::F1, PayloadSize::B8, 1);
    assert!(matches!(
        can.apply_layout(&layout),
        Err(Error::NotInConfigMode)
    ));
    spi.done();
}

#[test]
fn set_filter_exact_standard_id() {
    let id = Id::Standard(StandardId::new(0x123).unwrap());
    let mut e = Vec::new();
    e.extend(w8(0x1D0, 0x00)); // disable filter 0 while editing
    e.extend(w32(0x1F0, 0x0000_0123)); // FLTOBJ
    e.extend(w32(0x1F4, 0x4000_07FF)); // MASK: SID bits + MIDE
    e.extend(w8(0x1D0, 0x82)); // enable, point to FIFO 2
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.set_filter(Filter::F0, FilterMatch::exact(id), Fifo::F2)
        .unwrap();
    spi.done();
}
