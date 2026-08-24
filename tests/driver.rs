//! Byte-exact integration tests for the sync driver against a mock SPI bus.

use embedded_can::Frame as _;
use embedded_can::{Id, StandardId};
use embedded_hal_mock::eh1::delay::NoopDelay;
use embedded_hal_mock::eh1::spi::{Mock, Transaction};
use mcp251xfd::{
    CiInt, ClockConfig, Config, DataBitTiming, Error, Event, FdFrame, Fifo, FifoLayout, Filter,
    FilterMatch, Frame, FrameFlags, MCP251xFd, NominalBitTiming, OperationMode, PayloadSize,
    ReceivedFrame, Variant,
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
fn set_mode_times_out_when_chip_never_switches() {
    let mut e = Vec::new();
    e.extend(r32(0x000, 0x0480_0020)); // OPMOD=Config, REQOP=Config, ISOCRC
    e.extend(w32(0x000, 0x0080_0020)); // REQOP := NormalFd (0)
    for _ in 0..80 {
        e.extend(r32(0x000, 0x0080_0020)); // OPMOD stays Config; never switches
    }
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(matches!(
        can.set_mode(OperationMode::NormalFd, &mut NoopDelay),
        Err(Error::ModeChangeTimeout)
    ));
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

#[test]
fn disable_filter_writes_zero_byte() {
    let mut e = Vec::new();
    e.extend(w8(0x1D0, 0x00));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.disable_filter(Filter::F0).unwrap();
    spi.done();
}

#[test]
fn transmit_classic_frame() {
    let frame = Frame::new(StandardId::new(0x123).unwrap(), &[1, 2, 3, 4]).unwrap();
    let mut e = Vec::new();
    // First transmit: seq 0.
    e.extend(r32(0x060, 0x0000_0001)); // CiFIFOSTA1: not full
    e.extend(r32(0x064, 0x0000_0000)); // CiFIFOUA1: offset 0
    e.extend(wram(
        0x400,
        &[
            0x23, 0x01, 0x00, 0x00, // T0: SID 0x123
            0x04, 0x00, 0x00, 0x00, // T1: DLC 4, SEQ 0
            0x01, 0x02, 0x03, 0x04, // payload
        ],
    ));
    e.extend(w8(0x05D, 0x03)); // CiFIFOCON1 byte1: UINC | TXREQ
    // Second transmit: seq increments, chip UA advanced to 0x10.
    e.extend(r32(0x060, 0x0000_0001));
    e.extend(r32(0x064, 0x0000_0010));
    e.extend(wram(
        0x410,
        &[
            0x23, 0x01, 0x00, 0x00, 0x04, 0x02, 0x00, 0x00, // T1: DLC 4, SEQ 1 (1 << 9)
            0x01, 0x02, 0x03, 0x04,
        ],
    ));
    e.extend(w8(0x05D, 0x03));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.transmit(Fifo::F1, &frame).unwrap();
    can.transmit(Fifo::F1, &frame).unwrap();
    spi.done();
}

#[test]
fn transmit_full_fifo_errors() {
    let frame = Frame::new(StandardId::new(0x123).unwrap(), &[]).unwrap();
    let mut e = Vec::new();
    e.extend(r32(0x060, 0x0000_0000)); // full: TFNRFNIF clear
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(matches!(
        can.transmit(Fifo::F1, &frame),
        Err(Error::TxFifoFull)
    ));
    spi.done();
}

#[test]
fn transmit_fd_frame_with_brs() {
    // 12-byte payload -> DLC 9; FDF | BRS set; payload already word-aligned.
    let frame = FdFrame::new(
        StandardId::new(0x7F).unwrap(),
        &[
            0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB,
        ],
        FrameFlags {
            brs: true,
            esi: false,
        },
    )
    .unwrap();
    let mut e = Vec::new();
    e.extend(r32(0x060, 0x0000_0001));
    e.extend(r32(0x064, 0x0000_0000));
    // T1 = DLC 9 | BRS(1<<6) | FDF(1<<7) | SEQ 0 = 0xC9.
    let mut obj = vec![0x7F, 0x00, 0x00, 0x00, 0xC9, 0x00, 0x00, 0x00];
    obj.extend_from_slice(frame.data());
    e.extend(wram(0x400, &obj));
    e.extend(w8(0x05D, 0x03));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    can.transmit_fd(Fifo::F1, &frame).unwrap();
    spi.done();
}

#[test]
fn transmit_rejects_out_of_range_ua() {
    // FIFO not full, but CiFIFOUA reads back >= message RAM size (0x800):
    // implausible (chip possibly still in Configuration mode). No RAM or
    // CiFIFOCON traffic should follow.
    let frame = Frame::new(StandardId::new(0x123).unwrap(), &[1, 2, 3, 4]).unwrap();
    let mut e = Vec::new();
    e.extend(r32(0x060, 0x0000_0001)); // not full
    e.extend(r32(0x064, 0x0000_0800)); // UA == RAM_SIZE: out of range
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(matches!(
        can.transmit(Fifo::F1, &frame),
        Err(Error::CommunicationCheckFailed)
    ));
    spi.done();
}

#[test]
fn transmit_rejects_object_running_past_end_of_ram() {
    // A 64-byte FD frame writes 8 + 64 = 72 bytes. CiFIFOUA reads back
    // 0x7C0, which is inside message RAM (0x800 bytes) but only 0x40 bytes
    // from its end — the object would run to 0x808. That means this FIFO's
    // PLSIZE is smaller than the frame; refuse before touching RAM.
    let frame = FdFrame::new(
        StandardId::new(0x123).unwrap(),
        &[0x5A; 64],
        FrameFlags::default(),
    )
    .unwrap();
    let mut e = Vec::new();
    e.extend(r32(0x060, 0x0000_0001)); // not full
    e.extend(r32(0x064, 0x0000_07C0)); // UA + 72 > RAM_SIZE
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(matches!(
        can.transmit_fd(Fifo::F1, &frame),
        Err(Error::CommunicationCheckFailed)
    ));
    // `Mock::done` fails if any further transaction (the RAM write, the
    // CiFIFOCON byte) had been issued.
    spi.done();
}

#[test]
fn receive_zeroes_stale_padding_bytes() {
    // DLC 5 reads a 8-byte-padded word run; bytes 5..8 belong to whatever
    // occupied this RAM slot before and must not reach the frame, or the
    // derived PartialEq/Debug compare and print bus leftovers.
    let mut e = Vec::new();
    e.extend(r32(0x06C, 0x0000_0001)); // CiFIFOSTA2: not empty
    e.extend(r32(0x070, 0x0000_0020)); // CiFIFOUA2: offset 0x20
    e.extend(rram(
        0x420,
        &[0x23, 0x01, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00],
    )); // R0: SID 0x123, R1: DLC 5
    e.extend(rram(
        0x428,
        &[0x01, 0x02, 0x03, 0x04, 0x05, 0xDE, 0xAD, 0xBE],
    )); // 5 payload bytes + 3 stale bytes of the previous occupant
    e.extend(w8(0x069, 0x01)); // CiFIFOCON2 byte1: UINC
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    let rx = can.receive(Fifo::F2).unwrap();
    match rx.frame {
        ReceivedFrame::Classic(f) => {
            assert_eq!(f.data(), &[1, 2, 3, 4, 5]);
            let expected = Frame::new(StandardId::new(0x123).unwrap(), &[1, 2, 3, 4, 5]).unwrap();
            assert_eq!(f, expected);
        }
        ReceivedFrame::Fd(_) => panic!("expected classic frame"),
    }
    spi.done();
}

#[test]
fn receive_remote_frame_skips_payload_and_stays_zeroed() {
    // A classic RTR frame carries no data bytes on the wire, so the payload
    // slot holds the RAM element's previous occupant. The driver must not
    // read it at all: the expectation list has no payload transaction, so
    // Mock::done() fails if one is issued, and the frame must equal
    // Frame::new_remote's all-zero-array construction.
    let mut e = Vec::new();
    e.extend(r32(0x06C, 0x0000_0001)); // CiFIFOSTA2: not empty
    e.extend(r32(0x070, 0x0000_0020)); // CiFIFOUA2: offset 0x20
    e.extend(rram(
        0x420,
        &[0x23, 0x01, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00],
    )); // R0: SID 0x123, R1: DLC 4 | RTR (bit 5)
    e.extend(w8(0x069, 0x01)); // CiFIFOCON2 byte1: UINC
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    let rx = can.receive(Fifo::F2).unwrap();
    match rx.frame {
        ReceivedFrame::Classic(f) => {
            assert!(f.is_remote_frame());
            assert_eq!(f.dlc(), 4);
            let expected = Frame::new_remote(StandardId::new(0x123).unwrap(), 4).unwrap();
            assert_eq!(f, expected);
        }
        ReceivedFrame::Fd(_) => panic!("expected classic frame"),
    }
    spi.done();
}

#[test]
fn receive_classic_frame() {
    let mut e = Vec::new();
    e.extend(r32(0x06C, 0x0000_0001)); // CiFIFOSTA2: not empty
    e.extend(r32(0x070, 0x0000_00A0)); // CiFIFOUA2: offset 0xA0
    e.extend(rram(
        0x4A0,
        &[0x23, 0x01, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00],
    )); // R0, R1
    e.extend(rram(0x4A8, &[0x01, 0x02, 0x03, 0x04])); // payload
    e.extend(w8(0x069, 0x01)); // CiFIFOCON2 byte1: UINC
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    let rx = can.receive(Fifo::F2).unwrap();
    match rx.frame {
        ReceivedFrame::Classic(f) => {
            assert_eq!(f.id(), Id::Standard(StandardId::new(0x123).unwrap()));
            assert_eq!(f.data(), &[1, 2, 3, 4]);
        }
        ReceivedFrame::Fd(_) => panic!("expected classic frame"),
    }
    assert_eq!(rx.timestamp, None);
    spi.done();
}

#[test]
fn receive_fd_frame() {
    let mut e = Vec::new();
    e.extend(r32(0x06C, 0x0000_0001));
    e.extend(r32(0x070, 0x0000_0000));
    // R1 = DLC 9 | BRS(1<<6) | FDF(1<<7) = 0xC9 -> 12 payload bytes.
    e.extend(rram(
        0x400,
        &[0x7F, 0x00, 0x00, 0x00, 0xC9, 0x00, 0x00, 0x00],
    ));
    e.extend(rram(
        0x408,
        &[
            0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB,
        ],
    ));
    e.extend(w8(0x069, 0x01));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    let rx = can.receive(Fifo::F2).unwrap();
    match rx.frame {
        ReceivedFrame::Fd(f) => {
            assert_eq!(f.data().len(), 12);
            assert_eq!(f.data()[11], 0xAB);
            assert!(f.flags().brs);
        }
        ReceivedFrame::Classic(_) => panic!("expected FD frame"),
    }
    spi.done();
}

#[test]
fn receive_empty_fifo_errors() {
    let mut e = Vec::new();
    e.extend(r32(0x06C, 0x0000_0000));
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(matches!(can.receive(Fifo::F2), Err(Error::RxFifoEmpty)));
    spi.done();
}

#[test]
fn receive_rejects_out_of_range_ua() {
    // FIFO not empty, but CiFIFOUA reads back >= message RAM size (0x800):
    // implausible. No RAM or CiFIFOCON traffic should follow.
    let mut e = Vec::new();
    e.extend(r32(0x06C, 0x0000_0001)); // not empty
    e.extend(r32(0x070, 0x0000_0800)); // UA == RAM_SIZE: out of range
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(matches!(
        can.receive(Fifo::F2),
        Err(Error::CommunicationCheckFailed)
    ));
    spi.done();
}

#[test]
fn receive_rejects_ua_too_close_to_ram_top_for_a_header() {
    // Every RX object opens with an 8-byte header, so the last UA that can
    // hold one is RAM_SIZE - 8 = 0x7F8. At 0x7F9 the header read alone would
    // run past the end of message RAM; reject before issuing it.
    let mut e = Vec::new();
    e.extend(r32(0x06C, 0x0000_0001)); // not empty
    e.extend(r32(0x070, 0x0000_07F9)); // UA + 8 > RAM_SIZE by one byte
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    assert!(matches!(
        can.receive(Fifo::F2),
        Err(Error::CommunicationCheckFailed)
    ));
    // `Mock::done` fails if the 8-byte header read had been issued.
    spi.done();
}

#[test]
fn interrupts_and_events() {
    let mut e = Vec::new();
    e.extend(r32(0x01C, 0x0000_0002)); // RXIF
    e.extend(w8(0x01C, 0xFD)); // clear_interrupts byte0: bit 1 (RXIF) written 0 but
    // read-only (ignored by hardware), rest written 1 (leave as-is)
    e.extend(w8(0x01D, 0xFF)); // byte1: all bits written 1 (leave as-is)
    e.extend(w8(0x01E, 0x02)); // enables: RXIE (bit 17 -> byte2 bit 1)
    e.extend(w8(0x01F, 0x00));
    e.extend(r32(0x018, 0x0000_0002)); // ICODE = 2 -> FIFO 2
    e.extend(r32(0x018, 0x0000_0040)); // ICODE = 0x40 -> none
    e.extend(r32(0x034, 0x0021_1503)); // TREC
    let mut spi = Mock::new(&e);
    let mut can = MCP251xFd::new(&mut spi);
    let flags = can.interrupt_flags().unwrap();
    assert!(flags.rxif());
    can.clear_interrupts(flags).unwrap();
    can.configure_interrupts(CiInt(0).with_rxie(true)).unwrap();
    assert_eq!(can.pending_event().unwrap(), Event::Fifo(Fifo::F2));
    assert_eq!(can.pending_event().unwrap(), Event::None);
    let trec = can.error_counters().unwrap();
    assert_eq!(trec.tec(), 0x15);
    assert!(trec.tx_bus_off());
    spi.done();
}
