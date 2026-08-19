//! Register-level definitions for the MCP251XFD family.
//!
//! Pure data — no I/O. Bit layouts follow the MCP2518FD datasheet
//! (DS20006027B) and the MCP25XXFD Family Reference Manual (DS20005678E).

/// Register and RAM addresses (12-bit SPI address space).
pub mod addr {
    use super::{Fifo, Filter};

    /// CAN control register.
    pub const C1CON: u16 = 0x000;
    /// Nominal bit timing configuration.
    pub const C1NBTCFG: u16 = 0x004;
    /// Data bit timing configuration.
    pub const C1DBTCFG: u16 = 0x008;
    /// Transmitter delay compensation.
    pub const C1TDC: u16 = 0x00C;
    /// Free-running time base counter.
    pub const C1TBC: u16 = 0x010;
    /// Interrupt vector code register.
    pub const C1VEC: u16 = 0x018;
    /// Interrupt flags (low half) and enables (high half).
    pub const C1INT: u16 = 0x01C;
    /// Transmit/receive error counters and bus state.
    pub const C1TREC: u16 = 0x034;
    /// Oscillator control (MCP2000-class SFR block).
    pub const OSC: u16 = 0xE00;
    /// I/O pin control. Byte access only (family erratum: multi-byte
    /// writes spanning bytes 2-3 clear LAT0/LAT1).
    pub const IOCON: u16 = 0xE04;
    /// ECC control.
    pub const ECCCON: u16 = 0xE0C;
    /// First address of the 2 KiB message RAM.
    pub const RAM_START: u16 = 0x400;
    /// Message RAM size in bytes.
    pub const RAM_SIZE: usize = 2048;

    /// Address of `CiFIFOCONm` for the given FIFO.
    pub const fn fifo_con(fifo: Fifo) -> u16 {
        0x05C + 12 * (fifo.index() as u16 - 1)
    }
    /// Address of `CiFIFOSTAm` for the given FIFO.
    pub const fn fifo_sta(fifo: Fifo) -> u16 {
        fifo_con(fifo) + 4
    }
    /// Address of `CiFIFOUAm` for the given FIFO.
    pub const fn fifo_ua(fifo: Fifo) -> u16 {
        fifo_con(fifo) + 8
    }
    /// Byte address of the filter-control byte for the given filter
    /// (one byte per filter inside `CiFLTCON0..7`).
    pub const fn flt_con_byte(filter: Filter) -> u16 {
        0x1D0 + filter.index() as u16
    }
    /// Address of `CiFLTOBJm` for the given filter.
    pub const fn flt_obj(filter: Filter) -> u16 {
        0x1F0 + 8 * filter.index() as u16
    }
    /// Address of `CiMASKm` for the given filter.
    pub const fn flt_mask(filter: Filter) -> u16 {
        flt_obj(filter) + 4
    }
}

macro_rules! index_consts {
    ($ty:ident, $($name:ident = $n:literal),* $(,)?) => {
        $(
            #[doc = concat!(stringify!($ty), " number ", stringify!($n), ".")]
            pub const $name: Self = Self($n);
        )*
    };
}

/// One of the 31 general-purpose message FIFOs (`1..=31`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Fifo(u8);

impl Fifo {
    index_consts!(
        Fifo,
        F1 = 1,
        F2 = 2,
        F3 = 3,
        F4 = 4,
        F5 = 5,
        F6 = 6,
        F7 = 7,
        F8 = 8,
        F9 = 9,
        F10 = 10,
        F11 = 11,
        F12 = 12,
        F13 = 13,
        F14 = 14,
        F15 = 15,
        F16 = 16,
        F17 = 17,
        F18 = 18,
        F19 = 19,
        F20 = 20,
        F21 = 21,
        F22 = 22,
        F23 = 23,
        F24 = 24,
        F25 = 25,
        F26 = 26,
        F27 = 27,
        F28 = 28,
        F29 = 29,
        F30 = 30,
        F31 = 31,
    );

    /// Creates a FIFO handle. Returns `None` unless `1 <= n <= 31`.
    pub const fn new(n: u8) -> Option<Self> {
        if matches!(n, 1..=31) {
            Some(Self(n))
        } else {
            None
        }
    }

    /// The FIFO number (`1..=31`).
    pub const fn index(self) -> u8 {
        self.0
    }
}

/// One of the 32 acceptance filters (`0..=31`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Filter(u8);

impl Filter {
    index_consts!(
        Filter,
        F0 = 0,
        F1 = 1,
        F2 = 2,
        F3 = 3,
        F4 = 4,
        F5 = 5,
        F6 = 6,
        F7 = 7,
        F8 = 8,
        F9 = 9,
        F10 = 10,
        F11 = 11,
        F12 = 12,
        F13 = 13,
        F14 = 14,
        F15 = 15,
        F16 = 16,
        F17 = 17,
        F18 = 18,
        F19 = 19,
        F20 = 20,
        F21 = 21,
        F22 = 22,
        F23 = 23,
        F24 = 24,
        F25 = 25,
        F26 = 26,
        F27 = 27,
        F28 = 28,
        F29 = 29,
        F30 = 30,
        F31 = 31,
    );

    /// Creates a filter handle. Returns `None` unless `n <= 31`.
    pub const fn new(n: u8) -> Option<Self> {
        if n <= 31 { Some(Self(n)) } else { None }
    }

    /// The filter number (`0..=31`).
    pub const fn index(self) -> u8 {
        self.0
    }
}

/// Payload size of a FIFO element (CAN FD DLC steps).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PayloadSize {
    /// 8 bytes.
    B8,
    /// 12 bytes.
    B12,
    /// 16 bytes.
    B16,
    /// 20 bytes.
    B20,
    /// 24 bytes.
    B24,
    /// 32 bytes.
    B32,
    /// 48 bytes.
    B48,
    /// 64 bytes.
    B64,
}

impl PayloadSize {
    /// Payload size in bytes.
    pub const fn bytes(self) -> usize {
        match self {
            Self::B8 => 8,
            Self::B12 => 12,
            Self::B16 => 16,
            Self::B20 => 20,
            Self::B24 => 24,
            Self::B32 => 32,
            Self::B48 => 48,
            Self::B64 => 64,
        }
    }

    /// The 3-bit `PLSIZE` register encoding (`0..=7`).
    pub const fn plsize_code(self) -> u32 {
        match self {
            Self::B8 => 0,
            Self::B12 => 1,
            Self::B16 => 2,
            Self::B20 => 3,
            Self::B24 => 4,
            Self::B32 => 5,
            Self::B48 => 6,
            Self::B64 => 7,
        }
    }

    /// Inverse of [`Self::plsize_code`]. Codes above 7 saturate to `B64`.
    pub const fn from_code(code: u32) -> Self {
        match code {
            0 => Self::B8,
            1 => Self::B12,
            2 => Self::B16,
            3 => Self::B20,
            4 => Self::B24,
            5 => Self::B32,
            6 => Self::B48,
            _ => Self::B64,
        }
    }
}

/// CAN controller operation modes (`CiCON.OPMOD` / `REQOP`, 3 bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OperationMode {
    /// Mixed CAN FD / classic frames (mode 0).
    NormalFd,
    /// Sleep (mode 1).
    Sleep,
    /// Internal loopback — TX internally routed to RX, nothing on the bus
    /// (mode 2).
    InternalLoopback,
    /// Listen-only (mode 3).
    ListenOnly,
    /// Configuration mode — required for layout/timing changes (mode 4).
    Configuration,
    /// External loopback (mode 5).
    ExternalLoopback,
    /// Classic CAN 2.0 only; FD frames cause errors (mode 6).
    Normal20,
    /// Restricted operation (mode 7).
    RestrictedOperation,
}

impl OperationMode {
    /// The 3-bit register encoding.
    pub const fn bits(self) -> u8 {
        match self {
            Self::NormalFd => 0,
            Self::Sleep => 1,
            Self::InternalLoopback => 2,
            Self::ListenOnly => 3,
            Self::Configuration => 4,
            Self::ExternalLoopback => 5,
            Self::Normal20 => 6,
            Self::RestrictedOperation => 7,
        }
    }

    /// Decodes the 3-bit register encoding (only the low 3 bits are used).
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0b111 {
            0 => Self::NormalFd,
            1 => Self::Sleep,
            2 => Self::InternalLoopback,
            3 => Self::ListenOnly,
            4 => Self::Configuration,
            5 => Self::ExternalLoopback,
            6 => Self::Normal20,
            _ => Self::RestrictedOperation,
        }
    }
}

/// Detected chip variant (see the spec: detection via `OSC.LPMEN`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Variant {
    /// MCP2517FD: 7-bit TX sequence numbers, Sleep mode only.
    Mcp2517Fd,
    /// MCP2518FD or MCP251863 (same die): 23-bit sequence numbers, LPM.
    Mcp2518Fd,
}

impl Variant {
    /// Mask applied to TX sequence numbers for this variant.
    pub const fn seq_mask(self) -> u32 {
        match self {
            Self::Mcp2517Fd => 0x7F,
            Self::Mcp2518Fd => 0x7F_FFFF,
        }
    }
}

macro_rules! bit {
    ($get:ident, $set:ident, $bit:literal, $doc:literal) => {
        #[doc = concat!("Reads ", $doc, " (bit ", stringify!($bit), ").")]
        pub const fn $get(self) -> bool {
            self.0 & (1 << $bit) != 0
        }
        #[doc = concat!("Sets ", $doc, " (bit ", stringify!($bit), ").")]
        pub const fn $set(self, v: bool) -> Self {
            if v {
                Self(self.0 | (1 << $bit))
            } else {
                Self(self.0 & !(1 << $bit))
            }
        }
    };
}

/// `OSC` — oscillator control register (0xE00). Datasheet §5.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Osc(/** Raw 32-bit register value. */ pub u32);

impl Osc {
    bit!(
        pll_enabled,
        with_pll_enabled,
        0,
        "`PLLEN` (10x PLL from 4 MHz-class input)"
    );
    bit!(
        osc_disabled,
        with_osc_disabled,
        2,
        "`OSCDIS` (clock off, sleep)"
    );
    bit!(
        lpmen,
        with_lpmen,
        3,
        "`LPMEN` (Low-Power Mode; writable on MCP2518FD only)"
    );
    bit!(
        sclk_div2,
        with_sclk_div2,
        4,
        "`SCLKDIV` (divide SYSCLK by 2)"
    );
    bit!(pll_ready, with_pll_ready, 8, "`PLLRDY` (read-only status)");
    bit!(osc_ready, with_osc_ready, 10, "`OSCRDY` (read-only status)");
    bit!(
        sclk_ready,
        with_sclk_ready,
        12,
        "`SCLKRDY` (read-only status)"
    );

    /// Sets `CLKODIV` (bits 6:5): CLKO pin divider code (0=/1, 1=/2, 2=/4,
    /// 3=/10 — the power-on default).
    pub const fn with_clko_div(self, code: u32) -> Self {
        Self((self.0 & !(0b11 << 5)) | ((code & 0b11) << 5))
    }
}

/// `CiCON` — CAN control register (0x000). Datasheet §3.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiCon(/** Raw 32-bit register value. */ pub u32);

impl CiCon {
    bit!(
        iso_crc_enabled,
        with_iso_crc_enabled,
        5,
        "`ISOCRCEN` (ISO 11898-1:2015 CRC)"
    );
    bit!(
        protocol_exception_disabled,
        with_protocol_exception_disabled,
        6,
        "`PXEDIS`"
    );
    bit!(
        brs_disabled,
        with_brs_disabled,
        12,
        "`BRSDIS` (ignore BRS on TX)"
    );
    bit!(
        restrict_retx,
        with_restrict_retx,
        16,
        "`RTXAT` (honor per-FIFO retransmission attempts)"
    );
    bit!(
        store_tef,
        with_store_tef,
        19,
        "`STEF` (transmit event FIFO enable)"
    );
    bit!(txq_enabled, with_txq_enabled, 20, "`TXQEN`");
    bit!(
        abort_all,
        with_abort_all,
        27,
        "`ABAT` (request abort of all pending TX)"
    );

    /// Current operation mode, `OPMOD` (read-only, bits 23:21).
    pub const fn op_mode(self) -> OperationMode {
        OperationMode::from_bits(((self.0 >> 21) & 0b111) as u8)
    }

    /// Sets the requested operation mode, `REQOP` (bits 26:24).
    pub const fn with_req_op_mode(self, mode: OperationMode) -> Self {
        Self((self.0 & !(0b111 << 24)) | ((mode.bits() as u32) << 24))
    }
}

/// `CiINT` — interrupt flags (bits 15:0) and enables (bits 31:16) (0x01C).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiInt(/** Raw 32-bit register value. */ pub u32);

impl CiInt {
    bit!(txif, with_txif, 0, "`TXIF` (any TX FIFO interrupt pending)");
    bit!(rxif, with_rxif, 1, "`RXIF` (any RX FIFO interrupt pending)");
    bit!(modif, with_modif, 3, "`MODIF` (operation mode changed)");
    bit!(tefif, with_tefif, 4, "`TEFIF`");
    bit!(eccif, with_eccif, 8, "`ECCIF`");
    bit!(spicrcif, with_spicrcif, 9, "`SPICRCIF`");
    bit!(txatif, with_txatif, 10, "`TXATIF` (TX attempts exhausted)");
    bit!(rxovif, with_rxovif, 11, "`RXOVIF` (RX FIFO overflow)");
    bit!(serrif, with_serrif, 12, "`SERRIF` (system error)");
    bit!(cerrif, with_cerrif, 13, "`CERRIF` (CAN bus error)");
    bit!(ivmif, with_ivmif, 15, "`IVMIF` (invalid message)");
    bit!(txie, with_txie, 16, "`TXIE` enable");
    bit!(rxie, with_rxie, 17, "`RXIE` enable");
    bit!(modie, with_modie, 19, "`MODIE` enable");
    bit!(txatie, with_txatie, 26, "`TXATIE` enable");
    bit!(rxovie, with_rxovie, 27, "`RXOVIE` enable");
    bit!(serrie, with_serrie, 28, "`SERRIE` enable");
    bit!(cerrie, with_cerrie, 29, "`CERRIE` enable");
    bit!(ivmie, with_ivmie, 31, "`IVMIE` enable");
}

/// `CiVEC` — interrupt vector codes (0x018).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiVec(/** Raw 32-bit register value. */ pub u32);

impl CiVec {
    /// `ICODE` (bits 6:0): highest-priority pending interrupt code.
    pub const fn icode(self) -> u8 {
        (self.0 & 0x7F) as u8
    }
    /// `RXCODE` (bits 30:24): highest-priority pending RX FIFO code.
    pub const fn rxcode(self) -> u8 {
        ((self.0 >> 24) & 0x7F) as u8
    }
    /// `TXCODE` (bits 22:16): highest-priority pending TX FIFO code.
    pub const fn txcode(self) -> u8 {
        ((self.0 >> 16) & 0x7F) as u8
    }
}

/// `CiTREC` — error counters and bus state (0x034).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiTrec(/** Raw 32-bit register value. */ pub u32);

impl CiTrec {
    /// Receive error counter (bits 7:0).
    pub const fn rec(self) -> u8 {
        (self.0 & 0xFF) as u8
    }
    /// Transmit error counter (bits 15:8).
    pub const fn tec(self) -> u8 {
        ((self.0 >> 8) & 0xFF) as u8
    }
    bit!(error_warning, with_error_warning, 16, "`EWARN`");
    bit!(rx_warning, with_rx_warning, 17, "`RXWARN`");
    bit!(tx_warning, with_tx_warning, 18, "`TXWARN`");
    bit!(rx_error_passive, with_rx_error_passive, 19, "`RXBP`");
    bit!(tx_error_passive, with_tx_error_passive, 20, "`TXBP`");
    bit!(tx_bus_off, with_tx_bus_off, 21, "`TXBO`");
}

/// `CiNBTCFG` — nominal bit timing (0x004). All fields stored as value − 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiNbtCfg(/** Raw 32-bit register value. */ pub u32);

impl CiNbtCfg {
    /// Builds the register from human values: `brp: 1..=256`,
    /// `tseg1: 2..=256`, `tseg2: 1..=128`, `sjw: 1..=128` (time quanta).
    /// Callers validate ranges (see `config::NominalBitTiming::validate`).
    pub const fn new(brp: u16, tseg1: u16, tseg2: u16, sjw: u16) -> Self {
        Self(
            (((brp - 1) as u32) << 24)
                | (((tseg1 - 1) as u32) << 16)
                | (((tseg2 - 1) as u32) << 8)
                | ((sjw - 1) as u32),
        )
    }
}

/// `CiDBTCFG` — data bit timing (0x008). All fields stored as value − 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiDbtCfg(/** Raw 32-bit register value. */ pub u32);

impl CiDbtCfg {
    /// Builds the register from human values: `brp: 1..=256`,
    /// `tseg1: 1..=32`, `tseg2: 1..=16`, `sjw: 1..=16` (time quanta).
    pub const fn new(brp: u16, tseg1: u8, tseg2: u8, sjw: u8) -> Self {
        Self(
            (((brp - 1) as u32) << 24)
                | (((tseg1 - 1) as u32) << 16)
                | (((tseg2 - 1) as u32) << 8)
                | ((sjw - 1) as u32),
        )
    }
}

/// Transmitter delay compensation mode (`CiTDC.TDCMOD`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TdcMode {
    /// TDC disabled (code 0).
    Disabled,
    /// Manual: SSP = TDCV + TDCO (code 1).
    Manual,
    /// Auto: chip measures the loop delay; SSP = measured + TDCO (code 2).
    /// Recommended for data rates ≥ 1 Mbit/s.
    Auto,
}

/// `CiTDC` — transmitter delay compensation (0x00C).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiTdc(/** Raw 32-bit register value. */ pub u32);

impl CiTdc {
    /// Sets `TDCMOD` (bits 17:16).
    pub const fn with_mode(self, mode: TdcMode) -> Self {
        let code = match mode {
            TdcMode::Disabled => 0,
            TdcMode::Manual => 1,
            TdcMode::Auto => 2,
        };
        Self((self.0 & !(0b11 << 16)) | (code << 16))
    }
    /// Sets `TDCO` (bits 14:8): SSP offset in SYSCLK cycles,
    /// two's complement `-64..=63`.
    /// **Note:** The MCP2518FD datasheet's TDCO value table is internally inconsistent
    /// (its "1111111 = -64" row contradicts its own "two's complement" statement).
    /// This field uses standard 7-bit two's complement (0x7F = -1), matching the Emandhal
    /// C driver and the Linux mcp251xfd driver, preventing future "corrections" from
    /// breaking the encoding.
    pub const fn with_tdco(self, tdco: i8) -> Self {
        Self((self.0 & !(0x7F << 8)) | (((tdco as u32) & 0x7F) << 8))
    }
    bit!(
        edge_filter,
        with_edge_filter,
        25,
        "`EDGFLTEN` (edge filtering, recommended for FD)"
    );
}

/// `CiFIFOCONm` — FIFO control (0x05C + 12·(m−1)).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiFifoCon(/** Raw 32-bit register value. */ pub u32);

impl CiFifoCon {
    /// Value for byte 1 (bits 15:8) setting `UINC` only — used to pop an
    /// RX FIFO element without touching configuration bits.
    pub const CON_BYTE1_UINC: u8 = 0x01;
    /// Value for byte 1 setting `UINC | TXREQ` — used to queue and flush a
    /// TX FIFO element.
    pub const CON_BYTE1_UINC_TXREQ: u8 = 0x03;

    bit!(
        not_full_empty_ie,
        with_not_full_empty_ie,
        0,
        "`TFNRFNIE` (not-full/not-empty interrupt enable)"
    );
    bit!(rx_overflow_ie, with_rx_overflow_ie, 3, "`RXOVIE`");
    bit!(rx_timestamp, with_rx_timestamp, 5, "`RXTSEN`");
    bit!(
        tx,
        with_tx,
        7,
        "`TXEN` (1 = transmit FIFO, 0 = receive FIFO)"
    );
    bit!(
        uinc,
        with_uinc,
        8,
        "`UINC` (increment head/tail; write-only)"
    );
    bit!(
        txreq,
        with_txreq,
        9,
        "`TXREQ` (request transmission; TX FIFOs)"
    );
    bit!(freset, with_freset, 10, "`FRESET` (FIFO reset)");

    /// Sets `FSIZE` (bits 28:24): FIFO depth in messages, `1..=32`,
    /// stored as depth − 1.
    pub const fn with_fifo_size(self, depth: u8) -> Self {
        Self((self.0 & !(0x1F << 24)) | ((((depth - 1) as u32) & 0x1F) << 24))
    }
    /// Sets `PLSIZE` (bits 31:29).
    pub const fn with_payload_size(self, p: PayloadSize) -> Self {
        Self((self.0 & !(0b111 << 29)) | (p.plsize_code() << 29))
    }
}

/// `CiFIFOSTAm` — FIFO status (0x060 + 12·(m−1)).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CiFifoSta(/** Raw 32-bit register value. */ pub u32);

impl CiFifoSta {
    bit!(
        not_full_or_not_empty,
        with_not_full_or_not_empty,
        0,
        "`TFNRFNIF` (TX: not full / RX: not empty)"
    );
    bit!(
        rx_overflow,
        with_rx_overflow,
        3,
        "`RXOVIF` (RX FIFO overflowed; a message was lost)"
    );
    bit!(
        tx_attempts_exhausted,
        with_tx_attempts_exhausted,
        4,
        "`TXATIF`"
    );

    /// `FIFOCI` (bits 12:8): current FIFO message index. Subject to the
    /// MCP2517FD corrupt-read erratum — do not build protocol logic on it.
    pub const fn fifo_index(self) -> u8 {
        ((self.0 >> 8) & 0x1F) as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_addresses_match_datasheet() {
        // Reference manual: CiFIFOCONm = 0x05C + 12 * (m - 1).
        assert_eq!(addr::fifo_con(Fifo::F1), 0x05C);
        assert_eq!(addr::fifo_sta(Fifo::F1), 0x060);
        assert_eq!(addr::fifo_ua(Fifo::F1), 0x064);
        assert_eq!(addr::fifo_con(Fifo::F2), 0x068);
        assert_eq!(addr::fifo_con(Fifo::F31), 0x1C4);
    }

    #[test]
    fn filter_addresses_match_datasheet() {
        assert_eq!(addr::flt_con_byte(Filter::F0), 0x1D0);
        assert_eq!(addr::flt_con_byte(Filter::F31), 0x1EF);
        assert_eq!(addr::flt_obj(Filter::F0), 0x1F0);
        assert_eq!(addr::flt_mask(Filter::F0), 0x1F4);
        assert_eq!(addr::flt_obj(Filter::F31), 0x2E8);
    }

    #[test]
    fn osc_round_trip() {
        let osc = Osc(0).with_pll_enabled(true).with_lpmen(true);
        assert!(osc.pll_enabled() && osc.lpmen());
        assert_eq!(osc.0, (1 << 0) | (1 << 3));
        // Ready bits per datasheet: PLLRDY=8, OSCRDY=10, SCLKRDY=12.
        assert!(Osc(1 << 10).osc_ready());
        assert!(Osc(1 << 8).pll_ready());
        assert!(Osc(1 << 12).sclk_ready());
    }

    #[test]
    fn cicon_round_trip() {
        let con = CiCon(0)
            .with_iso_crc_enabled(true)
            .with_req_op_mode(OperationMode::Configuration);
        assert_eq!(con.0, (1 << 5) | (0b100 << 24));
        // OPMOD is read-only, bits 23:21.
        assert_eq!(CiCon(0b100 << 21).op_mode(), OperationMode::Configuration);
        assert_eq!(CiCon(0b000 << 21).op_mode(), OperationMode::NormalFd);
        assert_eq!(CiCon(0b110 << 21).op_mode(), OperationMode::Normal20);
    }

    #[test]
    fn ciint_flags() {
        // CiINT low half: TXIF=0, RXIF=1, MODIF=3, CERRIF=13, RXOVIF=11.
        let i = CiInt(1 << 1 | 1 << 3);
        assert!(i.rxif() && i.modif() && !i.txif());
        assert_eq!(CiInt(0).with_rxie(true).0, 1 << 17);
        assert_eq!(CiInt(0).with_txie(true).0, 1 << 16);
    }

    #[test]
    fn citrec_counters() {
        let t = CiTrec(0x0021_1503);
        assert_eq!(t.rec(), 0x03);
        assert_eq!(t.tec(), 0x15);
        assert!(t.tx_bus_off()); // TXBO = bit 21
    }

    #[test]
    fn payload_size_codes() {
        assert_eq!(PayloadSize::B8.bytes(), 8);
        assert_eq!(PayloadSize::B64.bytes(), 64);
        assert_eq!(PayloadSize::B8.plsize_code(), 0);
        assert_eq!(PayloadSize::B64.plsize_code(), 7);
        assert_eq!(PayloadSize::from_code(5), PayloadSize::B32);
    }

    #[test]
    fn nbtcfg_stores_minus_one() {
        // 40 MHz, 500 kbit/s, 80 TQ: brp=1, tseg1=63, tseg2=16, sjw=16.
        let r = CiNbtCfg::new(1, 63, 16, 16);
        // BRP bits 31:24, TSEG1 23:16, TSEG2 14:8, SJW 6:0 — all value-1.
        assert_eq!(r.0, 0x003E_0F0F); // (0 << 24) | (62 << 16) | (15 << 8) | 15
    }

    #[test]
    fn dbtcfg_stores_minus_one() {
        // 2 Mbit/s data phase, 20 TQ: brp=1, tseg1=15, tseg2=4, sjw=4.
        let r = CiDbtCfg::new(1, 15, 4, 4);
        assert_eq!(r.0, 0x000E_0303); // (0 << 24) | (14 << 16) | (3 << 8) | 3
    }

    #[test]
    fn tdc_auto_mode() {
        let r = CiTdc(0)
            .with_mode(TdcMode::Auto)
            .with_tdco(15)
            .with_edge_filter(true);
        // TDCMOD bits 17:16 = 0b10, TDCO bits 14:8 (two's complement), EDGFLTEN bit 25.
        assert_eq!(r.0, (0b10 << 16) | (15 << 8) | (1 << 25));
        // Negative TDCO is 7-bit two's complement.
        assert_eq!(CiTdc(0).with_tdco(-1).0, 0x7F << 8);
    }

    #[test]
    fn fifocon_fields() {
        let r = CiFifoCon(0)
            .with_tx(true)
            .with_fifo_size(4)
            .with_payload_size(PayloadSize::B64)
            .with_freset(true);
        assert_eq!(r.0, (1 << 7) | (1 << 10) | (3 << 24) | (7 << 29));
        let rx = CiFifoCon(0)
            .with_not_full_empty_ie(true)
            .with_rx_overflow_ie(true);
        assert_eq!(rx.0, 1 | (1 << 3));
    }

    #[test]
    fn fifosta_fields() {
        assert!(CiFifoSta(1).not_full_or_not_empty());
        assert!(CiFifoSta(1 << 3).rx_overflow());
        assert_eq!(CiFifoSta(0x0500).fifo_index(), 5);
    }
}
