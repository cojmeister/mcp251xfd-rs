//! Driver configuration: clocks, bit timing, filters.

use crate::error::ConfigError;
use crate::registers::objects::pack_id;
use crate::registers::{CiDbtCfg, CiNbtCfg};
use embedded_can::Id;

/// The maximum safe SPI clock for a given SYSCLK, per the MCP2517FD
/// erratum DS80000792 item 5 ("SPI writes/reads of the RAM can be
/// corrupted"): `0.85 * SYSCLK / 2`.
///
/// The driver cannot observe the actual SPI clock through `SpiDevice` — you
/// must configure your bus at or below this cap yourself. Per the erratum,
/// corruption only occurs with simultaneous CAN bus activity during a RAM
/// read, so the init-time RAM echo test (no bus traffic, Configuration
/// mode) only proves wiring and byte/word order; it cannot confirm erratum
/// compliance at your chosen SPI clock.
pub const fn max_spi_hz(sysclk_hz: u32) -> u32 {
    (sysclk_hz / 2 / 100) * 85
}

/// Oscillator configuration.
///
/// SYSCLK = `xtal_hz * 10` (if `pll`) `/ 2` (if `sclk_div2`); SYSCLK must
/// land in `2..=40 MHz`, and the PLL input must be a 4 MHz-class crystal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ClockConfig {
    /// Crystal / external clock frequency in Hz (`4..=40 MHz` crystal,
    /// `2..=40 MHz` external clock).
    pub xtal_hz: u32,
    /// Enable the 10x PLL (only from a 4 MHz-class input).
    pub pll: bool,
    /// Divide SYSCLK by 2 (`OSC.SCLKDIV`).
    pub sclk_div2: bool,
}

impl ClockConfig {
    /// 40 MHz crystal, no PLL — the recommended configuration.
    pub const MHZ40: Self = Self {
        xtal_hz: 40_000_000,
        pll: false,
        sclk_div2: false,
    };
    /// 20 MHz crystal, no PLL.
    pub const MHZ20: Self = Self {
        xtal_hz: 20_000_000,
        pll: false,
        sclk_div2: false,
    };
    /// 4 MHz crystal with 10x PLL -> 40 MHz SYSCLK (PLL adds lock time).
    pub const MHZ4_PLL: Self = Self {
        xtal_hz: 4_000_000,
        pll: true,
        sclk_div2: false,
    };

    /// The resulting SYSCLK in Hz.
    pub const fn sysclk_hz(&self) -> u32 {
        let base = if self.pll {
            self.xtal_hz * 10
        } else {
            self.xtal_hz
        };
        if self.sclk_div2 { base / 2 } else { base }
    }

    /// Checks PLL-input and SYSCLK range constraints.
    pub const fn validate(&self) -> Result<(), ConfigError> {
        if self.pll && self.xtal_hz > 5_000_000 {
            return Err(ConfigError::Clock);
        }
        let sysclk = self.sysclk_hz();
        if sysclk < 2_000_000 || sysclk > 40_000_000 {
            return Err(ConfigError::Clock);
        }
        Ok(())
    }
}

/// Nominal (arbitration-phase) bit timing in time quanta (TQ = BRP/SYSCLK).
///
/// Bit time = `(1 + tseg1 + tseg2) * TQ`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NominalBitTiming {
    /// Baud rate prescaler (`1..=256`).
    pub brp: u16,
    /// Time segment 1: propagation + phase 1 (`2..=256` TQ).
    pub tseg1: u16,
    /// Time segment 2: phase 2 (`1..=128` TQ).
    pub tseg2: u16,
    /// Synchronization jump width (`1..=128` TQ, `<= tseg2`).
    pub sjw: u16,
}

impl NominalBitTiming {
    /// 125 kbit/s at 40 MHz SYSCLK (160 TQ, 80% sample point).
    pub const KBPS125_40MHZ: Self = Self {
        brp: 2,
        tseg1: 127,
        tseg2: 32,
        sjw: 32,
    };
    /// 250 kbit/s at 40 MHz SYSCLK (160 TQ, 80% sample point).
    pub const KBPS250_40MHZ: Self = Self {
        brp: 1,
        tseg1: 127,
        tseg2: 32,
        sjw: 32,
    };
    /// 500 kbit/s at 40 MHz SYSCLK (80 TQ, 80% sample point).
    pub const KBPS500_40MHZ: Self = Self {
        brp: 1,
        tseg1: 63,
        tseg2: 16,
        sjw: 16,
    };
    /// 1 Mbit/s at 40 MHz SYSCLK (40 TQ, 80% sample point).
    pub const MBPS1_40MHZ: Self = Self {
        brp: 1,
        tseg1: 31,
        tseg2: 8,
        sjw: 8,
    };

    /// Range-checks all fields.
    pub const fn validate(&self) -> Result<(), ConfigError> {
        if self.brp < 1
            || self.brp > 256
            || self.tseg1 < 2
            || self.tseg1 > 256
            || self.tseg2 < 1
            || self.tseg2 > 128
            || self.sjw < 1
            || self.sjw > 128
            || self.sjw > self.tseg2
        {
            return Err(ConfigError::NominalBitTiming);
        }
        Ok(())
    }

    /// Encodes into `CiNBTCFG`. Call [`Self::validate`] first.
    pub const fn to_reg(&self) -> CiNbtCfg {
        CiNbtCfg::new(self.brp, self.tseg1, self.tseg2, self.sjw)
    }
}

/// Data-phase bit timing for CAN FD (BRS), in time quanta.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DataBitTiming {
    /// Baud rate prescaler (`1..=256`; keep equal to the nominal BRP,
    /// ideally 1, to minimize quantization error).
    pub brp: u16,
    /// Time segment 1 (`1..=32` TQ).
    pub tseg1: u8,
    /// Time segment 2 (`1..=16` TQ).
    pub tseg2: u8,
    /// Synchronization jump width (`1..=16` TQ, `<= tseg2`).
    pub sjw: u8,
}

impl DataBitTiming {
    /// 2 Mbit/s at 40 MHz SYSCLK (20 TQ, 80% sample point).
    pub const MBPS2_40MHZ: Self = Self {
        brp: 1,
        tseg1: 15,
        tseg2: 4,
        sjw: 4,
    };
    /// 5 Mbit/s at 40 MHz SYSCLK (8 TQ, 75% sample point).
    pub const MBPS5_40MHZ: Self = Self {
        brp: 1,
        tseg1: 5,
        tseg2: 2,
        sjw: 2,
    };
    /// 8 Mbit/s at 40 MHz SYSCLK (5 TQ, 80% sample point).
    pub const MBPS8_40MHZ: Self = Self {
        brp: 1,
        tseg1: 3,
        tseg2: 1,
        sjw: 1,
    };

    /// Range-checks all fields.
    pub const fn validate(&self) -> Result<(), ConfigError> {
        if self.brp < 1
            || self.brp > 256
            || self.tseg1 < 1
            || self.tseg1 > 32
            || self.tseg2 < 1
            || self.tseg2 > 16
            || self.sjw < 1
            || self.sjw > 16
            || self.sjw > self.tseg2
        {
            return Err(ConfigError::DataBitTiming);
        }
        Ok(())
    }

    /// Encodes into `CiDBTCFG`. Call [`Self::validate`] first.
    pub const fn to_reg(&self) -> CiDbtCfg {
        CiDbtCfg::new(self.brp, self.tseg1, self.tseg2, self.sjw)
    }

    /// Transmitter delay compensation offset for auto TDC mode:
    /// `DBRP * DTSEG1`, clamped to the 7-bit signed maximum of 63
    /// (SYSCLK cycles).
    ///
    /// This is the recipe worked out in the family reference manual
    /// (DS20005678E §3.4.8, Table 3-5: DBRP=1/DTSEG1=15 -> TDCO=15).
    /// **Note:** mainline Linux (`can_calc_tdco`, since ~v5.16) instead
    /// computes `DBRP * (1 + DTSEG1)` per ISO 11898-1 §11.3.3 — one
    /// `T_SYSCLK` later than the FRM's own example (16 vs. 15 here). Older
    /// Linux (≤ v5.15) matched the FRM. This function intentionally keeps
    /// the FRM's `DBRP * DTSEG1` formula and value, to prevent a future
    /// "correction" from silently changing the encoding.
    pub const fn tdco(&self) -> i8 {
        let v = self.brp as u32 * self.tseg1 as u32;
        if v > 63 { 63 } else { v as i8 }
    }
}

/// Complete controller configuration for [`init`](crate::MCP251xFd::init).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Oscillator configuration.
    pub clock: ClockConfig,
    /// Nominal bit timing.
    pub nominal: NominalBitTiming,
    /// Data-phase bit timing; `None` disables CAN FD bit rate switching.
    pub data: Option<DataBitTiming>,
}

impl Config {
    /// Validates every part of the configuration.
    pub const fn validate(&self) -> Result<(), ConfigError> {
        if let Err(e) = self.clock.validate() {
            return Err(e);
        }
        if let Err(e) = self.nominal.validate() {
            return Err(e);
        }
        if let Some(d) = self.data {
            if let Err(e) = d.validate() {
                return Err(e);
            }
        }
        Ok(())
    }
}

/// An acceptance filter: `CiFLTOBJ` value plus `CiMASK` value.
///
/// A received frame matches when `(frame_id ^ fltobj) & mask == 0`
/// (per-bit: mask 1 = must match, 0 = don't care). Bit 30 (`MIDE`) makes
/// the standard/extended distinction part of the match.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FilterMatch {
    /// Raw `CiFLTOBJm` value (packed ID + `EXIDE` bit 30).
    pub fltobj: u32,
    /// Raw `CiMASKm` value (packed ID mask + `MIDE` bit 30).
    pub mask: u32,
}

impl FilterMatch {
    const IDE_BIT: u32 = 1 << 30;

    /// Matches exactly one identifier (and its standard/extended kind).
    pub fn exact(id: Id) -> Self {
        let (obj, id_mask) = match id {
            Id::Standard(_) => (pack_id(id), 0x7FF),
            Id::Extended(_) => (pack_id(id) | Self::IDE_BIT, 0x1FFF_FFFF),
        };
        Self {
            fltobj: obj,
            mask: id_mask | Self::IDE_BIT,
        }
    }

    /// Matches every frame, standard and extended.
    pub fn accept_all() -> Self {
        Self { fltobj: 0, mask: 0 }
    }

    /// Matches `id` under a custom mask over the *natural* identifier bits
    /// (11-bit for standard, 29-bit for extended); the mask is packed into
    /// the register layout for you. The standard/extended kind always
    /// participates in the match (`MIDE` set).
    pub fn with_mask(id: Id, id_mask: u32) -> Self {
        let packed_mask = match id {
            Id::Standard(_) => id_mask & 0x7FF,
            Id::Extended(_) => ((id_mask >> 18) & 0x7FF) | ((id_mask & 0x3_FFFF) << 11),
        };
        let obj = match id {
            Id::Standard(_) => pack_id(id),
            Id::Extended(_) => pack_id(id) | Self::IDE_BIT,
        };
        Self {
            fltobj: obj,
            mask: packed_mask | Self::IDE_BIT,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_can::{ExtendedId, Id, StandardId};

    #[test]
    fn spi_cap() {
        assert_eq!(max_spi_hz(40_000_000), 17_000_000);
        assert_eq!(max_spi_hz(20_000_000), 8_500_000);
    }

    #[test]
    fn clock_sysclk() {
        assert_eq!(ClockConfig::MHZ40.sysclk_hz(), 40_000_000);
        assert_eq!(ClockConfig::MHZ4_PLL.sysclk_hz(), 40_000_000);
        let div = ClockConfig {
            xtal_hz: 40_000_000,
            pll: false,
            sclk_div2: true,
        };
        assert_eq!(div.sysclk_hz(), 20_000_000);
        // PLL only valid from a 4 MHz-class input.
        assert!(
            ClockConfig {
                xtal_hz: 40_000_000,
                pll: true,
                sclk_div2: false
            }
            .validate()
            .is_err()
        );
    }

    #[test]
    fn preset_bit_counts() {
        // Preset invariant: sysclk / (brp * (1 + tseg1 + tseg2)) = bit rate.
        for (p, rate) in [
            (NominalBitTiming::KBPS125_40MHZ, 125_000u32),
            (NominalBitTiming::KBPS250_40MHZ, 250_000),
            (NominalBitTiming::KBPS500_40MHZ, 500_000),
            (NominalBitTiming::MBPS1_40MHZ, 1_000_000),
        ] {
            let tq = 1 + p.tseg1 as u32 + p.tseg2 as u32;
            assert_eq!(40_000_000 / (p.brp as u32 * tq), rate);
            assert!(p.validate().is_ok());
            assert!(p.sjw <= p.tseg2);
        }
        for (p, rate) in [
            (DataBitTiming::MBPS2_40MHZ, 2_000_000u32),
            (DataBitTiming::MBPS5_40MHZ, 5_000_000),
            (DataBitTiming::MBPS8_40MHZ, 8_000_000),
        ] {
            let tq = 1 + p.tseg1 as u32 + p.tseg2 as u32;
            assert_eq!(40_000_000 / (p.brp as u32 * tq), rate);
            assert!(p.validate().is_ok());
        }
        // Pin the exact register words (FRM DS20005678E Table 3-5) so a
        // future preset or `to_reg` change can't silently drift.
        assert_eq!(NominalBitTiming::KBPS500_40MHZ.to_reg().0, 0x003E_0F0F);
        assert_eq!(DataBitTiming::MBPS2_40MHZ.to_reg().0, 0x000E_0303);
    }

    #[test]
    fn timing_validation() {
        let mut t = NominalBitTiming::KBPS500_40MHZ;
        t.brp = 0;
        assert!(t.validate().is_err());
        t = NominalBitTiming::KBPS500_40MHZ;
        t.sjw = t.tseg2 + 1;
        assert!(t.validate().is_err());
        let mut d = DataBitTiming::MBPS2_40MHZ;
        d.tseg1 = 33;
        assert!(d.validate().is_err());
    }

    #[test]
    fn tdco_follows_recipe() {
        // TDCO = DBRP * DTSEG1, clamped to 63 (7-bit signed max).
        assert_eq!(DataBitTiming::MBPS2_40MHZ.tdco(), 15);
        let big = DataBitTiming {
            brp: 16,
            tseg1: 16,
            tseg2: 4,
            sjw: 4,
        };
        assert_eq!(big.tdco(), 63);
    }

    #[test]
    fn filter_match() {
        let id = Id::Standard(StandardId::new(0x123).unwrap());
        let m = FilterMatch::exact(id);
        assert_eq!(m.fltobj, 0x123);
        // Mask: all 11 SID bits + MIDE (bit 30).
        assert_eq!(m.mask, 0x7FF | (1 << 30));
        let eid = Id::Extended(ExtendedId::new(0x0CFE_6E01).unwrap());
        let me = FilterMatch::exact(eid);
        assert_eq!(me.fltobj & (1 << 30), 1 << 30); // EXIDE set
        assert_eq!(me.mask, 0x1FFF_FFFF | (1 << 30));
        assert_eq!(FilterMatch::accept_all().mask, 0);
    }
}
