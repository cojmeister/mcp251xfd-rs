//! Encoding and decoding of message objects in controller RAM.
//!
//! TX objects are two header words (T0, T1) followed by the payload;
//! RX objects mirror this (R0, R1). Family reference manual §4.

use embedded_can::{ExtendedId, Id, StandardId};

/// Converts a DLC code (`0..=15`) to a payload length in bytes.
/// For classic frames (`fdf == false`), DLC values above 8 mean 8 bytes.
pub const fn dlc_to_len(dlc: u8, fdf: bool) -> usize {
    match dlc {
        0..=8 => dlc as usize,
        9 if fdf => 12,
        10 if fdf => 16,
        11 if fdf => 20,
        12 if fdf => 24,
        13 if fdf => 32,
        14 if fdf => 48,
        15 if fdf => 64,
        _ => 8, // classic frame with DLC 9..=15
    }
}

/// Converts an exact payload length to its DLC code; `None` if the length
/// is not directly representable (e.g. 10 bytes FD, or > 8 bytes classic).
pub const fn len_to_dlc(len: usize, fdf: bool) -> Option<u8> {
    match len {
        0..=8 => Some(len as u8),
        12 if fdf => Some(9),
        16 if fdf => Some(10),
        20 if fdf => Some(11),
        24 if fdf => Some(12),
        32 if fdf => Some(13),
        48 if fdf => Some(14),
        64 if fdf => Some(15),
        _ => None,
    }
}

/// Rounds a length up to the next valid CAN FD payload length
/// (`0..=8, 12, 16, 20, 24, 32, 48, 64`). `None` if `len > 64`.
pub const fn padded_dlc_len(len: usize) -> Option<usize> {
    match len {
        0..=8 => Some(len),
        9..=12 => Some(12),
        13..=16 => Some(16),
        17..=20 => Some(20),
        21..=24 => Some(24),
        25..=32 => Some(32),
        33..=48 => Some(48),
        49..=64 => Some(64),
        _ => None,
    }
}

/// Packs a CAN ID into the T0/R0/FLTOBJ layout: `SID` in bits 10:0 and,
/// for extended IDs, `EID` (ID bits 17:0) in bits 28:11 with the base ID
/// (bits 28:18 of the 29-bit ID) in the `SID` field.
pub fn pack_id(id: Id) -> u32 {
    match id {
        Id::Standard(sid) => sid.as_raw() as u32,
        Id::Extended(eid) => {
            let raw = eid.as_raw();
            ((raw >> 18) & 0x7FF) | ((raw & 0x3_FFFF) << 11)
        }
    }
}

/// Inverse of [`pack_id`]. `extended` selects the layout (from `IDE`).
pub fn unpack_id(raw: u32, extended: bool) -> Id {
    if extended {
        let id29 = ((raw & 0x7FF) << 18) | ((raw >> 11) & 0x3_FFFF);
        // Both operands are masked to 29 bits, so this cannot fail.
        Id::Extended(ExtendedId::new(id29).unwrap_or(ExtendedId::ZERO))
    } else {
        Id::Standard(StandardId::new((raw & 0x7FF) as u16).unwrap_or(StandardId::ZERO))
    }
}

/// The fields of a TX message object header (words T0 and T1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxHeader {
    /// CAN identifier.
    pub id: Id,
    /// DLC code (`0..=15`).
    pub dlc: u8,
    /// Remote transmission request (classic frames only).
    pub rtr: bool,
    /// Bit rate switch (FD frames only).
    pub brs: bool,
    /// FD format frame.
    pub fdf: bool,
    /// Error state indicator.
    pub esi: bool,
    /// Sequence number echoed in the TEF (`0..=0x7F` on MCP2517FD,
    /// `0..=0x7F_FFFF` on MCP2518FD). Caller masks to the variant's width.
    pub seq: u32,
}

impl TxHeader {
    /// Encodes T0 and T1.
    pub fn to_words(&self) -> [u32; 2] {
        let t0 = pack_id(self.id);
        let ide = matches!(self.id, Id::Extended(_));
        let t1 = (self.dlc as u32 & 0xF)
            | ((ide as u32) << 4)
            | ((self.rtr as u32) << 5)
            | ((self.brs as u32) << 6)
            | ((self.fdf as u32) << 7)
            | ((self.esi as u32) << 8)
            | (self.seq << 9);
        [t0, t1]
    }
}

/// The fields decoded from an RX message object header (words R0 and R1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RxHeader {
    /// CAN identifier.
    pub id: Id,
    /// DLC code (`0..=15`).
    pub dlc: u8,
    /// Remote transmission request.
    pub rtr: bool,
    /// Bit rate switch was used.
    pub brs: bool,
    /// FD format frame.
    pub fdf: bool,
    /// Error state indicator of the transmitter.
    pub esi: bool,
    /// Index of the filter that accepted this frame (`0..=31`).
    pub filhit: u8,
}

impl RxHeader {
    /// Decodes R0 and R1.
    pub fn from_words(words: [u32; 2]) -> Self {
        let [r0, r1] = words;
        let ide = r1 & (1 << 4) != 0;
        Self {
            id: unpack_id(r0, ide),
            dlc: (r1 & 0xF) as u8,
            rtr: r1 & (1 << 5) != 0,
            brs: r1 & (1 << 6) != 0,
            fdf: r1 & (1 << 7) != 0,
            esi: r1 & (1 << 8) != 0,
            filhit: ((r1 >> 11) & 0x1F) as u8,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_can::{ExtendedId, Id, StandardId};

    #[test]
    fn dlc_len_mapping() {
        for (dlc, len) in [
            (0, 0),
            (8, 8),
            (9, 12),
            (10, 16),
            (11, 20),
            (12, 24),
            (13, 32),
            (14, 48),
            (15, 64),
        ] {
            assert_eq!(dlc_to_len(dlc, true), len);
            assert_eq!(len_to_dlc(len, true), Some(dlc));
        }
        // Classic CAN: DLC 9..15 all mean 8 data bytes on the wire.
        assert_eq!(dlc_to_len(12, false), 8);
        assert_eq!(len_to_dlc(9, false), None);
        assert_eq!(len_to_dlc(10, true), None); // 10 is not a valid FD length
        assert_eq!(padded_dlc_len(10), Some(12));
        assert_eq!(padded_dlc_len(64), Some(64));
        assert_eq!(padded_dlc_len(65), None);
    }

    #[test]
    fn id_packing_standard() {
        let id = Id::Standard(StandardId::new(0x123).unwrap());
        assert_eq!(pack_id(id), 0x123);
        assert_eq!(unpack_id(0x123, false), id);
    }

    #[test]
    fn id_packing_extended() {
        // 29-bit ID 0x0CFE_6E01: base (bits 28:18) -> SID field 10:0,
        // low 18 bits -> EID field 28:11.
        let raw29: u32 = 0x0CFE_6E01;
        let id = Id::Extended(ExtendedId::new(raw29).unwrap());
        let packed = pack_id(id);
        assert_eq!(packed & 0x7FF, raw29 >> 18);
        assert_eq!((packed >> 11) & 0x3_FFFF, raw29 & 0x3_FFFF);
        assert_eq!(unpack_id(packed, true), id);
    }

    #[test]
    fn tx_header_words() {
        let h = TxHeader {
            id: Id::Standard(StandardId::new(0x123).unwrap()),
            dlc: 4,
            rtr: false,
            brs: false,
            fdf: false,
            esi: false,
            seq: 1,
        };
        let [t0, t1] = h.to_words();
        assert_eq!(t0, 0x123);
        assert_eq!(t1, 4 | (1 << 9)); // DLC=4, SEQ=1
        let h_fd = TxHeader {
            dlc: 15,
            brs: true,
            fdf: true,
            seq: 0,
            ..h
        };
        let [_, t1] = h_fd.to_words();
        assert_eq!(t1, 15 | (1 << 6) | (1 << 7));
    }

    #[test]
    fn rx_header_round_trip() {
        // Extended FD frame, DLC 15, BRS, FILHIT 3.
        let raw29: u32 = 0x0CFE_6E01;
        let r0 = ((raw29 >> 18) & 0x7FF) | ((raw29 & 0x3_FFFF) << 11);
        let r1 = 15 | (1 << 4) | (1 << 6) | (1 << 7) | (3 << 11);
        let h = RxHeader::from_words([r0, r1]);
        assert_eq!(h.id, Id::Extended(ExtendedId::new(raw29).unwrap()));
        assert_eq!(h.dlc, 15);
        assert!(h.brs && h.fdf && !h.rtr && !h.esi);
        assert_eq!(h.filhit, 3);
    }
}
