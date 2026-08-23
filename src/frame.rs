//! CAN frame types.
//!
//! [`Frame`] is a classic CAN 2.0 frame and implements
//! [`embedded_can::Frame`] for ecosystem interop. [`FdFrame`] is a CAN FD
//! frame (no ecosystem-standard trait exists for FD).

use crate::registers::objects::{len_to_dlc, padded_dlc_len};
use embedded_can::Id;

/// A classic CAN 2.0 frame (up to 8 data bytes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    id: Id,
    dlc: u8,
    rtr: bool,
    data: [u8; 8],
}

impl Frame {
    /// The frame's CAN identifier.
    pub fn id(&self) -> Id {
        self.id
    }
    /// The data bytes (empty for remote frames).
    pub fn data(&self) -> &[u8] {
        if self.rtr {
            &[]
        } else {
            &self.data[..self.dlc as usize]
        }
    }
    /// The DLC (`0..=8`).
    pub fn dlc(&self) -> usize {
        self.dlc as usize
    }
    /// Whether this is a remote (RTR) frame.
    pub fn is_remote(&self) -> bool {
        self.rtr
    }
}

impl embedded_can::Frame for Frame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }
        let mut buf = [0u8; 8];
        buf[..data.len()].copy_from_slice(data);
        Some(Self {
            id: id.into(),
            dlc: data.len() as u8,
            rtr: false,
            data: buf,
        })
    }

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        if dlc > 8 {
            return None;
        }
        Some(Self {
            id: id.into(),
            dlc: dlc as u8,
            rtr: true,
            data: [0; 8],
        })
    }

    fn is_extended(&self) -> bool {
        matches!(self.id, Id::Extended(_))
    }
    fn is_remote_frame(&self) -> bool {
        self.rtr
    }
    fn id(&self) -> Id {
        self.id
    }
    fn dlc(&self) -> usize {
        self.dlc as usize
    }
    fn data(&self) -> &[u8] {
        Frame::data(self)
    }
}

impl Frame {
    #[allow(dead_code)] // used by driver RX path (Task 12)
    pub(crate) fn from_parts(id: Id, dlc: u8, rtr: bool, data: [u8; 8]) -> Self {
        Self { id, dlc, rtr, data }
    }
}

/// Per-frame CAN FD flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FrameFlags {
    /// Bit rate switch: transmit the data phase at the data bit rate.
    pub brs: bool,
    /// Error state indicator.
    pub esi: bool,
}

/// A CAN FD frame (up to 64 data bytes; length must be a valid DLC step).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FdFrame {
    id: Id,
    len: u8,
    flags: FrameFlags,
    data: [u8; 64],
}

impl FdFrame {
    /// Creates an FD frame. Returns `None` unless `data.len()` is exactly a
    /// valid CAN FD length (`0..=8, 12, 16, 20, 24, 32, 48, 64`).
    pub fn new(id: impl Into<Id>, data: &[u8], flags: FrameFlags) -> Option<Self> {
        len_to_dlc(data.len(), true)?;
        let mut buf = [0u8; 64];
        buf[..data.len()].copy_from_slice(data);
        Some(Self {
            id: id.into(),
            len: data.len() as u8,
            flags,
            data: buf,
        })
    }

    /// Creates an FD frame, zero-padding the payload up to the next valid
    /// CAN FD length. Returns `None` if `data.len() > 64`.
    pub fn new_padded(id: impl Into<Id>, data: &[u8], flags: FrameFlags) -> Option<Self> {
        let padded = padded_dlc_len(data.len())?;
        let mut buf = [0u8; 64];
        buf[..data.len()].copy_from_slice(data);
        Some(Self {
            id: id.into(),
            len: padded as u8,
            flags,
            data: buf,
        })
    }

    /// The frame's CAN identifier.
    pub fn id(&self) -> Id {
        self.id
    }
    /// The data bytes (padded length).
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }
    /// The FD flags.
    pub fn flags(&self) -> FrameFlags {
        self.flags
    }
}

impl FdFrame {
    #[allow(dead_code)] // used by driver RX path (Task 12)
    pub(crate) fn from_parts(id: Id, len: u8, flags: FrameFlags, data: [u8; 64]) -> Self {
        Self {
            id,
            len,
            flags,
            data,
        }
    }
}

/// A frame received from an RX FIFO.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RxFrame {
    /// The received frame.
    pub frame: ReceivedFrame,
    /// RX timestamp. Always `None` in this version (timestamping is not
    /// yet configurable).
    pub timestamp: Option<u32>,
}

/// Classic or FD payload of a received frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReceivedFrame {
    /// A classic CAN 2.0 frame.
    Classic(Frame),
    /// A CAN FD frame.
    Fd(FdFrame),
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_can::{Frame as _, StandardId};

    #[test]
    fn classic_frame_basics() {
        let id = StandardId::new(0x123).unwrap();
        let f = Frame::new(id, &[1, 2, 3, 4]).unwrap();
        assert_eq!(f.data(), &[1, 2, 3, 4]);
        assert_eq!(f.dlc(), 4);
        assert!(!f.is_remote_frame());
        assert!(Frame::new(id, &[0; 9]).is_none());
        let r = Frame::new_remote(id, 2).unwrap();
        assert!(r.is_remote_frame());
        assert_eq!(r.dlc(), 2);
        assert!(Frame::new_remote(id, 9).is_none());
    }

    #[test]
    fn fd_frame_lengths() {
        let id = StandardId::new(0x123).unwrap();
        assert!(FdFrame::new(id, &[0; 64], FrameFlags::default()).is_some());
        assert!(FdFrame::new(id, &[0; 10], FrameFlags::default()).is_none());
        let padded = FdFrame::new_padded(
            id,
            &[0xFF; 10],
            FrameFlags {
                brs: true,
                esi: false,
            },
        )
        .unwrap();
        assert_eq!(padded.data().len(), 12);
        assert_eq!(&padded.data()[..10], &[0xFF; 10]);
        assert_eq!(&padded.data()[10..], &[0, 0]);
        assert!(padded.flags().brs);
        assert!(FdFrame::new_padded(id, &[0; 65], FrameFlags::default()).is_none());
    }
}
