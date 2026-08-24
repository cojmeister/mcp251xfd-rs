//! Message RAM layout planning.
//!
//! The chip allocates configured FIFOs contiguously from the start of the
//! 2 KiB message RAM. The planner validates that the configuration fits;
//! actual element addresses are read back from `CiFIFOUAm` at runtime.
//!
//! Build the layout in a `const` and RAM overflow becomes a compile error:
//!
//! ```
//! use mcp251xfd::registers::ram::FifoLayout;
//! use mcp251xfd::registers::{Fifo, PayloadSize};
//! const LAYOUT: FifoLayout = FifoLayout::new()
//!     .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
//!     .rx_fifo(Fifo::F2, PayloadSize::B64, 8);
//! ```

use super::{Fifo, PayloadSize, addr};

/// Transfer direction of a FIFO.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoDirection {
    /// The FIFO transmits frames.
    Transmit,
    /// The FIFO receives frames.
    Receive,
}

/// Configuration of one FIFO within a [`FifoLayout`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoEntry {
    /// Transfer direction.
    pub direction: FifoDirection,
    /// Payload size of each element.
    pub payload: PayloadSize,
    /// Number of elements (`1..=32`).
    pub depth: u8,
}

impl FifoEntry {
    /// RAM bytes used by this FIFO: `(8 + payload) * depth`.
    pub const fn bytes(&self) -> usize {
        (8 + self.payload.bytes()) * self.depth as usize
    }
}

/// Why a FIFO could not be added to a [`FifoLayout`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LayoutError {
    /// The layout would exceed the 2048-byte message RAM.
    RamOverflow,
    /// Depth must be `1..=32`.
    BadDepth,
    /// This FIFO number is already configured in the layout.
    AlreadyConfigured,
}

/// A complete message-RAM layout: which FIFOs exist, their direction,
/// payload size, and depth.
///
/// Allocate FIFO numbers contiguously from [`Fifo::F1`] upwards. Any subset
/// of `F1..=F31` is accepted, but the planner's budget assumes the chip
/// reserves RAM only for the FIFOs that are actually configured — and the
/// address-generation formula (DS20005678E §3, "FIFO User Address",
/// Equation 3-20) leaves the RAM occupancy of *unconfigured* FIFOs
/// undefined. A gapped layout is therefore not validated against what the
/// silicon does with the gaps.
///
/// This is a fit-checking concern only, never a corruption one: the driver
/// computes no element addresses of its own, it reads `CiFIFOUAm` back from
/// the chip for every access, so a disagreement surfaces as
/// [`Error::CommunicationCheckFailed`](crate::Error::CommunicationCheckFailed)
/// rather than as a write into the wrong FIFO.
#[derive(Debug, Clone, Copy)]
pub struct FifoLayout {
    entries: [Option<FifoEntry>; 31],
    total: usize,
}

impl FifoLayout {
    /// An empty layout.
    pub const fn new() -> Self {
        Self {
            entries: [None; 31],
            total: 0,
        }
    }

    /// Adds a FIFO. Errors instead of panicking; use this for layouts built
    /// at runtime.
    pub const fn try_add(self, fifo: Fifo, entry: FifoEntry) -> Result<Self, LayoutError> {
        if entry.depth < 1 || entry.depth > 32 {
            return Err(LayoutError::BadDepth);
        }
        let slot = (fifo.index() - 1) as usize;
        if self.entries[slot].is_some() {
            return Err(LayoutError::AlreadyConfigured);
        }
        let total = self.total + entry.bytes();
        if total > addr::RAM_SIZE {
            return Err(LayoutError::RamOverflow);
        }
        let mut entries = self.entries;
        entries[slot] = Some(entry);
        Ok(Self { entries, total })
    }

    /// Adds a transmit FIFO; see [`Self::try_add`].
    pub const fn try_tx_fifo(
        self,
        fifo: Fifo,
        payload: PayloadSize,
        depth: u8,
    ) -> Result<Self, LayoutError> {
        self.try_add(
            fifo,
            FifoEntry {
                direction: FifoDirection::Transmit,
                payload,
                depth,
            },
        )
    }

    /// Adds a receive FIFO; see [`Self::try_add`].
    pub const fn try_rx_fifo(
        self,
        fifo: Fifo,
        payload: PayloadSize,
        depth: u8,
    ) -> Result<Self, LayoutError> {
        self.try_add(
            fifo,
            FifoEntry {
                direction: FifoDirection::Receive,
                payload,
                depth,
            },
        )
    }

    /// Adds a transmit FIFO.
    ///
    /// # Panics
    /// Panics if the layout would exceed RAM, the depth is out of range, or
    /// the FIFO is already configured. In a `const` context the panic is a
    /// compile error — declare layouts as `const` to get build-time checking.
    pub const fn tx_fifo(self, fifo: Fifo, payload: PayloadSize, depth: u8) -> Self {
        match self.try_tx_fifo(fifo, payload, depth) {
            Ok(l) => l,
            Err(LayoutError::RamOverflow) => panic!("FIFO layout exceeds 2048-byte message RAM"),
            Err(LayoutError::BadDepth) => panic!("FIFO depth must be 1..=32"),
            Err(LayoutError::AlreadyConfigured) => panic!("FIFO already configured"),
        }
    }

    /// Adds a receive FIFO. Panics like [`Self::tx_fifo`].
    pub const fn rx_fifo(self, fifo: Fifo, payload: PayloadSize, depth: u8) -> Self {
        match self.try_rx_fifo(fifo, payload, depth) {
            Ok(l) => l,
            Err(LayoutError::RamOverflow) => panic!("FIFO layout exceeds 2048-byte message RAM"),
            Err(LayoutError::BadDepth) => panic!("FIFO depth must be 1..=32"),
            Err(LayoutError::AlreadyConfigured) => panic!("FIFO already configured"),
        }
    }

    /// Total message-RAM bytes used (`<= 2048`).
    pub const fn total_bytes(&self) -> usize {
        self.total
    }

    /// Iterates over the configured FIFOs in FIFO-number order.
    pub fn entries(&self) -> impl Iterator<Item = (Fifo, FifoEntry)> + '_ {
        self.entries.iter().enumerate().filter_map(|(i, e)| {
            e.map(|entry| (Fifo::new(i as u8 + 1).expect("index in range"), entry))
        })
    }

    /// Looks up one FIFO's configuration.
    pub fn entry(&self, fifo: Fifo) -> Option<FifoEntry> {
        self.entries[(fifo.index() - 1) as usize]
    }
}

impl Default for FifoLayout {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registers::{Fifo, PayloadSize};

    #[test]
    fn element_and_total_sizes() {
        // (8 header + 64 payload) * 4 + (8 + 64) * 8 = 288 + 576 = 864.
        let l = FifoLayout::new()
            .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
            .rx_fifo(Fifo::F2, PayloadSize::B64, 8);
        assert_eq!(l.total_bytes(), 864);
        assert_eq!(l.entries().count(), 2);
        let e = l.entry(Fifo::F2).unwrap();
        assert_eq!(e.depth, 8);
        assert!(matches!(e.direction, FifoDirection::Receive));
    }

    #[test]
    fn exactly_full_is_ok() {
        // 16 bytes/element * 32 deep * 4 FIFOs = 2048 exactly.
        let l = FifoLayout::new()
            .rx_fifo(Fifo::F1, PayloadSize::B8, 32)
            .rx_fifo(Fifo::F2, PayloadSize::B8, 32)
            .rx_fifo(Fifo::F3, PayloadSize::B8, 32)
            .rx_fifo(Fifo::F4, PayloadSize::B8, 32);
        assert_eq!(l.total_bytes(), 2048);
    }

    #[test]
    fn overflow_is_err_at_runtime() {
        let l = FifoLayout::new().rx_fifo(Fifo::F1, PayloadSize::B64, 28); // 72 * 28 = 2016
        assert!(matches!(
            l.try_rx_fifo(Fifo::F2, PayloadSize::B64, 1),
            Err(LayoutError::RamOverflow)
        ));
    }

    #[test]
    fn bad_depth_and_duplicates() {
        let l = FifoLayout::new().tx_fifo(Fifo::F1, PayloadSize::B8, 1);
        assert!(matches!(
            l.try_tx_fifo(Fifo::F1, PayloadSize::B8, 1),
            Err(LayoutError::AlreadyConfigured)
        ));
        assert!(matches!(
            FifoLayout::new().try_tx_fifo(Fifo::F2, PayloadSize::B8, 0),
            Err(LayoutError::BadDepth)
        ));
        assert!(matches!(
            FifoLayout::new().try_tx_fifo(Fifo::F2, PayloadSize::B8, 33),
            Err(LayoutError::BadDepth)
        ));
    }

    #[test]
    fn const_layout_compiles() {
        const LAYOUT: FifoLayout = FifoLayout::new()
            .tx_fifo(Fifo::F1, PayloadSize::B64, 4)
            .rx_fifo(Fifo::F2, PayloadSize::B64, 8);
        assert_eq!(LAYOUT.total_bytes(), 864);
    }
}
