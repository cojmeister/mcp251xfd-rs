//! SPI transaction layer.
//!
//! Command format: 16-bit big-endian word = 4-bit opcode | 12-bit address,
//! followed by data bytes (registers little-endian). RAM accesses
//! (0x400..0xC00) must be 32-bit aligned and sized in 32-bit multiples.
//!
//! Everything runs inside `SpiDevice::transaction` so chip-select framing
//! is correct on shared buses; the driver never touches CS itself.

use crate::error::Error;
use embedded_hal::spi::{Operation, SpiDevice};
#[cfg(feature = "async")]
use embedded_hal_async::spi::SpiDevice as SpiDeviceAsync;

/// SPI instruction opcodes (high nibble of the command word).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Opcode {
    /// Reset the chip to Configuration mode with default registers.
    Reset = 0x0,
    /// Write bytes starting at an address.
    Write = 0x2,
    /// Read bytes starting at an address.
    Read = 0x3,
    // TODO: CRC-protected SPI transfers are not implemented; opcodes reserved below.
    // WriteCrc = 0xA, ReadCrc = 0xB, SafeWrite = 0xC.
}

/// Encodes the 2-byte command word.
pub(crate) const fn cmd(op: Opcode, addr: u16) -> [u8; 2] {
    (((op as u16) << 12) | (addr & 0x0FFF)).to_be_bytes()
}

/// Low-level register/RAM access over a shared-bus-capable SPI device.
#[maybe_async_cfg::maybe(sync(keep_self), async(feature = "async"))]
pub(crate) struct Bus<SPI> {
    pub(crate) spi: SPI,
}

#[maybe_async_cfg::maybe(
    sync(keep_self),
    async(feature = "async", idents(SpiDevice(async = "SpiDeviceAsync")))
)]
impl<SPI: SpiDevice> Bus<SPI> {
    /// Sends the RESET instruction.
    ///
    /// Per DS20006027B §4.1.1: should only be issued while the device is in
    /// Configuration mode, and it does not change Message Memory (RAM).
    pub(crate) async fn reset(&mut self) -> Result<(), Error<SPI::Error>> {
        self.spi
            .write(&cmd(Opcode::Reset, 0))
            .await
            .map_err(Error::Spi)
    }

    /// Reads one byte from an SFR address.
    #[allow(dead_code)] // Bus SFR API completeness; no driver caller yet.
    pub(crate) async fn read_sfr8(&mut self, addr: u16) -> Result<u8, Error<SPI::Error>> {
        let c = cmd(Opcode::Read, addr);
        let mut buf = [0u8; 1];
        self.spi
            .transaction(&mut [Operation::Write(&c), Operation::Read(&mut buf)])
            .await
            .map_err(Error::Spi)?;
        Ok(buf[0])
    }

    /// Writes one byte to an SFR address. Also the only safe way to touch
    /// IOCON: single-byte SFR WRITE is required for IOCON on all variants,
    /// per DS20006027B Register 3-2 Note 2 / §4.1.3.
    pub(crate) async fn write_sfr8(
        &mut self,
        addr: u16,
        value: u8,
    ) -> Result<(), Error<SPI::Error>> {
        let c = cmd(Opcode::Write, addr);
        self.spi
            .transaction(&mut [Operation::Write(&c), Operation::Write(&[value])])
            .await
            .map_err(Error::Spi)
    }

    /// Reads a 32-bit register (little-endian on the wire).
    pub(crate) async fn read_sfr32(&mut self, addr: u16) -> Result<u32, Error<SPI::Error>> {
        let c = cmd(Opcode::Read, addr);
        let mut buf = [0u8; 4];
        self.spi
            .transaction(&mut [Operation::Write(&c), Operation::Read(&mut buf)])
            .await
            .map_err(Error::Spi)?;
        Ok(u32::from_le_bytes(buf))
    }

    /// Writes a 32-bit register (little-endian on the wire).
    pub(crate) async fn write_sfr32(
        &mut self,
        addr: u16,
        value: u32,
    ) -> Result<(), Error<SPI::Error>> {
        let c = cmd(Opcode::Write, addr);
        self.spi
            .transaction(&mut [Operation::Write(&c), Operation::Write(&value.to_le_bytes())])
            .await
            .map_err(Error::Spi)
    }

    /// Reads from message RAM. `addr` must be 32-bit aligned and `buf.len()`
    /// a multiple of 4 (hardware requirement).
    pub(crate) async fn read_ram(
        &mut self,
        addr: u16,
        buf: &mut [u8],
    ) -> Result<(), Error<SPI::Error>> {
        debug_assert!(addr % 4 == 0 && buf.len() % 4 == 0);
        debug_assert!((0x400..0xC00).contains(&addr));
        debug_assert!(addr as usize + buf.len() <= 0xC00);
        let c = cmd(Opcode::Read, addr);
        self.spi
            .transaction(&mut [Operation::Write(&c), Operation::Read(buf)])
            .await
            .map_err(Error::Spi)
    }

    /// Writes to message RAM. Same alignment rules as [`Self::read_ram`].
    pub(crate) async fn write_ram(
        &mut self,
        addr: u16,
        data: &[u8],
    ) -> Result<(), Error<SPI::Error>> {
        debug_assert!(addr % 4 == 0 && data.len() % 4 == 0);
        debug_assert!((0x400..0xC00).contains(&addr));
        debug_assert!(addr as usize + data.len() <= 0xC00);
        let c = cmd(Opcode::Write, addr);
        self.spi
            .transaction(&mut [Operation::Write(&c), Operation::Write(data)])
            .await
            .map_err(Error::Spi)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::spi::{Mock, Transaction};

    #[test]
    fn command_encoding() {
        assert_eq!(cmd(Opcode::Read, 0x000), [0x30, 0x00]);
        assert_eq!(cmd(Opcode::Write, 0xE00), [0x2E, 0x00]);
        assert_eq!(cmd(Opcode::Read, 0xBFC), [0x3B, 0xFC]);
        assert_eq!(cmd(Opcode::Reset, 0x000), [0x00, 0x00]);
    }

    #[test]
    fn read_sfr32_is_little_endian() {
        let mut spi = Mock::new(&[
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x30, 0x00]),
            Transaction::read_vec(vec![0x60, 0x04, 0x00, 0x00]),
            Transaction::transaction_end(),
        ]);
        let mut bus = Bus { spi: &mut spi };
        assert_eq!(bus.read_sfr32(0x000).unwrap(), 0x0000_0460);
        spi.done();
    }

    #[test]
    fn write_sfr32_and_sfr8() {
        let mut spi = Mock::new(&[
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x2E, 0x00]),
            Transaction::write_vec(vec![0x60, 0x00, 0x00, 0x00]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x20, 0x5D]),
            Transaction::write_vec(vec![0x03]),
            Transaction::transaction_end(),
        ]);
        let mut bus = Bus { spi: &mut spi };
        bus.write_sfr32(0xE00, 0x0000_0060).unwrap();
        bus.write_sfr8(0x05D, 0x03).unwrap();
        spi.done();
    }

    #[test]
    fn ram_round_trip() {
        let mut spi = Mock::new(&[
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x2B, 0xFC]),
            Transaction::write_vec(vec![0x55, 0xAA, 0x55, 0xAA]),
            Transaction::transaction_end(),
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x3B, 0xFC]),
            Transaction::read_vec(vec![0x55, 0xAA, 0x55, 0xAA]),
            Transaction::transaction_end(),
        ]);
        let mut bus = Bus { spi: &mut spi };
        bus.write_ram(0xBFC, &0xAA55_AA55u32.to_le_bytes()).unwrap();
        let mut buf = [0u8; 4];
        bus.read_ram(0xBFC, &mut buf).unwrap();
        assert_eq!(u32::from_le_bytes(buf), 0xAA55_AA55);
        spi.done();
    }
}

#[cfg(all(test, feature = "async"))]
mod async_tests {
    use super::*;
    use embedded_hal_mock::eh1::spi::{Mock, Transaction};

    #[tokio::test]
    async fn async_read_sfr32() {
        let mut spi = Mock::new(&[
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x30, 0x00]),
            Transaction::read_vec(vec![0x60, 0x04, 0x00, 0x00]),
            Transaction::transaction_end(),
        ]);
        let mut bus = BusAsync { spi: &mut spi };
        assert_eq!(bus.read_sfr32(0x000).await.unwrap(), 0x0000_0460);
        spi.done();
    }
}
