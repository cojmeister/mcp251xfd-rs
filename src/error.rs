//! Error types returned by the driver.

/// Errors returned by driver operations.
///
/// `E` is the error type of the underlying [`embedded_hal::spi::SpiDevice`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error<E> {
    /// The SPI transaction itself failed.
    Spi(E),
    /// The init-time RAM echo test (write/read `0xAA55AA55`) failed.
    ///
    /// Usually wiring, a wrong CS line, or an SPI clock above the erratum
    /// limit of `0.85 * SYSCLK / 2` (see [`crate::max_spi_hz`]).
    CommunicationCheckFailed,
    /// The oscillator/PLL did not report ready within the timeout (~4 ms).
    ClockNotReady,
    /// The chip did not reach the requested operation mode within the timeout.
    ModeChangeTimeout,
    /// The operation requires Configuration mode, but the chip is not in it.
    NotInConfigMode,
    /// The target TX FIFO has no free slot. Retry after a slot frees up.
    TxFifoFull,
    /// The RX FIFO holds no message.
    RxFifoEmpty,
    /// The requested FIFO layout does not fit the 2048-byte message RAM.
    RamOverflow,
    /// A configuration value is out of range.
    InvalidConfig(ConfigError),
    /// A frame payload length is not a valid CAN (FD) length.
    InvalidPayloadLength,
    /// Waiting on the interrupt pin failed.
    IntPin,
}

/// Which part of a [`crate::config::Config`] was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// Nominal bit timing field out of range (see field docs for ranges).
    NominalBitTiming,
    /// Data bit timing field out of range (see field docs for ranges).
    DataBitTiming,
    /// Clock configuration invalid (e.g. PLL from a non-4 MHz-class input).
    Clock,
}

impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Spi(e) => write!(f, "SPI error: {e:?}"),
            Error::CommunicationCheckFailed => {
                f.write_str("RAM echo test failed (wiring or SPI clock too fast)")
            }
            Error::ClockNotReady => f.write_str("oscillator/PLL not ready"),
            Error::ModeChangeTimeout => f.write_str("operation mode change timed out"),
            Error::NotInConfigMode => f.write_str("chip is not in Configuration mode"),
            Error::TxFifoFull => f.write_str("TX FIFO is full"),
            Error::RxFifoEmpty => f.write_str("RX FIFO is empty"),
            Error::RamOverflow => f.write_str("FIFO layout exceeds 2048-byte message RAM"),
            Error::InvalidConfig(ConfigError::NominalBitTiming) => {
                f.write_str("invalid nominal bit timing")
            }
            Error::InvalidConfig(ConfigError::DataBitTiming) => {
                f.write_str("invalid data bit timing")
            }
            Error::InvalidConfig(ConfigError::Clock) => f.write_str("invalid clock configuration"),
            Error::InvalidPayloadLength => f.write_str("invalid payload length"),
            Error::IntPin => f.write_str("interrupt pin wait failed"),
        }
    }
}

impl<E: core::fmt::Debug> core::error::Error for Error<E> {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_is_debug_and_display() {
        let e: Error<()> = Error::TxFifoFull;
        assert_eq!(format!("{e:?}"), "TxFifoFull");
        assert_eq!(format!("{e}"), "TX FIFO is full");
        let e: Error<()> = Error::InvalidConfig(ConfigError::NominalBitTiming);
        assert!(format!("{e}").contains("nominal"));
    }
}
