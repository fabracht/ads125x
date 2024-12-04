use crate::{Ads1256, Ads1256Error};
use core::fmt::Debug;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiDevice;
use nb::block;

use crate::constants::*;

/// Non-blocking reading interface for the ADS1256
pub trait Ads1256NonBlocking {
    type Error: Debug;

    /// Start a conversion
    ///
    /// This method initiates an ADC conversion.
    fn start_conversion(&mut self) -> Result<(), Self::Error>;

    /// Read the result of the conversion
    ///
    /// Returns `nb::Error::WouldBlock` if the conversion is not complete.
    /// Returns the conversion result as a signed 32-bit integer if the conversion is complete.
    fn read_conversion(&mut self) -> nb::Result<i32, Self::Error>;
}

impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256NonBlocking
    for Ads1256<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
    SpiError: Debug,
    GpioError: Debug,
{
    type Error = Ads1256Error<SpiError, GpioError>;

    fn start_conversion(&mut self) -> Result<(), Self::Error> {
        // Wake up the ADC if it was in standby
        self.send_command(CMD_WAKEUP)?;

        // Synchronize the ADC
        self.send_command(CMD_SYNC)?;
        self.delay.delay_us(100);

        // Start conversion by waking up again
        self.send_command(CMD_WAKEUP)
    }

    fn read_conversion(&mut self) -> nb::Result<i32, Self::Error> {
        // Check if DRDY is high (conversion not complete)
        if self.drdy.is_high().map_err(Ads1256Error::Gpio)? {
            return Err(nb::Error::WouldBlock);
        }

        // Read the conversion result
        match self.read_data() {
            Ok(value) => Ok(value),
            Err(e) => Err(nb::Error::Other(e)),
        }
    }
}

/// Extension trait to add blocking operations
pub trait Ads1256Ext: Ads1256NonBlocking {
    /// Perform a blocking read
    ///
    /// This method will block until a conversion is complete.
    fn read_blocking(&mut self) -> Result<i32, Self::Error> {
        self.start_conversion()?;
        Ok(block!(self.read_conversion())?)
    }
}

impl<T: Ads1256NonBlocking> Ads1256Ext for T {}

#[cfg(feature = "defmt")]
impl<SpiError, GpioError> defmt::Format for Ads1256Error<SpiError, GpioError>
where
    SpiError: Debug,
    GpioError: Debug,
{
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::Spi(_) => defmt::write!(f, "SPI error"),
            Self::Gpio(_) => defmt::write!(f, "GPIO error"),
            Self::Timeout => defmt::write!(f, "Operation timed out"),
            Self::InvalidInputChannel => defmt::write!(f, "Invalid input channel"),
            Self::InvalidGainSetting => defmt::write!(f, "Invalid gain setting"),
            Self::InvalidDataRate => defmt::write!(f, "Invalid data rate"),
            Self::CalibrationError => defmt::write!(f, "Calibration failed"),
            Self::BufferConfigError => defmt::write!(f, "Buffer configuration error"),
            Self::ContinuousReadError => defmt::write!(f, "Continuous read mode error"),
            Self::SyncError => defmt::write!(f, "Synchronization error"),
            Self::PowerDownError => defmt::write!(f, "Power down error"),
            Self::DrdyTimeout {
                current_state,
                wait_time,
            } => {
                defmt::write!(
                    f,
                    "DRDY timeout: state={}, waited={}μs",
                    current_state,
                    wait_time
                )
            }
            Self::RegisterError {
                register,
                operation,
            } => {
                defmt::write!(f, "Register error: {} on 0x{:02X}", operation, register)
            }
        }
    }
}
