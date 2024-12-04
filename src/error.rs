use core::fmt;

#[derive(Debug)]
pub enum Ads1256Error<SpiError, GpioError> {
    Spi(SpiError),
    Gpio(GpioError),
    Timeout,
    InvalidInputChannel,
    InvalidResponse,
}

impl<SpiError, GpioError> fmt::Display for Ads1256Error<SpiError, GpioError>
where
    SpiError: fmt::Debug,
    GpioError: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Ads1256Error::Spi(e) => write!(f, "SPI error: {:?}", e),
            Ads1256Error::Gpio(e) => write!(f, "GPIO error: {:?}", e),
            Ads1256Error::InvalidResponse => write!(f, "Invalid response from ADS1256"),
            Ads1256Error::Timeout => write!(f, "Timeout error"),
            Ads1256Error::InvalidInputChannel => write!(f, "Invalid Input Channel error"),
        }
    }
}
