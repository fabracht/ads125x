#[derive(Debug)]
pub enum Ads1256Error<SpiError, GpioError> {
    Spi(SpiError),
    Gpio(GpioError),
    Timeout,
}
