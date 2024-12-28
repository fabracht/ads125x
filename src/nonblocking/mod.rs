// src/nonblocking/mod.rs
pub mod calibration;
pub mod channels;
pub mod modes;
pub mod operations;
pub mod power;

use crate::constants::*;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};
use modes::Mode;

/// Non-blocking ADS1256 driver
pub struct Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY> {
    pub(crate) spi: SPI,
    pub(crate) cs: CS,
    pub(crate) drdy: DRDY,
    pub(crate) pdwn: PDWN,
    pub(crate) delay: DELAY,
    mode: Mode,
    gain: Gain,
    data_rate: DataRate,
    current_channel: Option<u8>,
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError>
    Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: Wait + OutputPin<Error = GpioError>,
    DRDY: Wait + InputPin<Error = GpioError>,
    PDWN: Wait + OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    /// Creates a new non-blocking ADS1256 instance
    pub fn new(
        spi: SPI,
        cs: CS,
        drdy: DRDY,
        pdwn: PDWN,
        delay: DELAY,
        gain: Gain,
        data_rate: DataRate,
    ) -> Self {
        Self {
            spi,
            cs,
            drdy,
            pdwn,
            delay,
            mode: Mode::OneShot,
            gain,
            data_rate,
            current_channel: None,
        }
    }
}
