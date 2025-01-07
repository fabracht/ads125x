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
pub struct Ads1256NonBlocking<SPI, DRDY, PDWN, DELAY> {
    pub(crate) spi: SPI,
    pub(crate) drdy: DRDY,
    pub(crate) pdwn: PDWN,
    pub(crate) delay: DELAY,
    vref: f64,
    mode: Mode,
    gain: Gain,
    data_rate: DataRate,
    current_channel: Option<u8>,
}

impl<SPI, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256NonBlocking<SPI, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    DRDY: Wait + InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    /// Creates a new non-blocking ADS1256 instance
    pub fn new(
        spi: SPI,
        drdy: DRDY,
        pdwn: PDWN,
        delay: DELAY,
        gain: Gain,
        data_rate: DataRate,
    ) -> Self {
        Self {
            spi,
            drdy,
            pdwn,
            delay,
            vref: DEFAULT_VREF,
            mode: Mode::OneShot,
            gain,
            data_rate,
            current_channel: None,
        }
    }
    /// Get the current reference voltage
    pub fn vref(&self) -> f64 {
        self.vref
    }

    pub fn set_vref(&mut self, vref: f64) {
        self.vref = vref;
    }

    /// Get the current gain
    pub fn gain(&self) -> Gain {
        self.gain
    }

    /// Get the current gain value
    pub fn gain_value(&self) -> f64 {
        self.gain.value()
    }
}
