// src/driver/mod.rs
pub mod calibration;
pub mod channels;
pub mod modes;
pub mod operations;
pub mod power;

use crate::constants::*;

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    spi::SpiDevice,
};
use modes::OperatingMode;

/// ADS1256 driver
#[derive(Debug)]
pub struct Ads1256<SPI, CS, DRDY, PDWN, DELAY> {
    spi: SPI,
    cs: CS,
    pub drdy: DRDY,
    pdwn: PDWN,
    pub delay: DELAY,
    gain: Gain,
    data_rate: DataRate,
    vref: f64,
    mode: OperatingMode,
}

impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    /// Creates a new ADS1256 driver instance
    pub fn new(
        spi: SPI,
        cs: CS,
        drdy: DRDY,
        pdwn: PDWN,
        delay: DELAY,
        gain: Gain,
        data_rate: DataRate,
    ) -> Self {
        Ads1256 {
            spi,
            cs,
            drdy,
            pdwn,
            delay,
            gain,
            data_rate,
            vref: DEFAULT_VREF,
            mode: OperatingMode::OneShot,
        }
    }

    /// Creates a new ADS1256 driver instance with custom reference voltage
    pub fn new_with_vref(
        spi: SPI,
        cs: CS,
        drdy: DRDY,
        pdwn: PDWN,
        delay: DELAY,
        gain: Gain,
        data_rate: DataRate,
        vref: f64,
    ) -> Self {
        Ads1256 {
            spi,
            cs,
            drdy,
            pdwn,
            delay,
            gain,
            data_rate,
            vref,
            mode: OperatingMode::OneShot,
        }
    }

    /// Get the current reference voltage
    pub fn vref(&self) -> f64 {
        self.vref
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
