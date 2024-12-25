// src/driver/power.rs
use super::Ads1256;
use crate::constants::*;
use crate::error::Ads1256Error;
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    spi::SpiDevice,
};

impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = CS::Error>,
    PDWN: OutputPin<Error = CS::Error>,
    DELAY: DelayNs,
{
    pub fn reset(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Pull PDWN low
        self.pdwn.set_low().map_err(Ads1256Error::Gpio)?;
        self.delay.delay_ms(10);

        // Pull PDWN high
        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;
        self.delay.delay_ms(10);

        // Send reset command
        self.send_command(CMD_RESET)?;
        self.delay.delay_ms(50); // Increased delay after reset command

        // Wait for DRDY to go low
        self.wait_for_drdy()?;

        // Perform self-calibration
        self.send_command(CMD_SELFCAL)?;
        self.wait_for_drdy()?;

        Ok(())
    }

    pub fn enter_power_down(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Hold SYNC/PDWN low for 20 DRDY periods
        self.pdwn.set_low().map_err(Ads1256Error::Gpio)?;

        // Calculate wait time based on data rate period
        let drdy_period_us = self.data_rate.period_us();
        self.delay.delay_ms((20.0 * drdy_period_us / 1000.0) as u32);

        Ok(())
    }

    pub fn exit_power_down(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;
        self.delay.delay_ms(30); // Wait for oscillator startup
        Ok(())
    }

    pub fn synchronize(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Use SYNC pin to synchronize
        self.pdwn.set_low().map_err(Ads1256Error::Gpio)?;

        // Wait for t16 (timing for the SYNC pulse)
        self.delay.delay_us(T16_DELAY);

        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;

        // Wait for DRDY to indicate new conversion started
        self.wait_for_drdy()?;

        Ok(())
    }
}
