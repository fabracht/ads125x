// src/driver/power.rs
use super::Ads1256NonBlocking;
use crate::constants::*;
use crate::error::Ads1256Error;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::{delay::DelayNs, digital::Wait, spi::SpiBus};

impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiBus<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: Wait + InputPin<Error = CS::Error>,
    PDWN: OutputPin<Error = CS::Error>,
    DELAY: DelayNs,
{
    pub async fn reset(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Pull PDWN low
        self.pdwn.set_low().map_err(Ads1256Error::Gpio)?;
        self.delay.delay_ms(10).await;

        // Pull PDWN high
        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;
        self.delay.delay_ms(10).await;

        // Send reset command
        self.send_command(CMD_RESET).await?;
        self.delay.delay_ms(50).await; // Increased delay after reset command

        // Wait for DRDY to go low
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;

        // Perform self-calibration
        self.send_command(CMD_SELFCAL).await?;
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;

        Ok(())
    }

    pub async fn enter_power_down(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Hold SYNC/PDWN low for 20 DRDY periods
        self.pdwn.set_low().map_err(Ads1256Error::Gpio)?;

        // Calculate wait time based on data rate period
        let drdy_period_us = self.data_rate.period_us();
        self.delay
            .delay_ms((20.0 * drdy_period_us / 1000.0) as u32)
            .await;

        Ok(())
    }

    pub async fn exit_power_down(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;
        self.delay.delay_ms(30).await; // Wait for oscillator startup
        Ok(())
    }

    pub async fn synchronize(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Use SYNC pin to synchronize
        self.pdwn.set_low().map_err(Ads1256Error::Gpio)?;

        // Wait for t16 (timing for the SYNC pulse)
        self.delay.delay_us(T16_DELAY).await;

        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;

        // Wait for DRDY to indicate new conversion started
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;

        Ok(())
    }
}
