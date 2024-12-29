// src/nonblocking/modes.rs
use crate::{constants::*, error::Ads1256Error, nonblocking::Ads1256NonBlocking};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

/// ADC operation mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mode {
    OneShot,
    Continuous,
}

impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: Wait + OutputPin<Error = GpioError>,
    DRDY: Wait + InputPin<Error = GpioError>,
    PDWN: Wait + OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    /// Enter continuous conversion mode
    pub async fn enter_continuous_mode(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;

        // Exit standby if in one-shot mode
        if self.mode == Mode::OneShot {
            self.send_command(CMD_WAKEUP).await?;
            // Wait for modulator power-up (33.3μs @ 7.68MHz)
            self.delay.delay_us(34).await;
        }

        // Enter continuous mode
        self.send_command(CMD_RDATAC).await?;
        self.delay.delay_us(T6_DELAY).await;
        self.mode = Mode::Continuous;
        Ok(())
    }

    /// Exit continuous mode
    pub async fn enter_one_shot_mode(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        if self.mode == Mode::Continuous {
            self.send_command(CMD_SDATAC).await?;
            self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;
        }

        // Enter standby mode
        self.send_command(CMD_STANDBY).await?;
        self.mode = Mode::OneShot;
        Ok(())
    }

    /// Read data in continuous mode
    pub async fn read_continuous(&mut self) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        if self.mode != Mode::Continuous {
            return Err(Ads1256Error::InvalidState("Not in continuous mode"));
        }

        // Wait for DRDY in non-blocking way
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;

        // Read the data
        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        let mut buffer = [0u8; 3];
        self.spi
            .read(&mut buffer)
            .await
            .map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;

        // Convert to signed 24-bit value
        let raw_value = ((buffer[0] as i32) << 16) | ((buffer[1] as i32) << 8) | (buffer[2] as i32);
        let value = if raw_value & 0x800000 != 0 {
            raw_value | !0xFFFFFF
        } else {
            raw_value
        };

        Ok(value)
    }

    /// Handle input step change in continuous mode
    pub async fn handle_input_step_change(
        &mut self,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        if self.mode != Mode::Continuous {
            return Ok(());
        }

        // Wait for DRDY
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;

        // Perform synchronization
        self.send_command(CMD_SYNC).await?;
        self.delay.delay_us(T11_DELAY).await;
        self.send_command(CMD_WAKEUP).await?;

        Ok(())
    }
}
