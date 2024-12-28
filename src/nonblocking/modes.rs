// src/nonblocking/modes.rs
use crate::{
    constants::*,
    error::Ads1256Error,
    nonblocking::{utils::yield_now, Ads1256NonBlocking, Mode, State},
};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    /// Enter continuous conversion mode
    pub async fn enter_continuous_mode(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        match self.state {
            State::Idle | State::Standby => {
                // Exit standby if needed
                if self.state == State::Standby {
                    self.send_command(CMD_WAKEUP).await?;
                    self.state = State::PoweringUp;

                    // Wait for power-up
                    while self.poll_state()? == State::PoweringUp {
                        yield_now().await;
                    }
                }

                // Enter continuous mode
                self.send_command(CMD_RDATAC).await?;
                self.delay.delay_us(T6_DELAY).await;
                self.mode = Mode::Continuous;
                self.state = State::Converting;
                Ok(())
            }
            _ => Err(Ads1256Error::InvalidState(
                "Cannot enter continuous mode in current state",
            )),
        }
    }

    /// Exit continuous mode
    pub async fn enter_one_shot_mode(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        if self.mode != Mode::Continuous {
            return Ok(());
        }

        // Wait for any ongoing conversion
        while self.state == State::Converting || self.state == State::Reading {
            yield_now().await;
        }

        self.send_command(CMD_SDATAC).await?;
        self.mode = Mode::OneShot;
        self.state = State::Idle;
        Ok(())
    }

    /// Read data in continuous mode
    pub async fn read_continuous(&mut self) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        if self.mode != Mode::Continuous {
            return Err(Ads1256Error::InvalidState("Not in continuous mode"));
        }

        // Wait for DRDY in non-blocking way
        while !self.drdy.is_low().map_err(Ads1256Error::Gpio)? {
            yield_now().await;
        }

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
    pub async fn handle_step_change(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        if self.mode != Mode::Continuous {
            return Ok(());
        }

        // Wait for DRDY
        while !self.drdy.is_low().map_err(Ads1256Error::Gpio)? {
            yield_now().await;
        }

        // Perform synchronization
        self.send_command(CMD_SYNC).await?;
        self.delay.delay_us(T11_DELAY).await;
        self.send_command(CMD_WAKEUP).await?;

        Ok(())
    }
}
