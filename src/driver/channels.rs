// src/driver/channels.rs
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
    /// Sets the input multiplexer for single-ended input
    pub fn set_channel(&mut self, channel: u8) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        if channel > 7 {
            return Err(Ads1256Error::InvalidInputChannel);
        }

        let positive = channel & 0x07;
        let negative = 0x08; // AINCOM
        let mux = (positive << 4) | negative;

        self.write_register(REG_MUX, &[mux])?;

        // Synchronize after channel change
        self.send_command(CMD_SYNC)?;
        self.delay.delay_us(T11_DELAY);
        self.send_command(CMD_WAKEUP)?;

        Ok(())
    }

    /// Sets the input multiplexer for differential input
    pub fn set_differential_channel(
        &mut self,
        positive: u8,
        negative: u8,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        if positive > 7 || negative > 7 {
            return Err(Ads1256Error::InvalidInputChannel);
        }

        let mux = ((positive & 0x07) << 4) | (negative & 0x07);
        self.write_register(REG_MUX, &[mux])?;

        // Synchronize after channel change
        self.send_command(CMD_SYNC)?;
        self.delay.delay_us(T11_DELAY);
        self.send_command(CMD_WAKEUP)?;

        Ok(())
    }

    /// Cycle to next channel efficiently as per datasheet Figure 19
    pub fn cycle_channel(&mut self, channel: u8) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        if channel > 7 {
            return Err(Ads1256Error::InvalidInputChannel);
        }

        // Wait for DRDY to go low (indicating data is ready)
        self.wait_for_drdy()?;

        // Step 1: Update MUX register for next reading
        let positive = channel & 0x07;
        let negative = 0x08; // AINCOM
        let mux = (positive << 4) | negative;
        self.write_register(REG_MUX, &[mux])?;

        // Step 2: Restart conversion process
        self.send_command(CMD_SYNC)?;
        self.delay.delay_us(T11_DELAY);
        self.send_command(CMD_WAKEUP)?;

        // Step 3: Read data from previous conversion
        let result = self.read_data()?;

        Ok(result)
    }

    /// Cycle to next differential channel pair efficiently
    pub fn cycle_differential_channel(
        &mut self,
        positive: u8,
        negative: u8,
    ) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        if positive > 7 || negative > 7 {
            return Err(Ads1256Error::InvalidInputChannel);
        }

        // Wait for DRDY to go low before changing MUX
        self.wait_for_drdy()?;

        // Step 1: Update MUX register for next reading
        let mux = ((positive & 0x07) << 4) | (negative & 0x07);
        self.write_register(REG_MUX, &[mux])?;

        // Step 2: Restart conversion process
        self.send_command(CMD_SYNC)?;
        self.delay.delay_us(T11_DELAY);
        self.send_command(CMD_WAKEUP)?;

        // Step 3: Read data from previous conversion
        let result = self.read_data()?;

        Ok(result)
    }

    /// Handle step changes in continuous mode
    pub fn handle_input_step_change(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.wait_for_drdy()?;
        self.send_command(CMD_SYNC)?;
        self.delay.delay_us(T11_DELAY);
        self.send_command(CMD_WAKEUP)?;
        Ok(())
    }
}
