// src/driver/modes.rs
use super::types::OperatingMode;
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
    /// Initialize with specific operating mode
    pub fn init_with_mode(
        &mut self,
        mode: OperatingMode,
        buffer_enabled: bool,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.init(buffer_enabled)?;
        match mode {
            OperatingMode::OneShot => self.enter_one_shot_mode()?,
            OperatingMode::Continuous => self.enter_continuous_mode()?,
        }
        self.mode = mode;
        Ok(())
    }

    /// Enter one-shot mode
    pub fn enter_one_shot_mode(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Stop continuous mode if active
        if self.mode == OperatingMode::Continuous {
            self.send_command(CMD_SDATAC)?;
            self.wait_for_drdy()?;
        }

        // Enter standby mode
        self.send_command(CMD_STANDBY)?;
        self.mode = OperatingMode::OneShot;
        Ok(())
    }

    /// Enter continuous mode
    pub fn enter_continuous_mode(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Wait for DRDY to go low before entering continuous mode
        self.wait_for_drdy()?;

        // Exit standby if in one-shot mode
        if self.mode == OperatingMode::OneShot {
            self.send_command(CMD_WAKEUP)?;
            // Wait for modulator power-up (33.3μs @ 7.68MHz)
            self.delay.delay_us(34);
        }

        // Enter continuous mode
        self.send_command(CMD_RDATAC)?;
        self.delay.delay_us(T6_DELAY);
        self.mode = OperatingMode::Continuous;
        Ok(())
    }

    /// Read a single conversion in one-shot mode
    pub fn read_one_shot(&mut self) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        if self.mode != OperatingMode::OneShot {
            return Err(Ads1256Error::InvalidState("Not in one-shot mode"));
        }

        // Wake up the ADC to start conversion
        self.send_command(CMD_WAKEUP)?;

        // Wait for modulator power-up (33.3μs @ 7.68MHz)
        self.delay.delay_us(34);

        // Wait for conversion to complete
        self.wait_for_drdy()?;

        // Read the conversion result
        let result = self.read_data()?;

        // Return to standby for power saving
        self.send_command(CMD_STANDBY)?;

        Ok(result)
    }

    /// Read in continuous mode
    pub fn read_continuous(&mut self) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        if self.mode != OperatingMode::Continuous {
            return Err(Ads1256Error::InvalidState("Not in continuous mode"));
        }

        self.wait_for_drdy()?;

        // In continuous mode, just read the data without sending RDATA command
        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        let mut buffer = [0u8; 3];
        self.spi.read(&mut buffer).map_err(Ads1256Error::Spi)?;
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
}
