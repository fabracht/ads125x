// src/driver/operations.rs
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
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    /// Sends a command to the ADS1256
    pub(crate) fn send_command(
        &mut self,
        command: u8,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.spi.write(&[command]).map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;
        Ok(())
    }

    /// Writes data to a register
    pub(crate) fn write_register(
        &mut self,
        reg: u8,
        data: &[u8],
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let command = CMD_WREG | (reg & 0x0F);
        let count = (data.len() - 1) as u8;

        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.spi
            .write(&[command, count])
            .map_err(Ads1256Error::Spi)?;
        self.delay.delay_us(5);
        self.spi.write(data).map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;
        Ok(())
    }

    /// Reads data from a register
    pub(crate) fn read_register(
        &mut self,
        reg: u8,
        buffer: &mut [u8],
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let command = CMD_RREG | (reg & 0x0F);
        let count = (buffer.len() - 1) as u8;

        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.spi
            .write(&[command, count])
            .map_err(Ads1256Error::Spi)?;
        self.delay.delay_us(5);
        self.spi.read(buffer).map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;
        Ok(())
    }

    /// Reads raw data from the ADC
    pub(crate) fn read_data(&mut self) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.spi.write(&[CMD_RDATA]).map_err(Ads1256Error::Spi)?;

        self.delay.delay_us(T6_DELAY);

        let mut buffer = [0u8; 3];
        self.spi.read(&mut buffer).map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;

        let raw_value = ((buffer[0] as i32) << 16) | ((buffer[1] as i32) << 8) | (buffer[2] as i32);
        let value = if raw_value & 0x800000 != 0 {
            raw_value | !0xFFFFFF
        } else {
            raw_value
        };

        Ok(value)
    }

    /// Converts raw ADC code to voltage
    pub fn code_to_voltage(&self, code: i32) -> f64 {
        let gain = self.gain.value();
        let max_code = 8388607.0; // Maximum positive ADC value (2^23 - 1)
        (code as f64 * (2.0 * self.vref)) / (gain * max_code)
    }

    /// Initializes the ADS1256 ADC module
    pub fn init(&mut self, buffer_enabled: bool) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Power up sequence
        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;
        self.delay.delay_ms(10);

        // Reset the device
        self.send_command(CMD_RESET)?;
        self.wait_for_drdy()?;

        // Stop continuous read mode
        self.send_command(CMD_SDATAC)?;
        self.wait_for_drdy()?;

        // Configure STATUS register with BUFEN setting
        let mut status = [0u8; 1];
        self.read_register(REG_STATUS, &mut status)?;
        if buffer_enabled {
            status[0] |= 0x02;
        } else {
            status[0] &= !0x02;
        }
        self.write_register(REG_STATUS, &status)?;

        // Configure ADCON register (PGA setting)
        let adcon = self.gain as u8;
        self.write_register(REG_ADCON, &[adcon])?;

        // Set data rate
        self.write_register(REG_DRATE, &[self.data_rate as u8])?;

        // Configure IO register (all GPIOs as outputs)
        self.write_register(REG_IO, &[0x00])?;

        // Initial MUX setting (AIN0 to AINCOM)
        let mux = 0x08;
        self.write_register(REG_MUX, &[mux])?;

        // Perform self-calibration
        self.send_command(CMD_SELFCAL)?;
        self.wait_for_drdy()?;

        Ok(())
    }

    /// Blocking wait for DRDY to go low
    pub(crate) fn wait_for_drdy(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let timeout = 50000;
        for _ in 0..timeout {
            if self.drdy.is_low().map_err(Ads1256Error::Gpio)? {
                return Ok(());
            }
            self.delay.delay_us(self.data_rate.period_us() as u32);
        }
        Err(Ads1256Error::Timeout)
    }

    pub fn wait_for_drdy_high(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let timeout = 50000;
        for _ in 0..timeout {
            if self.drdy.is_high().map_err(Ads1256Error::Gpio)? {
                return Ok(());
            }
            self.delay.delay_us(self.data_rate.period_us() as u32);
        }
        Err(Ads1256Error::Timeout)
    }
}
