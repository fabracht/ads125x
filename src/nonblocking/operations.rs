// src/nonblocking/operations.rs

use crate::{constants::*, error::Ads1256Error};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::{delay::DelayNs, digital::Wait, spi::SpiDevice};

impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError>
    crate::nonblocking::Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: Wait + InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    /// Sends a command to the ADS1256
    pub(crate) async fn send_command(
        &mut self,
        command: u8,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.spi
            .write(&[command])
            .await
            .map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;
        Ok(())
    }

    /// Writes data to a register
    pub(crate) async fn write_register(
        &mut self,
        reg: u8,
        data: &[u8],
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let command = CMD_WREG | (reg & 0x0F);
        let count = (data.len() - 1) as u8;

        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.spi
            .write(&[command, count])
            .await
            .map_err(Ads1256Error::Spi)?;
        self.delay.delay_us(5).await;
        self.spi.write(data).await.map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;
        Ok(())
    }

    /// Reads data from a register
    pub(crate) async fn read_register(
        &mut self,
        reg: u8,
        buffer: &mut [u8],
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let command = CMD_RREG | (reg & 0x0F);
        let count = (buffer.len() - 1) as u8;

        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.spi
            .write(&[command, count])
            .await
            .map_err(Ads1256Error::Spi)?;
        self.delay.delay_us(5).await;
        self.spi.read(buffer).await.map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;
        Ok(())
    }

    /// Reads raw data from the ADC
    pub(crate) async fn read_data(&mut self) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.spi
            .write(&[CMD_RDATA])
            .await
            .map_err(Ads1256Error::Spi)?;
        self.delay.delay_us(T6_DELAY).await;

        let mut buffer = [0u8; 3];
        self.spi
            .read(&mut buffer)
            .await
            .map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;

        let raw_value = ((buffer[0] as i32) << 16) | ((buffer[1] as i32) << 8) | (buffer[2] as i32);
        let value = if raw_value & 0x800000 != 0 {
            raw_value | !0xFFFFFF
        } else {
            raw_value
        };

        Ok(value)
    }

    /// Initialize the device in a specific mode
    pub async fn init(
        &mut self,
        buffer_enabled: bool,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        log::info!("Initializing the device");
        // Power up sequence
        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;
        log::info!("Powering up the device");
        self.delay.delay_ms(10).await;
        log::info!("Powered up the device");
        // Reset the device
        self.send_command(CMD_RESET).await?;
        log::info!("Resetting the device");
        // Wait for DRDY in non-blocking way
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;
        log::info!("Waiting for DRDY to go low");
        // Stop continuous read mode if active
        self.send_command(CMD_SDATAC).await?;
        log::info!("Stopping continuous read mode");
        // Wait for DRDY
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;
        log::info!("Waiting for DRDY to go low");
        // Configure STATUS register with BUFEN setting
        let mut status = [0u8; 1];
        self.read_register(REG_STATUS, &mut status).await?;
        if buffer_enabled {
            status[0] |= 0x02;
        } else {
            status[0] &= !0x02;
        }
        self.write_register(REG_STATUS, &status).await?;
        log::info!("Configuring STATUS register with BUFEN setting");
        // Configure ADCON register (PGA setting)
        let adcon = self.gain as u8;
        self.write_register(REG_ADCON, &[adcon]).await?;
        log::info!("Configuring ADCON register with PGA setting");
        // Set data rate
        self.write_register(REG_DRATE, &[self.data_rate as u8])
            .await?;
        log::info!("Setting data rate");
        // Configure IO register (all GPIOs as outputs)
        self.write_register(REG_IO, &[0x00]).await?;
        log::info!("Configuring IO register with all GPIOs as outputs");
        // Initial MUX setting (AIN0 to AINCOM)
        let mux = 0x08;
        self.write_register(REG_MUX, &[mux]).await?;
        log::info!("Initial MUX setting (AIN0 to AINCOM)");
        // Perform self-calibration
        self.send_command(CMD_SELFCAL).await?;
        log::info!("Performing self-calibration");
        // Wait for DRDY
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;
        log::info!("Waiting for DRDY to go low");
        Ok(())
    }

    /// Converts raw ADC code to voltage
    pub fn code_to_voltage(&self, code: i32) -> f64 {
        let gain = self.gain.value();
        let max_code = 8388607.0; // Maximum positive ADC value (2^23 - 1)
        (code as f64 * (2.0 * DEFAULT_VREF)) / (gain * max_code)
    }
}
