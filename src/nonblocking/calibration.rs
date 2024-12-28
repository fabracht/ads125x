// src/nonblocking/calibration.rs
use crate::{constants::*, error::Ads1256Error, nonblocking::Ads1256NonBlocking};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::{delay::DelayNs, digital::Wait, spi::SpiDevice};

impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: Wait + OutputPin<Error = GpioError>,
    DRDY: Wait + InputPin<Error = GpioError>,
    PDWN: Wait + OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    /// Perform self-calibration
    pub async fn self_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SELFCAL).await?;

        // Wait for DRDY in non-blocking way
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;

        Ok(())
    }

    /// Perform self offset calibration
    pub async fn self_offset_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SELFOCAL).await?;

        // Wait for DRDY in non-blocking way
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;

        Ok(())
    }

    /// Perform self gain calibration
    pub async fn self_gain_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SELFGCAL).await?;

        // Wait for DRDY in non-blocking way
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;

        Ok(())
    }

    /// Perform system offset calibration
    pub async fn system_offset_calibrate(
        &mut self,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SYSOCAL).await?;

        // Wait for DRDY in non-blocking way
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;

        Ok(())
    }

    /// Perform system gain calibration
    pub async fn system_gain_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SYSGCAL).await?;

        // Wait for DRDY in non-blocking way
        self.drdy.wait_for_low().await.map_err(Ads1256Error::Gpio)?;

        Ok(())
    }

    /// Read offset calibration registers
    pub async fn read_offset_calibration(
        &mut self,
    ) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        let mut buffer = [0u8; 3];
        self.read_register(REG_OFC0, &mut buffer[0..1]).await?;
        self.read_register(REG_OFC1, &mut buffer[1..2]).await?;
        self.read_register(REG_OFC2, &mut buffer[2..3]).await?;
        Ok(((buffer[0] as i32) << 16) | ((buffer[1] as i32) << 8) | (buffer[2] as i32))
    }

    /// Read gain calibration registers
    pub async fn read_gain_calibration(
        &mut self,
    ) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        let mut buffer = [0u8; 3];
        self.read_register(REG_FSC0, &mut buffer[0..1]).await?;
        self.read_register(REG_FSC1, &mut buffer[1..2]).await?;
        self.read_register(REG_FSC2, &mut buffer[2..3]).await?;
        Ok(((buffer[0] as i32) << 16) | ((buffer[1] as i32) << 8) | (buffer[2] as i32))
    }

    /// Write to offset calibration registers
    pub async fn write_offset_calibration(
        &mut self,
        value: i32,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let data = [(value >> 16) as u8, (value >> 8) as u8, value as u8];
        self.write_register(REG_OFC0, &data[0..1]).await?;
        self.write_register(REG_OFC1, &data[1..2]).await?;
        self.write_register(REG_OFC2, &data[2..3]).await?;
        Ok(())
    }

    /// Write to gain calibration registers
    pub async fn write_gain_calibration(
        &mut self,
        value: i32,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let data = [(value >> 16) as u8, (value >> 8) as u8, value as u8];
        self.write_register(REG_FSC0, &data[0..1]).await?;
        self.write_register(REG_FSC1, &data[1..2]).await?;
        self.write_register(REG_FSC2, &data[2..3]).await?;
        Ok(())
    }

    /// Set PGA gain and perform self calibration
    pub async fn set_gain(&mut self, gain: Gain) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let adcon = gain as u8;
        self.write_register(REG_ADCON, &[adcon]).await?;
        self.gain = gain;
        self.self_calibrate().await?;
        Ok(())
    }

    /// Set data rate and perform self calibration
    pub async fn set_data_rate(
        &mut self,
        data_rate: DataRate,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.write_register(REG_DRATE, &[data_rate as u8]).await?;
        self.data_rate = data_rate;
        self.self_calibrate().await?;
        Ok(())
    }

    /// Print all register values for debugging
    pub async fn print_registers(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let registers = [
            (REG_STATUS, "STATUS"),
            (REG_MUX, "MUX"),
            (REG_ADCON, "ADCON"),
            (REG_DRATE, "DRATE"),
            (REG_IO, "IO"),
            (REG_OFC0, "OFC0"),
            (REG_OFC1, "OFC1"),
            (REG_OFC2, "OFC2"),
            (REG_FSC0, "FSC0"),
            (REG_FSC1, "FSC1"),
            (REG_FSC2, "FSC2"),
        ];

        for (reg, name) in registers.iter() {
            let mut buffer = [0u8; 1];
            self.read_register(*reg, &mut buffer).await?;
            log::debug!("Register {}: 0x{:02X}", name, buffer[0]);
        }

        Ok(())
    }
}
