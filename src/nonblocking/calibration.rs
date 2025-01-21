// src/nonblocking/calibration.rs
use crate::{constants::*, error::Ads1256Error, nonblocking::Ads1256NonBlocking};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::{delay::DelayNs, digital::Wait, spi::SpiDevice};

impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: Wait + InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
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
        // Read all registers first
        let registers = [
            (
                REG_STATUS,
                "STATUS",
                ["ID3", "ID2", "ID1", "ID0", "ORDER", "ACAL", "BUFEN", "DRDY"],
            ),
            (
                REG_MUX,
                "MUX",
                [
                    "PSEL3", "PSEL2", "PSEL1", "PSEL0", "NSEL3", "NSEL2", "NSEL1", "NSEL0",
                ],
            ),
            (
                REG_ADCON,
                "ADCON",
                [
                    "0", "CLK1", "CLK0", "SDCS1", "SDCS0", "PGA2", "PGA1", "PGA0",
                ],
            ),
            (
                REG_DRATE,
                "DRATE",
                ["DR7", "DR6", "DR5", "DR4", "DR3", "DR2", "DR1", "DR0"],
            ),
            (
                REG_IO,
                "IO",
                [
                    "DIR3", "DIR2", "DIR1", "DIR0", "DIO3", "DIO2", "DIO1", "DIO0",
                ],
            ),
            (
                REG_OFC0,
                "OFC0",
                [
                    "OFC07", "OFC06", "OFC05", "OFC04", "OFC03", "OFC02", "OFC01", "OFC00",
                ],
            ),
            (
                REG_OFC1,
                "OFC1",
                [
                    "OFC15", "OFC14", "OFC13", "OFC12", "OFC11", "OFC10", "OFC09", "OFC08",
                ],
            ),
            (
                REG_OFC2,
                "OFC2",
                [
                    "OFC23", "OFC22", "OFC21", "OFC20", "OFC19", "OFC18", "OFC17", "OFC16",
                ],
            ),
            (
                REG_FSC0,
                "FSC0",
                [
                    "FSC07", "FSC06", "FSC05", "FSC04", "FSC03", "FSC02", "FSC01", "FSC00",
                ],
            ),
            (
                REG_FSC1,
                "FSC1",
                [
                    "FSC15", "FSC14", "FSC13", "FSC12", "FSC11", "FSC10", "FSC09", "FSC08",
                ],
            ),
            (
                REG_FSC2,
                "FSC2",
                [
                    "FSC23", "FSC22", "FSC21", "FSC20", "FSC19", "FSC18", "FSC17", "FSC16",
                ],
            ),
        ];

        // Print header
        log::info!("Register Map:");
        log::info!("ADDRESS REGISTER VALUE  BIT7  BIT6  BIT5  BIT4  BIT3  BIT2  BIT1  BIT0");
        log::info!("------- -------- ------ ----- ----- ----- ----- ----- ----- ----- -----");

        for (reg, name, bits) in registers.iter() {
            let mut buffer = [0u8; 1];
            self.read_register(*reg, &mut buffer).await?;
            let value = buffer[0];

            // Print register info with bit values
            log::info!("{:02X}h     {:6}   0x{:02X}   {:4}   {:4}   {:4}   {:4}   {:4}   {:4}   {:4}   {:4}", 
                reg, name, value,
                if (value & 0x80) != 0 {"1"} else {"0"},
                if (value & 0x40) != 0 {"1"} else {"0"},
                if (value & 0x20) != 0 {"1"} else {"0"},
                if (value & 0x10) != 0 {"1"} else {"0"},
                if (value & 0x08) != 0 {"1"} else {"0"},
                if (value & 0x04) != 0 {"1"} else {"0"},
                if (value & 0x02) != 0 {"1"} else {"0"},
                if (value & 0x01) != 0 {"1"} else {"0"}
            );

            // Print bit names
            log::info!(
                "                     {:4}   {:4}   {:4}   {:4}   {:4}   {:4}   {:4}   {:4}",
                bits[0],
                bits[1],
                bits[2],
                bits[3],
                bits[4],
                bits[5],
                bits[6],
                bits[7]
            );
            log::info!("");
        }

        Ok(())
    }
}
