use crate::constants::*;
use crate::constants::{DataRate, Gain};
use crate::error::Ads1256Error;
use core::result::Result;
use core::result::Result::Ok;

use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

use embedded_hal::digital::{InputPin, OutputPin};

/// ADS1256 driver
pub struct Ads1256<SPI, CS, DRDY, PDWN, DELAY> {
    spi: SPI,
    cs: CS,
    drdy: DRDY,
    pdwn: PDWN,
    delay: DELAY,
    gain: Gain,
    data_rate: DataRate,
}

impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    /// Creates a new ADS1256 driver instance
    pub fn new(
        spi: SPI,
        cs: CS,
        drdy: DRDY,
        pdwn: PDWN,
        delay: DELAY,
        gain: Gain,
        data_rate: DataRate,
    ) -> Self {
        Ads1256 {
            spi,
            cs,
            drdy,
            pdwn,
            delay,
            gain,
            data_rate,
        }
    }

    /// Initializes the ADS1256 ADC module with default settings.
    pub async fn init(
        &mut self,
        buffer_enabled: bool,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Bring PDWN high to enable the device
        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;
        self.delay.delay_ms(10);

        // Reset the device
        self.send_command(CMD_RESET).await?;
        self.wait_for_drdy().await?; // Wait for DRDY instead of fixed delay

        // Stop Read Data Continuously mode
        self.send_command(CMD_SDATAC).await?;
        self.wait_for_drdy().await?;

        // Read current STATUS register
        // Configure STATUS register with BUFEN setting
        let mut status = [0u8; 1];
        self.read_register(REG_STATUS, &mut status).await?;
        if buffer_enabled {
            status[0] |= 0x02; // Set BUFEN bit
        } else {
            status[0] &= !0x02; // Clear BUFEN bit
        }
        self.write_register(REG_STATUS, &status).await?;

        // Configure ADCON register
        // Corrected to set BUFEN (Bit 4)
        let adcon = 0x00 | (self.gain as u8); // Set BUFEN as needed
        self.write_register(REG_ADCON, &[adcon]).await?;

        // Set data rate
        self.write_register(REG_DRATE, &[self.data_rate as u8])
            .await?;

        // Set IO register (all GPIOs as outputs)
        self.write_register(REG_IO, &[0x00]).await?;

        // Configure MUX register for initial channel (e.g., AIN0 single-ended)
        let mux = (0x00 << 4) | 0x08; // AINP = AIN0, AINN = AINCOM
        self.write_register(REG_MUX, &[mux]).await?;

        // Perform self-calibration
        self.send_command(CMD_SELFCAL).await?;
        self.wait_for_drdy().await?;

        Ok(())
    }

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
        self.wait_for_drdy().await?;

        // Perform self-calibration
        self.send_command(CMD_SELFCAL).await?;
        self.wait_for_drdy().await?;

        Ok(())
    }

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

    /// Sends a command to the ADS1256
    async fn send_command(&mut self, command: u8) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        log::debug!("Sending command: 0x{:02X}", command);
        self.spi
            .write(&[command])
            .await
            .map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;
        Ok(())
    }

    /// Writes data to a register
    pub async fn write_register(
        &mut self,
        reg: u8,
        data: &[u8],
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let command = CMD_WREG | (reg & 0x0F);
        let count = (data.len() - 1) as u8;

        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.wait_for_drdy().await?;
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
    pub async fn read_register(
        &mut self,
        reg: u8,
        buffer: &mut [u8],
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let command = CMD_RREG | (reg & 0x0F);
        let count = (buffer.len() - 1) as u8;

        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.wait_for_drdy().await?;
        self.spi
            .write(&[command, count])
            .await
            .map_err(Ads1256Error::Spi)?;
        self.delay.delay_us(5).await;
        self.spi.read(buffer).await.map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;
        Ok(())
    }

    /// Waits for DRDY pin to go low
    async fn wait_for_drdy(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let timeout = 1000;
        for _ in 0..timeout {
            if self.drdy.is_low().map_err(Ads1256Error::Gpio)? {
                return Ok(());
            }
            self.delay.delay_us(200).await;
        }
        log::error!("DRDY pin did not go low");
        Err(Ads1256Error::Timeout)
    }

    async fn wait_for_drdy_high(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let mut attempts = 0;
        while self.drdy.is_low().map_err(Ads1256Error::Gpio)? {
            self.delay.delay_us(200).await;
            attempts += 1;
            if attempts > 1000 {
                log::error!("DRDY pin did not go high");
                return Err(Ads1256Error::Timeout);
            }
        }
        Ok(())
    }

    /// Reads raw data from the ADC
    pub async fn read_data(&mut self) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        self.wait_for_drdy().await?;

        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.spi
            .write(&[CMD_RDATA])
            .await
            .map_err(Ads1256Error::Spi)?;

        let mut buffer = [0u8; 3];
        self.spi
            .read(&mut buffer)
            .await
            .map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;

        log::debug!(
            "Raw data: {:02X} {:02X} {:02X}",
            buffer[0],
            buffer[1],
            buffer[2]
        );
        // Convert 24-bit data to signed 32-bit integer
        let raw_value = ((buffer[0] as i32) << 16) | ((buffer[1] as i32) << 8) | (buffer[2] as i32);
        // Sign extension for negative values
        let value = if raw_value & 0x800000 != 0 {
            raw_value | !0xFFFFFF
        } else {
            raw_value
        };

        Ok(value)
    }

    /// Reads the voltage from the ADC
    pub async fn read_voltage(&mut self) -> Result<f64, Ads1256Error<SpiError, GpioError>> {
        let code = self.read_data().await?;
        let voltage = self.code_to_voltage(code);
        Ok(voltage)
    }

    /// Converts raw ADC code to voltage
    fn code_to_voltage(&self, code: i32) -> f64 {
        let v_ref = 2.5; // Reference voltage
        let gain = self.gain.value(); // Get the gain value based on the current PGA setting
        let max_code = 8388607.0; // 0x7FFFFF

        (code as f64 * (2.0 * v_ref)) / (gain * max_code)
    }

    pub async fn set_buffer_enabled(
        &mut self,
        enabled: bool,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Read the current STATUS register value
        let mut status = [0u8; 1];
        self.read_register(REG_STATUS, &mut status).await?;

        // Modify the BUFEN bit
        if enabled {
            status[0] |= 0x02; // Set BUFEN bit (Bit 1)
        } else {
            status[0] &= !0x02; // Clear BUFEN bit (Bit 1)
        }

        // Write back to the STATUS register
        self.write_register(REG_STATUS, &status).await?;

        // Perform self-calibration if necessary
        // It's recommended to recalibrate after changing the buffer setting
        self.send_command(CMD_SELFCAL).await?;
        self.wait_for_drdy().await?;

        Ok(())
    }

    /// Sets the input multiplexer for single-ended input
    pub async fn set_channel(
        &mut self,
        channel: u8,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Check if channel number is within valid range
        if channel > 7 {
            return Err(Ads1256Error::InvalidInputChannel);
        }

        // Set AINP to the desired channel (AIN0 to AIN7)
        let positive = channel & 0x07;

        // Set AINN to AINCOM (assuming AINCOM is connected via GND pins)
        let negative = 0x08; // Code for AINCOM

        // Construct MUX register value
        let mux = (positive << 4) | negative;

        log::debug!(
            "Setting MUX register to: 0x{:02X} (AINP = AIN{}, AINN = AINCOM)",
            mux,
            positive
        );

        // Write to MUX register
        self.write_register(REG_MUX, &[mux]).await?;

        // Synchronize conversions
        self.send_command(CMD_SYNC).await?;

        // Wait for minimum delay t11
        self.delay.delay_us(100).await; // Adjust based on tCLKIN

        // Wake up the ADC
        self.send_command(CMD_WAKEUP).await?;

        // Wait for DRDY to go high and then low (new data ready)
        self.wait_for_drdy_high().await?; // Wait for DRDY to go high
        self.wait_for_drdy().await?; // Wait for DRDY to go low

        Ok(())
    }

    /// Sets the input multiplexer for differential input
    pub async fn set_input_channel(
        &mut self,
        positive: u8,
        negative: u8,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let mut mux = 0x00;
        self.wait_for_drdy().await?;
        // Validate and set positive input
        if positive == 0x08 {
            // AINCOM is not available
            return Err(Ads1256Error::InvalidInputChannel);
        } else if positive <= 0x07 {
            mux |= (positive & 0x07) << 4;
        } else {
            return Err(Ads1256Error::InvalidInputChannel);
        }

        // Validate and set negative input
        if negative == 0x08 {
            // AINCOM is not available
            return Err(Ads1256Error::InvalidInputChannel);
        } else if negative <= 0x07 {
            mux |= negative & 0x07;
        } else {
            return Err(Ads1256Error::InvalidInputChannel);
        }

        // Write MUX register and synchronize
        self.write_register(REG_MUX, &[mux]).await?;
        self.send_command(CMD_SYNC).await?;
        self.delay.delay_us(100).await; // Adjust based on tCLKIN
        self.send_command(CMD_WAKEUP).await?;

        self.wait_for_drdy_high().await?; // Wait for DRDY to go high
        self.wait_for_drdy().await?; // Wait for DRDY to go low
        Ok(())
    }

    /// Sets the gain for the ADC
    pub async fn set_gain(&mut self, gain: Gain) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let adcon = 0x20 | (gain as u8);
        self.write_register(REG_ADCON, &[adcon]).await?;
        self.gain = gain;
        Ok(())
    }

    /// Sets the data rate for the ADC
    pub async fn set_data_rate(
        &mut self,
        data_rate: DataRate,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.write_register(REG_DRATE, &[data_rate as u8]).await?;
        self.data_rate = data_rate;
        Ok(())
    }
}

// Implement self-calibration options
impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    // Perform a full self-calibration (both offset and gain)
    pub async fn self_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SELFCAL).await?;
        self.wait_for_drdy().await?;
        Ok(())
    }

    // Perform only offset self-calibration
    pub async fn self_offset_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SELFOCAL).await?;
        self.wait_for_drdy().await?;
        Ok(())
    }

    // Perform only gain self-calibration
    pub async fn self_gain_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SELFGCAL).await?;
        self.wait_for_drdy().await?;
        Ok(())
    }
}

// Implement system calibration options
impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    // Perform a system offset calibration (requires zero differential input)
    pub async fn system_offset_calibrate(
        &mut self,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Step 1: Configure inputs for zero differential input
        // For example, short AIN0 and AIN1
        self.set_input_channel(0x00, 0x01).await?; // AINP = AIN0, AINN = AIN1

        // Ensure that AIN0 is properly connected (e.g., connected to AGND)
        // This may require hardware setup outside of the ADC

        // Step 2: Perform calibration
        self.send_command(CMD_SYSOCAL).await?;
        self.wait_for_drdy().await?;
        Ok(())
    }

    // Perform a system gain calibration (requires full-scale input)
    pub async fn system_gain_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Step 1: Configure inputs for full-scale differential input
        self.set_input_channel(0x00, 0x01).await?; // AINP = AIN0, AINN = AIN1

        // Apply full-scale voltage across AIN0 and AIN1
        // The voltage should be (±2 * VREF) / PGA
        // Ensure that the applied voltage does not exceed the absolute maximum ratings

        // Step 2: Perform calibration
        self.send_command(CMD_SYSGCAL).await?;
        self.wait_for_drdy().await?;
        Ok(())
    }
}

// Read from the calibration registers
impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    // Function to read the Offset Calibration registers (OFC0, OFC1, OFC2)
    pub async fn read_offset_calibration(
        &mut self,
    ) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        let mut buffer = [0u8; 3];
        self.read_register(REG_OFC0, &mut buffer).await?;
        Ok(((buffer[0] as i32) << 16) | ((buffer[1] as i32) << 8) | (buffer[2] as i32))
    }

    // Function to read the Full-Scale Calibration registers (FSC0, FSC1, FSC2)
    pub async fn read_fullscale_calibration(
        &mut self,
    ) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        let mut buffer = [0u8; 3];
        self.read_register(REG_FSC0, &mut buffer).await?;
        Ok(((buffer[0] as i32) << 16) | ((buffer[1] as i32) << 8) | (buffer[2] as i32))
    }
}

// Writing to calibration register
impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    // Manually write to the Offset Calibration registers
    pub async fn write_offset_calibration(
        &mut self,
        value: i32,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let data = [(value >> 16) as u8, (value >> 8) as u8, value as u8];
        self.write_register(REG_OFC0, &data).await
    }

    // Manually write to the Full-Scale Calibration registers
    pub async fn write_fullscale_calibration(
        &mut self,
        value: i32,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let data = [(value >> 16) as u8, (value >> 8) as u8, value as u8];
        self.write_register(REG_FSC0, &data).await
    }
}

// Implement Synchronization and power down
impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    // Synchronize using the SYNC/PDWN pin
    pub async fn synchronize_with_pin(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Bring SYNC/PDWN low
        self.pdwn.set_low().map_err(Ads1256Error::Gpio)?;

        // Wait for t16 (timing for the SYNC pulse), this is typically quite short, ~4 clock cycles
        self.delay.delay_us(5).await; // Adjust this based on timing specs (t16)

        // Bring SYNC/PDWN high to complete synchronization
        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;

        // Wait for the ADS1256 to be ready again (DRDY will go high until data is ready)
        self.wait_for_drdy().await?;

        Ok(())
    }

    // Power down the ADS1256 by holding the SYNC/PDWN pin low for 20 DRDY periods
    pub async fn enter_power_down(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Hold SYNC/PDWN low for 20 DRDY periods (we'll wait in a loop)
        self.pdwn.set_low().map_err(Ads1256Error::Gpio)?;

        // Wait for 20 DRDY periods, each DRDY period depends on the data rate
        // Assuming a 1000 SPS data rate, the period is 1ms (adjust according to your data rate)
        for _ in 0..20 {
            self.delay.delay_ms(1).await; // Wait for 1 DRDY period (1ms for 1000 SPS)
        }

        Ok(())
    }
}

// Implement synchronization via SYNC command
impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    // Synchronize using the SYNC and WAKEUP commands
    pub async fn synchronize_with_commands(
        &mut self,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Send the SYNC command to stop conversion and wait for the device to sync
        self.send_command(CMD_SYNC).await?;

        // To resume and synchronize conversions, send the WAKEUP command
        self.send_command(CMD_WAKEUP).await?;

        // After a synchronization, DRDY stays high until data is ready
        self.wait_for_drdy().await?;

        Ok(())
    }
}
