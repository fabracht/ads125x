use crate::constants::*;
use crate::constants::{DataRate, Gain};
use crate::error::Ads1256Error;
use core::result::Result;
use core::result::Result::Ok;

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    spi::SpiDevice,
};

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
    pub fn init(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Bring PDWN high to enable the device
        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;
        self.delay.delay_ms(10);

        // Reset the device
        self.send_command(CMD_RESET)?;
        self.delay.delay_ms(5);

        // Stop Read Data Continuously mode
        self.send_command(CMD_SDATAC)?;
        self.delay.delay_ms(1);

        // Configure ADCON register
        let adcon = 0x20 | (self.gain as u8); // Clock out frequency = fCLKIN, gain setting
        self.write_register(REG_ADCON, &[adcon])?;

        // Set data rate
        self.write_register(REG_DRATE, &[self.data_rate as u8])?;

        // Set IO register (all GPIOs as outputs)
        self.write_register(REG_IO, &[0x00])?;

        // Perform self-calibration
        self.send_command(CMD_SELFCAL)?;
        self.wait_for_drdy()?;

        Ok(())
    }

    /// Sends a command to the ADS1256
    fn send_command(&mut self, command: u8) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        log::info!("Sending command: 0x{:02X}", command);
        self.spi.write(&[command]).map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;
        Ok(())
    }

    /// Writes data to a register
    pub fn write_register(
        &mut self,
        reg: u8,
        data: &[u8],
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let command = CMD_WREG | (reg & 0x0F);
        let count = (data.len() - 1) as u8;

        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.wait_for_drdy()?;
        self.spi
            .write(&[command, count])
            .map_err(Ads1256Error::Spi)?;
        self.delay.delay_us(1);
        self.spi.write(data).map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;
        Ok(())
    }

    /// Reads data from a register
    pub fn read_register(
        &mut self,
        reg: u8,
        buffer: &mut [u8],
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let command = CMD_RREG | (reg & 0x0F);
        let count = (buffer.len() - 1) as u8;

        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.wait_for_drdy()?;
        self.spi
            .write(&[command, count])
            .map_err(Ads1256Error::Spi)?;
        self.delay.delay_us(1);
        self.spi.read(buffer).map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;
        Ok(())
    }

    /// Waits for DRDY pin to go low
    fn wait_for_drdy(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        while self.drdy.is_high().map_err(Ads1256Error::Gpio)? {
            log::info!("Waiting for DRDY to go low...");
        }
        Ok(())
    }

    /// Reads raw data from the ADC
    pub fn read_data(&mut self) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        self.wait_for_drdy()?;

        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.spi.write(&[CMD_RDATA]).map_err(Ads1256Error::Spi)?;
        self.delay.delay_ms(1);

        let mut buffer = [0u8; 3];
        self.spi.read(&mut buffer).map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;

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
    pub fn read_voltage(&mut self) -> Result<f64, Ads1256Error<SpiError, GpioError>> {
        let code = self.read_data()?;
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

    /// Sets the input multiplexer for single-ended input
    pub fn set_channel(&mut self, channel: u8) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let mux = channel & 0x07; // Adjust based on your channel mapping
        log::info!("Setting channel: {}", mux);
        self.write_register(REG_MUX, &[mux])?;
        self.send_command(CMD_SYNC)?;
        self.send_command(CMD_WAKEUP)?;
        Ok(())
    }

    /// Sets the input multiplexer for differential input
    pub fn set_input_channel(
        &mut self,
        positive: u8,
        negative: u8,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let mux = ((positive & 0x0F) << 4) | (negative & 0x0F);
        self.write_register(REG_MUX, &[mux])?;
        self.send_command(CMD_SYNC)?;
        self.send_command(CMD_WAKEUP)?;
        Ok(())
    }

    /// Sets the gain for the ADC
    pub fn set_gain(&mut self, gain: Gain) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let adcon = 0x20 | (gain as u8);
        self.write_register(REG_ADCON, &[adcon])?;
        self.gain = gain;
        Ok(())
    }

    /// Sets the data rate for the ADC
    pub fn set_data_rate(
        &mut self,
        data_rate: DataRate,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.write_register(REG_DRATE, &[data_rate as u8])?;
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
    pub fn self_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SELFCAL)?;
        self.wait_for_drdy()?;
        Ok(())
    }

    // Perform only offset self-calibration
    pub fn self_offset_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SELFOCAL)?;
        self.wait_for_drdy()?;
        Ok(())
    }

    // Perform only gain self-calibration
    pub fn self_gain_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SELFGCAL)?;
        self.wait_for_drdy()?;
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
    pub fn system_offset_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SYSOCAL)?;
        self.wait_for_drdy()?;
        Ok(())
    }

    // Perform a system gain calibration (requires full-scale input)
    pub fn system_gain_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SYSGCAL)?;
        self.wait_for_drdy()?;
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
    pub fn read_offset_calibration(&mut self) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        let mut buffer = [0u8; 3];
        self.read_register(REG_OFC0, &mut buffer)?;
        Ok(((buffer[0] as i32) << 16) | ((buffer[1] as i32) << 8) | (buffer[2] as i32))
    }

    // Function to read the Full-Scale Calibration registers (FSC0, FSC1, FSC2)
    pub fn read_fullscale_calibration(&mut self) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        let mut buffer = [0u8; 3];
        self.read_register(REG_FSC0, &mut buffer)?;
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
    pub fn write_offset_calibration(
        &mut self,
        value: i32,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let data = [(value >> 16) as u8, (value >> 8) as u8, value as u8];
        self.write_register(REG_OFC0, &data)
    }

    // Manually write to the Full-Scale Calibration registers
    pub fn write_fullscale_calibration(
        &mut self,
        value: i32,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let data = [(value >> 16) as u8, (value >> 8) as u8, value as u8];
        self.write_register(REG_FSC0, &data)
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
    pub fn synchronize_with_pin(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Bring SYNC/PDWN low
        self.pdwn.set_low().map_err(Ads1256Error::Gpio)?;

        // Wait for t16 (timing for the SYNC pulse), this is typically quite short, ~4 clock cycles
        self.delay.delay_us(1); // Adjust this based on timing specs (t16)

        // Bring SYNC/PDWN high to complete synchronization
        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;

        // Wait for the ADS1256 to be ready again (DRDY will go high until data is ready)
        self.wait_for_drdy()?;

        Ok(())
    }

    // Power down the ADS1256 by holding the SYNC/PDWN pin low for 20 DRDY periods
    pub fn enter_power_down(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Hold SYNC/PDWN low for 20 DRDY periods (we'll wait in a loop)
        self.pdwn.set_low().map_err(Ads1256Error::Gpio)?;

        // Wait for 20 DRDY periods, each DRDY period depends on the data rate
        // Assuming a 1000 SPS data rate, the period is 1ms (adjust according to your data rate)
        for _ in 0..20 {
            self.delay.delay_ms(1); // Wait for 1 DRDY period (1ms for 1000 SPS)
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
    pub fn synchronize_with_commands(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Send the SYNC command to stop conversion and wait for the device to sync
        self.send_command(CMD_SYNC)?;

        // To resume and synchronize conversions, send the WAKEUP command
        self.send_command(CMD_WAKEUP)?;

        // After a synchronization, DRDY stays high until data is ready
        self.wait_for_drdy()?;

        Ok(())
    }
}
