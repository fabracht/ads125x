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
    pub drdy: DRDY,
    pdwn: PDWN,
    pub delay: DELAY,
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
            status[0] |= 0x02; // Set BUFEN bit
        } else {
            status[0] &= !0x02; // Clear BUFEN bit
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
        let mux = (0x00 << 4) | 0x08;
        self.write_register(REG_MUX, &[mux])?;

        // Perform self-calibration
        self.send_command(CMD_SELFCAL)?;
        self.wait_for_drdy()?;

        Ok(())
    }

    /// Sends a command to the ADS1256
    pub fn send_command(&mut self, command: u8) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        log::debug!("Sending command: 0x{:02X}", command);
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
        self.delay.delay_us(5);
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
        self.delay.delay_us(5);
        self.spi.read(buffer).map_err(Ads1256Error::Spi)?;
        self.cs.set_high().map_err(Ads1256Error::Gpio)?;
        Ok(())
    }

    // fn wait_for_drdy(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
    //     let period_us = (self.data_rate.period_ms() * 1000.0) as u32;
    //     let max_attempts = 5; // Allow for a few periods worth of waiting
    //     let mut attempts = 0;

    //     while attempts < max_attempts {
    //         // Check DRDY state
    //         if !self.drdy.is_high().map_err(Ads1256Error::Gpio)? {
    //             return Ok(());
    //         }

    //         // Wait for one period
    //         self.delay.delay_us(period_us);
    //         attempts += 1;
    //     }

    //     Err(Ads1256Error::DrdyTimeout {
    //         current_state: true,
    //         wait_time: period_us * max_attempts,
    //     })
    // }

    // fn wait_for_drdy_high(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
    //     let period_us = (self.data_rate.period_ms() * 1000.0) as u32;
    //     let max_attempts = 5; // Allow for a few periods worth of waiting
    //     let mut attempts = 0;

    //     while attempts < max_attempts {
    //         // Check DRDY state
    //         if self.drdy.is_high().map_err(Ads1256Error::Gpio)? {
    //             return Ok(());
    //         }

    //         // Wait for one period
    //         self.delay.delay_us(period_us);
    //         attempts += 1;
    //     }

    //     Err(Ads1256Error::DrdyTimeout {
    //         current_state: false,
    //         wait_time: period_us * max_attempts,
    //     })
    // }

    fn wait_for_drdy(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let timeout = 50000;
        for _ in 0..timeout {
            if self.drdy.is_low().map_err(Ads1256Error::Gpio)? {
                return Ok(());
            }
            self.delay.delay_us(200);
        }
        log::error!("DRDY pin did not go low");
        Err(Ads1256Error::Timeout)
    }

    fn wait_for_drdy_high(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let mut attempts = 0;
        while self.drdy.is_low().map_err(Ads1256Error::Gpio)? {
            self.delay.delay_us(200);
            attempts += 1;
            if attempts > 50000 {
                log::error!("DRDY pin did not go high");
                return Err(Ads1256Error::Timeout);
            }
        }
        Ok(())
    }

    /// Reads raw data from the ADC
    pub fn read_data(&mut self) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        // self.wait_for_drdy()?;

        self.cs.set_low().map_err(Ads1256Error::Gpio)?;
        self.spi.write(&[CMD_RDATA]).map_err(Ads1256Error::Spi)?;

        // Add t6 delay as per datasheet (50 * CLKIN period)
        self.delay.delay_us(7); // For 7.68MHz clock

        let mut buffer = [0u8; 3];
        self.spi.read(&mut buffer).map_err(Ads1256Error::Spi)?;
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

    pub fn read_voltage(&mut self) -> Result<f64, Ads1256Error<SpiError, GpioError>> {
        let code = self.read_data()?;
        let voltage = self.code_to_voltage(code);
        Ok(voltage)
    }

    /// Converts raw ADC code to voltage
    pub fn code_to_voltage(&self, code: i32) -> f64 {
        // Assuming VREF is 2.5V and PGA gain is 1
        let v_ref = 2.5;
        let gain = self.gain.value();
        let max_code = 8388607.0; // Maximum positive ADC value (2^23 - 1)

        // Convert code to voltage
        (code as f64 * (2.0 * v_ref)) / (gain * max_code)
    }

    pub fn set_buffer_enabled(
        &mut self,
        enabled: bool,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Read the current STATUS register value
        let mut status = [0u8; 1];
        self.read_register(REG_STATUS, &mut status)?;

        // Modify the BUFEN bit
        if enabled {
            status[0] |= 0x02; // Set BUFEN bit (Bit 1)
        } else {
            status[0] &= !0x02; // Clear BUFEN bit (Bit 1)
        }

        // Write back to the STATUS register
        self.write_register(REG_STATUS, &status)?;

        // Perform self-calibration after changing buffer setting
        self.send_command(CMD_SELFCAL)?;
        self.wait_for_drdy()?;

        Ok(())
    }

    /// Sets the input multiplexer for single-ended input
    pub fn set_channel(&mut self, channel: u8) -> Result<(), Ads1256Error<SpiError, GpioError>> {
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
        self.write_register(REG_MUX, &[mux])?;

        // Synchronize conversions
        self.send_command(CMD_SYNC)?;
        self.delay.delay_us(100); // Adjust based on tCLKIN
        self.send_command(CMD_WAKEUP)?;

        // Wait for DRDY to go high and then low (new data ready)
        self.wait_for_drdy_high()?; // Wait for DRDY to go high
        self.wait_for_drdy()?; // Wait for DRDY to go low

        Ok(())
    }

    /// Sets the input multiplexer for differential input
    pub fn set_input_channel(
        &mut self,
        positive: u8,
        negative: u8,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let mut mux = 0x00;
        self.wait_for_drdy()?;
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
        self.write_register(REG_MUX, &[mux])?;
        self.send_command(CMD_SYNC)?;
        self.delay.delay_us(100); // Adjust based on tCLKIN
        self.send_command(CMD_WAKEUP)?;

        self.wait_for_drdy_high()?; // Wait for DRDY to go high
        self.wait_for_drdy()?; // Wait for DRDY to go low
        Ok(())
    }

    /// Sets the gain for the ADC
    pub fn set_gain(&mut self, gain: Gain) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let adcon = 0x20 | (gain as u8);
        self.write_register(REG_ADCON, &[adcon])?;
        self.gain = gain;

        // Perform self-calibration
        self.send_command(CMD_SELFCAL)?;
        self.wait_for_drdy()?;
        Ok(())
    }

    /// Sets the data rate for the ADC
    pub fn set_data_rate(
        &mut self,
        data_rate: DataRate,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.write_register(REG_DRATE, &[data_rate as u8])?;
        self.data_rate = data_rate;

        // Perform self-calibration
        self.send_command(CMD_SELFCAL)?;
        self.wait_for_drdy()?;
        Ok(())
    }

    // Self-calibration methods
    pub fn self_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SELFCAL)?;
        self.wait_for_drdy()?;
        Ok(())
    }

    pub fn self_offset_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SELFOCAL)?;
        self.wait_for_drdy()?;
        Ok(())
    }

    pub fn self_gain_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SELFGCAL)?;
        self.wait_for_drdy()?;
        Ok(())
    }

    pub fn system_offset_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SYSOCAL)?;
        self.wait_for_drdy()?;
        Ok(())
    }

    pub fn system_gain_calibrate(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.send_command(CMD_SYSGCAL)?;
        self.wait_for_drdy()?;
        Ok(())
    }

    // Read calibration registers
    pub fn read_offset_calibration(&mut self) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        let mut buffer = [0u8; 3];
        self.read_register(REG_OFC0, &mut buffer)?;
        Ok(((buffer[0] as i32) << 16) | ((buffer[1] as i32) << 8) | (buffer[2] as i32))
    }

    pub fn read_fullscale_calibration(&mut self) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        let mut buffer = [0u8; 3];
        self.read_register(REG_FSC0, &mut buffer)?;
        Ok(((buffer[0] as i32) << 16) | ((buffer[1] as i32) << 8) | (buffer[2] as i32))
    }

    pub fn write_offset_calibration(
        &mut self,
        value: i32,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let data = [(value >> 16) as u8, (value >> 8) as u8, value as u8];
        self.write_register(REG_OFC0, &data)
    }

    pub fn write_fullscale_calibration(
        &mut self,
        value: i32,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        let data = [(value >> 16) as u8, (value >> 8) as u8, value as u8];
        self.write_register(REG_FSC0, &data)
    }

    pub fn print_registers(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
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
            self.read_register(*reg, &mut buffer)?;
            log::debug!("Register {}: 0x{:02X}", name, buffer[0]);
        }

        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Pull PDWN low
        self.pdwn.set_low().map_err(Ads1256Error::Gpio)?;
        self.delay.delay_ms(10);

        // Pull PDWN high
        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;
        self.delay.delay_ms(10);

        // Send reset command
        self.send_command(CMD_RESET)?;
        self.delay.delay_ms(50); // Increased delay after reset command

        // Wait for DRDY to go low
        self.wait_for_drdy()?;

        // Perform self-calibration
        self.send_command(CMD_SELFCAL)?;
        self.wait_for_drdy()?;

        Ok(())
    }

    pub fn synchronize_with_pin(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Bring SYNC/PDWN low
        self.pdwn.set_low().map_err(Ads1256Error::Gpio)?;

        // Wait for t16 (timing for the SYNC pulse), typically quite short, ~4 clock cycles
        self.delay.delay_us(5); // Adjust based on timing specs (t16)

        // Bring SYNC/PDWN high to complete synchronization
        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;

        // Wait for the ADS1256 to be ready again (DRDY goes high until data is ready)
        self.wait_for_drdy()?;

        Ok(())
    }

    pub fn enter_power_down(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        // Hold SYNC/PDWN low for 20 DRDY periods
        self.pdwn.set_low().map_err(Ads1256Error::Gpio)?;

        // Wait for 20 DRDY periods (depends on the data rate)
        let drdy_period_ms = self.data_rate.period_ms();
        self.delay.delay_ms((20.0 * drdy_period_ms) as u32);

        Ok(())
    }

    pub fn exit_power_down(&mut self) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        self.pdwn.set_high().map_err(Ads1256Error::Gpio)?;
        self.delay.delay_ms(30); // Wait for oscillator startup
        Ok(())
    }

    /// Cycles to a new channel using the optimized sequence from the datasheet.
    /// This method follows the recommended sequence:
    /// 1. Update MUX register when DRDY is low
    /// 2. Issue SYNC and WAKEUP commands
    /// 3. Return data from previous conversion
    ///
    /// Returns the data from the previous channel's conversion.
    pub fn cycle_channel(&mut self, channel: u8) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        // Wait for DRDY to ensure we can change the MUX
        self.wait_for_drdy()?;

        // Set the new channel in MUX register
        // Check if channel number is within valid range
        if channel > 7 {
            return Err(Ads1256Error::InvalidInputChannel);
        }

        // Set AINP to the desired channel (AIN0 to AIN7)
        let positive = channel & 0x07;
        // Set AINN to AINCOM
        let negative = 0x08;
        // Construct MUX register value
        let mux = (positive << 4) | negative;

        log::debug!(
            "Setting MUX register to: 0x{:02X} (AINP = AIN{}, AINN = AINCOM)",
            mux,
            positive
        );

        // Write to MUX register
        self.write_register(REG_MUX, &[mux])?;

        // Restart conversion process with SYNC and WAKEUP
        self.send_command(CMD_SYNC)?;
        self.delay.delay_us(100); // t11 delay between commands
        self.send_command(CMD_WAKEUP)?;

        // Wait for DRDY to go high and then low (new data ready)
        self.wait_for_drdy_high()?; // Wait for DRDY to go high
        self.wait_for_drdy()?; // Wait for DRDY to go low

        // Read and return the data from the previous conversion
        self.read_data()
    }

    /// Cycles to a new differential input channel pair using the optimized sequence.
    /// This method allows selecting both positive and negative inputs for differential measurements.
    pub fn cycle_differential_channel(
        &mut self,
        positive: u8,
        negative: u8,
    ) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        // Wait for DRDY to ensure we can change the MUX
        self.wait_for_drdy()?;

        // Validate inputs
        if positive > 7 || negative > 7 {
            return Err(Ads1256Error::InvalidInputChannel);
        }

        // Construct MUX register value
        let mux = (positive << 4) | negative;

        log::debug!(
            "Setting MUX register to: 0x{:02X} (AINP = AIN{}, AINN = AIN{})",
            mux,
            positive,
            negative
        );

        // Write to MUX register
        self.write_register(REG_MUX, &[mux])?;

        // Restart conversion process with SYNC and WAKEUP
        self.send_command(CMD_SYNC)?;
        self.delay.delay_us(100); // t11 delay between commands
        self.send_command(CMD_WAKEUP)?;

        // Wait for DRDY to go high and then low (new data ready)
        self.wait_for_drdy_high()?; // Wait for DRDY to go high
        self.wait_for_drdy()?; // Wait for DRDY to go low

        // Read and return the data from the previous conversion
        self.read_data()
    }
}
