// src/nonblocking/channels.rs

// src/nonblocking/channels.rs
use crate::{
    constants::*,
    error::Ads1256Error,
    nonblocking::{utils::yield_now, Ads1256NonBlocking},
};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};
use heapless::Vec;

const MAX_CHANNELS: usize = 8;

impl<SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    /// Change channel with non-blocking operations
    pub async fn change_channel(
        &mut self,
        channel: u8,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        if channel > 7 {
            return Err(Ads1256Error::InvalidInputChannel);
        }

        // Wait for DRDY in non-blocking way
        while !self.drdy.is_low().map_err(Ads1256Error::Gpio)? {
            yield_now().await;
        }

        // Update MUX register
        let positive = channel & 0x07;
        let negative = 0x08; // AINCOM
        let mux = (positive << 4) | negative;
        self.write_register(REG_MUX, &[mux]).await?;

        // Synchronize the ADC
        self.send_command(CMD_SYNC).await?;
        self.delay.delay_us(T11_DELAY).await;
        self.send_command(CMD_WAKEUP).await?;

        self.current_channel = Some(channel);
        Ok(())
    }

    /// Efficient channel cycling implementation
    pub async fn cycle_channel(
        &mut self,
        channel: u8,
    ) -> Result<i32, Ads1256Error<SpiError, GpioError>> {
        if channel > 7 {
            return Err(Ads1256Error::InvalidInputChannel);
        }

        // Wait for DRDY
        while !self.drdy.is_low().map_err(Ads1256Error::Gpio)? {
            yield_now().await;
        }

        // Update MUX register for next reading
        let positive = channel & 0x07;
        let negative = 0x08; // AINCOM
        let mux = (positive << 4) | negative;
        self.write_register(REG_MUX, &[mux]).await?;

        // Restart conversion process
        self.send_command(CMD_SYNC).await?;
        self.delay.delay_us(T11_DELAY).await;
        self.send_command(CMD_WAKEUP).await?;

        // Read data from previous conversion
        let result = self.read_data().await?;

        self.current_channel = Some(channel);
        Ok(result)
    }

    /// Create a channel sequencer for automated cycling
    pub fn create_channel_sequence(
        &mut self,
        channels: &[u8],
    ) -> ChannelSequencer<'_, SPI, CS, DRDY, PDWN, DELAY> {
        let mut seq_channels: Vec<u8, MAX_CHANNELS> = Vec::new();
        for &ch in channels.iter().take(MAX_CHANNELS) {
            if ch <= 7 {
                // Ignore invalid channels
                let _ = seq_channels.push(ch);
            }
        }
        ChannelSequencer::new(self, seq_channels)
    }
}

/// Channel sequencer for automated channel cycling
pub struct ChannelSequencer<'a, SPI, CS, DRDY, PDWN, DELAY> {
    adc: &'a mut Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY>,
    channels: Vec<u8, MAX_CHANNELS>,
    current_index: usize,
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY> ChannelSequencer<'a, SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice,
    CS: OutputPin,
    DRDY: InputPin,
    PDWN: OutputPin,
    DELAY: DelayNs,
{
    fn new(
        adc: &'a mut Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY>,
        channels: Vec<u8, MAX_CHANNELS>,
    ) -> Self {
        Self {
            adc,
            channels,
            current_index: 0,
        }
    }

    /// Gets the next reading in the sequence
    pub async fn next<SpiError, GpioError>(
        &mut self,
    ) -> Option<Result<(u8, i32), Ads1256Error<SpiError, GpioError>>>
    where
        SPI: SpiDevice<Error = SpiError>,
        CS: OutputPin<Error = GpioError>,
        DRDY: InputPin<Error = GpioError>,
        PDWN: OutputPin<Error = GpioError>,
    {
        if self.current_index >= self.channels.len() {
            self.current_index = 0;
            if self.channels.is_empty() {
                return None;
            }
        }

        let channel = self.channels[self.current_index];
        let result = self
            .adc
            .cycle_channel(channel)
            .await
            .map(|value| (channel, value));

        self.current_index += 1;
        Some(result)
    }
}
