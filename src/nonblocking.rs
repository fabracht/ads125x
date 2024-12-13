use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;

use crate::{Ads1256, Ads1256Error};

/// Represents the current state of an ADC conversion
#[derive(Debug)]
enum ConversionState {
    /// Ready to start conversion
    Ready,
    /// Conversion has been initiated and waiting for completion
    Converting,
    /// Conversion is complete
    Complete,
}

/// A future that represents a non-blocking ADC conversion
pub struct AdcConversion<'a, SPI, CS, DRDY, PDWN, DELAY, GpioError> {
    adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>,
    state: ConversionState,
    config: ConversionConfig,
    _gpio_error: core::marker::PhantomData<GpioError>,
}

/// Configuration for different types of conversions
#[derive(Debug)]
enum ConversionConfig {
    /// Single-ended measurement on specified channel
    SingleEnded(u8),
    /// Differential measurement between specified channels
    Differential(u8, u8),
    /// Direct conversion using current configuration
    Direct,
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY, GpioError>
    AdcConversion<'a, SPI, CS, DRDY, PDWN, DELAY, GpioError>
{
    /// Creates a new ADC conversion using current settings
    pub fn new(adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>) -> Self {
        Self {
            adc,
            state: ConversionState::Ready,
            config: ConversionConfig::Direct,
            _gpio_error: core::marker::PhantomData,
        }
    }

    /// Creates a new single-ended conversion on specified channel
    pub fn new_single_ended(adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>, channel: u8) -> Self {
        Self {
            adc,
            state: ConversionState::Ready,
            config: ConversionConfig::SingleEnded(channel),
            _gpio_error: core::marker::PhantomData,
        }
    }

    /// Creates a new differential conversion between specified channels
    pub fn new_differential(
        adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>,
        positive: u8,
        negative: u8,
    ) -> Self {
        Self {
            adc,
            state: ConversionState::Ready,
            config: ConversionConfig::Differential(positive, negative),
            _gpio_error: core::marker::PhantomData,
        }
    }

    // Helper function to start the appropriate type of conversion
    fn start_conversion(&mut self) -> Result<(), Ads1256Error<SPI::Error, GpioError>>
    where
        SPI: SpiDevice,
        CS: OutputPin<Error = GpioError>,
        DRDY: InputPin<Error = GpioError>,
        PDWN: OutputPin<Error = GpioError>,
        DELAY: DelayNs,
    {
        match self.config {
            ConversionConfig::SingleEnded(channel) => {
                let _ = self.adc.cycle_channel(channel)?;
                Ok(())
            }
            ConversionConfig::Differential(pos, neg) => {
                let _ = self.adc.cycle_differential_channel(pos, neg)?;
                Ok(())
            }
            ConversionConfig::Direct => self.adc.start_one_shot(),
        }
    }
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Future
    for AdcConversion<'a, SPI, CS, DRDY, PDWN, DELAY, GpioError>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    type Output = Result<i32, Ads1256Error<SpiError, GpioError>>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = unsafe { self.get_unchecked_mut() };

        match this.state {
            ConversionState::Ready => {
                // Start the appropriate type of conversion
                match this.start_conversion() {
                    Ok(()) => {
                        this.state = ConversionState::Converting;
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Err(e) => Poll::Ready(Err(e)),
                }
            }
            ConversionState::Converting => {
                // Check if conversion is complete
                match this.adc.read_one_shot_nb() {
                    Ok(value) => {
                        this.state = ConversionState::Complete;
                        Poll::Ready(Ok(value))
                    }
                    Err(nb::Error::WouldBlock) => {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Err(nb::Error::Other(e)) => Poll::Ready(Err(e)),
                }
            }
            ConversionState::Complete => Poll::Ready(Err(Ads1256Error::InvalidState(
                "Conversion already complete",
            ))),
        }
    }
}

/// Iterator for continuous sampling of multiple channels
pub struct ContinuousSampling<'a, SPI, CS, DRDY, PDWN, DELAY, GpioError, const N: usize> {
    adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>,
    channels: Option<&'a [u8; N]>,
    differential_pairs: Option<&'a [(u8, u8); N]>,
    current_index: usize,
    _gpio_error: core::marker::PhantomData<GpioError>,
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY, GpioError, const N: usize>
    ContinuousSampling<'a, SPI, CS, DRDY, PDWN, DELAY, GpioError, N>
{
    /// Creates a new continuous sampling iterator for single-ended channels
    pub fn with_channels(
        adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>,
        channels: &'a [u8; N],
    ) -> Self {
        Self {
            adc,
            channels: Some(channels),
            differential_pairs: None,
            current_index: 0,
            _gpio_error: core::marker::PhantomData,
        }
    }

    /// Creates a new continuous sampling iterator for differential channel pairs
    pub fn with_differential_channels(
        adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>,
        pairs: &'a [(u8, u8); N],
    ) -> Self {
        Self {
            adc,
            channels: None,
            differential_pairs: Some(pairs),
            current_index: 0,
            _gpio_error: core::marker::PhantomData,
        }
    }
}

pub trait LendingIterator {
    type Item<'a>
    where
        Self: 'a;
    fn next(&mut self) -> Option<Self::Item<'_>>;
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError, const N: usize> LendingIterator
    for ContinuousSampling<'a, SPI, CS, DRDY, PDWN, DELAY, GpioError, N>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    type Item<'b> = AdcConversion<'b, SPI, CS, DRDY, PDWN, DELAY, GpioError> where Self: 'b;

    fn next(&mut self) -> Option<Self::Item<'_>> {
        if let Some(channels) = self.channels {
            let channel = channels[self.current_index];
            self.current_index = (self.current_index + 1) % N;
            Some(AdcConversion::new_single_ended(self.adc, channel))
        } else if let Some(pairs) = self.differential_pairs {
            let (pos, neg) = pairs[self.current_index];
            self.current_index = (self.current_index + 1) % N;
            Some(AdcConversion::new_differential(self.adc, pos, neg))
        } else {
            Some(AdcConversion::new(self.adc))
        }
    }
}
