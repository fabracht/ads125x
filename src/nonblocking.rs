use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;

use crate::{Ads1256, Ads1256Error};

#[derive(Debug)]
enum ConversionState {
    NotStarted,
    WaitingForConversion,
    Complete,
}

/// A future that represents a non-blocking ADC conversion
pub struct AdcConversion<'a, SPI, CS, DRDY, PDWN, DELAY, GpioError> {
    adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>,
    state: ConversionState,
    channel: Option<u8>,
    differential: Option<(u8, u8)>,
    _gpio_error: core::marker::PhantomData<GpioError>,
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError>
    AdcConversion<'a, SPI, CS, DRDY, PDWN, DELAY, GpioError>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    fn new(adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>) -> Self {
        Self {
            adc,
            state: ConversionState::NotStarted,
            channel: None,
            differential: None,
            _gpio_error: core::marker::PhantomData,
        }
    }

    fn new_single_ended(adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>, channel: u8) -> Self {
        Self {
            adc,
            state: ConversionState::NotStarted,
            channel: Some(channel),
            differential: None,
            _gpio_error: core::marker::PhantomData,
        }
    }

    fn new_differential(
        adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>,
        positive: u8,
        negative: u8,
    ) -> Self {
        Self {
            adc,
            state: ConversionState::NotStarted,
            channel: None,
            differential: Some((positive, negative)),
            _gpio_error: core::marker::PhantomData,
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
            ConversionState::NotStarted => {
                if let Some(channel) = this.channel {
                    if let Err(e) = this.adc.cycle_channel(channel) {
                        return Poll::Ready(Err(e));
                    }
                } else if let Some((positive, negative)) = this.differential {
                    if let Err(e) = this.adc.cycle_differential_channel(positive, negative) {
                        return Poll::Ready(Err(e));
                    }
                } else {
                    if let Err(e) = this.adc.start_one_shot() {
                        return Poll::Ready(Err(e));
                    }
                }

                this.state = ConversionState::WaitingForConversion;
                cx.waker().wake_by_ref();
                Poll::Pending
            }
            ConversionState::WaitingForConversion => match this.adc.read_one_shot_nb() {
                Ok(value) => {
                    this.state = ConversionState::Complete;
                    Poll::Ready(Ok(value))
                }
                Err(nb::Error::WouldBlock) => {
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
                Err(nb::Error::Other(e)) => Poll::Ready(Err(e)),
            },
            ConversionState::Complete => Poll::Ready(Err(Ads1256Error::InvalidState(
                "Conversion already complete",
            ))),
        }
    }
}

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

    fn next<'a>(&'a mut self) -> Option<Self::Item<'a>>;
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
    type Item<'b> = AdcConversion<'b, SPI, CS, DRDY, PDWN, DELAY, GpioError>
    where
        Self: 'b;

    fn next<'b>(&'b mut self) -> Option<Self::Item<'b>> {
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
