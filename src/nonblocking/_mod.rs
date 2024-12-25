// src/driver/calibration.rs
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;

use crate::{Ads1256, Ads1256Error};

/// Future for waiting for DRDY to go low (indicating data ready)
pub struct WaitForDrdyLow<'a, SPI, CS, DRDY, PDWN, DELAY> {
    adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>,
}

/// Future for waiting for DRDY to go high
pub struct WaitForDrdyHigh<'a, SPI, CS, DRDY, PDWN, DELAY> {
    adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>,
}

/// Future for reading ADC data with voltage conversion
pub struct ReadVoltage<'a, SPI, CS, DRDY, PDWN, DELAY> {
    adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>,
    state: ReadState,
    vref: f64,
    gain: f64,
}

/// Future for cycling channels with voltage conversion
pub struct CycleChannel<'a, SPI, CS, DRDY, PDWN, DELAY> {
    adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>,
    state: CycleState,
    channel: u8,
    vref: f64,
    gain: f64,
}

/// Future for cycling differential channels with voltage conversion
pub struct CycleDifferentialChannel<'a, SPI, CS, DRDY, PDWN, DELAY> {
    adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>,
    state: CycleState,
    positive: u8,
    negative: u8,
    vref: f64,
    gain: f64,
}

/// Enum representing all possible ADC futures
pub enum AdcFuture<'a, SPI, CS, DRDY, PDWN, DELAY> {
    SingleEnded(CycleChannel<'a, SPI, CS, DRDY, PDWN, DELAY>),
    Differential(CycleDifferentialChannel<'a, SPI, CS, DRDY, PDWN, DELAY>),
    Direct(ReadVoltage<'a, SPI, CS, DRDY, PDWN, DELAY>),
}

#[derive(Debug)]
enum ReadState {
    WaitingForData,
    ReadingData,
}

#[derive(Debug)]
enum CycleState {
    WaitingForDrdyLow,
    SettingMux,
    WaitingForDrdyHigh,
    WaitingForDataReady,
    ReadingData,
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY> Ads1256<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice,
    CS: OutputPin,
    DRDY: InputPin<Error = CS::Error>,
    PDWN: OutputPin<Error = CS::Error>,
    DELAY: DelayNs,
{
    /// Non-blocking wait for DRDY to go low
    pub fn wait_drdy_low_async(&mut self) -> WaitForDrdyLow<'_, SPI, CS, DRDY, PDWN, DELAY> {
        WaitForDrdyLow { adc: self }
    }

    /// Non-blocking wait for DRDY to go high
    pub fn wait_drdy_high_async(&mut self) -> WaitForDrdyHigh<'_, SPI, CS, DRDY, PDWN, DELAY> {
        WaitForDrdyHigh { adc: self }
    }

    /// Asynchronously read voltage from the ADC
    pub fn read_voltage_async(&mut self) -> ReadVoltage<'_, SPI, CS, DRDY, PDWN, DELAY> {
        let vref = self.vref();
        let gain = self.gain_value();
        ReadVoltage {
            adc: self,
            state: ReadState::WaitingForData,
            vref,
            gain,
        }
    }

    /// Asynchronously cycle to a new channel
    pub fn cycle_channel_async(
        &mut self,
        channel: u8,
    ) -> CycleChannel<'_, SPI, CS, DRDY, PDWN, DELAY> {
        let vref = self.vref();
        let gain = self.gain_value();
        CycleChannel {
            adc: self,
            state: CycleState::WaitingForDrdyLow,
            channel,
            vref,
            gain,
        }
    }

    /// Asynchronously cycle to a new differential channel pair
    pub fn cycle_differential_channel_async(
        &mut self,
        positive: u8,
        negative: u8,
    ) -> CycleDifferentialChannel<'_, SPI, CS, DRDY, PDWN, DELAY> {
        let vref = self.vref();
        let gain = self.gain_value();
        CycleDifferentialChannel {
            adc: self,
            state: CycleState::WaitingForDrdyLow,
            positive,
            negative,
            vref,
            gain,
        }
    }
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY> Future for AdcFuture<'a, SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice,
    CS: OutputPin,
    DRDY: InputPin<Error = CS::Error>,
    PDWN: OutputPin<Error = CS::Error>,
    DELAY: DelayNs,
{
    type Output = Result<f64, Ads1256Error<SPI::Error, CS::Error>>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Get a mutable reference to the inner enum variant
        let this = unsafe { self.get_unchecked_mut() };
        match this {
            AdcFuture::SingleEnded(future) => {
                let future = unsafe { Pin::new_unchecked(future) };
                future.poll(cx)
            }
            AdcFuture::Differential(future) => {
                let future = unsafe { Pin::new_unchecked(future) };
                future.poll(cx)
            }
            AdcFuture::Direct(future) => {
                let future = unsafe { Pin::new_unchecked(future) };
                future.poll(cx)
            }
        }
    }
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY> Future for WaitForDrdyLow<'a, SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice,
    CS: OutputPin,
    DRDY: InputPin<Error = CS::Error>,
    PDWN: OutputPin<Error = CS::Error>,
    DELAY: DelayNs,
{
    type Output = Result<(), Ads1256Error<SPI::Error, CS::Error>>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = unsafe { self.get_unchecked_mut() };
        match this.adc.drdy.is_low().map_err(Ads1256Error::Gpio) {
            Ok(true) => Poll::Ready(Ok(())),
            Ok(false) => {
                cx.waker().wake_by_ref();
                Poll::Pending
            }
            Err(e) => Poll::Ready(Err(e)),
        }
    }
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY> Future for WaitForDrdyHigh<'a, SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice,
    CS: OutputPin,
    DRDY: InputPin<Error = CS::Error>,
    PDWN: OutputPin<Error = CS::Error>,
    DELAY: DelayNs,
{
    type Output = Result<(), Ads1256Error<SPI::Error, CS::Error>>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = unsafe { self.get_unchecked_mut() };
        match this.adc.drdy.is_high().map_err(Ads1256Error::Gpio) {
            Ok(true) => Poll::Ready(Ok(())),
            Ok(false) => {
                cx.waker().wake_by_ref();
                Poll::Pending
            }
            Err(e) => Poll::Ready(Err(e)),
        }
    }
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY> Future for ReadVoltage<'a, SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice,
    CS: OutputPin,
    DRDY: InputPin<Error = CS::Error>,
    PDWN: OutputPin<Error = CS::Error>,
    DELAY: DelayNs,
{
    type Output = Result<f64, Ads1256Error<SPI::Error, CS::Error>>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = unsafe { self.get_unchecked_mut() };

        match this.state {
            ReadState::WaitingForData => match this.adc.drdy.is_low().map_err(Ads1256Error::Gpio) {
                Ok(true) => {
                    this.state = ReadState::ReadingData;
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
                Ok(false) => {
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
                Err(e) => Poll::Ready(Err(e)),
            },
            ReadState::ReadingData => match this.adc.read_data() {
                Ok(raw_value) => {
                    let max_code = 8388607.0;
                    let voltage = (raw_value as f64 * (2.0 * this.vref)) / (this.gain * max_code);
                    Poll::Ready(Ok(voltage))
                }
                Err(e) => Poll::Ready(Err(e)),
            },
        }
    }
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY> Future for CycleChannel<'a, SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice,
    CS: OutputPin,
    DRDY: InputPin<Error = CS::Error>,
    PDWN: OutputPin<Error = CS::Error>,
    DELAY: DelayNs,
{
    type Output = Result<f64, Ads1256Error<SPI::Error, CS::Error>>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = unsafe { self.get_unchecked_mut() };

        match this.state {
            CycleState::WaitingForDrdyLow => {
                match this.adc.drdy.is_low().map_err(Ads1256Error::Gpio) {
                    Ok(true) => {
                        this.state = CycleState::SettingMux;
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Ok(false) => {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Err(e) => Poll::Ready(Err(e)),
                }
            }
            CycleState::SettingMux => {
                if this.channel > 7 {
                    return Poll::Ready(Err(Ads1256Error::InvalidInputChannel));
                }

                let positive = this.channel & 0x07;
                let negative = 0x08;
                let mux = (positive << 4) | negative;

                match this.adc.write_register(crate::constants::REG_MUX, &[mux]) {
                    Ok(()) => {
                        if let Err(e) = this.adc.send_command(crate::constants::CMD_SYNC) {
                            return Poll::Ready(Err(e));
                        }
                        this.adc.delay.delay_us(100);
                        if let Err(e) = this.adc.send_command(crate::constants::CMD_WAKEUP) {
                            return Poll::Ready(Err(e));
                        }
                        this.state = CycleState::WaitingForDrdyHigh;
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Err(e) => Poll::Ready(Err(e)),
                }
            }
            CycleState::WaitingForDrdyHigh => {
                match this.adc.drdy.is_high().map_err(Ads1256Error::Gpio) {
                    Ok(true) => {
                        this.state = CycleState::WaitingForDataReady;
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Ok(false) => {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Err(e) => Poll::Ready(Err(e)),
                }
            }
            CycleState::WaitingForDataReady => {
                match this.adc.drdy.is_low().map_err(Ads1256Error::Gpio) {
                    Ok(true) => {
                        this.state = CycleState::ReadingData;
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Ok(false) => {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Err(e) => Poll::Ready(Err(e)),
                }
            }
            CycleState::ReadingData => match this.adc.read_data() {
                Ok(raw_value) => {
                    let max_code = 8388607.0;
                    let voltage = (raw_value as f64 * (2.0 * this.vref)) / (this.gain * max_code);
                    Poll::Ready(Ok(voltage))
                }
                Err(e) => Poll::Ready(Err(e)),
            },
        }
    }
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY> Future
    for CycleDifferentialChannel<'a, SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice,
    CS: OutputPin,
    DRDY: InputPin<Error = CS::Error>,
    PDWN: OutputPin<Error = CS::Error>,
    DELAY: DelayNs,
{
    type Output = Result<f64, Ads1256Error<SPI::Error, CS::Error>>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = unsafe { self.get_unchecked_mut() };

        match this.state {
            CycleState::WaitingForDrdyLow => {
                match this.adc.drdy.is_low().map_err(Ads1256Error::Gpio) {
                    Ok(true) => {
                        this.state = CycleState::SettingMux;
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Ok(false) => {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Err(e) => Poll::Ready(Err(e)),
                }
            }
            CycleState::SettingMux => {
                if this.positive > 7 || this.negative > 7 {
                    return Poll::Ready(Err(Ads1256Error::InvalidInputChannel));
                }

                let mux = (this.positive << 4) | this.negative;

                match this.adc.write_register(crate::constants::REG_MUX, &[mux]) {
                    Ok(()) => {
                        if let Err(e) = this.adc.send_command(crate::constants::CMD_SYNC) {
                            return Poll::Ready(Err(e));
                        }
                        this.adc.delay.delay_us(100);
                        if let Err(e) = this.adc.send_command(crate::constants::CMD_WAKEUP) {
                            return Poll::Ready(Err(e));
                        }
                        this.state = CycleState::WaitingForDrdyHigh;
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Err(e) => Poll::Ready(Err(e)),
                }
            }
            CycleState::WaitingForDrdyHigh => {
                match this.adc.drdy.is_high().map_err(Ads1256Error::Gpio) {
                    Ok(true) => {
                        this.state = CycleState::WaitingForDataReady;
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Ok(false) => {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Err(e) => Poll::Ready(Err(e)),
                }
            }
            CycleState::WaitingForDataReady => {
                match this.adc.drdy.is_low().map_err(Ads1256Error::Gpio) {
                    Ok(true) => {
                        this.state = CycleState::ReadingData;
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Ok(false) => {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                    Err(e) => Poll::Ready(Err(e)),
                }
            }
            CycleState::ReadingData => match this.adc.read_data() {
                Ok(raw_value) => {
                    let max_code = 8388607.0;
                    let voltage = (raw_value as f64 * (2.0 * this.vref)) / (this.gain * max_code);
                    Poll::Ready(Ok(voltage))
                }
                Err(e) => Poll::Ready(Err(e)),
            },
        }
    }
}

/// Iterator for continuous sampling of multiple channels
pub struct ContinuousSampling<'a, SPI, CS, DRDY, PDWN, DELAY, const N: usize> {
    adc: &'a mut Ads1256<SPI, CS, DRDY, PDWN, DELAY>,
    channels: Option<&'a [u8; N]>,
    differential_pairs: Option<&'a [(u8, u8); N]>,
    current_index: usize,
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY, const N: usize>
    ContinuousSampling<'a, SPI, CS, DRDY, PDWN, DELAY, N>
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
        }
    }

    /// Returns a future that will complete when the next sample is ready
    pub fn next_sample(&mut self) -> Option<AdcFuture<'_, SPI, CS, DRDY, PDWN, DELAY>>
    where
        SPI: SpiDevice,
        CS: OutputPin,
        DRDY: InputPin<Error = CS::Error>,
        PDWN: OutputPin<Error = CS::Error>,
        DELAY: DelayNs,
    {
        if let Some(channels) = self.channels {
            let channel = channels[self.current_index];
            self.current_index = (self.current_index + 1) % N;
            Some(AdcFuture::SingleEnded(
                self.adc.cycle_channel_async(channel),
            ))
        } else if let Some(pairs) = self.differential_pairs {
            let (pos, neg) = pairs[self.current_index];
            self.current_index = (self.current_index + 1) % N;
            Some(AdcFuture::Differential(
                self.adc.cycle_differential_channel_async(pos, neg),
            ))
        } else {
            Some(AdcFuture::Direct(self.adc.read_voltage_async()))
        }
    }
}
