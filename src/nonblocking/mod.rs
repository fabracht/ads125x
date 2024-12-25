// src/nonblocking/mod.rs
pub mod calibration;
pub mod channels;
pub mod modes;
pub mod operations;
pub mod utils;

use crate::{constants::*, error::Ads1256Error};
use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll, Waker},
};
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    spi::SpiDevice,
};

/// ADC operating state
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum State {
    Idle,
    PoweringUp,
    Converting,
    WaitingForData,
    Reading,
    EnteringStandby,
    Standby,
    ChangingChannel { channel: u8, previous_data: bool },
    Error,
}

/// ADC operation mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mode {
    OneShot,
    Continuous,
}

/// A non-blocking conversion future
pub struct Conversion<'a, SPI, CS, DRDY, PDWN, DELAY> {
    adc: &'a mut Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY>,
    waker: Option<Waker>,
}

/// Non-blocking ADS1256 driver
pub struct Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY> {
    pub(crate) spi: SPI,
    pub(crate) cs: CS,
    pub(crate) drdy: DRDY,
    pub(crate) pdwn: PDWN,
    pub(crate) delay: DELAY,
    state: State,
    mode: Mode,
    gain: Gain,
    data_rate: DataRate,
    current_channel: Option<u8>,
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError>
    Ads1256NonBlocking<SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    /// Creates a new non-blocking ADS1256 instance
    pub fn new(
        spi: SPI,
        cs: CS,
        drdy: DRDY,
        pdwn: PDWN,
        delay: DELAY,
        gain: Gain,
        data_rate: DataRate,
    ) -> Self {
        Self {
            spi,
            cs,
            drdy,
            pdwn,
            delay,
            state: State::Idle,
            mode: Mode::OneShot,
            gain,
            data_rate,
            current_channel: None,
        }
    }

    /// Starts a one-shot conversion
    pub fn start_conversion(&mut self) -> Conversion<'_, SPI, CS, DRDY, PDWN, DELAY> {
        Conversion {
            adc: self,
            waker: None,
        }
    }

    /// Poll the current state
    pub fn poll_state(&mut self) -> Result<State, Ads1256Error<SpiError, GpioError>> {
        match self.state {
            State::Idle => Ok(State::Idle),
            State::PoweringUp => {
                // TODO: Implement proper timing check
                self.state = State::Converting;
                Ok(self.state)
            }
            State::Converting | State::WaitingForData => {
                if self.drdy.is_low().map_err(Ads1256Error::Gpio)? {
                    self.state = State::Reading;
                }
                Ok(self.state)
            }
            _ => Ok(self.state),
        }
    }

    /// Non-blocking channel change
    pub fn start_channel_change(
        &mut self,
        new_channel: u8,
    ) -> Result<(), Ads1256Error<SpiError, GpioError>> {
        if new_channel > 7 {
            return Err(Ads1256Error::InvalidInputChannel);
        }

        match self.state {
            State::Idle | State::Standby => {
                self.state = State::ChangingChannel {
                    channel: new_channel,
                    previous_data: false,
                };
                Ok(())
            }
            _ => Err(Ads1256Error::InvalidState(
                "Cannot change channel in current state",
            )),
        }
    }
}

impl<'a, SPI, CS, DRDY, PDWN, DELAY, SpiError, GpioError> Future
    for Conversion<'a, SPI, CS, DRDY, PDWN, DELAY>
where
    SPI: SpiDevice<Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    DRDY: InputPin<Error = GpioError>,
    PDWN: OutputPin<Error = GpioError>,
    DELAY: DelayNs,
{
    type Output = Result<i32, Ads1256Error<SpiError, GpioError>>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Store waker for future notifications
        self.waker = Some(cx.waker().clone());

        match self.adc.poll_state()? {
            State::Reading => {
                // Read the conversion result
                let result = self.adc.read_data()?;
                Poll::Ready(Ok(result))
            }
            _ => Poll::Pending,
        }
    }
}
