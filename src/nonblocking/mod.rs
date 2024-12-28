// src/nonblocking/mod.rs
pub mod calibration;
pub mod channels;
pub mod modes;
pub mod operations;
pub mod utils;

use crate::{constants::*, error::Ads1256Error};
// use core::{
//     future::Future,
//     pin::Pin,
//     task::{Context, Poll},
// };
// use embedded_io_async::{Read, Write};

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

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
