#![no_std]

mod constants;
#[cfg(not(feature = "async"))]
mod driver;

#[cfg(feature = "async")]
mod driver_async;

mod error;

pub use constants::{DataRate, Gain};

#[cfg(not(feature = "async"))]
pub use driver::Ads1256;

#[cfg(feature = "async")]
pub use driver_async::Ads1256;

pub use error::Ads1256Error;
