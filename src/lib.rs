#![no_std]

#[cfg(feature = "std")]
extern crate std;

pub mod constants;
pub mod driver;
pub mod error;

pub use constants::{DataRate, Gain};
pub use driver::Ads1256;
pub use error::Ads1256Error;
