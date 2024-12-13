#![no_std]
#![doc = include_str!("../README.md")]
#![cfg_attr(docsrs, feature(doc_cfg))]

extern crate alloc;

pub mod constants;
pub mod driver;
pub mod error;
// pub mod nb;
pub mod nonblocking;

pub use constants::{DataRate, Gain};
pub use driver::Ads1256;
pub use error::Ads1256Error;
// pub use nb::Ads1256NonBlocking;

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use constants::*;
    use embedded_hal_mock::eh1::delay::NoopDelay as MockDelay;
    use embedded_hal_mock::eh1::digital::Mock as MockPin;
    use embedded_hal_mock::eh1::spi::Mock as MockSpi;

    #[test]
    fn test_gain_value() {
        assert_eq!(Gain::Gain1.value(), 1.0);
        assert_eq!(Gain::Gain64.value(), 64.0);
    }

    #[test]
    fn test_new_adc() {
        let expectations = vec![];
        let mut spi = MockSpi::new(&expectations);
        let mut cs = MockPin::new(&[]);
        let mut drdy = MockPin::new(&[]);
        let mut pdwn = MockPin::new(&[]);
        let delay = MockDelay::new();

        let _adc = Ads1256::new(
            spi.clone(),
            cs.clone(),
            drdy.clone(),
            pdwn.clone(),
            delay,
            Gain::Gain1,
            DataRate::Sps100,
        );
        spi.done();
        cs.done();
        drdy.done();
        pdwn.done();
    }

    #[test]
    fn test_code_to_voltage() {
        let expectations = vec![];
        let mut spi = MockSpi::new(&expectations);
        let mut cs = MockPin::new(&[]);
        let mut drdy = MockPin::new(&[]);
        let mut pdwn = MockPin::new(&[]);
        let delay = MockDelay::new();

        let adc = Ads1256::new(
            spi.clone(),
            cs.clone(),
            drdy.clone(),
            pdwn.clone(),
            delay,
            Gain::Gain1,
            DataRate::Sps100,
        );

        // Test with maximum positive value
        let max_code = 0x7FFFFF;
        let voltage = adc.code_to_voltage(max_code);
        assert!(
            (voltage - 5.0).abs() < 1e-6,
            "Max code: {}, Voltage: {}",
            (voltage - 5.0).abs(),
            voltage
        );

        // Test with maximum negative value
        let min_code = -0x800000;
        let voltage = adc.code_to_voltage(min_code);
        assert!(
            (voltage + 5.0).abs() < 1e-6,
            "Min code: {}, Voltage: {}",
            min_code,
            voltage
        );

        // Test with zero
        let zero_code = 0;
        let voltage = adc.code_to_voltage(zero_code);
        assert!(
            voltage.abs() < 1e-6,
            "Zero code: {}, Voltage: {}",
            zero_code,
            voltage
        );

        spi.done();
        cs.done();
        drdy.done();
        pdwn.done();
    }
}
