# ADS1256 Rust Driver

This crate provides a Rust driver for the ADS1256 (and ADS1255) Analog-to-Digital Converter (ADC) from Texas Instruments. The ADS1256 is a high-precision, 24-bit ADC with an integrated Programmable Gain Amplifier (PGA) and self-calibration functionality. It supports up to 30 kSPS, eight single-ended or four differential input channels, and has an SPI communication interface.

## Key Features

- **24-bit Analog-to-Digital Conversion**

  - Up to eight single-ended or four differential input channels.
  - Data rates: 2.5 SPS to 30,000 SPS.
  - ±0.0010% nonlinearity (maximum).

- **SPI Interface**

  - Fully compatible with the `embedded-hal` SPI traits.

- **Programmable Gain Amplifier (PGA)**

  - Gain settings: 1x to 64x for adjustable input ranges.

- **Calibration**
  - Self-calibration and manual system calibration (offset/gain).
  - Access to `OFC` and `FSC` registers for verification or manual settings.

## Usage

### Example: Reading Voltage with Embedded-HAL Traits

```ignore
use ads125x::{Ads1256, Ads1256Error, DataRate, Gain};

type CommonAds1256Error = Ads1256Error<embedded_hal::spi::ErrorKind, embedded_hal_mock::eh1::MockError>;

fn main() -> Result<(), CommonAds1256Error> {
    let spi = ..... // Create SPI instance
    let cs = ..... // Create CS pin
    let drdy = ..... // Create DRDY pin
    let pdwn = ..... // Create PDWN pin
    let delay = ..... // Create Delay instance

    let mut adc = Ads1256::new(spi, cs, drdy, pdwn, delay, Gain::Gain1, DataRate::Sps1000);
    adc.init(true)?;

    adc.set_channel(0)?;
    let voltage = adc.read_voltage()?;
    println!("Voltage: {:.6} V", voltage);

    Ok(())
}
```
