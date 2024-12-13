# ADS125x

A `no_std` Rust driver for the ADS1255/ADS1256 24-bit analog-to-digital converters.

[![Crates.io Version](https://img.shields.io/crates/v/ads125x.svg)](https://crates.io/crates/ads125x)
[![Documentation](https://docs.rs/ads125x/badge.svg)](https://docs.rs/ads125x)

## Features

- Full support for both ADS1255 and ADS1256 variants
- Implementation of all ADC operating modes:
  - Single-shot conversions
  - Continuous sampling
  - Channel cycling
  - Differential measurements
- Non-blocking operation support with async/await
- Programmable gain amplifier (PGA) control
- Flexible input multiplexer configuration
- Built-in self and system calibration
- Buffer control and sensor detect functionality
- Complete error handling with descriptive error types

## Hardware Support

- 24-bit resolution
- Data rates from 2.5 SPS to 30,000 SPS
- Up to 8 single-ended or 4 differential inputs (ADS1256)
- 2 single-ended or 1 differential input (ADS1255)
- Programmable gain from 1x to 64x
- SPI interface with clock rates up to 7.68MHz

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
ads125x = "0.1.0"
```

### Basic Example

```rust
use ads125x::{Ads1256, DataRate, Gain};

fn main() -> Result<(), Error> {
    let spi = // Initialize your SPI
    let cs = // Initialize CS pin
    let drdy = // Initialize DRDY pin
    let pdwn = // Initialize PDWN pin
    let delay = // Initialize delay provider

    // Create ADC instance
    let mut adc = Ads1256::new(spi, cs, drdy, pdwn, delay, Gain::Gain1, DataRate::Sps1000);

    // Initialize with buffer enabled
    adc.init(true)?;

    // Read voltage from channel 0
    adc.set_channel(0)?;
    let voltage = adc.read_voltage()?;
    println!("Voltage: {:.6} V", voltage);

    Ok(())
}
```

### Non-blocking Operation

```rust
use ads125x::nonblocking::AdcConversion;

async fn read_voltage(adc: &mut Ads1256<...>) -> Result<f64, Error> {
    // Start a non-blocking conversion
    let raw = AdcConversion::new(adc).await?;

    // Convert to voltage
    Ok(adc.code_to_voltage(raw))
}
```

### Continuous Channel Sampling

```rust
use ads125x::nonblocking::ContinuousSampling;

async fn sample_channels(adc: &mut Ads1256<...>) -> Result<(), Error> {
    // Define channels to sample
    let channels = [0, 1, 2, 3];

    // Create continuous sampling iterator
    let mut sampling = ContinuousSampling::with_channels(adc, &channels);

    // Sample each channel in sequence
    while let Some(conversion) = sampling.next() {
        let value = conversion.await?;
        println!("Channel value: {}", value);
    }

    Ok(())
}
```

### Differential Measurements

```rust
// Configure differential measurement between AIN0 (positive) and AIN1 (negative)
adc.set_input_channel(0, 1)?;
let voltage = adc.read_voltage()?;
```

## Configuration Options

### Data Rates

Available sampling rates (at 7.68MHz clock):

- 30,000 SPS
- 15,000 SPS
- 7,500 SPS
- 3,750 SPS
- 2,000 SPS
- 1,000 SPS
- 500 SPS
- 100 SPS
- 60 SPS
- 50 SPS
- 30 SPS
- 25 SPS
- 15 SPS
- 10 SPS
- 5 SPS
- 2.5 SPS

### Gain Settings

Available PGA gains:

- 1x (default)
- 2x
- 4x
- 8x
- 16x
- 32x
- 64x

## Advanced Features

### Calibration

```rust
// Perform self-calibration
adc.self_calibrate()?;

// System offset calibration (with inputs shorted)
adc.system_offset_calibrate()?;

// System gain calibration (with known reference)
adc.system_gain_calibrate()?;
```

### Buffer Control

```rust
// Enable input buffer
adc.set_buffer_enabled(true)?;
```

### Sensor Detection

```rust
// Configure sensor detect current sources
adc.set_sensor_detect_current(SensorDetectCurrent::Current_0_5uA)?;
```

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
