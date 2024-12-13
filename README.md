# ADS125x

A `no_std` Rust driver for the ADS1255/ADS1256 24-bit analog-to-digital converters.

[![Crates.io Version](https://img.shields.io/crates/v/ads125x.svg)](https://crates.io/crates/ads125x)
[![Documentation](https://docs.rs/ads125x/badge.svg)](https://docs.rs/ads125x)

## Features

- Complete support for ADS1255/ADS1256 ADCs
- Both blocking and non-blocking operation modes
- Single-ended and differential measurements
- Continuous channel sampling
- Programmable gain amplifier (PGA) control
- Flexible input multiplexer configuration
- Calibration support (self and system)
- Buffer control and sensor detection

## Hardware Support

- 24-bit resolution
- Data rates from 2.5 SPS to 30,000 SPS
- Up to 8 single-ended or 4 differential inputs (ADS1256)
- 2 single-ended or 1 differential input (ADS1255)
- Programmable gain: 1x to 64x
- SPI interface up to 7.68MHz

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
ads125x = "0.1.0"
```

### Blocking Operation

```rust
use ads125x::{Ads1256, DataRate, Gain};

fn main() -> Result<(), Error> {
    let spi = // Initialize your SPI
    let cs = // Initialize CS pin
    let drdy = // Initialize DRDY pin
    let pdwn = // Initialize PDWN pin
    let delay = // Initialize delay provider

    let mut adc = Ads1256::new(spi, cs, drdy, pdwn, delay, Gain::Gain1, DataRate::Sps1000);
    adc.init(true)?;

    // Single channel reading
    adc.set_channel(0)?;
    let voltage = adc.read_voltage()?;

    // Differential reading
    adc.set_input_channel(0, 1)?;  // AIN0 = positive, AIN1 = negative
    let diff_voltage = adc.read_voltage()?;

    Ok(())
}
```

### Non-blocking Operation

The non-blocking API uses Rust's async/await to provide non-blocking ADC readings:

```rust
use ads125x::nonblocking::AdcConversion;

async fn read_voltage(adc: &mut Ads1256<...>) -> Result<f64, Error> {
    // Single channel non-blocking conversion
    let conversion = AdcConversion::new_single_ended(adc, 0);
    let raw = conversion.await?;

    // Convert to voltage
    Ok(adc.code_to_voltage(raw))
}

async fn read_differential(adc: &mut Ads1256<...>) -> Result<f64, Error> {
    // Differential non-blocking conversion
    let conversion = AdcConversion::new_differential(adc, 0, 1);
    let raw = conversion.await?;

    Ok(adc.code_to_voltage(raw))
}
```

### Continuous Channel Sampling

For applications requiring sampling of multiple channels:

```rust
use ads125x::nonblocking::ContinuousSampling;

async fn sample_channels(adc: &mut Ads1256<...>) -> Result<(), Error> {
    // Define channels to sample
    let channels = [0, 1, 2, 3];

    // Create continuous sampling iterator
    let mut sampling = ContinuousSampling::with_channels(adc, &channels);

    // Sample all channels in sequence
    while let Some(conversion) = sampling.next() {
        let value = conversion.await?;
        println!("Channel value: {}", value);
    }

    Ok(())
}

// Or for differential measurements:
async fn sample_differential_pairs(adc: &mut Ads1256<...>) -> Result<(), Error> {
    // Define differential pairs (positive, negative)
    let pairs = [(0, 1), (2, 3), (4, 5)];

    let mut sampling = ContinuousSampling::with_differential_channels(adc, &pairs);

    while let Some(conversion) = sampling.next() {
        let value = conversion.await?;
        println!("Differential value: {}", value);
    }

    Ok(())
}
```

## Configuration

### Data Rates

Available sampling rates (at 7.68MHz clock):

- 30,000 SPS to 2.5 SPS

```rust
adc.set_data_rate(DataRate::Sps15000)?;
```

### Gain Settings

PGA gains from 1x to 64x:

```rust
adc.set_gain(Gain::Gain8)?;  // Sets PGA to 8x
```

## Advanced Features

### Buffer Control

```rust
// Enable input buffer
adc.set_buffer_enabled(true)?;
```

### Calibration

```rust
// Self calibration
adc.self_calibrate()?;

// System calibration
adc.system_offset_calibrate()?;
adc.system_gain_calibrate()?;
```

### Sensor Detection

```rust
adc.set_sensor_detect_current(SensorDetectCurrent::Current_0_5uA)?;
```

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you shall be dual licensed as above, without any additional terms or conditions.
