# ADS1256/ADS1255 Driver

A no_std driver for the ADS1255/ADS1256 24-bit analog-to-digital converter (ADC). This driver provides both blocking and non-blocking APIs for interfacing with Texas Instruments' ADS1255/ADS1256 high-precision ADCs.

## Features

- Complete `no_std` support
- Blocking and non-blocking (async) operation modes
- Support for single-ended and differential measurements
- Built-in calibration routines (self and system)
- Configurable PGA gain settings (1-64)
- Adjustable data rates (2.5 SPS to 30,000 SPS)
- Flexible input multiplexer control
- Buffer enable/disable support
- Comprehensive error handling
- Full register-level access when needed

## Hardware Support

The driver is built on embedded-hal 1.0 traits and supports:

- SPI interface up to 10MHz
- GPIO pins for CS, DRDY, and PDWN control
- System clock configurations from 1-10MHz (7.68MHz typical)

## Development Setup

This project uses Visual Studio Code Dev Containers for development. This ensures a consistent development environment for all contributors.

### Prerequisites

1. [Docker](https://www.docker.com/get-started)
2. [Visual Studio Code](https://code.visualstudio.com/)
3. [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Getting Started with Dev Container

1. Clone the repository:
   ```bash
   git clone https://github.com/fabracht/ads125x.git
   cd ads125x
   ```

2. Open the project in VS Code:
   ```bash
   code .
   ```

3. When prompted, click "Reopen in Container" or press `F1` and select "Dev Containers: Reopen in Container"

The container will be built automatically with all necessary dependencies installed. This includes:
- Rust toolchain
- Required system packages
- GitHub CLI
- Development tools and extensions

### Building and Testing

Once inside the dev container, you can use standard cargo commands:

```bash
cargo build
cargo test
cargo clippy
```

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
ads125x = "0.1.0"
```

## Basic Usage

### Blocking API Example

```rust ignore
use ads125x::{Ads1256, DataRate, Gain};

fn main() -> Result<(), Error> {
    // Create SPI and GPIO instances
    let spi = // ...
    let cs = // ...
    let drdy = // ...
    let pdwn = // ...
    let delay = // ...

    // Initialize ADC
    let mut adc = Ads1256::new(spi, cs, drdy, pdwn, delay,
        Gain::Gain1, DataRate::Sps1000);
    adc.init(true)?; // true enables input buffer

    // Single-ended measurement on AIN0
    adc.set_channel(0)?;
    let voltage = adc.read_voltage()?;
    println!("Voltage: {:.6} V", voltage);

    Ok(())
}
```

### Non-Blocking API Example

```rust ignore
use ads125x::nonblocking::Ads1256NonBlocking;

async fn measure_voltage(mut adc: Ads1256NonBlocking<...>) -> Result<f64, Error> {
    // Configure for AIN0
    adc.set_channel(0).await?;

    // Start conversion and wait for result
    let raw = adc.read_data().await?;
    let voltage = adc.code_to_voltage(raw);

    Ok(voltage)
}
```

## Advanced Usage

### Differential Measurements

```rust ignore
// Measure differential between AIN0 (P) and AIN1 (N)
adc.set_input_channel(0, 1)?;
let diff_voltage = adc.read_voltage()?;
```

### Channel Cycling

```rust ignore
// Cycle through channels quickly
let channels = [0, 1, 2, 3];
for &channel in &channels {
    let value = adc.cycle_channel(channel)?;
    println!("CH{}: {}", channel, value);
}
```

### Calibration

```rust ignore
// Self calibration
adc.self_calibrate()?;

// System calibration
adc.system_offset_calibrate()?;  // With inputs shorted
adc.system_gain_calibrate()?;    // With known reference
```

### Register Access

```rust ignore
// Direct register operations if needed
adc.write_register(REG_MUX, &[0x01])?;
let mut status = [0u8; 1];
adc.read_register(REG_STATUS, &mut status)?;
```

### Power Management

```rust ignore
// Enter standby mode
adc.enter_standby()?;

// Wake up
adc.exit_standby()?;

// Enter power-down
adc.enter_power_down()?;
```

## Configuration Options

### Gain Settings

- Available gains: 1, 2, 4, 8, 16, 32, 64
- Controlled via `Gain` enum
- Affects input voltage range

### Data Rates

- Ranges from 2.5 SPS to 30,000 SPS
- Tradeoff between speed and noise
- Set via `DataRate` enum

### Input Buffer

- Can be enabled/disabled
- Provides high input impedance when enabled
- Some performance impact

### Digital Interface

- SPI mode 1 (CPOL=0, CPHA=1)
- Maximum SCLK frequency: fCLKIN/4
- CS, DRDY, PDWN control pins

## Error Handling

The driver uses a comprehensive error type that covers:

- Communication errors (SPI, GPIO)
- Configuration errors
- Timing/state errors
- Invalid parameter errors

All operations return `Result<T, Ads1256Error<SpiError, GpioError>>`.

## Advanced Topics

### Non-Blocking Operation Details

The non-blocking API provides:

1. Future-based conversions
2. Channel cycling
3. Continuous sampling
4. Async calibration

Example of continuous sampling:

```rust ignore
let channels = [0, 1, 2, 3];
let mut sampling = ContinuousSampling::with_channels(&mut adc, &channels);

while let Some(conversion) = sampling.next() {
    let value = conversion.await?;
    // Process value...
}
```

### Performance Optimization

1. Use appropriate data rate for application
2. Consider buffer impact on measurements
3. Optimize channel cycling sequence
4. Use non-blocking API for better system utilization

### Noise Considerations

1. Use lower data rates for better noise rejection
2. Enable input buffer for high-impedance sources
3. Consider PGA settings impact on noise
4. External filtering may be needed

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## License

Licensed under either:

- MIT license or
- Apache License, Version 2.0

at your option.

## Resources

- [ADS1256 Datasheet](https://www.ti.com/lit/ds/symlink/ads1256.pdf)
- [Application Notes](https://www.ti.com/product/ADS1256#tech-docs)
