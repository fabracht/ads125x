# ADS1256 Rust Driver

This crate provides a Rust driver for the ADS1256 (and ADS1255) Analog-to-Digital Converter (ADC) from Texas Instruments. The ADS1256 is a high-precision, 24-bit ADC with an integrated Programmable Gain Amplifier (PGA) and self-calibration functionality. It supports up to 30 kSPS, eight single-ended or four differential input channels, and has an SPI communication interface.

## Key Features

- **24-bit Analog-to-Digital Conversion**

  - Supports eight single-ended or four differential input channels.
  - Data output rates from 2.5 SPS to 30,000 SPS.
  - Nonlinearity of ±0.0010% maximum.

- **SPI Interface**

  - Communicates with the ADS1256 via SPI, compatible with the `embedded-hal` SPI traits.

- **Programmable Gain Amplifier (PGA)**

  - Adjustable gain from 1x to 64x for different input ranges.

- **Calibration**

  - Supports both **self-calibration** and **system calibration** (manual offset and gain calibration).
  - Access to the Offset Calibration (`OFC`) and Full-Scale Calibration (`FSC`) registers for verification or manual calibration.

- **Synchronization**
  - Synchronization can be controlled via the SYNC/PDWN pin or SPI commands, allowing precise control over conversion timing.
- **GPIO Control**
  - Manages the SYNC/PDWN, CS, and DRDY pins using the `embedded-hal` digital traits.

## Usage

### Example 1: Basic Setup and Reading Voltage

```rust
use ads125x::{Ads1256, DataRate, Gain};
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::{Gpio10, Gpio11, Gpio12};
use esp_idf_hal::spi::{SpiDeviceDriver, SpiDriver};

// Assuming you have a setup with the ESP32 and `esp-idf-hal`
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Set up peripherals (SPI, GPIO, etc.)
    let peripherals = Peripherals::take().unwrap();
    let spi = peripherals.spi2;
    let sclk = peripherals.pins.gpio6;
    let mosi = peripherals.pins.gpio7;
    let miso = peripherals.pins.gpio8;

    let spi_driver = SpiDriver::new(spi, sclk, mosi, miso, None, &SpiConfig::new().baudrate(1.MHz()))?;
    let spi_device = SpiDeviceDriver::new(&spi_driver, None);

    let cs = peripherals.pins.gpio10.into_output()?;
    let drdy = peripherals.pins.gpio11.into_input()?;
    let pdwn = peripherals.pins.gpio12.into_output()?;

    let delay = Ets;

    // Create an ADS1256 instance
    let mut adc = Ads1256::new(spi_device, cs, drdy, pdwn, delay, Gain::Gain1, DataRate::Sps1000);

    // Initialize the ADC
    adc.init()?;

    // Set channel and read voltage
    adc.set_channel(0)?;
    let voltage = adc.read_voltage()?;
    println!("Measured voltage: {:.6} V", voltage);

    Ok(())
}
```

### Example 2: Performing Self-Calibration

```rust
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize the ADS1256 as shown in the first example...

    // Perform self-calibration after initialization
    adc.self_calibrate()?;

    // Read and verify the calibration registers
    let offset_cal = adc.read_offset_calibration()?;
    let fullscale_cal = adc.read_fullscale_calibration()?;

    println!("Offset Calibration: {}", offset_cal);
    println!("Full-Scale Calibration: {}", fullscale_cal);

    Ok(())
}
```

### Example 3: Synchronization

The ADS1256 can be synchronized using either the SYNC/PDWN pin or SPI commands. This can be useful for aligning conversions with external events.

#### Synchronizing with the SYNC/PDWN Pin

```rust
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize the ADS1256...

    // Synchronize using the SYNC/PDWN pin
    adc.synchronize_with_pin()?;

    // Now perform a regular conversion
    adc.set_channel(0)?;
    let voltage = adc.read_voltage()?;
    println!("Measured voltage: {:.6} V", voltage);

    Ok(())
}
```

#### Synchronizing with SPI SYNC Command

```rust
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize the ADS1256...

    // Synchronize using the SYNC and WAKEUP commands
    adc.synchronize_with_commands()?;

    // Now perform a regular conversion
    adc.set_channel(0)?;
    let voltage = adc.read_voltage()?;
    println!("Measured voltage: {:.6} V", voltage);

    Ok(())
}
```

## Calibration Features

This driver supports both **self-calibration** and **system calibration**:

- **Self-Calibration**:
  - Automatically calibrates both offset and gain values.
  - Can be performed after initialization or when changing critical configurations like the data rate or PGA setting.
- **System Calibration**:
  - The user must provide specific input signals (e.g., zero differential for offset, full-scale for gain) to perform system calibration.
  - Use the `system_offset_calibrate()` and `system_gain_calibrate()` methods to perform system calibration.

```rust
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize the ADS1256...

    // Perform system offset calibration (apply zero differential input signal)
    adc.system_offset_calibrate()?;

    // Perform system gain calibration (apply full-scale input signal)
    adc.system_gain_calibrate()?;

    Ok(())
}
```

## Pinout and Configurations

Ensure that your SPI and GPIO pins are properly configured based on your hardware. For ESP32, for example, the following typical pin configuration might be used:

- **SPI Pins**:
  - SCLK (Clock)
  - MOSI (Master Out Slave In)
  - MISO (Master In Slave Out)
- **GPIO Pins**:
  - CS (Chip Select)
  - DRDY (Data Ready)
  - PDWN (Power Down / Sync)

## Supported Platforms

This driver is designed to work with any platform that implements the [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits for SPI, GPIO, and delay functionalities. It has been tested with:

- **ESP32-S3** using the `esp-idf-hal` crate

## Next Steps

- Implement and verify system calibration with real-world inputs.
- Perform detailed testing with various data rates and input configurations.
- Add more examples, including differential input mode and GPIO control.
