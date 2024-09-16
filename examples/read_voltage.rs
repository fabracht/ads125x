// src/main.rs

use embedded_hal::spi::SpiDevice;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::{InputPin, OutputPin};
use esp_idf_hal::prelude::*;
use esp_idf_hal::spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver};

use ads125x::{Ads1256, Ads1256Error, DataRate, Gain};
type SpiError = esp_idf_hal::spi::Error;
type GpioError = esp_idf_hal::gpio::Error;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    esp_idf_sys::link_patches();

    // Initialize peripherals
    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    // Set up SPI interface
    let spi = peripherals.spi2; // Use SPI2 or SPI3
    let sclk = pins.gpio6; // SPI Clock
    let mosi = pins.gpio7; // MOSI
    let miso = pins.gpio8; // MISO

    let spi_config = SpiConfig::new().baudrate(1.MHz().into());

    let spi_driver = SpiDriver::new(spi, sclk, mosi, miso, &spi_config)?;

    let spi_device = SpiDeviceDriver::new(&spi_driver, None, &spi_config);

    // Set up GPIO pins
    let cs = pins.gpio10.into_output()?;
    let drdy = pins.gpio11.into_input()?;
    let pdwn = pins.gpio12.into_output()?;

    // Set up delay provider
    let mut delay = Ets;

    // Implement DelayNs for Ets
    impl embedded_hal::delay::DelayNs for Ets {
        fn delay_ns(&mut self, ns: u32) {
            self.delay_us((ns + 999) / 1000); // Approximate ns delay
        }
    }

    // Create an instance of the ADS1256 driver
    let mut adc = Ads1256::new(
        spi_device,
        cs,
        drdy,
        pdwn,
        delay,
        Gain::Gain1,
        DataRate::Sps1000,
    );

    // Initialize the ADC
    adc.init()?;

    // Configure the ADC
    adc.set_gain(Gain::Gain1)?;
    adc.set_data_rate(DataRate::Sps1000)?;
    adc.set_channel(0)?; // Single-ended input on channel 0

    // Read voltage
    let voltage = adc.read_voltage()?;
    println!("Voltage: {:.6} V", voltage);

    Ok(())
}
