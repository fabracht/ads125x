# ADS1256 Driver Overview

## Device Specifications

- 24-bit Analog-to-Digital Converter
- Data output rates up to 30kSPS
- Nonlinearity: ±0.0010% max
- 8 single-ended or 4 differential inputs
- Suitable for measuring analog voltage within 3V
- Operating voltage: 5V

## Pin Descriptions

- 5V, GND: 5V power input
- SCLK, DIN, DOUT, CS: SPI communication interface
- DRDY: Data Ready output (Active low)
- PDWN: Sync / Power down input (Active low)
- AIN0-AIN7: Analog voltage inputs

## Key Features

- Programmable Gain Amplifier (PGA)
- Digital Filter with selectable data rates
- Self and System Calibration
- Flexible Input Multiplexer
