# ADS1256 Configuration

## Programmable Gain Amplifier (PGA)

- Available gains: 1, 2, 4, 8, 16, 32, 64
- Controlled by bits PGA2, PGA1, PGA0 in ADCON register

## Data Rates

- 16 programmable data rates from 2.5 SPS to 30,000 SPS
- Controlled by DRATE register

## Input Multiplexer

- Configurable for 8 single-ended or 4 differential inputs
- Controlled by MUX register

## Analog Input Buffer

- Can be enabled/disabled
- Controlled by BUFEN bit in STATUS register

## Calibration

- Self-calibration and system calibration options available
- Calibration coefficients stored in OFC and FSC registers
