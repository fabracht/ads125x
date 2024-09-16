# ADS1256 Registers

| Address | Register | Reset Value | Description                         |
| ------- | -------- | ----------- | ----------------------------------- |
| 00h     | STATUS   | x1h         | Status Control Register             |
| 01h     | MUX      | 01h         | Input Multiplexer Control Register  |
| 02h     | ADCON    | 20h         | A/D Control Register                |
| 03h     | DRATE    | F0h         | A/D Data Rate Control Register      |
| 04h     | IO       | E0h         | GPIO Control Register               |
| 05h-07h | OFC[2:0] | xxh         | Offset Calibration Coefficients     |
| 08h-0Ah | FSC[2:0] | xxh         | Full-Scale Calibration Coefficients |

Detailed bit descriptions for each register are available in the datasheet.
