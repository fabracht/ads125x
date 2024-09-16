# ADS1256 Commands

| Command  | Description                           | Byte    |
| -------- | ------------------------------------- | ------- |
| WAKEUP   | Completes SYNC and Exits Standby Mode | 00h/FFh |
| RDATA    | Read Data                             | 01h     |
| RDATAC   | Read Data Continuously                | 03h     |
| SDATAC   | Stop Read Data Continuously           | 0Fh     |
| RREG     | Read from Register                    | 1xh     |
| WREG     | Write to Register                     | 5xh     |
| SELFCAL  | Offset and Gain Self-Calibration      | F0h     |
| SELFOCAL | Offset Self-Calibration               | F1h     |
| SELFGCAL | Gain Self-Calibration                 | F2h     |
| SYSOCAL  | System Offset Calibration             | F3h     |
| SYSGCAL  | System Gain Calibration               | F4h     |
| SYNC     | Synchronize the A/D Conversion        | FCh     |
| STANDBY  | Begin Standby Mode                    | FDh     |
| RESET    | Reset to Power-Up Values              | FEh     |

Detailed usage for each command is available in the datasheet.
