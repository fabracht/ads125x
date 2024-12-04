use core::fmt;

#[derive(Debug)]
pub enum Ads1256Error<SpiError, GpioError> {
    Spi(SpiError),
    Gpio(GpioError),
    Timeout,
    InvalidInputChannel,
    InvalidResponse,
    InputVoltageOutOfRange,
    CalibrationError,
    RegisterError {
        register: u8,
        operation: &'static str,
    },
    DrdyTimeout {
        current_state: bool,
        wait_time: u32,
    },
    BufferConfigError,
    InvalidGainSetting,
    InvalidDataRate,
    ContinuousReadError,
    SyncError,
    PowerDownError,
}

impl<SpiError, GpioError> fmt::Display for Ads1256Error<SpiError, GpioError>
where
    SpiError: fmt::Debug,
    GpioError: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Ads1256Error::Spi(e) => write!(f, "SPI error: {:?}", e),
            Ads1256Error::Gpio(e) => write!(f, "GPIO error: {:?}", e),
            Ads1256Error::InvalidResponse => write!(f, "Invalid response from ADS1256"),
            Ads1256Error::Timeout => write!(f, "Operation timed out"),
            Ads1256Error::InvalidInputChannel => write!(f, "Invalid input channel"),
            Ads1256Error::InputVoltageOutOfRange => write!(f, "Input voltage out of range"),
            Ads1256Error::CalibrationError => write!(f, "Calibration failed"),
            Ads1256Error::RegisterError {
                register,
                operation,
            } => {
                write!(
                    f,
                    "Register error: {} on register 0x{:02X}",
                    operation, register
                )
            }
            Ads1256Error::DrdyTimeout {
                current_state,
                wait_time,
            } => {
                write!(
                    f,
                    "DRDY timeout - state: {}, waited: {}μs",
                    current_state, wait_time
                )
            }
            Ads1256Error::BufferConfigError => write!(f, "Buffer configuration error"),
            Ads1256Error::InvalidGainSetting => write!(f, "Invalid gain setting"),
            Ads1256Error::InvalidDataRate => write!(f, "Invalid data rate"),
            Ads1256Error::ContinuousReadError => write!(f, "Continuous read mode error"),
            Ads1256Error::SyncError => write!(f, "Synchronization error"),
            Ads1256Error::PowerDownError => write!(f, "Power down error"),
        }
    }
}
