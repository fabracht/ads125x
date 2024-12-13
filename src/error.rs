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
    InvalidState(&'static str),
    ChannelCyclingError {
        channel: u8,
        reason: &'static str,
    },
    DifferentialChannelError {
        positive: u8,
        negative: u8,
        reason: &'static str,
    },
    AsyncOperationError(&'static str),
    ConversionInProgress,
    NotReady,
    OperationCancelled,
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
            Ads1256Error::InvalidState(msg) => write!(f, "Invalid state: {}", msg),
            Ads1256Error::ChannelCyclingError { channel, reason } => {
                write!(
                    f,
                    "Channel cycling error on channel {}: {}",
                    channel, reason
                )
            }
            Ads1256Error::DifferentialChannelError {
                positive,
                negative,
                reason,
            } => {
                write!(
                    f,
                    "Differential channel error (P:{}, N:{}): {}",
                    positive, negative, reason
                )
            }
            Ads1256Error::AsyncOperationError(msg) => write!(f, "Async operation error: {}", msg),
            Ads1256Error::ConversionInProgress => write!(f, "Conversion already in progress"),
            Ads1256Error::NotReady => write!(f, "Device not ready for operation"),
            Ads1256Error::OperationCancelled => write!(f, "Operation was cancelled"),
        }
    }
}

#[cfg(feature = "defmt")]
impl<SpiError, GpioError> defmt::Format for Ads1256Error<SpiError, GpioError>
where
    SpiError: core::fmt::Debug,
    GpioError: core::fmt::Debug,
{
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::Spi(_) => defmt::write!(f, "SPI error"),
            Self::Gpio(_) => defmt::write!(f, "GPIO error"),
            Self::Timeout => defmt::write!(f, "Operation timed out"),
            Self::InvalidInputChannel => defmt::write!(f, "Invalid input channel"),
            Self::InvalidGainSetting => defmt::write!(f, "Invalid gain setting"),
            Self::InvalidDataRate => defmt::write!(f, "Invalid data rate"),
            Self::CalibrationError => defmt::write!(f, "Calibration failed"),
            Self::BufferConfigError => defmt::write!(f, "Buffer configuration error"),
            Self::ContinuousReadError => defmt::write!(f, "Continuous read mode error"),
            Self::SyncError => defmt::write!(f, "Synchronization error"),
            Self::PowerDownError => defmt::write!(f, "Power down error"),
            Self::DrdyTimeout {
                current_state,
                wait_time,
            } => {
                defmt::write!(
                    f,
                    "DRDY timeout: state={}, waited={}μs",
                    current_state,
                    wait_time
                )
            }
            Self::RegisterError {
                register,
                operation,
            } => {
                defmt::write!(f, "Register error: {} on 0x{:02X}", operation, register)
            }
            Self::InvalidState(msg) => {
                defmt::write!(f, "Invalid state: {}", msg)
            }
            Self::ChannelCyclingError { channel, reason } => {
                defmt::write!(f, "Channel cycling error on {}: {}", channel, reason)
            }
            Self::DifferentialChannelError {
                positive,
                negative,
                reason,
            } => {
                defmt::write!(
                    f,
                    "Differential error (P:{}, N:{}): {}",
                    positive,
                    negative,
                    reason
                )
            }
            Self::AsyncOperationError(msg) => defmt::write!(f, "Async error: {}", msg),
            Self::ConversionInProgress => defmt::write!(f, "Conversion in progress"),
            Self::NotReady => defmt::write!(f, "Device not ready"),
            Self::OperationCancelled => defmt::write!(f, "Operation cancelled"),
            _ => defmt::write!(f, "Unknown error"),
        }
    }
}
