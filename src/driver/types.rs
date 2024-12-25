// src/driver/types.rs
/// Operating mode of the ADC
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OperatingMode {
    /// One-shot mode using STANDBY/WAKEUP for power efficiency
    OneShot,
    /// Continuous conversion mode
    Continuous,
}
