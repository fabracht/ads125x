/// ADS1256 Commands
pub const CMD_WAKEUP: u8 = 0x00;
pub const CMD_RDATA: u8 = 0x01;
// pub const CMD_RDATAC: u8 = 0x03;
pub const CMD_SDATAC: u8 = 0x0F;
pub const CMD_RREG: u8 = 0x10;
pub const CMD_WREG: u8 = 0x50;
pub const CMD_SELFCAL: u8 = 0xF0;
pub const CMD_SELFOCAL: u8 = 0xF1;
pub const CMD_SELFGCAL: u8 = 0xF2;
pub const CMD_SYSOCAL: u8 = 0xF3;
pub const CMD_SYSGCAL: u8 = 0xF4;
pub const CMD_SYNC: u8 = 0xFC;
// pub const CMD_STANDBY: u8 = 0xFD;
pub const CMD_RESET: u8 = 0xFE;

/// ADS1256 Registers
pub const REG_STATUS: u8 = 0x00;
pub const REG_MUX: u8 = 0x01;
pub const REG_ADCON: u8 = 0x02;
pub const REG_DRATE: u8 = 0x03;
pub const REG_IO: u8 = 0x04;
pub const REG_OFC0: u8 = 0x05;
pub const REG_OFC1: u8 = 0x06;
pub const REG_OFC2: u8 = 0x07;
pub const REG_FSC0: u8 = 0x08;
pub const REG_FSC1: u8 = 0x09;
pub const REG_FSC2: u8 = 0x0A;

/// Gain settings for the ADS1256 programmable gain amplifier (PGA)
#[derive(Clone, Copy, Debug)]
pub enum Gain {
    Gain1 = 0b000,
    Gain2 = 0b001,
    Gain4 = 0b010,
    Gain8 = 0b011,
    Gain16 = 0b100,
    Gain32 = 0b101,
    Gain64 = 0b110,
}

impl Gain {
    /// Returns the gain value as a floating-point number
    pub fn value(&self) -> f64 {
        match self {
            Gain::Gain1 => 1.0,
            Gain::Gain2 => 2.0,
            Gain::Gain4 => 4.0,
            Gain::Gain8 => 8.0,
            Gain::Gain16 => 16.0,
            Gain::Gain32 => 32.0,
            Gain::Gain64 => 64.0,
        }
    }
}

/// Data rates for the ADS1256
#[derive(Clone, Copy, Debug)]
pub enum DataRate {
    Sps30000 = 0xF0,
    Sps15000 = 0xE0,
    Sps7500 = 0xD0,
    Sps3750 = 0xC0,
    Sps2000 = 0xB0,
    Sps1000 = 0xA1,
    Sps500 = 0x92,
    Sps100 = 0x82,
    Sps60 = 0x72,
    Sps50 = 0x63,
    Sps30 = 0x53,
    Sps25 = 0x43,
    Sps15 = 0x33,
    Sps10 = 0x23,
    Sps5 = 0x13,
    Sps2_5 = 0x03,
}
