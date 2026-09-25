//! Shared plunger calibration, travel conversion and position-limit validation.
pub const COUNTS_PER_MM: u32 = 16_688;

#[derive(Clone, Copy)]
pub struct Limits {
    pub min: u8,
    pub max: u8,
}
impl Default for Limits {
    fn default() -> Self {
        Self { min: 30, max: 75 }
    }
}
impl Limits {
    pub fn new(min: u8, max: u8) -> Result<Self, &'static str> {
        if min >= max || max > 100 {
            return Err("Limits must satisfy 0 <= min < max <= 100");
        }
        Ok(Self { min, max })
    }
}

pub fn distance_counts(um: u32) -> Option<u32> {
    if um == 0 {
        return None;
    }
    u32::try_from((u64::from(um) * u64::from(COUNTS_PER_MM) + 500) / 1000)
        .ok()
        .filter(|&counts| counts <= i32::MAX as u32)
}
