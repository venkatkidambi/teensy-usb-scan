//! Hardware-independent plunger control. Calibration comes from Motor.h.
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
    pub fn allows(self, up: bool, pot: Option<u16>) -> bool {
        let Some(value) = pot.filter(|v| *v <= 4095) else {
            return false;
        };
        // Round inward so ADC quantization cannot extend the requested window.
        if up {
            value < (u32::from(self.max) * 4095 / 100) as u16
        } else {
            value > (u32::from(self.min) * 4095).div_ceil(100) as u16
        }
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

pub fn encoder_delta(previous: u8, current: u8) -> i32 {
    match (previous << 2) | current {
        0b0001 | 0b0111 | 0b1110 | 0b1000 => 1,
        0b0010 | 0b1011 | 0b1101 | 0b0100 => -1,
        _ => 0,
    }
}

/// Actual pad levels and PWM register readback, separate from commanded state.
#[derive(Clone, Copy, Default)]
pub struct Io {
    pub standby: bool,
    pub ain1: bool,
    pub ain2: bool,
    pub encoder: u8,
    pub pwm_count: i16,
    pub pwm_compare: i16,
    pub pwm_running: bool,
    pub pwm_enabled: bool,
}

#[derive(Clone, Copy, Default)]
pub struct Motion {
    pub limits: Limits,
    pub position: i32,
    pub running: bool,
    pub speed: u8,
    pub up: bool,
    start: i32,
    target: u32,
}
impl Motion {
    pub fn start(&mut self, up: bool, speed: u8, um: u32) -> Result<(), &'static str> {
        if self.running {
            return Err("Plunger is already moving");
        }
        if speed == 0 {
            return Err("Speed must be 1..255");
        }
        let target = distance_counts(um).ok_or("Invalid travel distance")?;
        self.start = self.position;
        self.target = target;
        self.speed = speed;
        self.up = up;
        self.running = true;
        Ok(())
    }
    pub fn step(&mut self, delta: i32) -> bool {
        self.position = self.position.wrapping_add(delta);
        if self.running && self.position.wrapping_sub(self.start).unsigned_abs() >= self.target {
            self.stop();
            return true;
        }
        false
    }
    pub fn stop(&mut self) {
        self.running = false;
        self.speed = 0;
    }
    pub fn zero(&mut self) -> Result<(), &'static str> {
        if self.running {
            return Err("Stop the plunger before zeroing");
        }
        self.position = 0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn potentiometer_limits_and_recovery() {
        let limits = Limits::default();
        assert_eq!((limits.min, limits.max), (30, 75));
        assert!(limits.allows(true, Some(3070)));
        assert!(!limits.allows(true, Some(3071)));
        assert!(!limits.allows(false, Some(1229)));
        assert!(limits.allows(false, Some(1230)));
        assert!(limits.allows(true, Some(0)));
        assert!(!limits.allows(false, Some(0)));
        assert!(limits.allows(false, Some(4095)));
        assert!(!limits.allows(true, Some(4095)));
        for up in [true, false] {
            assert!(!limits.allows(up, None));
            assert!(!limits.allows(up, Some(4096)));
        }
        assert!(Limits::new(0, 100).is_ok());
        for (min, max) in [(30, 30), (75, 30), (0, 101)] {
            assert!(Limits::new(min, max).is_err());
        }
        let custom = Limits::new(40, 60).unwrap();
        assert!(!custom.allows(true, Some(2457)));
        assert!(!custom.allows(false, Some(1638)));
    }
    #[test]
    fn calibration_and_limits() {
        assert_eq!(distance_counts(1), Some(17));
        assert_eq!(distance_counts(1000), Some(16688));
        assert_eq!(distance_counts(0), None);
        assert_eq!(distance_counts(u32::MAX), None);
    }
    #[test]
    fn quadrature_and_target_stop() {
        let mut motion = Motion::default();
        motion.start(true, 100, 1).unwrap();
        assert!(motion.zero().is_err());
        assert!(motion.start(false, 100, 1).is_err());
        let states = [0, 1, 3, 2, 0];
        for pair in states.windows(2) {
            assert_eq!(encoder_delta(pair[0], pair[1]), 1);
        }
        for pair in states.windows(2) {
            assert_eq!(encoder_delta(pair[1], pair[0]), -1);
        }
        assert_eq!(encoder_delta(0, 3), 0);
        for _ in 0..16 {
            assert!(!motion.step(1));
        }
        assert!(motion.step(1));
        assert!(!motion.running);
        assert_eq!(motion.speed, 0);
        motion.zero().unwrap();
        motion.start(false, 255, 1).unwrap();
        for _ in 0..16 {
            motion.step(-1);
        }
        assert!(motion.step(-1));
    }
}
