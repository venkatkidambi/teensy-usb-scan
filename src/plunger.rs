//! Hardware-independent plunger control. Calibration comes from Motor.h.
pub const COUNTS_PER_MM: u32 = 16_688;

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

#[derive(Clone, Copy, Default)]
pub struct Motion {
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
