//! Hardware-independent plunger control. Calibration comes from Motor.h.
pub use crate::plunger_config::{COUNTS_PER_MM, Limits, distance_counts};

impl Limits {
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
