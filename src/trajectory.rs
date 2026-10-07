//! Integer, absolute-time sinusoidal position and bounded edge generation.
//! Shared with native tests. No feedback correction or elapsed-loop accumulator.
use crate::experiment_data::ExperimentConfig;
pub const TICK_US: u32 = 25;
const Q: i64 = 1 << 30;
const TURN: u64 = 1_000_000_000; // microseconds * millihertz

/// Range-reduced Q30 sine polynomial. Accepted amplitudes keep approximation
/// error below 0.01 encoder count; cardinal phases are exact.
fn cosine(phase: u64) -> i64 {
    let phase = (phase + TURN / 4) % TURN;
    let signed = if phase > TURN / 2 {
        phase as i64 - TURN as i64
    } else {
        phase as i64
    };
    let folded = if signed > TURN as i64 / 4 {
        TURN as i64 / 2 - signed
    } else if signed < -(TURN as i64) / 4 {
        -(TURN as i64) / 2 - signed
    } else {
        signed
    };
    if folded == TURN as i64 / 4 {
        return Q;
    }
    if folded == -(TURN as i64) / 4 {
        return -Q;
    }
    let x = folded * 6_746_518_852 / TURN as i64;
    let xx = x * x / Q;
    let mut p = -27; // -1/11!
    for c in [2959, -213044, 8947849, -178956971, Q] {
        p = c + xx * p / Q;
    }
    (x * p / Q).clamp(-Q, Q)
}

#[derive(Clone, Copy)]
pub struct Trajectory {
    center_nm: i64,
    amplitude_nm: u32,
    frequency_mhz: u32,
    pub duration_us: u32,
    pub start: i32,
}
impl Trajectory {
    pub fn new(config: ExperimentConfig) -> Result<Self, &'static str> {
        config.validate()?;
        let ExperimentConfig::Sinusoidal {
            center_nm,
            amplitude_nm,
            frequency_mhz,
            cycles,
            ..
        } = config
        else {
            return Err("GPIO requires sinusoidal configuration");
        };
        Ok(Self {
            center_nm,
            amplitude_nm,
            frequency_mhz,
            duration_us: (u64::from(cycles) * TURN).div_ceil(u64::from(frequency_mhz)) as u32,
            start: config.start()?,
        })
    }
    /// Derivative in um/s; integer phase avoids accumulated timing drift.
    pub fn velocity(&self, elapsed_us: u32) -> f64 {
        if elapsed_us >= self.duration_us {
            return 0.0;
        }
        let phase = u64::from(elapsed_us) * u64::from(self.frequency_mhz) % TURN;
        crate::experiment_data::sine_max_velocity(self.amplitude_nm, self.frequency_mhz)
            * cosine((phase + 3 * TURN / 4) % TURN) as f64
            / Q as f64
    }
    pub fn position(&self, elapsed_us: u32) -> i32 {
        if elapsed_us >= self.duration_us {
            return self.start;
        }
        let phase = u64::from(elapsed_us) * u64::from(self.frequency_mhz) % TURN;
        let nm = self.center_nm * Q - i64::from(self.amplitude_nm) * cosine(phase);
        ((nm + if nm < 0 { -39 * Q } else { 39 * Q }) / (78 * Q)) as i32
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Running,
    Complete,
    Stopped,
    Fault,
}
/// Returned pin levels, applied in ENABLE/PULSE-low, DIRECTION, PULSE-high order.
pub struct Pulses {
    pub trajectory: Trajectory,
    pub position: i32,
    pub high: bool,
    pub positive: bool,
    pub status: Status,
    last_tick: u32,
    last_rise: u32,
}
impl Pulses {
    pub fn new(trajectory: Trajectory) -> Self {
        Self {
            position: trajectory.start,
            trajectory,
            high: false,
            positive: true,
            status: Status::Running,
            last_tick: 0,
            last_rise: 0,
        }
    }
    pub fn stop(&mut self) {
        self.high = false;
        self.status = Status::Stopped;
    }
    pub fn tick(&mut self, elapsed: u32) {
        if self.status != Status::Running {
            self.high = false;
            return;
        }
        if elapsed.wrapping_sub(self.last_tick) > 75 {
            self.high = false;
            self.status = Status::Fault;
            return;
        }
        self.last_tick = elapsed;
        let terminal = elapsed >= self.trajectory.duration_us;
        if self.high {
            self.high = false;
            if terminal && self.position == self.trajectory.start {
                self.status = Status::Complete;
            }
            return;
        }
        let delta = self.trajectory.position(elapsed) - self.position;
        if terminal && delta == 0 {
            self.status = Status::Complete;
            return;
        }
        // A half-count-tie endpoint may need its final edge at the boundary.
        // Hold that exact target until its falling edge, never enter cycle N+1.
        if delta.abs() > 1 {
            self.status = Status::Fault;
            return;
        }
        if delta == 0 {
            return;
        }
        let positive = delta > 0;
        if positive != self.positive {
            self.positive = positive; // one full low tick of direction setup
            return;
        }
        if elapsed.wrapping_sub(self.last_rise) < 75 {
            return;
        }
        self.position += delta;
        self.high = true;
        self.last_rise = elapsed;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::experiment_data::counts;
    fn config(a: u32, f: u32, n: u32) -> ExperimentConfig {
        ExperimentConfig::Sinusoidal {
            center_nm: 78000,
            amplitude_nm: a,
            frequency_mhz: f,
            cycles: n,
            control: crate::experiment_data::SineControl::Gpio,
        }
    }
    #[test]
    fn conversion_cardinals_rate_and_bounds() {
        assert_eq!(counts(1000), Ok(13)); // 1 um / 0.078
        assert_eq!(counts(-1000), Ok(-13));
        let t = Trajectory::new(config(7800, 1000, 1)).unwrap();
        assert_eq!(
            [0, 250000, 500000, 750000, 1000000].map(|us| t.position(us)),
            [900, 1000, 1100, 1000, 900]
        );
        assert!(config(124_140, 1000, 1).validate().is_ok());
        assert!(config(124_141, 1000, 1).validate().is_err());
        assert!(config(u32::MAX, 1000, 1).validate().is_err());
    }
    #[test]
    fn half_count_terminal_tie_finishes_without_an_extra_cycle() {
        let t = Trajectory::new(ExperimentConfig::Sinusoidal {
            center_nm: 39,
            amplitude_nm: 78,
            frequency_mhz: 1000,
            cycles: 1,
            control: crate::experiment_data::SineControl::Gpio,
        })
        .unwrap();
        let mut p = Pulses::new(t);
        for us in (25..=t.duration_us + 100).step_by(25) {
            p.tick(us);
        }
        assert_eq!(p.status, Status::Complete);
        assert_eq!(p.position, -1);
        assert!(!p.high);
    }
    #[test]
    fn pulses_reverse_complete_exact_cycles_and_stop_on_abort_or_deadline() {
        for (a, f, n) in [
            (7800, 1000, 3),
            (124140, 1000, 1),
            (12414, 10000, 2),
            (1_241_400, 100, 1),
            (7800, 1300, 2),
        ] {
            let t = Trajectory::new(config(a, f, n)).unwrap();
            let mut p = Pulses::new(t);
            let (mut rises, mut previous, mut last) = (0, false, 0);
            let (mut direction, mut changes) = (true, 0);
            for us in (TICK_US..t.duration_us).step_by(TICK_US as usize) {
                p.tick(us);
                assert_eq!(p.status, Status::Running, "{us}");
                if p.positive != direction {
                    assert!(!p.high); // direction changes only while PULSE is low
                    direction = p.positive;
                    changes += 1;
                }
                if p.high && !previous {
                    rises += 1;
                    assert!(us - last >= 75);
                    last = us;
                }
                previous = p.high;
                let reference = (78000.0
                    - f64::from(a)
                        * (f64::from(us) * f64::from(f) * core::f64::consts::TAU / 1e9).cos())
                    / 78.0;
                assert!((f64::from(t.position(us)) - reference).abs() < 0.51);
            }
            p.tick(t.duration_us);
            assert_eq!(p.status, Status::Complete);
            assert_eq!(p.position, t.start);
            assert_eq!(changes, 2 * n - 1);
            let max = counts(78000 + i64::from(a)).unwrap();
            assert_eq!(rises, 2 * (max - t.start) as u32 * n);
            let mut p = Pulses::new(t);
            let mut us = TICK_US;
            while !p.high {
                p.tick(us);
                us += TICK_US;
            }
            p.stop();
            p.tick(us);
            assert!(!p.high);
            assert_eq!(p.status, Status::Stopped);
            let mut p = Pulses::new(t);
            p.tick(100);
            assert_eq!(p.status, Status::Fault);
            assert!(!p.high);
        }
    }
}
