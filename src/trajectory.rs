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

/// Validate controller limits and establish an exact pulse reference before ENABLE.
/// The cached DPOS is read-only diagnostics; no serial target is ever sent.
pub fn gpio_origin(
    config: ExperimentConfig,
    t: &crate::protocol::Telemetry,
    reference: Option<i32>,
    now: u32,
) -> Result<i32, &'static str> {
    use crate::experiment_data::{MAX_POSITION, MIN_POSITION, counts};
    config.validate()?;
    let ExperimentConfig::Sinusoidal {
        center_nm,
        amplitude_nm,
        ..
    } = config
    else {
        return Err("GPIO requires sinusoidal configuration");
    };
    if !t.gpio.iter().all(|s| s.valid) || t.gpio[0].value != 2 || t.gpio[1].value != 1 {
        return Err("GPIO configuration or travel limits unconfirmed");
    }
    let low = t.gpio[2].value;
    let high = t.gpio[3].value;
    let start = config.start()?;
    let end = counts(center_nm + i64::from(amplitude_nm))?;
    if low < i64::from(MIN_POSITION)
        || high > i64::from(MAX_POSITION)
        || low >= high
        || [i64::from(start), i64::from(end), t.samples[0].value]
            .iter()
            .any(|p| *p < low || *p > high)
    {
        return Err("GPIO trajectory exceeds controller travel limits");
    }
    if [t.samples[0], t.samples[2]]
        .iter()
        .any(|s| !s.valid || now.wrapping_sub(s.ms) >= 250)
        || crate::xeryon_motion::moving(t.samples[2].value)
    {
        return Err("GPIO requires fresh stationary encoder/status feedback");
    }
    let origin = match reference {
        Some(position) => i64::from(position),
        None if t.samples[1].valid && now.wrapping_sub(t.samples[1].ms) < 250 => t.samples[1].value,
        None => return Err("GPIO reference diagnostic is stale"),
    };
    if origin != t.samples[0].value {
        return Err(
            "GPIO reference unconfirmed; encoder and retained target must agree before START",
        );
    }
    Ok(origin as i32)
}

#[derive(Clone, Copy)]
pub struct Trajectory {
    center_nm: i64,
    amplitude_nm: u32,
    frequency_mhz: u32,
    pub duration_us: u32,
    pub start: i32,
    approach: Option<i32>,
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
            approach: None,
        })
    }
    /// Half-cosine approach: zero endpoint velocity, peak <4 kHz.
    pub fn approach(from: i32, to: i32) -> Result<Self, &'static str> {
        use crate::experiment_data::{MAX_DURATION_MS, MAX_POSITION, MIN_POSITION};
        if [from, to]
            .iter()
            .any(|p| !(MIN_POSITION..=MAX_POSITION).contains(p))
        {
            return Err("GPIO approach exceeds encoder range");
        }
        let duration = u64::from(from.abs_diff(to)) * 400;
        if duration + 1_000_000 > u64::from(MAX_DURATION_MS) * 1000 {
            return Err("GPIO approach exceeds positioning time budget");
        }
        Ok(Self {
            center_nm: 0,
            amplitude_nm: 0,
            frequency_mhz: 0,
            duration_us: duration.max(1000) as u32,
            start: from,
            approach: Some(to),
        })
    }
    pub fn end(&self) -> i32 {
        self.approach.unwrap_or(self.start)
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
            return self.end();
        }
        if let Some(to) = self.approach {
            let phase = u64::from(elapsed_us) * TURN / (2 * u64::from(self.duration_us));
            let delta = i64::from(to) - i64::from(self.start);
            let displacement = delta * (Q - cosine(phase));
            return self.start + ((displacement + if delta < 0 { -Q } else { Q }) / (2 * Q)) as i32;
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
            if terminal && self.position == self.trajectory.end() {
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
    fn gpio_preflight_checks_limits_reference_and_freshness_before_enable() {
        let mut t = crate::protocol::Telemetry::new();
        for (tag, value) in [
            (b"GPIO", 2),
            (b"STPS", 1),
            (b"LLIM", -2000),
            (b"HLIM", 2000),
            (b"EPOS", 1000),
            (b"DPOS", 1000),
            (b"STAT", 1344),
        ] {
            t.record(tag, value, u32::MAX - 10);
        }
        assert_eq!(gpio_origin(config(7800, 1000, 1), &t, None, 1), Ok(1000));
        t.record(b"DPOS", -1000, 1);
        assert!(gpio_origin(config(7800, 1000, 1), &t, None, 1).is_err());
        assert_eq!(
            gpio_origin(config(7800, 1000, 1), &t, Some(1000), 1),
            Ok(1000)
        );
        assert!(gpio_origin(config(7800, 1000, 1), &t, Some(1000), 251).is_err());
        t.record(b"HLIM", 1050, 1);
        assert!(gpio_origin(config(7800, 1000, 1), &t, Some(1000), 1).is_err());
        t.record(b"HLIM", 2000, 1);
        t.record(b"LLIM", 950, 1);
        assert!(gpio_origin(config(7800, 1000, 1), &t, Some(1000), 1).is_err());
    }
    #[test]
    fn approach_is_bounded_monotonic_and_returns_exact_pulse_count_with_irq_jitter() {
        for (from, to) in [(0, 1), (1, 0), (-1000, 1000), (1000, -1000), (0, 0)] {
            let t = Trajectory::approach(from, to).unwrap();
            let mut p = Pulses::new(t);
            let mut elapsed = 0;
            let mut rises = 0;
            while p.status == Status::Running {
                elapsed += 28; // includes bounded interrupt work/jitter
                p.tick(elapsed);
                if p.high {
                    rises += 1;
                }
                assert!((from.min(to)..=from.max(to)).contains(&p.position));
                assert!(elapsed < t.duration_us + 200);
            }
            assert_eq!(p.status, Status::Complete);
            assert_eq!(p.position, to);
            assert_eq!(rises, from.abs_diff(to));
            assert!(!p.high);
        }
        assert!(Trajectory::approach(i32::MIN, 0).is_err());
        assert!(Trajectory::approach(0, 1_000_000).is_err());
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
