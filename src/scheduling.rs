//! Bounded lookahead on the existing 1 ms monotonic clock. No phase accumulator.
use super::{reversal::TxProgress, sine_velocity};
use crate::{experiment_data::*, xeryon_motion::ScanCommand};

pub const MIN_CONTROL_INTERVAL: u32 = 4;
pub const MAX_CONTROL_INTERVAL: u32 = 10;
pub const MAX_VELOCITY_ERROR: f64 = 0.07; // fraction of peak, plus rounding allowance

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Fixed100Hz,
    Adaptive,
}
impl Mode {
    pub fn name(self) -> &'static str {
        match self {
            Self::Fixed100Hz => "fixed100hz",
            Self::Adaptive => "adaptive",
        }
    }
}
pub fn tolerance(amplitude: u32, frequency: u32) -> f64 {
    (sine_max_velocity(amplitude, frequency) * MAX_VELOCITY_ERROR + 0.5).max(1.0)
}
pub fn crossing_after(frequency: u32, elapsed: u32) -> u32 {
    let phase = u64::from(frequency) * u64::from(elapsed);
    let crossing = if phase < 250_000 {
        250_000
    } else {
        ((phase - 250_000) / 500_000 + 1) * 500_000 + 250_000
    };
    crossing.div_ceil(u64::from(frequency)) as u32
}
/// Continuous-time maximum, including any velocity extrema between endpoints.
/// This is a queued-command approximation, not inferred physical velocity.
pub fn held_error(amplitude: u32, frequency: u32, held: f64, from: u32, to: u32) -> f64 {
    let mut error = (held - sine_velocity(amplitude, frequency, from))
        .abs()
        .max((held - sine_velocity(amplitude, frequency, to)).abs());
    let first = (u64::from(from) * u64::from(frequency)).div_ceil(500_000);
    let last = u64::from(to) * u64::from(frequency) / 500_000;
    // At most two extrema are needed even for a missed, multi-cycle poll.
    for k in first..=last.min(first + 1) {
        let peak = sine_max_velocity(amplitude, frequency) * if k % 2 == 0 { 1.0 } else { -1.0 };
        error = error.max((held - peak).abs());
    }
    error
}
pub fn next(mode: Mode, amplitude: u32, frequency: u32, elapsed: u32, held: f64) -> u32 {
    if mode == Mode::Fixed100Hz {
        return (elapsed / 10 + 1) * 10;
    }
    let mut interval = MIN_CONTROL_INTERVAL;
    for dt in MIN_CONTROL_INTERVAL..=MAX_CONTROL_INTERVAL {
        if held_error(amplitude, frequency, held, elapsed, elapsed + dt)
            > tolerance(amplitude, frequency)
        {
            break;
        }
        interval = dt;
    }
    (elapsed + interval).min(crossing_after(frequency, elapsed))
}

#[derive(Default)]
pub struct Stats {
    pub count: u32,
    min: u32,
    max: u32,
    sum: u64,
}
impl Stats {
    pub fn add(&mut self, value: u32) {
        self.min = if self.count == 0 {
            value
        } else {
            self.min.min(value)
        };
        self.max = self.max.max(value);
        self.sum += u64::from(value);
        self.count += 1;
    }
    fn write(&self, out: &mut impl core::fmt::Write) -> core::fmt::Result {
        if self.count == 0 {
            return write!(out, "null");
        }
        write!(
            out,
            "{{\"count\":{},\"min\":{},\"mean\":{:.3},\"max\":{}}}",
            self.count,
            self.min,
            self.sum as f64 / f64::from(self.count),
            self.max
        )
    }
}
#[derive(Clone, Copy, Default)]
struct Pending {
    sequence: u32,
    scheduled: u32,
    queued: u32,
}
pub struct Diagnostics {
    pub mode: Mode,
    pub evaluations: u32,
    pub attempted: u32,
    pub sent: u32,
    pub sspd: u32,
    pub busy_due: u32,
    pub queue_peak: u32,
    pub max_error: f64,
    pub over_budget: u32,
    pub lateness: Stats,
    pub reversal_lateness: Stats,
    intervals: Stats,
    queue_delay: Stats,
    completion_delay: Stats,
    delivery_delay: Stats,
    pending: [Pending; 2],
    last_speed_ms: Option<u32>,
    observed_ms: u32,
    pub held: Option<f64>,
    pub waveform_ms: u32,
}
impl Default for Diagnostics {
    #[allow(unexpected_cfgs)] // desktop interoperability tests include this firmware module
    fn default() -> Self {
        Self {
            mode: if cfg!(feature = "fixed-100hz") {
                Mode::Fixed100Hz
            } else {
                Mode::Adaptive
            },
            evaluations: 0,
            attempted: 0,
            sent: 0,
            sspd: 0,
            busy_due: 0,
            queue_peak: 0,
            max_error: 0.0,
            over_budget: 0,
            lateness: Stats::default(),
            reversal_lateness: Stats::default(),
            intervals: Stats::default(),
            queue_delay: Stats::default(),
            completion_delay: Stats::default(),
            delivery_delay: Stats::default(),
            pending: [Pending::default(); 2],
            last_speed_ms: None,
            observed_ms: 0,
            held: None,
            waveform_ms: 0,
        }
    }
}
impl Diagnostics {
    pub fn observe(&mut self, amplitude: u32, frequency: u32, elapsed: u32) {
        if let Some(held) = self.held {
            let error = held_error(amplitude, frequency, held, self.observed_ms, elapsed);
            self.max_error = self.max_error.max(error);
            self.over_budget += u32::from(
                elapsed != self.observed_ms && error > tolerance(amplitude, frequency) + 1e-6,
            );
        }
        self.observed_ms = elapsed;
        self.waveform_ms = elapsed;
    }
    pub fn refresh(&mut self, progress: TxProgress) {
        self.queue_peak = self
            .queue_peak
            .max(progress.0[0].wrapping_sub(progress.0[1]));
        for pending in &mut self.pending {
            if pending.sequence == 0 {
                continue;
            }
            let ago = progress.0[1].wrapping_sub(pending.sequence);
            if ago < 2 {
                let completed = progress.0[5 - ago as usize];
                self.completion_delay
                    .add(completed.wrapping_sub(pending.queued));
                self.delivery_delay
                    .add(completed.wrapping_sub(pending.scheduled));
                pending.sequence = 0;
            }
        }
    }
    pub fn sent(
        &mut self,
        commands: &[ScanCommand],
        progress: TxProgress,
        scheduled: u32,
        now: u32,
        elapsed: u32,
        direction: i8,
    ) {
        self.sent += commands.len() as u32;
        self.queue_peak = self
            .queue_peak
            .max(progress.0[0].wrapping_sub(progress.0[1]));
        for (i, command) in commands.iter().enumerate() {
            let ago = commands.len() - 1 - i;
            // A zero sequence is a test double with no transport instrumentation.
            let queued = if progress.0[0] == 0 {
                now
            } else {
                progress.0[3 - ago]
            };
            if progress.0[0] != 0 {
                self.queue_delay.add(queued.wrapping_sub(scheduled));
                self.pending[i] = Pending {
                    sequence: progress.0[0] - ago as u32,
                    scheduled,
                    queued,
                };
            }
            if let ScanCommand::Speed(speed) = command {
                self.sspd += 1;
                if let Some(last) = self.last_speed_ms {
                    self.intervals.add(queued.wrapping_sub(last));
                }
                self.last_speed_ms = Some(queued);
                self.held = Some(f64::from(*speed) * f64::from(direction));
                self.observed_ms = elapsed;
            }
        }
    }
    pub fn write(
        &self,
        out: &mut impl core::fmt::Write,
        amplitude: u32,
        frequency: u32,
    ) -> core::fmt::Result {
        write!(
            out,
            ",\"scheduler\":{{\"mode\":\"{}\",\"min_ms\":{},\"max_ms\":{},\"max_velocity_error_um_s\":{:.3},\"evaluations\":{},\"commands_attempted\":{},\"commands_sent\":{},\"sspd_sent\":{},\"sspd_hz\":{:.3},\"held_error_um_s\":{:.3},\"over_budget_observations\":{},\"busy_due\":{},\"queue_peak\":{},\"completion_unobserved\":{}",
            self.mode.name(),
            if self.mode == Mode::Adaptive {
                MIN_CONTROL_INTERVAL
            } else {
                MAX_CONTROL_INTERVAL
            },
            MAX_CONTROL_INTERVAL,
            tolerance(amplitude, frequency),
            self.evaluations,
            self.attempted,
            self.sent,
            self.sspd,
            f64::from(self.sspd) * 1000.0 / f64::from(self.waveform_ms.max(1)),
            self.max_error,
            self.over_budget,
            self.busy_due,
            self.queue_peak,
            self.sent.saturating_sub(self.completion_delay.count)
        )?;
        for (name, stats) in [
            ("sspd_interval_ms", &self.intervals),
            ("lateness_ms", &self.lateness),
            ("reversal_lateness_ms", &self.reversal_lateness),
            ("scheduled_to_queue_ms", &self.queue_delay),
            ("queue_to_complete_ms", &self.completion_delay),
            ("scheduled_to_complete_ms", &self.delivery_delay),
        ] {
            write!(out, ",\"{name}\":")?;
            stats.write(out)?;
        }
        write!(out, "}}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xeryon_motion::ScanState;

    #[test]
    fn position_centers_need_less_density_than_extrema_and_bounds_are_explicit() {
        let a = 1_000_000;
        let f = 2000;
        let at = |t| next(Mode::Adaptive, a, f, t, sine_velocity(a, f, t)) - t;
        assert_eq!(at(0), MAX_CONTROL_INTERVAL); // position center, velocity maximum
        assert!(at(110) < at(0)); // approaching position extremum, large acceleration
        assert_eq!(next(Mode::Adaptive, a, f, 123, 0.0), 125); // priority crossing < minimum
        for t in 0..500 {
            let dt = at(t);
            assert!(dt <= MAX_CONTROL_INTERVAL);
            assert!(dt >= MIN_CONTROL_INTERVAL || t + dt == crossing_after(f, t));
        }
        assert_eq!(next(Mode::Fixed100Hz, a, f, 123, 0.0), 130);
    }

    fn simulate(a: u32, f: u32, mode: Mode) -> (f64, u32, f64) {
        let mut scan = ScanState::default();
        let mut held = 0.0;
        let mut time = 0;
        let end = 2_000_000u32.div_ceil(f);
        let mut error: f64 = 0.0;
        let mut sent = 0;
        let mut turns = 0;
        let mut last_command = None;
        while time < end {
            let direction = sine_direction(f, time);
            let crossing = scan.direction.is_some_and(|d| d != direction);
            scan.update(sine_velocity(a, f, time), direction, |commands| {
                for command in commands {
                    assert_ne!(*command, ScanCommand::Direction(0));
                    if let ScanCommand::Speed(speed) = command {
                        if !crossing {
                            assert_ne!(last_command, Some(*speed));
                        }
                        last_command = Some(*speed);
                        held = f64::from(*speed) * f64::from(direction);
                        sent += 1;
                    }
                }
                if crossing {
                    assert_eq!(
                        commands,
                        &[ScanCommand::Speed(0), ScanCommand::Direction(direction)]
                    );
                    turns += 1;
                }
                Ok(())
            })
            .unwrap();
            let to = next(mode, a, f, time, held).min(end);
            error = error.max(held_error(a, f, held, time, to));
            if mode == Mode::Adaptive {
                // Independent dense sampling of the analytic cosine, including sub-ms times.
                for tenth in time * 10..=to * 10 {
                    let v = sine_max_velocity(a, f)
                        * (core::f64::consts::TAU * f64::from(f) * f64::from(tenth) / 10_000_000.0)
                            .cos();
                    assert!(
                        (held - v).abs() <= tolerance(a, f) + 1e-6,
                        "a={a} f={f} t={tenth} error={} tolerance={}",
                        (held - v).abs(),
                        tolerance(a, f)
                    );
                }
            }
            time = to;
        }
        assert_eq!(turns, 4);
        (error, sent, f64::from(sent) * 1000.0 / f64::from(end))
    }

    #[test]
    fn simulated_quantized_hold_respects_error_budget_and_deduplicates() {
        for a in [800, 7_800, 1_000_000, 1_000_000_000] {
            for f in [100, 333, 1000, 1300, 1999, 2000] {
                let (error, _, _) = simulate(a, f, Mode::Adaptive);
                assert!(error <= tolerance(a, f) + 1e-6);
            }
        }
        for f in [1000, 1300, 2000] {
            let adaptive = simulate(1_000_000, f, Mode::Adaptive);
            let fixed = simulate(1_000_000, f, Mode::Fixed100Hz);
            println!("SIM f={f}mHz A=1000um adaptive(error,count,hz)={adaptive:?} fixed={fixed:?}");
        }
        assert!(simulate(800, 100, Mode::Adaptive).1 < 100); // many unchanged integer speeds
    }

    #[test]
    fn latency_pairs_ordered_commands_across_wrap_and_retains_missing_completions() {
        let mut d = Diagnostics::default();
        let scheduled = u32::MAX - 3;
        let queued = u32::MAX - 1;
        d.sent(
            &[ScanCommand::Speed(0), ScanCommand::Direction(-1)],
            TxProgress([2, 0, queued, queued, 0, 0]),
            scheduled,
            queued,
            10,
            -1,
        );
        d.refresh(TxProgress([2, 1, queued, queued, 0, 1]));
        assert_eq!(d.completion_delay.count, 1);
        assert_eq!(d.completion_delay.min, 3);
        d.refresh(TxProgress([2, 2, queued, queued, 1, 3]));
        d.refresh(TxProgress([2, 2, queued, queued, 1, 3]));
        assert_eq!(d.completion_delay.count, 2);
        assert_eq!(d.completion_delay.sum, 8);
        assert_eq!(d.delivery_delay.min, 5);
        assert_eq!(d.delivery_delay.max, 7);
        assert_eq!(d.queue_delay.sum, 4);
        assert_eq!(d.queue_peak, 2);
        d.sent(
            &[ScanCommand::Speed(100)],
            TxProgress([3, 2, queued, 10, 1, 3]),
            9,
            10,
            21,
            -1,
        );
        assert_eq!(d.intervals.min, 12);
        let mut json = String::new();
        d.write(&mut json, 1_000_000, 2000).unwrap();
        assert!(json.contains("\"completion_unobserved\":1"));
    }

    #[test]
    fn error_includes_hidden_velocity_extrema_and_late_poll_violations() {
        let peak = sine_max_velocity(1_000_000, 2000);
        assert_eq!(held_error(1_000_000, 2000, 0.0, 200, 300), peak);
        let mut d = Diagnostics::default();
        d.held = Some(0.0);
        d.observe(1_000_000, 2000, 500);
        assert_eq!(d.max_error, peak);
        assert_eq!(d.over_budget, 1);
    }
}
