//! Foreground-only autonomous controller. Hardware IO stays behind a Rust trait.
use crate::{experiment_data::*, pressure::Reading, protocol::Telemetry, xeryon_motion};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Dashboard,
    Validate,
    MoveToStart,
    Armed,
    Running,
    Restoring,
    Complete,
    Aborted,
    Fault,
    DataReady,
    Transferring,
}
pub trait Actuator {
    fn target(&mut self, counts: i32, now: u32, track_completion: bool)
    -> Result<(), &'static str>;
    fn stop(&mut self) -> Result<(), &'static str>;
    fn tx_idle(&self) -> bool;
    /// True once accepted; false means busy, without queuing behind an old packet.
    fn command(&mut self, command: crate::sine_control::Command) -> Result<bool, &'static str>;
}
pub struct Engine<'a> {
    pub state: State,
    pub config: Option<ExperimentConfig>,
    pub run: u32,
    pub outcome: &'static str,
    pub reason: &'static str,
    pub stop_queued: bool,
    pub started_ms: u32,
    pub duration_ms: u32,
    pub elapsed_ms: u32,
    pub missed_samples: u32,
    pub max_lateness_ms: u32,
    pub len: usize,
    pub required_samples: usize,
    samples: &'a mut [Record],
    crc: u32,
    move_started: u32,
    next_sample: u32,
    next_target: u32,
    step: u32,
    hold_started: Option<u32>,
    target: i32,
    epoch: u32,
    transfer_next: usize,
    settling: bool,
    center_sent: bool,
    last_ms: u32,
    scan: crate::sine_control::Session,
    pub settings_restore_sent: bool,
    pub control_updates: u32,
    pub max_control_lateness_ms: u32,
}
impl<'a> Engine<'a> {
    pub fn new(samples: &'a mut [Record]) -> Self {
        Self {
            state: State::Dashboard,
            config: None,
            run: 0,
            outcome: "none",
            reason: "none",
            stop_queued: false,
            started_ms: 0,
            duration_ms: 0,
            elapsed_ms: 0,
            missed_samples: 0,
            max_lateness_ms: 0,
            len: 0,
            required_samples: 0,
            samples,
            crc: !0,
            move_started: 0,
            next_sample: 0,
            next_target: 0,
            step: 0,
            hold_started: None,
            target: 0,
            epoch: 0,
            transfer_next: 0,
            settling: false,
            center_sent: false,
            last_ms: 0,
            scan: crate::sine_control::Session::new(),
            settings_restore_sent: true,
            control_updates: 0,
            max_control_lateness_ms: 0,
        }
    }
    pub fn active(&self) -> bool {
        matches!(
            self.state,
            State::MoveToStart | State::Armed | State::Running | State::Restoring
        )
    }
    pub fn owns_hardware(&self) -> bool {
        self.state != State::Dashboard
    }
    pub fn data_ready(&self) -> bool {
        matches!(self.state, State::DataReady | State::Transferring)
    }
    pub fn checksum(&self) -> u32 {
        !self.crc
    }
    fn healthy(t: &Telemetry, p: Reading, now: u32, scanning: bool) -> Result<(), &'static str> {
        if !xeryon_motion::ready(t) {
            return Err("USB2 disconnected");
        }
        if !xeryon_motion::fresh(t, 0, now) || !xeryon_motion::fresh(t, 2, now) {
            return Err("Xeryon feedback stale");
        }
        if t.samples[2].value & xeryon_motion::FAULTS != 0 {
            return Err("Xeryon controller fault");
        }
        let incompatible =
            (1 << 1) | (1 << 14) | (1 << 15) | (1 << 9) | if scanning { 0 } else { 1 << 13 };
        if t.samples[2].value & incompatible != 0 {
            return Err("Xeryon limit or incompatible motion");
        }
        // SCAN=0 / STOP can temporarily leave closed-loop mode. During owned
        // scan control permit that stopped transition, but an active scan must
        // still report closed loop and the encoder must always remain valid.
        let stat = t.samples[2].value;
        let needs_closed_loop = !scanning || stat & (1 << 13) != 0;
        if stat & (1 << 8) == 0 || (needs_closed_loop && stat & (1 << 6) == 0) {
            return Err("Xeryon closed loop or encoder invalid");
        }
        if !(i64::from(MIN_POSITION)..=i64::from(MAX_POSITION)).contains(&t.samples[0].value) {
            return Err("Invalid encoder position");
        }
        if !p.valid || p.state >= 3 || now.wrapping_sub(p.ms) >= 250 {
            return Err("Pressure failure or stale sample");
        }
        Ok(())
    }
    pub fn accept(
        &mut self,
        config: ExperimentConfig,
        t: &Telemetry,
        p: Reading,
        plunger_running: bool,
        now: u32,
    ) -> Result<(), &'static str> {
        if self.owns_hardware() {
            return Err("Experiment busy; acknowledge retained data first");
        }
        self.state = State::Validate;
        let validation = (|| {
            let duration = config.validate()?;
            let required = (duration / SAMPLE_MS + 1) as usize;
            if required > self.samples.len() {
                return Err("Experiment buffer too small");
            }
            Self::healthy(t, p, now, false)?;
            if plunger_running
                || t.xeryon_motion.is_some_and(|m| m.state == 1)
                || t.samples[2].value & (1 << 10) == 0
            {
                return Err("Stop manual motion before START");
            }
            Ok((duration, required, config.start()?))
        })();
        let (duration, required, target) = match validation {
            Ok(v) => v,
            Err(e) => {
                self.state = State::Dashboard;
                return Err(e);
            }
        };
        self.run = self.run.wrapping_add(1).max(1);
        self.config = Some(config);
        self.duration_ms = duration;
        self.required_samples = required;
        self.len = 0;
        self.crc = !0;
        self.elapsed_ms = 0;
        self.started_ms = 0;
        self.missed_samples = 0;
        self.max_lateness_ms = 0;
        self.outcome = "none";
        self.reason = "none";
        self.stop_queued = false;
        self.move_started = now;
        self.target = target;
        self.epoch = t.epoch;
        self.step = 0;
        self.settling = false;
        self.center_sent = false;
        self.last_ms = now;
        self.scan = crate::sine_control::Session::new();
        self.settings_restore_sent = true;
        self.control_updates = 0;
        self.max_control_lateness_ms = 0;
        self.hold_started = None;
        self.state = State::MoveToStart;
        // First target is sent by begin(), only after all validation succeeds.
        Ok(())
    }
    pub fn begin(&mut self, now: u32, io: &mut impl Actuator) {
        if let Err(e) = io.target(self.target, now, true) {
            self.finish(State::Fault, e, io);
        }
    }
    fn finish(&mut self, state: State, reason: &'static str, io: &mut impl Actuator) {
        #[cfg(not(test))]
        crate::fault::mark(5);
        self.state = state;
        self.outcome = match state {
            State::Complete => "completed",
            State::Aborted => "aborted",
            _ => "faulted",
        };
        self.reason = reason;
        self.stop_queued = io.stop().is_ok();
        if !self.stop_queued && state == State::Complete {
            self.outcome = "faulted";
            self.reason = "STOP could not be queued";
        }
        self.scan.begin_restore(self.last_ms);
        if self.scan.changed() {
            self.settings_restore_sent = false;
            self.state = State::Restoring;
        } else {
            self.state = State::DataReady;
        }
    }
    pub fn abort(&mut self, io: &mut impl Actuator) {
        if self.state == State::Restoring {
            self.stop_queued = io.stop().is_ok(); // retries remain useful without restarting cleanup
            return;
        }
        if self.active() {
            self.finish(State::Aborted, "PC ABORT", io);
        }
    }
    pub fn poll(&mut self, now: u32, t: &Telemetry, p: Reading, io: &mut impl Actuator) {
        if !self.active() {
            return;
        }
        self.last_ms = now;
        if self.state == State::Restoring {
            let restored = if t.epoch != self.epoch {
                Err("Controller changed during restore")
            } else {
                self.scan.restore(t, now, io)
            };
            match restored {
                Ok(true) => {
                    self.settings_restore_sent = true;
                    self.state = State::DataReady;
                }
                Err(e) => {
                    if self.outcome == "completed" {
                        self.outcome = "faulted";
                        self.reason = e;
                    }
                    self.state = State::DataReady;
                }
                _ => {}
            }
            return;
        }
        if self.state == State::Running {
            self.elapsed_ms = now.wrapping_sub(self.started_ms);
        }
        let fault = Self::healthy(t, p, now, self.scan.scan_started)
            .err()
            .or_else(|| (t.epoch != self.epoch).then_some("Xeryon connection changed"))
            .or_else(|| {
                t.xeryon_motion
                    .filter(|m| m.state == 4)
                    .map(|m| m.interruption_reason.unwrap_or("Xeryon motion failed"))
            });
        if let Some(e) = fault {
            self.finish(State::Fault, e, io);
            return;
        }
        if self.state == State::MoveToStart {
            if now.wrapping_sub(self.move_started) >= MOVE_TIMEOUT_MS {
                self.finish(State::Fault, "Start move deadline", io);
                return;
            }
            if !t
                .xeryon_motion
                .is_some_and(|m| m.state == 2 && m.target == Some(self.target))
            {
                return;
            }
            self.state = State::Armed;
            return;
        }
        if self.state == State::Armed {
            if let Some(config @ ExperimentConfig::Sinusoidal { .. }) = self.config {
                match self.scan.prepare(config, t, now, io) {
                    Ok(false) => return,
                    Err(e) => {
                        self.finish(State::Fault, e, io);
                        return;
                    }
                    Ok(true) => {}
                }
            }
            self.started_ms = now;
            self.next_sample = 0;
            self.next_target = 0;
            self.state = State::Running;
            #[cfg(not(test))]
            crate::fault::mark(4);
        }
        let elapsed = now.wrapping_sub(self.started_ms);
        self.elapsed_ms = elapsed;
        if let Some(ExperimentConfig::Sinusoidal {
            center_nm,
            amplitude_nm,
            frequency_mhz,
            ..
        }) = self.config
        {
            if !self.settling {
                if let Err(e) = self.scan.service(now, io) {
                    self.finish(State::Fault, e, io);
                    return;
                }
                // This field is the desired sine position, not a DPOS sent during SCAN.
                self.target = sine_target(center_nm, amplitude_nm, frequency_mhz, elapsed);
            }
        }
        // A deadline is also a hard bound on storage. Never synthesize catch-up samples.
        if elapsed > self.duration_ms {
            self.finish(State::Fault, "Acquisition deadline missed", io);
            return;
        }
        if elapsed >= self.next_sample {
            let late = elapsed - self.next_sample;
            self.max_lateness_ms = self.max_lateness_ms.max(late);
            self.missed_samples += late / SAMPLE_MS;
            self.next_sample = (elapsed / SAMPLE_MS + 1) * SAMPLE_MS;
            let time = t.samples[6];
            let record = Record {
                elapsed_ms: elapsed,
                epos: t.samples[0].value as i32,
                pressure_raw: p.raw,
                stat: t.samples[2].value as u32,
                epos_age_ms: now.wrapping_sub(t.samples[0].ms),
                pressure_age_ms: now.wrapping_sub(p.ms),
                stat_age_ms: now.wrapping_sub(t.samples[2].ms),
                target: self.target,
                controller_ticks: time.value as u32,
                controller_age_ms: if time.valid && (0..=u32::MAX as i64).contains(&time.value) {
                    now.wrapping_sub(time.ms)
                } else {
                    u32::MAX
                },
            };
            if self.len >= self.required_samples || self.len >= self.samples.len() {
                self.finish(State::Fault, "Buffer invariant failed", io);
                return;
            }
            self.samples[self.len] = record;
            self.len += 1;
            self.crc = crc32(self.crc, &record.encode());
        }
        let result = match self.config.unwrap() {
            ExperimentConfig::QuasiStatic {
                start_nm,
                step_nm,
                hold_ms,
                steps,
            } => {
                if self.step != 0 && self.hold_started.is_none() {
                    if now.wrapping_sub(self.move_started) >= MOVE_TIMEOUT_MS {
                        Err("Step move deadline")
                    } else if t
                        .xeryon_motion
                        .is_some_and(|m| m.state == 2 && m.target == Some(self.target))
                    {
                        self.hold_started = Some(now);
                        Ok(false)
                    } else {
                        Ok(false)
                    }
                } else if self.step == 0
                    || self
                        .hold_started
                        .is_some_and(|ms| now.wrapping_sub(ms) >= hold_ms)
                {
                    if self.step == steps {
                        Ok(true)
                    } else {
                        self.step += 1;
                        self.target = counts(start_nm + step_nm * i64::from(self.step)).unwrap();
                        self.hold_started = None;
                        self.move_started = now;
                        io.target(self.target, now, true).map(|_| false)
                    }
                } else {
                    Ok(false)
                }
            }
            ExperimentConfig::Sinusoidal { center_nm, .. } => {
                let waveform_ms = self.duration_ms - MOVE_TIMEOUT_MS;
                if self.settling {
                    if now.wrapping_sub(self.move_started) >= MOVE_TIMEOUT_MS {
                        Err("Final center move deadline")
                    } else if !self.center_sent {
                        self.scan.center(self.target, now, io).map(|sent| {
                            self.center_sent = sent;
                            false
                        })
                    } else {
                        Ok(t.xeryon_motion
                            .is_some_and(|m| m.state == 2 && m.target == Some(self.target))
                            && t.samples[2].value & (1 << 13) == 0)
                    }
                } else if elapsed >= waveform_ms {
                    if elapsed - waveform_ms >= TRAJECTORY_MS {
                        Err("Trajectory deadline missed")
                    } else {
                        self.target = counts(center_nm).unwrap();
                        self.settling = true;
                        self.move_started = now;
                        self.scan.cancel();
                        io.stop().map(|_| false)
                    }
                } else if elapsed >= self.next_target {
                    self.max_control_lateness_ms =
                        self.max_control_lateness_ms.max(elapsed - self.next_target);
                    if elapsed - self.next_target >= TRAJECTORY_MS {
                        Err("Trajectory deadline missed")
                    } else {
                        self.next_target = (elapsed / TRAJECTORY_MS + 1) * TRAJECTORY_MS;
                        crate::sine_control::velocity(self.config.unwrap(), elapsed, now, t)
                            .and_then(|velocity| self.scan.velocity(velocity, now, io))
                            .map(|_| {
                                self.control_updates += 1;
                                false
                            })
                    }
                } else {
                    Ok(false)
                }
            }
        };
        match result {
            Ok(true) => self.finish(State::Complete, "none", io),
            Err(e) => self.finish(State::Fault, e, io),
            _ => {}
        }
    }
    pub fn metadata(&self, out: &mut impl core::fmt::Write, id: u32) -> core::fmt::Result {
        write!(
            out,
            "{{\"type\":\"data_meta\",\"id\":{id},\"version\":1,\"run\":{},\"config\":",
            self.run
        )?;
        self.config.unwrap().write_json(out)?;
        write!(
            out,
            ",\"control_updates\":{},\"max_control_lateness_ms\":{}",
            self.control_updates, self.max_control_lateness_ms
        )?;
        if matches!(self.config, Some(ExperimentConfig::Sinusoidal { .. })) {
            write!(
                out,
                ",\"control\":\"sspd_scan_p\",\"kp_per_s\":{SCAN_KP},\"correction_fraction\":{SCAN_CORRECTION},\"feedback_max_age_ms\":{SCAN_FEEDBACK_MS},\"target_semantics\":\"reference_position\",\"settings_restore_sent\":{}",
                self.settings_restore_sent
            )?;
        }
        writeln!(
            out,
            ",\"count\":{},\"record_bytes\":{RECORD_BYTES},\"bytes\":{},\"crc\":{},\"outcome\":\"{}\",\"reason\":\"{}\",\"stop_queued\":{},\"sample_ms\":{SAMPLE_MS},\"trajectory_ms\":{TRAJECTORY_MS},\"move_timeout_ms\":{MOVE_TIMEOUT_MS},\"started_ms\":{},\"elapsed_ms\":{},\"duration_bound_ms\":{},\"required_samples\":{},\"missed_samples\":{},\"max_lateness_ms\":{}}}",
            self.len,
            self.len * RECORD_BYTES,
            self.checksum(),
            self.outcome,
            self.reason,
            self.stop_queued,
            self.started_ms,
            self.elapsed_ms,
            self.duration_ms,
            self.required_samples,
            self.missed_samples,
            self.max_lateness_ms
        )
    }
    pub fn transfer(&mut self, run: u32) -> Result<(), &'static str> {
        if !self.data_ready() || run != self.run {
            return Err("No matching retained dataset");
        }
        self.state = State::Transferring;
        self.transfer_next = 0;
        Ok(())
    }
    pub fn chunk(&mut self, run: u32, offset: usize) -> Result<&[Record], &'static str> {
        if self.state != State::Transferring
            || run != self.run
            || offset != self.transfer_next
            || offset >= self.len
        {
            return Err("Invalid dataset offset; restart GET_DATA");
        }
        let end = (offset + CHUNK_RECORDS).min(self.len);
        self.transfer_next = end;
        Ok(&self.samples[offset..end])
    }
    pub fn acknowledge(&mut self, run: u32, count: usize, crc: u32) -> Result<(), &'static str> {
        if self.state != State::Transferring
            || run != self.run
            || count != self.len
            || self.transfer_next != self.len
            || crc != self.checksum()
        {
            return Err("Dataset acknowledgement mismatch");
        }
        self.state = State::Dashboard;
        // Bytes remain intact until the next successfully validated START.
        Ok(())
    }
}

/// Quantize once, directly to the nearest encoder count (ties away from zero).
/// Configuration validation bounds the entire trajectory to the encoder range.
fn sine_target(center_nm: i64, amplitude_nm: u32, frequency_mhz: u32, elapsed: u32) -> i32 {
    let phase = (u64::from(elapsed) * u64::from(frequency_mhz)) % 1_000_000;
    let wave = sine(phase as f64 / 1_000_000.0).clamp(-1.0, 1.0);
    let position = (center_nm as f64 + f64::from(amplitude_nm) * wave) / 78.0;
    (position + if position < 0.0 { -0.5 } else { 0.5 }) as i32
}

/// Range-reduced Taylor polynomial, |error| < 5e-14 over a cycle; no libm/heap.
pub(crate) fn sine(turns: f64) -> f64 {
    let mut x = turns * core::f64::consts::TAU;
    if x > core::f64::consts::PI {
        x -= core::f64::consts::TAU;
    }
    if x > core::f64::consts::FRAC_PI_2 {
        x = core::f64::consts::PI - x;
    }
    if x < -core::f64::consts::FRAC_PI_2 {
        x = -core::f64::consts::PI - x;
    }
    let xx = x * x;
    x * (1.0
        + xx * (-1.0 / 6.0
            + xx * (1.0 / 120.0
                + xx * (-1.0 / 5040.0
                    + xx * (1.0 / 362880.0
                        + xx * (-1.0 / 39916800.0
                            + xx * (1.0 / 6227020800.0
                                + xx * (-1.0 / 1307674368000.0 + xx / 355687428096000.0))))))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sine_control::test_support::Io;
    fn feedback(now: u32, target: i32, reached: bool) -> (Telemetry, Reading) {
        let mut t = Telemetry::new();
        t.mounted = true;
        t.cdc = 2;
        t.vid = 0x04d8;
        t.pid = 0xef4a;
        t.record(b"EPOS", i64::from(target), now);
        t.record(b"DPOS", i64::from(target), now);
        t.record(b"STAT", if reached { 1344 } else { 352 }, now);
        let mut motion = xeryon_motion::Motion::new();
        motion.start(target, now);
        motion.state = if reached { 2 } else { 1 };
        t.xeryon_motion = Some(motion);
        (
            t,
            Reading {
                valid: true,
                state: 2,
                raw: 123,
                ms: now,
                ..Reading::default()
            },
        )
    }
    fn sinusoidal() -> ExperimentConfig {
        ExperimentConfig::Sinusoidal {
            center_nm: 0,
            amplitude_nm: 7800,
            frequency_mhz: 1000,
            cycles: 1,
        }
    }
    fn start(e: &mut Engine<'_>, config: ExperimentConfig, now: u32, io: &mut Io) {
        let (t, p) = feedback(now, 0, true);
        e.accept(config, &t, p, false, now).unwrap();
        e.begin(now, io);
        let (t, p) = feedback(now.wrapping_add(1), e.target, true);
        e.poll(now.wrapping_add(1), &t, p, io);
        assert_eq!(e.state, State::Armed);
        for offset in 2..1_000 {
            let tick = now.wrapping_add(offset);
            let (mut t, p) = feedback(tick, e.target, true);
            io.feedback(&mut t, tick);
            e.poll(tick, &t, p, io);
            if e.state == State::Running {
                return;
            }
            assert_eq!(e.state, State::Armed, "{}", e.reason);
        }
        panic!("start did not arm");
    }
    #[test]
    fn validation_rejects_before_motion_and_reserves_worst_case() {
        assert_eq!(sinusoidal().validate(), Ok(11000));
        assert_eq!(
            ExperimentConfig::QuasiStatic {
                start_nm: 0,
                step_nm: -78,
                hold_ms: 100,
                steps: 2
            }
            .validate(),
            Ok(20200)
        );
        for config in [
            ExperimentConfig::QuasiStatic {
                start_nm: 0,
                step_nm: 0,
                hold_ms: 1,
                steps: 1,
            },
            ExperimentConfig::QuasiStatic {
                start_nm: 0,
                step_nm: -78,
                hold_ms: u32::MAX,
                steps: u32::MAX,
            },
            ExperimentConfig::QuasiStatic {
                start_nm: 0,
                step_nm: i64::MAX,
                hold_ms: 1,
                steps: 2,
            },
            ExperimentConfig::Sinusoidal {
                center_nm: i64::MAX,
                amplitude_nm: 78,
                frequency_mhz: 100,
                cycles: 1,
            },
            ExperimentConfig::Sinusoidal {
                center_nm: 0,
                amplitude_nm: 78,
                frequency_mhz: 2001,
                cycles: 1,
            },
            ExperimentConfig::Sinusoidal {
                center_nm: 0,
                amplitude_nm: 78,
                frequency_mhz: 100,
                cycles: 21,
            },
        ] {
            assert!(config.validate().is_err(), "{config:?}");
        }
        let mut records = [Record::EMPTY; 20];
        let mut e = Engine::new(&mut records);
        let (t, p) = feedback(0, 0, true);
        assert!(e.accept(sinusoidal(), &t, p, false, 0).is_err());
        assert_eq!(e.state, State::Dashboard);
        assert_eq!(e.run, 0);
        assert_eq!(counts(-117), Ok(-2));
    }
    #[test]
    fn quasi_static_records_motion_and_holds_and_uses_confirmed_reach() {
        let mut records = [Record::EMPTY; 1000];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        start(
            &mut e,
            ExperimentConfig::QuasiStatic {
                start_nm: 0,
                step_nm: -780,
                hold_ms: 100,
                steps: 2,
            },
            0,
            &mut io,
        );
        assert_eq!(io.targets.last().unwrap().0, -10);
        for now in 3..=502 {
            let reached = now >= 102 && (e.step == 1 || now >= 302);
            let (t, p) = feedback(now, e.target, reached);
            e.poll(now, &t, p, &mut io);
            if e.data_ready() {
                break;
            }
        }
        assert_eq!(e.outcome, "completed");
        assert_eq!(io.stops, 1);
        assert_eq!(
            io.targets.iter().map(|v| v.0).collect::<Vec<_>>(),
            [0, -10, -20]
        );
        assert_eq!(e.elapsed_ms, 400);
        assert_eq!(e.len, 9);
        assert!(e.samples[..e.len].iter().any(|r| r.stat & (1 << 10) == 0));
    }
    #[test]
    fn sine_uses_absolute_time_wraps_clock_and_finishes_without_pc() {
        let mut records = [Record::EMPTY; 256];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        let original = io.settings;
        start(&mut e, sinusoidal(), u32::MAX - 500, &mut io);
        for elapsed in 1..=1200 {
            let now = e.started_ms.wrapping_add(elapsed);
            let (t, p) = feedback(now, e.target, true);
            e.poll(now, &t, p, &mut io);
        }
        assert_eq!(e.outcome, "completed", "{}", e.reason);
        assert!(e.data_ready() && e.settings_restore_sent);
        assert_eq!(e.control_updates, 100);
        assert_eq!(e.max_control_lateness_ms, 0);
        assert_eq!(io.settings, original);
        assert_eq!(e.len, 21);
        assert_eq!(io.targets.len(), 2); // only initial and final center DPOS
        assert_eq!(io.targets[1].0, 0);
        assert!(io.targets[1].2);
        assert_eq!(e.samples[20].elapsed_ms, 1000);
        assert_eq!(io.stops, 2); // stop scanning, then terminal STOP
        for direction in [-1, 1] {
            assert!(
                io.commands
                    .contains(&crate::sine_control::Command::set("SCAN", direction))
            );
        }
        for i in 0..10000 {
            let phase = i as f64 / 10000.0;
            assert!((sine(phase) - (phase * core::f64::consts::TAU).sin()).abs() < 5e-14);
        }
    }
    #[test]
    fn sine_evaluates_velocity_every_ten_ms_and_logs_position_reference() {
        let mut records = [Record::EMPTY; 256];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        io.settings[2] = 20_000;
        start(
            &mut e,
            ExperimentConfig::Sinusoidal {
                center_nm: -39,
                amplitude_nm: 780_000,
                frequency_mhz: 2000,
                cycles: 1,
            },
            0,
            &mut io,
        );
        for elapsed in 1..=700 {
            let now = e.started_ms + elapsed;
            let ideal = -0.5 + 10_000.0 * (elapsed as f64 * core::f64::consts::TAU / 500.0).sin();
            let (t, p) = feedback(
                now,
                if elapsed < 500 {
                    ideal.round() as i32
                } else {
                    -1
                },
                true,
            );
            e.poll(now, &t, p, &mut io);
            if elapsed < 500 {
                assert_eq!(e.state, State::Running, "{}", e.reason);
                assert_eq!(e.next_target, (elapsed / 10 + 1) * 10);
            }
        }
        assert_eq!(e.outcome, "completed", "{}", e.reason);
        assert!(e.data_ready());
        assert_eq!(io.targets.len(), 2);
        for record in &e.samples[..e.len] {
            let ideal =
                -0.5 + 10_000.0 * (record.elapsed_ms as f64 * core::f64::consts::TAU / 500.0).sin();
            assert!((f64::from(record.target) - ideal).abs() <= 0.50001);
        }
    }
    #[test]
    fn sine_quantization_is_precise_across_valid_travel_and_small_amplitudes() {
        for (center, amplitude) in [
            (0, 78),
            (-39, 79),
            (39, 79),
            (0, 2_617_245_618),
            (i64::from(MAX_POSITION) * 78 - 78 + 38, 78),
            (i64::from(MIN_POSITION) * 78 + 78 - 38, 78),
        ] {
            for elapsed in 0..10_000 {
                let target = sine_target(center, amplitude, 100, elapsed);
                let ideal = (center as f64
                    + f64::from(amplitude)
                        * (elapsed as f64 * core::f64::consts::TAU / 10_000.0).sin())
                    / 78.0;
                assert!((f64::from(target) - ideal).abs() <= 0.50001);
                assert!((MIN_POSITION..=MAX_POSITION).contains(&target));
            }
        }
    }
    #[test]
    fn sine_jitter_uses_current_phase_without_drift_and_stalls_fault() {
        for delay in [TRAJECTORY_MS - 1, TRAJECTORY_MS] {
            let mut records = [Record::EMPTY; 256];
            let mut e = Engine::new(&mut records);
            let mut io = Io::default();
            start(&mut e, sinusoidal(), 0, &mut io);
            for ms in 1..=2 {
                let now = e.started_ms + ms;
                let (t, p) = feedback(now, e.target, true);
                e.poll(now, &t, p, &mut io);
            }
            for slot in 1..=10 {
                let elapsed = slot * TRAJECTORY_MS + delay;
                let now = e.started_ms + elapsed;
                let (t, p) = feedback(now, e.target, true);
                e.poll(now, &t, p, &mut io);
                if delay == TRAJECTORY_MS {
                    assert_eq!(e.reason, "Trajectory deadline missed");
                    assert_eq!(io.targets.len(), 1); // no catch-up burst
                    assert_eq!(io.stops, 1);
                    break;
                }
                assert_eq!(e.state, State::Running);
                let ideal = 100.0 * (elapsed as f64 * core::f64::consts::TAU / 1000.0).sin();
                assert!((f64::from(e.target) - ideal).abs() <= 0.50001);
                e.scan.service(now + 1, &mut io).unwrap();
                e.scan.service(now + 2, &mut io).unwrap();
            }
        }
    }
    #[test]
    fn sub_resolution_speed_stops_scanning_and_final_center_is_confirmed() {
        let mut records = [Record::EMPTY; 512];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        start(
            &mut e,
            ExperimentConfig::Sinusoidal {
                center_nm: 0,
                amplitude_nm: 78,
                frequency_mhz: 100,
                cycles: 1,
            },
            0,
            &mut io,
        );
        for elapsed in 1..=10_020 {
            let now = e.started_ms + elapsed;
            let (t, p) = feedback(now, e.target, true);
            e.poll(now, &t, p, &mut io);
        }
        assert_eq!(e.outcome, "completed");
        assert_eq!(io.targets.iter().map(|v| v.0).collect::<Vec<_>>(), [0, 0]);
        assert!(io.targets.last().unwrap().2);
        assert!(
            !io.commands
                .iter()
                .any(|c| c.tag == "SCAN" && c.value != Some(0))
        );
        assert_eq!(e.len, 201); // acquisition remains 20 Hz
    }
    #[test]
    fn abort_cancels_pending_scan_direction_and_restores_before_releasing_data() {
        let mut records = [Record::EMPTY; 256];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        let original = io.settings;
        start(&mut e, sinusoidal(), 0, &mut io);
        // First speed command has been accepted, direction is still in the plan.
        e.abort(&mut io);
        assert_eq!(e.state, State::Restoring);
        assert!(e.transfer(e.run).is_err());
        e.abort(&mut io); // repeat ABORT resends STOP without restarting restoration
        for elapsed in 1..200 {
            let now = e.started_ms + elapsed;
            let (t, p) = feedback(now, 0, true);
            e.poll(now, &t, p, &mut io);
        }
        assert!(e.data_ready() && e.settings_restore_sent);
        assert_eq!(e.outcome, "aborted");
        assert_eq!(io.settings, original);
        assert_eq!(io.stops, 2);
        assert!(!io.commands.iter().any(|c| c.tag == "SCAN"));
    }
    #[test]
    fn scan_status_is_allowed_only_during_owned_sine_and_stale_encoder_stops() {
        let mut records = [Record::EMPTY; 256];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        let (mut t, p) = feedback(0, 0, true);
        t.record(b"STAT", 352 | (1 << 13), 0);
        assert!(e.accept(sinusoidal(), &t, p, false, 0).is_err());
        start(&mut e, sinusoidal(), 0, &mut io);
        for elapsed in 1..=60 {
            let now = e.started_ms + elapsed;
            let (mut t, p) = feedback(now, e.target, true);
            t.record(b"STAT", 352 | (1 << 13), now);
            t.samples[0].ms = e.started_ms;
            e.poll(now, &t, p, &mut io);
            if elapsed < 50 {
                assert_eq!(e.state, State::Running, "{}", e.reason);
            }
        }
        assert_eq!(e.reason, "Scan feedback stale");
        assert_eq!(io.stops, 1);
    }
    #[test]
    fn zero_speed_transition_can_clear_closed_loop_without_false_fault() {
        let mut records = [Record::EMPTY; 256];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        start(&mut e, sinusoidal(), 0, &mut io);
        for elapsed in 1..=260 {
            let now = e.started_ms + elapsed;
            let (mut t, p) = feedback(now, e.target, true);
            if (250..255).contains(&elapsed) {
                t.record(b"STAT", 1 << 8, now);
            }
            e.poll(now, &t, p, &mut io);
            assert_eq!(e.state, State::Running, "{}", e.reason);
        }
        // An active scan without closed-loop control is still invalid.
        let now = e.started_ms + 261;
        let (mut t, p) = feedback(now, e.target, true);
        t.record(b"STAT", (1 << 8) | (1 << 13), now);
        e.poll(now, &t, p, &mut io);
        assert_eq!(e.reason, "Xeryon closed loop or encoder invalid");
    }
    #[test]
    fn faults_abort_and_deadline_preserve_partial_data() {
        for condition in 0..8 {
            let mut records = [Record::EMPTY; 256];
            let mut e = Engine::new(&mut records);
            let mut io = Io::default();
            start(&mut e, sinusoidal(), 0, &mut io);
            let tick = e.started_ms + 1;
            let (mut t, mut p) = feedback(tick, e.target, true);
            let now = match condition {
                0 => {
                    t.mounted = false;
                    tick
                }
                1 => {
                    t.record(b"STAT", 1 << 12, tick);
                    tick
                }
                2 => {
                    p.valid = false;
                    tick
                }
                3 => {
                    p.ms = tick.wrapping_sub(300);
                    tick
                }
                4 => {
                    e.abort(&mut io);
                    tick
                }
                5 => {
                    io.fail = true;
                    tick
                }
                6 => e.started_ms + 30,
                _ => {
                    t.epoch = 1;
                    tick
                }
            };
            e.poll(now, &t, p, &mut io);
            for offset in 1..=2100 {
                let later = now + offset;
                let (mut t, p) = feedback(later, e.target, true);
                if condition == 0 {
                    t.mounted = false;
                }
                if condition == 7 {
                    t.epoch = 1;
                }
                e.poll(later, &t, p, &mut io);
                if e.data_ready() {
                    break;
                }
            }
            assert!(e.data_ready(), "{condition}: {}", e.reason);
            assert_eq!(e.len, 1);
            assert_eq!(io.stops, 1);
            if condition == 5 {
                assert_eq!(e.reason, "transmit failure");
            }
            assert_eq!(
                e.outcome,
                if condition == 4 { "aborted" } else { "faulted" }
            );
            assert!(e.accept(sinusoidal(), &t, p, false, now).is_err());
        }
    }
    #[test]
    fn transfer_never_releases_on_wrong_or_premature_ack_and_can_restart() {
        let mut records = [Record::EMPTY; 256];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        start(&mut e, sinusoidal(), 0, &mut io);
        e.abort(&mut io);
        for ms in 1..=200 {
            let now = e.started_ms + ms;
            let (t, p) = feedback(now, e.target, true);
            e.poll(now, &t, p, &mut io);
        }
        assert!(e.data_ready());
        let run = e.run;
        let crc = e.checksum();
        assert!(e.acknowledge(run, e.len, crc).is_err());
        e.transfer(run).unwrap();
        assert!(e.acknowledge(run, e.len, crc).is_err());
        assert!(e.chunk(run, 1).is_err());
        assert_eq!(e.chunk(run, 0).unwrap().len(), 1);
        e.transfer(run).unwrap(); // reconnect/download from zero, same payload
        assert_eq!(e.chunk(run, 0).unwrap()[0].pressure_raw, 123);
        assert!(e.acknowledge(run + 1, 1, crc).is_err());
        assert!(e.acknowledge(run, 1, crc ^ 1).is_err());
        e.acknowledge(run, 1, crc).unwrap();
        assert_eq!(e.state, State::Dashboard);
        assert_eq!(e.len, 1);
        let (t, p) = feedback(5, 0, true);
        assert!(
            e.accept(
                ExperimentConfig::Sinusoidal {
                    center_nm: 0,
                    amplitude_nm: 0,
                    frequency_mhz: 1000,
                    cycles: 1
                },
                &t,
                p,
                false,
                5
            )
            .is_err()
        );
        assert_eq!(e.len, 1);
        assert_eq!(e.checksum(), crc);
        assert_eq!(!crc32(!0, b"123456789"), 0xcbf4_3926);
        let mut meta = String::new();
        e.metadata(&mut meta, 1).unwrap();
        assert!(meta.contains("\"outcome\":\"aborted\""));
    }
    #[test]
    fn acquisition_bound_is_enforced_and_late_samples_are_not_fabricated() {
        let mut records = [Record::EMPTY; CAPACITY];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        let config = ExperimentConfig::QuasiStatic {
            start_nm: 0,
            step_nm: 78,
            hold_ms: 194_750,
            steps: 1,
        };
        start(&mut e, config, 0, &mut io);
        assert_eq!(e.required_samples, CAPACITY);
        let (t, p) = feedback(202, 1, true);
        e.poll(202, &t, p, &mut io);
        assert_eq!(e.len, 2);
        assert_eq!(e.missed_samples, 3);
        for elapsed in (250..=MAX_DURATION_MS).step_by(50) {
            let now = e.started_ms + elapsed;
            let (t, p) = feedback(now, e.target, true);
            e.poll(now, &t, p, &mut io);
        }
        assert!(e.len <= CAPACITY);
        assert_eq!(e.outcome, "completed");
    }
    #[test]
    fn manual_heartbeat_is_preserved_but_autonomous_policy_ignores_it() {
        let (t, _) = feedback(6000, 0, false);
        let mut motion = xeryon_motion::Motion::new();
        motion.start(0, 0);
        assert!(!motion.poll_policy(&t, 6000, true));
        assert!(motion.poll(&t, 6000));
        assert_eq!(
            motion.interruption_reason,
            Some("desktop heartbeat missing")
        );
    }
}
