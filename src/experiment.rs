//! Foreground-only autonomous controller. Hardware IO stays behind a Rust trait.
use crate::{experiment_data::*, pressure::Reading, protocol::Telemetry, xeryon_motion};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Dashboard,
    Validate,
    MoveToStart,
    Armed,
    Running,
    Complete,
    Aborted,
    Fault,
    DataReady,
    Transferring,
}
pub trait Actuator {
    /// Set and request SSPD in separate, ordered USB transfers.
    fn speed(&mut self, speed_um_s: u32) -> Result<(), &'static str>;
    fn query_motor_timeout(&mut self) -> Result<(), &'static str>;
    fn tx_idle(&self) -> bool;
    fn target(&mut self, counts: i32, now: u32, track_completion: bool)
    -> Result<(), &'static str>;
    fn stop(&mut self) -> Result<(), &'static str>;
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
    move_timeout: u32,
    speed_pending: bool,
    speed_updates: u32,
    timeout_pending: bool,
    timeout_updates: u32,
    next_sample: u32,
    next_target: u32,
    step: u32,
    hold_started: Option<u32>,
    target: i32,
    epoch: u32,
    transfer_next: usize,
    settling: bool,
    fault_stat: Option<u32>,
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
            move_timeout: MOVE_TIMEOUT_MS,
            speed_pending: false,
            speed_updates: 0,
            timeout_pending: false,
            timeout_updates: 0,
            next_sample: 0,
            next_target: 0,
            step: 0,
            hold_started: None,
            target: 0,
            epoch: 0,
            transfer_next: 0,
            settling: false,
            fault_stat: None,
        }
    }
    pub fn active(&self) -> bool {
        matches!(
            self.state,
            State::MoveToStart | State::Armed | State::Running
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
    fn healthy(
        t: &Telemetry,
        p: Reading,
        now: u32,
        require_closed_loop: bool,
    ) -> Result<(), &'static str> {
        if !xeryon_motion::ready(t) {
            return Err("USB2 disconnected");
        }
        if !xeryon_motion::fresh(t, 0, now) || !xeryon_motion::fresh(t, 2, now) {
            return Err("Xeryon feedback stale");
        }
        if let Some(reason) = xeryon_motion::controller_fault(t.samples[2].value) {
            return Err(reason);
        }
        if t.samples[2].value & ((1 << 1) | (1 << 14) | (1 << 15) | (1 << 9) | (1 << 13)) != 0 {
            return Err("Xeryon limit or incompatible motion");
        }
        // STAT bit 8 means the index reference is known, not encoder health.
        // Experiments use the current EPOS coordinate system, just like manual
        // DPOS moves. Encoder errors (bit 12) are already rejected by FAULTS.
        if require_closed_loop && t.samples[2].value & (1 << 6) == 0 {
            return Err("Xeryon closed loop inactive");
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
            // DPOS enters closed-loop positioning; a stopped controller need
            // not already be in that mode before the initial target is sent.
            Self::healthy(t, p, now, false)?;
            if plunger_running
                || t.xeryon_motion.is_some_and(|m| m.state == 1)
                || (t.samples[2].value & (1 << 5) != 0 && t.samples[2].value & (1 << 10) == 0)
            {
                return Err("Stop manual motion before START");
            }
            let target = config.start()?;
            let distance = (i64::from(target) - t.samples[0].value).unsigned_abs() * 78;
            Ok((duration, required, target, config.move_timeout(distance)?))
        })();
        let (duration, required, target, move_timeout) = match validation {
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
        self.move_timeout = move_timeout;
        self.speed_pending = config.speed().is_some();
        self.speed_updates = t.speed_updates;
        self.timeout_pending = false;
        self.target = target;
        self.epoch = t.epoch;
        self.step = 0;
        self.settling = false;
        self.fault_stat = None;
        self.hold_started = None;
        self.state = State::MoveToStart;
        // First target is sent by begin(), only after all validation succeeds.
        Ok(())
    }
    pub fn begin(&mut self, now: u32, io: &mut impl Actuator) {
        let result = match self.config.and_then(ExperimentConfig::speed) {
            Some(speed) => io.speed(speed),
            None => io.target(self.target, now, true),
        };
        if let Err(e) = result {
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
        self.state = State::DataReady;
    }
    pub fn abort(&mut self, io: &mut impl Actuator) {
        if self.active() {
            self.finish(State::Aborted, "PC ABORT", io);
        }
    }
    pub fn poll(&mut self, now: u32, t: &Telemetry, p: Reading, io: &mut impl Actuator) {
        if !self.active() {
            return;
        }
        if self.state == State::Running {
            self.elapsed_ms = now.wrapping_sub(self.started_ms);
        }
        // Initial positioning can still see pre-DPOS status. Its existing
        // deadline and echo/reached checks bound this transition; require
        // closed loop once armed, before starting acquisition or the waveform.
        let fault = Self::healthy(t, p, now, self.state != State::MoveToStart)
            .err()
            .or_else(|| (t.epoch != self.epoch).then_some("Xeryon connection changed"))
            .or_else(|| {
                t.xeryon_motion
                    // During setup this tracker still describes the previous
                    // manual move. The first DPOS starts a new tracked move.
                    .filter(|m| m.state == 4 && !self.speed_pending && !self.timeout_pending)
                    .map(|m| m.interruption_reason.unwrap_or("Xeryon motion failed"))
            });
        if let Some(e) = fault {
            // Preserve the triggering word before STOP and later publications
            // replace the live Dashboard status, even between logging ticks.
            if t.samples[2].valid && t.samples[2].value & xeryon_motion::FAULTS != 0 {
                self.fault_stat = Some(t.samples[2].value as u32);
            }
            self.finish(State::Fault, e, io);
            return;
        }
        if self.state == State::MoveToStart {
            if self.speed_pending {
                if now.wrapping_sub(self.move_started) >= 3000 {
                    self.finish(State::Fault, "Xeryon speed readback timeout", io);
                } else if io.tx_idle() && t.speed.valid && t.speed_updates != self.speed_updates {
                    if Some(t.speed.value) != self.config.unwrap().speed().map(i64::from) {
                        self.finish(State::Fault, "Xeryon speed readback mismatch", io);
                    } else {
                        self.speed_pending = false;
                        self.timeout_pending = true;
                        self.timeout_updates = t.motor_timeout_updates;
                        self.move_started = now;
                        if let Err(e) = io.query_motor_timeout() {
                            self.finish(State::Fault, e, io);
                        }
                    }
                }
                return;
            }
            if self.timeout_pending {
                if now.wrapping_sub(self.move_started) >= 3000 {
                    self.finish(State::Fault, "Xeryon TOU2 readback timeout", io);
                } else if io.tx_idle()
                    && t.motor_timeout.valid
                    && t.motor_timeout_updates != self.timeout_updates
                {
                    let seconds = t.motor_timeout.value;
                    let longest = self
                        .move_timeout
                        .max(self.config.unwrap().step_timeout().unwrap());
                    // TOU2 is queried after SSPD, so its reply also lets any
                    // trailing SSPD readback arrive before the first target.
                    if Some(t.speed.value) != self.config.unwrap().speed().map(i64::from) {
                        self.finish(State::Fault, "Xeryon speed readback mismatch", io);
                    } else if !(0..=65535).contains(&seconds) {
                        self.finish(State::Fault, "Invalid Xeryon TOU2 readback", io);
                    } else if seconds != 0 && i64::from(longest) >= seconds * 1000 {
                        self.finish(
                            State::Fault,
                            "Move budget reaches TOU2; increase speed or reduce travel",
                            io,
                        );
                    } else {
                        self.timeout_pending = false;
                        self.move_started = now;
                        if let Err(e) = io.target(self.target, now, true) {
                            self.finish(State::Fault, e, io);
                        }
                    }
                }
                return;
            }
            if now.wrapping_sub(self.move_started) >= self.move_timeout {
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
            self.started_ms = now;
            self.next_sample = 0;
            self.next_target = 0;
            self.state = State::Running;
            #[cfg(not(test))]
            crate::fault::mark(4);
        }
        let elapsed = now.wrapping_sub(self.started_ms);
        self.elapsed_ms = elapsed;
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
                ..
            } => {
                if self.step != 0 && self.hold_started.is_none() {
                    if now.wrapping_sub(self.move_started) >= self.move_timeout {
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
                        self.move_timeout = self.config.unwrap().step_timeout().unwrap();
                        io.target(self.target, now, true).map(|_| false)
                    }
                } else {
                    Ok(false)
                }
            }
            ExperimentConfig::Sinusoidal {
                center_nm,
                amplitude_nm,
                frequency_mhz,
                ..
            } => {
                let waveform_ms = self.duration_ms - MOVE_TIMEOUT_MS;
                if self.settling {
                    if now.wrapping_sub(self.move_started) >= MOVE_TIMEOUT_MS {
                        Err("Final center move deadline")
                    } else {
                        Ok(t.xeryon_motion
                            .is_some_and(|m| m.state == 2 && m.target == Some(self.target)))
                    }
                } else if elapsed >= waveform_ms {
                    if elapsed - waveform_ms >= TRAJECTORY_MS {
                        Err("Trajectory deadline missed")
                    } else {
                        self.target = counts(center_nm).unwrap();
                        self.settling = true;
                        self.move_started = now;
                        io.target(self.target, now, true).map(|_| false)
                    }
                } else if elapsed >= self.next_target {
                    if elapsed - self.next_target >= TRAJECTORY_MS {
                        Err("Trajectory deadline missed")
                    } else {
                        self.next_target = (elapsed / TRAJECTORY_MS + 1) * TRAJECTORY_MS;
                        // Phase comes from the run clock; never integrate or replay missed ticks.
                        self.target = sine_target(center_nm, amplitude_nm, frequency_mhz, elapsed);
                        io.target(self.target, now, false).map(|_| false)
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
        if let Some(stat) = self.fault_stat {
            write!(out, ",\"fault_stat\":{stat}")?;
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
    #[derive(Default)]
    struct Io {
        targets: Vec<(i32, u32, bool)>,
        stops: usize,
        fail: bool,
        busy: bool,
        speeds: Vec<u32>,
    }
    impl Actuator for Io {
        fn query_motor_timeout(&mut self) -> Result<(), &'static str> {
            if self.fail {
                Err("timeout query transmit failure")
            } else {
                Ok(())
            }
        }
        fn speed(&mut self, speed: u32) -> Result<(), &'static str> {
            if self.fail {
                return Err("speed transmit failure");
            }
            self.speeds.push(speed);
            Ok(())
        }
        fn tx_idle(&self) -> bool {
            !self.busy
        }
        fn target(&mut self, target: i32, now: u32, track: bool) -> Result<(), &'static str> {
            if self.fail {
                return Err("transmit failure");
            }
            self.targets.push((target, now, track));
            Ok(())
        }
        fn stop(&mut self) -> Result<(), &'static str> {
            self.stops += 1;
            if self.fail {
                Err("stop failure")
            } else {
                Ok(())
            }
        }
    }
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
        if let Some(speed) = config.speed() {
            let (mut t, p) = feedback(now, 0, true);
            t.record(b"SSPD", i64::from(speed), now);
            e.poll(now, &t, p, io);
            assert!(!e.speed_pending);
            t.record(b"TOU2", 60, now);
            e.poll(now, &t, p, io);
            assert!(!e.timeout_pending);
        }
        let (t, p) = feedback(now.wrapping_add(1), e.target, true);
        e.poll(now.wrapping_add(1), &t, p, io);
        assert_eq!(e.state, State::Armed);
        let (t, p) = feedback(now.wrapping_add(2), e.target, true);
        e.poll(now.wrapping_add(2), &t, p, io);
    }
    #[test]
    fn validation_rejects_before_motion_and_reserves_worst_case() {
        assert_eq!(sinusoidal().validate(), Ok(11000));
        assert_eq!(
            ExperimentConfig::QuasiStatic {
                speed_um_s: 10,
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
                speed_um_s: 10,
                start_nm: 0,
                step_nm: 0,
                hold_ms: 1,
                steps: 1,
            },
            ExperimentConfig::QuasiStatic {
                speed_um_s: 10,
                start_nm: 0,
                step_nm: -78,
                hold_ms: u32::MAX,
                steps: u32::MAX,
            },
            ExperimentConfig::QuasiStatic {
                speed_um_s: 10,
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

        let mut records = [Record::EMPTY; 256];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        let (mut t, p) = feedback(0, 0, true);
        // Reported hardware status: closed loop + reached, index not known.
        t.record(b"STAT", 1089 | (1 << 12), 0);
        assert_eq!(
            e.accept(sinusoidal(), &t, p, false, 0),
            Err("Xeryon encoder error")
        );
        t.record(b"STAT", 1089, 0);
        e.accept(sinusoidal(), &t, p, false, 0).unwrap();
        e.begin(0, &mut io);
        e.poll(1, &t, p, &mut io);
        e.poll(2, &t, p, &mut io);
        assert_eq!(e.state, State::Running);
        e.abort(&mut io);

        // STOP can leave both closed-loop and reached clear. DPOS re-enters
        // positioning; pre-command feedback must not immediately fault it.
        let mut e = Engine::new(&mut records);
        t.record(b"STAT", 1 | (1 << 5), 0);
        assert_eq!(
            e.accept(sinusoidal(), &t, p, false, 0),
            Err("Stop manual motion before START")
        );
        t.record(b"STAT", 1, 0);
        e.accept(sinusoidal(), &t, p, false, 0).unwrap();
        e.begin(0, &mut io);
        t.xeryon_motion.as_mut().unwrap().state = 1;
        e.poll(1, &t, p, &mut io);
        assert_eq!(e.state, State::MoveToStart);
        t.record(b"STAT", 1089, 2);
        t.xeryon_motion.as_mut().unwrap().state = 2;
        e.poll(2, &t, p, &mut io);
        e.poll(3, &t, p, &mut io);
        assert_eq!(e.state, State::Running);
        t.record(b"STAT", 1, 4);
        e.poll(4, &t, p, &mut io);
        assert_eq!(e.reason, "Xeryon closed loop inactive");
    }
    #[test]
    fn quasi_static_setup_requires_fresh_speed_and_timeout_before_any_motion() {
        let config = ExperimentConfig::QuasiStatic {
            start_nm: 780,
            step_nm: 780,
            hold_ms: 100,
            steps: 1,
            speed_um_s: 10,
        };
        for failure in [
            "none",
            "timeout_disabled",
            "stale",
            "mismatch",
            "late_mismatch",
            "speed_tx",
            "timeout_tx",
            "timeout_missing",
            "timeout_invalid",
            "timeout_short",
            "abort",
        ] {
            let mut records = [Record::EMPTY; CAPACITY];
            let mut e = Engine::new(&mut records);
            let mut io = Io::default();
            let (mut t, p) = feedback(0, 0, true);
            t.record(b"SSPD", 10, 0); // Cached value must not satisfy the new run.
            t.record(b"TOU2", 60, 0);
            // A healthy controller can retain a failed manual tracking result.
            t.xeryon_motion.as_mut().unwrap().state = 4;
            e.accept(config, &t, p, false, 0).unwrap();
            io.fail = failure == "speed_tx";
            e.begin(0, &mut io);
            io.fail = false;
            e.poll(0, &t, p, &mut io);
            assert!(io.targets.is_empty());
            if failure == "abort" {
                e.abort(&mut io);
            }
            if failure != "stale" {
                // Same millisecond is valid when the readback generation advances.
                t.record(b"SSPD", if failure == "mismatch" { 11 } else { 10 }, 0);
            }
            io.busy = true;
            e.poll(0, &t, p, &mut io);
            assert!(io.targets.is_empty()); // Wait for the query transfer too.
            io.busy = false;
            io.fail = failure == "timeout_tx";
            e.poll(0, &t, p, &mut io);
            io.fail = false;
            assert!(io.targets.is_empty());
            e.poll(0, &t, p, &mut io);
            assert!(io.targets.is_empty()); // Cached TOU2 is insufficient.
            if failure == "late_mismatch" {
                t.record(b"SSPD", 11, 0);
            }
            if failure != "timeout_missing" {
                t.record(
                    b"TOU2",
                    match failure {
                        "timeout_invalid" => -1,
                        "timeout_short" => 10,
                        "timeout_disabled" => 0,
                        _ => 60,
                    },
                    0,
                );
            }
            e.poll(0, &t, p, &mut io);
            if matches!(failure, "none" | "timeout_disabled") {
                assert_eq!(io.speeds, [10]);
                assert_eq!(io.targets, [(10, 0, true)]);
                assert_eq!(e.state, State::MoveToStart);
                let mut metadata = String::new();
                e.metadata(&mut metadata, 1).unwrap();
                assert!(metadata.contains("\"speed_um_s\":10"));
            } else {
                let (mut t, p) = feedback(3000, 0, true);
                t.record(b"SSPD", 10, 3000);
                e.poll(3000, &t, p, &mut io);
                assert!(io.targets.is_empty(), "{failure}");
                assert!(e.data_ready(), "{failure}");
                assert_eq!(e.len, 0);
                assert_eq!(io.stops, 1);
                match failure {
                    "stale" => assert_eq!(e.reason, "Xeryon speed readback timeout"),
                    "mismatch" => assert_eq!(e.reason, "Xeryon speed readback mismatch"),
                    "timeout_missing" => assert_eq!(e.reason, "Xeryon TOU2 readback timeout"),
                    "abort" => assert_eq!(e.outcome, "aborted"),
                    _ => assert_eq!(e.outcome, "faulted"),
                }
            }
        }
    }
    #[test]
    fn slow_moves_use_speed_budget_for_start_steps_and_storage_across_clock_wrap() {
        let config = ExperimentConfig::QuasiStatic {
            start_nm: 39_000,
            step_nm: -39_000,
            hold_ms: 50,
            steps: 1,
            speed_um_s: 1,
        };
        assert_eq!(config.step_timeout(), Ok(79_156));
        assert_eq!(config.validate(), Ok(79_206));
        let mut records = [Record::EMPTY; CAPACITY];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        let begin = u32::MAX - 5;
        let (mut t, p) = feedback(begin, 0, true);
        e.accept(config, &t, p, false, begin).unwrap();
        e.begin(begin, &mut io);
        t.record(b"SSPD", 1, begin);
        e.poll(begin, &t, p, &mut io);
        t.record(b"TOU2", 120, begin);
        e.poll(begin, &t, p, &mut io);
        let later = begin.wrapping_add(60_001);
        let (t, p) = feedback(later, e.target, false);
        let mut motion = xeryon_motion::Motion::new();
        motion.start(e.target, begin);
        assert!(!motion.poll_policy(&t, later, true));
        e.poll(later, &t, p, &mut io);
        assert_eq!(e.state, State::MoveToStart);
        let now = begin.wrapping_add(70_000);
        let (t, p) = feedback(now, e.target, true);
        e.poll(now, &t, p, &mut io);
        e.poll(now, &t, p, &mut io);
        assert_eq!(e.state, State::Running);
        assert_eq!(io.targets.last().unwrap().0, 0);
        assert_eq!(e.required_samples, 1585);
        let later = now.wrapping_add(60_001);
        let (t, p) = feedback(later, e.target, false);
        e.poll(later, &t, p, &mut io);
        assert_eq!(e.state, State::Running);
        let deadline = now.wrapping_add(79_156);
        let (t, p) = feedback(deadline, e.target, false);
        e.poll(deadline, &t, p, &mut io);
        assert_eq!(e.reason, "Step move deadline");
        assert_eq!(io.stops, 1);
        assert!(config.move_timeout(200_000).is_err());
        assert!(
            ExperimentConfig::QuasiStatic {
                start_nm: 39_000,
                step_nm: -39_000,
                hold_ms: 50,
                steps: 3,
                speed_um_s: 1,
            }
            .validate()
            .is_err()
        );
        for speed_um_s in [0, MAX_SPEED_UM_S + 1] {
            assert!(
                ExperimentConfig::QuasiStatic {
                    start_nm: 0,
                    step_nm: 78,
                    hold_ms: 50,
                    steps: 1,
                    speed_um_s,
                }
                .validate()
                .is_err()
            );
        }
    }
    #[test]
    fn quasi_static_records_motion_and_holds_and_uses_confirmed_reach() {
        let mut records = [Record::EMPTY; 1000];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        start(
            &mut e,
            ExperimentConfig::QuasiStatic {
                speed_um_s: 10,
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
        let begin = u32::MAX - 500;
        start(&mut e, sinusoidal(), begin, &mut io);
        for elapsed in 1..=1001 {
            let now = e.started_ms.wrapping_add(elapsed);
            let (t, p) = feedback(now, e.target, true);
            e.poll(now, &t, p, &mut io);
        }
        assert_eq!(e.outcome, "completed");
        assert_eq!(e.len, 21);
        assert_eq!(io.targets.len(), 102); // initial, 100 waveform targets, final center
        for (tick, &(target, now, track)) in io.targets[1..101].iter().enumerate() {
            let elapsed = tick as u32 * 10;
            assert_eq!(now.wrapping_sub(e.started_ms), elapsed);
            let expected = (100.0 * (f64::from(elapsed) * core::f64::consts::TAU / 1000.0).sin())
                .round() as i32;
            assert_eq!(target, expected);
            assert!(!track);
        }
        assert_eq!(io.targets.last().unwrap().0, 0);
        assert!(io.targets.last().unwrap().2);
        assert_eq!(e.samples[20].elapsed_ms, 1000);
        assert_eq!(io.stops, 1);
        for i in 0..10000 {
            let phase = i as f64 / 10000.0;
            assert!((sine(phase) - (phase * core::f64::consts::TAU).sin()).abs() < 7e-10);
        }
    }
    #[test]
    fn faults_abort_and_deadline_preserve_partial_data() {
        for condition in 0..8 {
            let mut records = [Record::EMPTY; 256];
            let mut e = Engine::new(&mut records);
            let mut io = Io::default();
            start(&mut e, sinusoidal(), 0, &mut io);
            let (mut t, mut p) = feedback(12, e.target, true);
            let now = match condition {
                0 => {
                    t.mounted = false;
                    12
                }
                1 => {
                    t.record(b"STAT", 1 << 12, 12);
                    12
                }
                2 => {
                    p.valid = false;
                    12
                }
                3 => {
                    p.ms = u32::MAX - 300;
                    12
                }
                4 => {
                    e.abort(&mut io);
                    12
                }
                5 => {
                    io.fail = true;
                    12
                }
                6 => 42,
                _ => {
                    t.epoch = 1;
                    12
                }
            };
            e.poll(now, &t, p, &mut io);
            assert!(e.data_ready(), "{condition}");
            assert_eq!(e.len, 1);
            assert_eq!(io.stops, 1);
            if condition == 1 {
                assert_eq!(e.reason, "Xeryon encoder error");
                // A later healthy status must not erase the fault evidence.
                t.record(b"STAT", 1089, now + 1);
                e.poll(now + 1, &t, p, &mut io);
                let mut meta = String::new();
                e.metadata(&mut meta, 1).unwrap();
                assert!(meta.contains("\"fault_stat\":4096"));
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
            speed_um_s: 10,
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
