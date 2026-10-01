// Foreground-only autonomous controller. Hardware IO stays behind a Rust trait.
use crate::{experiment_data::*, pressure::Reading, protocol::Telemetry, xeryon_motion};

#[path = "envelope.rs"]
pub mod envelope;
#[path = "final_return.rs"]
mod final_return;
#[path = "reversal.rs"]
pub mod reversal;
#[path = "scheduling.rs"]
pub mod scheduling;
#[path = "waveform_tx.rs"]
mod waveform_tx;

const MODE_FEEDBACK_GRACE_MS: u32 = 100;

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
    /// Preparation completes before any experiment motion command.
    fn prepare_feedback(&mut self, _now: u32) -> Result<bool, &'static str> {
        Ok(true)
    }
    fn start_timing(&mut self) {}
    fn end_timing(&mut self) {}
    fn tx_progress(&self) -> reversal::TxProgress {
        reversal::TxProgress::default()
    }
    /// One or two ordered, separately flushed SSPD/SCAN lines; no readback.
    fn scan(&mut self, commands: &[xeryon_motion::ScanCommand]) -> Result<(), &'static str>;
    /// Set and request SSPD in separate, ordered USB transfers.
    fn speed(&mut self, speed_um_s: u32) -> Result<(), &'static str>;
    fn query_motor_timeout(&mut self) -> Result<(), &'static str>;
    fn tx_idle(&self) -> bool;
    fn tx_failed(&self) -> bool {
        false
    }
    fn target(&mut self, counts: i32, now: u32, track_completion: bool)
    -> Result<(), &'static str>;
    fn stop(&mut self) -> Result<(), &'static str>;
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CenterMove {
    None,
    Stopping,
    SettingSpeed,
    Positioning,
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
    center_move: CenterMove,
    final_return: final_return::Diagnostics,
    scan: xeryon_motion::ScanState,
    waveform_tx: waveform_tx::Transport,
    pub scheduling: scheduling::Diagnostics,
    pub reversals: reversal::Diagnostics<'a>,
    pub envelope: envelope::Envelope,
    motion_changed_ms: u32,
    fault_stat: Option<u32>,
    feedback_pending: bool,
    epos_arrivals: crate::protocol::ArrivalStats,
    pressure_arrivals: crate::protocol::ArrivalStats,
    transport: crate::protocol::TransportStats,
    last_pressure: u32,
    logged_epos: Option<u32>,
    logged_pressure: Option<u32>,
    repeated_epos: u32,
    repeated_pressure: u32,
    max_epos_age: u32,
    max_pressure_age: u32,
    malformed_start: u32,
    malformed: u32,
    feedback_before: [Option<i64>; 2],
    feedback_verified: bool,
    epos_start_generation: u32,
    epos_logged_updates: u32,
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
            center_move: CenterMove::None,
            final_return: Default::default(),
            scan: xeryon_motion::ScanState::default(),
            waveform_tx: waveform_tx::Transport::default(),
            scheduling: scheduling::Diagnostics::default(),
            reversals: reversal::Diagnostics::new(&mut []),
            envelope: envelope::Envelope::configured(),
            motion_changed_ms: 0,
            fault_stat: None,
            feedback_pending: false,
            epos_arrivals: Default::default(),
            pressure_arrivals: Default::default(),
            transport: Default::default(),
            last_pressure: 0,
            logged_epos: None,
            logged_pressure: None,
            repeated_epos: 0,
            repeated_pressure: 0,
            max_epos_age: 0,
            max_pressure_age: 0,
            malformed_start: 0,
            malformed: 0,
            feedback_before: [None; 2],
            feedback_verified: false,
            epos_start_generation: 0,
            epos_logged_updates: 0,
        }
    }
    pub fn with_reversals(mut self, records: &'a mut [reversal::Reversal]) -> Self {
        self.reversals = reversal::Diagnostics::new(records);
        self
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
        allow_scan: bool,
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
        let incompatible =
            (1 << 1) | (1 << 14) | (1 << 15) | (1 << 9) | if allow_scan { 0 } else { 1 << 13 };
        if t.samples[2].value & incompatible != 0 {
            return Err("Xeryon limit or incompatible motion");
        }
        // STAT bit 8 means the index reference is known, not encoder health.
        // Experiments use the current EPOS coordinate system, just like manual
        // DPOS moves. controller_fault already rejects encoder errors (bit 12).
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
            Self::healthy(t, p, now, false, false)?;
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
        self.reversals.reset();
        self.envelope.reset(t.epos_generation);
        let mode = self.scheduling.mode;
        self.scheduling = scheduling::Diagnostics::default();
        self.scheduling.mode = mode;
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
        self.center_move = CenterMove::None;
        self.final_return = Default::default();
        self.scan = xeryon_motion::ScanState::default();
        self.waveform_tx = waveform_tx::Transport::default();
        self.fault_stat = None;
        self.feedback_pending = true;
        self.epos_arrivals = Default::default();
        self.pressure_arrivals = Default::default();
        self.transport = Default::default();
        self.logged_epos = None;
        self.logged_pressure = None;
        self.repeated_epos = 0;
        self.repeated_pressure = 0;
        self.max_epos_age = 0;
        self.max_pressure_age = 0;
        self.malformed = 0;
        self.feedback_before = [None; 2];
        self.feedback_verified = false;
        self.epos_logged_updates = 0;
        self.hold_started = None;
        self.state = State::MoveToStart;
        // First target is sent by begin(), only after all validation succeeds.
        Ok(())
    }
    pub fn begin(&mut self, now: u32, io: &mut impl Actuator) {
        match io.prepare_feedback(now) {
            Ok(false) => return,
            Err(e) => {
                self.finish(State::Fault, e, io);
                return;
            }
            Ok(true) => {
                self.feedback_pending = false;
                self.feedback_verified = true;
            }
        }
        self.move_started = now;
        let result = match self.config.and_then(ExperimentConfig::speed) {
            Some(speed) => io.speed(speed),
            None => io.target(self.target, now, true),
        };
        if let Err(e) = result {
            self.finish(State::Fault, e, io);
        }
    }
    fn finish(&mut self, state: State, reason: &'static str, io: &mut impl Actuator) {
        if self.final_return.boundary.is_some() {
            self.final_return.finished = Some(self.started_ms.wrapping_add(self.elapsed_ms));
        }
        self.waveform_tx
            .close_busy(self.elapsed_ms, &mut self.scheduling);
        if self.state == State::Running && self.center_move == CenterMove::None {
            if let Some(ExperimentConfig::Sinusoidal {
                amplitude_nm,
                frequency_mhz,
                ..
            }) = self.config
            {
                self.scheduling.observe(
                    amplitude_nm,
                    frequency_mhz,
                    self.elapsed_ms.min(self.duration_ms - MOVE_TIMEOUT_MS),
                );
            }
        }
        if self.state == State::Running {
            self.scheduling.refresh(io.tx_progress());
        }
        self.reversals.refresh(io.tx_progress(), self.started_ms);
        self.waveform_tx = waveform_tx::Transport::default();
        io.end_timing();
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
        self.final_return.observe(t, io.tx_progress());
        if self.final_return.boundary.is_some() {
            self.elapsed_ms = now.wrapping_sub(self.started_ms);
            self.final_return.expired = now.wrapping_sub(self.move_started) >= MOVE_TIMEOUT_MS;
        }
        if io.tx_failed() {
            if self.state == State::Running {
                self.elapsed_ms = now.wrapping_sub(self.started_ms);
            }
            self.scheduling.usb_faults += 1;
            self.finish(State::Fault, "Xeryon USB transfer failed", io);
            return;
        }
        if self.state == State::Running {
            self.elapsed_ms = now.wrapping_sub(self.started_ms);
            self.scheduling.refresh(io.tx_progress());
            self.reversals.refresh(io.tx_progress(), self.started_ms);
            if self.center_move == CenterMove::None {
                self.reversals.observe(
                    self.config.unwrap(),
                    self.elapsed_ms,
                    now,
                    t,
                    !io.tx_idle(),
                );
            }
            if self.center_move == CenterMove::None {
                if let Some(ExperimentConfig::Sinusoidal {
                    amplitude_nm,
                    frequency_mhz,
                    ..
                }) = self.config
                    && self.elapsed_ms >= self.duration_ms - MOVE_TIMEOUT_MS
                {
                    self.scheduling.observe(
                        amplitude_nm,
                        frequency_mhz,
                        self.elapsed_ms.min(self.duration_ms - MOVE_TIMEOUT_MS),
                    );
                }
            }
            self.epos_arrivals = t.epos_arrivals;
            self.transport = t.transport;
            self.malformed = t.malformed.wrapping_sub(self.malformed_start);
            if p.valid && p.samples != self.last_pressure {
                self.pressure_arrivals.observe(p.ms);
                self.last_pressure = p.samples;
            }
            self.max_epos_age = self.max_epos_age.max(now.wrapping_sub(t.samples[0].ms));
            self.max_pressure_age = self.max_pressure_age.max(now.wrapping_sub(p.ms));
        }
        // Initial positioning can still see pre-DPOS status. Its existing
        // deadline and echo/reached checks bound this transition; require
        // closed loop once armed, before starting acquisition or the waveform.
        let sine_running = self.state == State::Running
            && matches!(self.config, Some(ExperimentConfig::Sinusoidal { .. }));
        // STOP may clear closed-loop status. Allow the deliberately
        // stopped phases and a bounded 100 ms mode-change feedback grace after
        // restarting SCAN or DPOS. All other health checks stay in force.
        let stopped_transition = sine_running
            && (matches!(
                self.center_move,
                CenterMove::Stopping | CenterMove::SettingSpeed
            ) || (self.center_move == CenterMove::None && self.scan.direction == Some(0))
                || now.wrapping_sub(self.motion_changed_ms) < MODE_FEEDBACK_GRACE_MS);
        let fault = Self::healthy(
            t,
            p,
            now,
            self.state != State::MoveToStart && !stopped_transition,
            sine_running
                && (self.center_move != CenterMove::Positioning
                    || now.wrapping_sub(self.motion_changed_ms) < MODE_FEEDBACK_GRACE_MS),
        )
        .err()
        .or_else(|| (t.epoch != self.epoch).then_some("Xeryon connection changed"))
        .or_else(|| {
            t.xeryon_motion
                // During setup this tracker still describes the previous
                // manual move. The first DPOS starts a new tracked move.
                .filter(|m| {
                    m.state == 4
                        && !self.feedback_pending
                        && !self.speed_pending
                        && !self.timeout_pending
                })
                .map(|m| m.interruption_reason.unwrap_or("Xeryon motion failed"))
        });
        if let Some(e) = fault {
            // Preserve the triggering word before STOP and later publications
            // replace the live Dashboard status, even between logging ticks.
            if t.samples[2].valid && xeryon_motion::controller_fault(t.samples[2].value).is_some() {
                self.fault_stat = Some(t.samples[2].value as u32);
            }
            self.finish(State::Fault, e, io);
            return;
        }
        if self.state == State::MoveToStart {
            if self.feedback_pending {
                self.feedback_before = t.feedback_before;
                // Preparation can take seconds while old requested-parameter
                // broadcasts continue. Only SSPD replies after begin's write
                // may satisfy the existing speed verification.
                self.speed_updates = t.speed_updates;
                self.begin(now, io);
                return;
            }
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
            // Drain positioning traffic before t=0 so the first waveform command
            // is exactly SSPD=0 followed by SCAN=+1, never a delayed nonzero retry.
            if matches!(self.config, Some(ExperimentConfig::Sinusoidal { .. })) && !io.tx_idle() {
                if now.wrapping_sub(self.move_started) >= self.move_timeout {
                    self.finish(State::Fault, "Start move deadline", io);
                }
                return;
            }
            io.start_timing();
            self.last_pressure = p.samples;
            self.malformed_start = t.malformed;
            self.feedback_before = t.feedback_before;
            self.epos_start_generation = t.epos_generation;
            self.max_epos_age = now.wrapping_sub(t.samples[0].ms);
            self.max_pressure_age = now.wrapping_sub(p.ms);
            self.started_ms = now;
            self.envelope.reset(t.epos_generation);
            self.next_sample = 0;
            self.next_target = 0;
            self.scan = xeryon_motion::ScanState::default();
            self.motion_changed_ms = now;
            self.state = State::Running;
            #[cfg(not(test))]
            crate::fault::mark(4);
        }
        let elapsed = now.wrapping_sub(self.started_ms);
        self.elapsed_ms = elapsed;
        if self.center_move == CenterMove::None
            && self.envelope.exceeded(self.config.unwrap(), t, now)
        {
            self.finish(State::Fault, "Sinusoidal trajectory envelope exceeded", io);
            return;
        }
        // A deadline is also a hard bound on storage. Never synthesize catch-up samples.
        if elapsed > self.duration_ms {
            self.finish(State::Fault, "Acquisition deadline missed", io);
            return;
        }
        // Log the theoretical position at the actual sample time, including
        // between control ticks. It is never sent as DPOS during the waveform.
        if let ExperimentConfig::Sinusoidal {
            center_nm,
            amplitude_nm,
            frequency_mhz,
            ..
        } = self.config.unwrap()
            && (elapsed >= self.next_sample
                || (self.center_move == CenterMove::None && elapsed >= self.next_target))
        {
            self.target = if self.center_move != CenterMove::None {
                counts(center_nm).unwrap()
            } else if elapsed >= self.duration_ms - MOVE_TIMEOUT_MS {
                // Clamp the terminal sample to the exact endpoint before STOP.
                self.config.unwrap().start().unwrap()
            } else {
                sine_target(center_nm, amplitude_nm, frequency_mhz, elapsed)
            };
        }
        if elapsed >= self.next_sample {
            let late = elapsed - self.next_sample;
            self.max_lateness_ms = self.max_lateness_ms.max(late);
            self.missed_samples += late / SAMPLE_MS;
            self.next_sample = (elapsed / SAMPLE_MS + 1) * SAMPLE_MS;
            let time = t.samples[6];
            if self.logged_epos != Some(t.epos_generation)
                && t.epos_generation != self.epos_start_generation
            {
                self.epos_logged_updates += 1;
            }
            self.repeated_epos += u32::from(self.logged_epos == Some(t.epos_generation));
            self.repeated_pressure += u32::from(self.logged_pressure == Some(p.samples));
            self.logged_epos = Some(t.epos_generation);
            self.logged_pressure = Some(p.samples);
            let record = Record {
                epos_generation: t.epos_generation,
                pressure_generation: p.samples,
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
                if self.center_move != CenterMove::None {
                    if now.wrapping_sub(self.move_started) >= MOVE_TIMEOUT_MS {
                        Err("Final center move deadline")
                    } else if self.center_move == CenterMove::Stopping && io.tx_idle() {
                        // Never return at the last (possibly zero) waveform speed.
                        // STOP has drained before changing SSPD; DPOS waits too.
                        // STOP can discard queued waveform bytes. The scan cache
                        // describes acceptance, not controller execution, so never
                        // deduplicate this positioning speed against that cache.
                        let speed = xeryon_motion::ScanCommand::Speed(velocity_speed(
                            sine_max_velocity(amplitude_nm, frequency_mhz),
                        ));
                        io.scan(&[speed]).map(|_| {
                            self.scan.commit(speed);
                            self.center_move = CenterMove::SettingSpeed;
                            false
                        })
                    } else if self.center_move == CenterMove::SettingSpeed && io.tx_idle() {
                        self.final_return.dpos.scheduled = Some(now);
                        io.target(self.target, now, true).map(|_| {
                            self.final_return.dpos.queued(now, io.tx_progress());
                            self.center_move = CenterMove::Positioning;
                            self.motion_changed_ms = now;
                            false
                        })
                    } else {
                        Ok(self.center_move == CenterMove::Positioning
                            && io.tx_idle()
                            && self.center_arrived(t))
                    }
                } else if elapsed >= waveform_ms {
                    self.final_return = final_return::Diagnostics {
                        boundary: Some(self.started_ms.wrapping_add(waveform_ms)),
                        observed: now,
                        theoretical_counts: self.config.unwrap().start().unwrap(),
                        target: counts(center_nm).unwrap(),
                        speed: velocity_speed(sine_max_velocity(amplitude_nm, frequency_mhz)),
                        waveform_pending: !io.tx_idle(),
                        intent_pending: self.waveform_tx.intent_pending(),
                        settings: t.positioning,
                        deadline: now.wrapping_add(MOVE_TIMEOUT_MS),
                        ..Default::default()
                    };
                    self.final_return.observe(t, io.tx_progress());
                    self.final_return.boundary_feedback = self.final_return.last;
                    self.waveform_tx.close_busy(elapsed, &mut self.scheduling);
                    self.waveform_tx = waveform_tx::Transport::default();
                    if elapsed - waveform_ms >= TRAJECTORY_MS {
                        Err("Trajectory deadline missed")
                    } else {
                        self.target = counts(center_nm).unwrap();
                        self.center_move = CenterMove::Stopping;
                        self.move_started = now;
                        self.final_return.stop.scheduled = Some(now);
                        io.stop().map(|_| {
                            self.final_return.stop.queued(now, io.tx_progress());
                            self.scan.stopped();
                            false
                        })
                    }
                } else {
                    self.poll_waveform(now, t, amplitude_nm, frequency_mhz, io)
                }
            }
        };
        match result {
            Ok(true) => self.finish(State::Complete, "none", io),
            Err(e) => self.finish(State::Fault, e, io),
            _ => {}
        }
    }
    /// Use the same tracked DPOS echo/Position Reached confirmation as startup.
    /// PTOL/PTO2/TOUT/DLAY belong to the controller: never gate on EPOS error.
    fn center_arrived(&self, t: &Telemetry) -> bool {
        let status = t.samples[2];
        let age = status.ms.wrapping_sub(self.motion_changed_ms);
        age > 0
            && age < 0x8000_0000
            && status.value & ((1 << 6) | (1 << 9) | (1 << 13)) == (1 << 6)
            && t.xeryon_motion
                .is_some_and(|m| m.state == 2 && m.target == Some(self.target))
    }
    pub fn metadata(&self, out: &mut impl core::fmt::Write, id: u32) -> core::fmt::Result {
        write!(
            out,
            "{{\"type\":\"data_meta\",\"id\":{id},\"version\":{DATA_VERSION},\"run\":{},\"config\":",
            self.run
        )?;
        self.config.unwrap().write_json(out)?;
        self.envelope.write(out)?;
        if matches!(self.config, Some(ExperimentConfig::Sinusoidal { .. })) {
            write!(
                out,
                ",\"sine_control\":\"velocity_scan\",\"sine_initialization\":\"negative_extremum_zero_velocity\",\"reversal_count\":{},\"reversal_total\":{},\"reversal_diagnostic_version\":2,\"reversal_window_ms\":{},\"reversals\":[]",
                self.transferred_reversals(),
                self.reversals.total,
                reversal::WINDOW_MS
            )?;
        }
        if let Some(ExperimentConfig::Sinusoidal {
            amplitude_nm,
            frequency_mhz,
            ..
        }) = self.config
        {
            self.scheduling.write(out, amplitude_nm, frequency_mhz)?;
        }
        self.final_return.write(out)?;
        if let Some(stat) = self.fault_stat {
            write!(out, ",\"fault_stat\":{stat}")?;
        }
        write!(out, ",\"timing\":{{\"epos\":")?;
        self.epos_arrivals.write_json(out, self.elapsed_ms)?;
        write!(out, ",\"pressure\":")?;
        self.pressure_arrivals.write_json(out, self.elapsed_ms)?;
        write!(
            out,
            ",\"feedback_verified\":{},\"epos_unlogged_updates\":{},\"epos_source_drops\":null",
            self.feedback_verified,
            self.epos_arrivals
                .count
                .saturating_sub(self.epos_logged_updates)
        )?;
        write!(
            out,
            ",\"epos_repeated_records\":{},\"pressure_repeated_records\":{},\"max_epos_age_ms\":{},\"max_pressure_age_ms\":{},\"malformed_lines\":{},\"rx_peak_bytes\":{},\"rx_backlog_polls\":{},\"max_usb_service_gap_ms\":{},\"max_tx_busy_ms\":{},\"rx_overflow\":null,\"epos_rtt\":null,\"epos_timestamp\":\"host_receipt_ms\",\"pressure_timestamp\":\"i2c_completion_ms\",\"info\":{},\"poli_ms\":{},\"before\":[",
            self.repeated_epos,
            self.repeated_pressure,
            self.max_epos_age,
            self.max_pressure_age,
            self.malformed,
            self.transport.rx_peak,
            self.transport.rx_backlog_polls,
            self.transport.max_service_gap_ms,
            self.transport.max_tx_busy_ms,
            crate::xeryon::EXPERIMENT_INFO,
            crate::xeryon::EXPERIMENT_POLI_MS
        )?;
        for (i, value) in self.feedback_before.iter().enumerate() {
            if i != 0 {
                write!(out, ",")?;
            }
            match value {
                Some(v) => write!(out, "{v}")?,
                None => write!(out, "null")?,
            }
        }
        write!(out, "]}}")?;
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
    // One diagnostic per two samples covers all supported reversal frequencies.
    fn transferred_reversals(&self) -> usize {
        self.reversals.len.min(self.len.div_ceil(CHUNK_RECORDS))
    }
    /// One bounded diagnostic per two-record chunk. Kept out of RUNNING
    /// traffic; the desktop collects them in metadata for raw CSV export.
    pub fn chunk_header(
        &self,
        out: &mut impl core::fmt::Write,
        id: u32,
        run: u32,
        offset: usize,
        bytes: usize,
    ) -> core::fmt::Result {
        write!(
            out,
            "{{\"type\":\"data_chunk\",\"id\":{id},\"run\":{run},\"offset\":{offset},\"bytes\":{bytes},\"reversal_offset\":{},\"reversals\":[",
            (offset / CHUNK_RECORDS).min(self.transferred_reversals())
        )?;
        let first = offset / CHUNK_RECORDS;
        for i in first..(first + 1).min(self.transferred_reversals()) {
            if i != first {
                write!(out, ",")?;
            }
            self.reversals.records[i].write(out)?;
        }
        writeln!(out, "]}}")
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
    let phase = (u64::from(elapsed) * u64::from(frequency_mhz) + 250_000) % 1_000_000;
    let wave = sine(phase as f64 / 1_000_000.0).clamp(-1.0, 1.0);
    let position = (center_nm as f64 - f64::from(amplitude_nm) * wave) / 78.0;
    (position + if position < 0.0 { -0.5 } else { 0.5 }) as i32
}

/// v(t) = 2*pi*f*A*sin(2*pi*f*t), in um/s, from actual elapsed milliseconds.
fn sine_velocity(amplitude_nm: u32, frequency_mhz: u32, elapsed: u32) -> f64 {
    let phase = (u64::from(elapsed) * u64::from(frequency_mhz)) % 1_000_000;
    sine_max_velocity(amplitude_nm, frequency_mhz)
        * sine(phase as f64 / 1_000_000.0).clamp(-1.0, 1.0)
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
