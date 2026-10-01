//! Bounded post-run diagnostics. No additional commands, sampling or allocation.
use crate::{experiment_data::*, protocol::Telemetry, xeryon_motion::ScanCommand};

// Configuration reserves every reversal; bounded samples within each window
// explicitly count truncation. Transport emits one reversal per sample chunk.
pub const CAPACITY: usize = crate::experiment_data::REVERSAL_CAPACITY;
pub const WINDOW_MS: u32 = 40;

#[derive(Clone, Copy, Default)]
pub struct TxProgress(pub [u32; 6]); // queued, completed, last two queue/done times

#[derive(Clone, Copy, Default)]
pub struct Event {
    kind: i32, // 0 absent, 1 SSPD, 2 SCAN
    value: i32,
    queued: u32,
    done: u32, // relative completion + 1; zero means unobserved
    sequence: u32,
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn window_metrics_use_raw_feedback_receipt_time_and_bounded_storage() {
        let mut records = [Reversal::EMPTY; 1];
        let mut d = Diagnostics::new(&mut records);
        let config = ExperimentConfig::Sinusoidal {
            center_nm: 0,
            amplitude_nm: 1_000_000,
            frequency_mhz: 1000,
            cycles: 2,
            compensation_enabled: false,
            compensation_ms: 17,
        };
        let mut t = Telemetry::new();
        for ms in 450..=550 {
            if ms % 20 == 0 {
                t.record(
                    b"EPOS",
                    i64::from(super::super::sine_target(0, 1_000_000, 1000, ms)) + 9,
                    ms,
                );
                t.record(b"STAT", 8256, ms);
            }
            if ms == 490 {
                t.transport.rx_backlog_polls += 1;
            }
            d.observe(config, ms, ms, &t, ms == 502);
            if ms % 10 == 0 {
                d.tick(ms, ms, &t, if ms < 500 { 1 } else { -1 }, ms == 500);
            }
        }
        assert_eq!(d.records[0].max_error_counts, 9);
        assert_eq!(d.records[0].backlog_polls, 1);
        assert_eq!(d.records[0].busy_polls, 1);
        assert_eq!(d.records[0].stat_or, 8256);
        let mut before = String::new();
        d.records[0].write(&mut before).unwrap();
        d.tick(750, 750, &t, 1, true);
        d.sent(
            &[ScanCommand::Speed(0), ScanCommand::Direction(1)],
            TxProgress::default(),
            0,
            750,
            true,
        );
        let mut after = String::new();
        d.records[0].write(&mut after).unwrap();
        assert_eq!(before, after); // truncation cannot overwrite a retained reversal
        assert_eq!((d.len, d.total), (1, 2));
        d.reset();
        assert_eq!((d.len, d.total), (0, 0));
    }
    #[test]
    fn ordered_command_completion_times_and_partial_reversals_survive_wrap() {
        let mut records = [Reversal::EMPTY; 2];
        let mut d = Diagnostics::new(&mut records);
        let start = u32::MAX - 260;
        let mut t = Telemetry::new();
        t.record(b"EPOS", 42, start.wrapping_add(240));
        t.record(b"STAT", 8256, start.wrapping_add(240));
        for (i, ms, speed) in [(1, 220, 3), (2, 230, 2), (3, 240, 1)] {
            let now = start.wrapping_add(ms);
            d.tick(ms, now, &t, 1, false);
            d.sent(
                &[ScanCommand::Speed(speed)],
                TxProgress([i, i - 1, 0, now, 0, 0]),
                start,
                ms,
                false,
            );
            d.refresh(TxProgress([i, i, 0, now, 0, now.wrapping_add(1)]), start);
        }
        d.tick(250, start.wrapping_add(250), &t, -1, true);
        d.sent(
            &[ScanCommand::Speed(0), ScanCommand::Direction(-1)],
            TxProgress([5, 3, start.wrapping_add(250), start.wrapping_add(250), 0, 0]),
            start,
            250,
            true,
        );
        d.refresh(
            TxProgress([5, 4, 0, 0, start.wrapping_add(241), start.wrapping_add(251)]),
            start,
        );
        assert_eq!(d.records[0].events[3].done, 252);
        assert_eq!(d.records[0].events[4].done, 0);
        d.refresh(
            TxProgress([5, 5, 0, 0, start.wrapping_add(251), start.wrapping_add(253)]),
            start,
        );
        d.tick(260, start.wrapping_add(260), &t, -1, false);
        d.sent(
            &[ScanCommand::Speed(1)],
            TxProgress([6, 5, 0, start.wrapping_add(260), 0, 0]),
            start,
            260,
            false,
        );
        d.refresh(
            TxProgress([6, 6, 0, 0, start.wrapping_add(253), start.wrapping_add(262)]),
            start,
        );
        let mut text = String::new();
        d.records[0].write(&mut text).unwrap();
        assert!(text.contains(
            "[1,3,220,221],[1,2,230,231],[1,1,240,241],[1,0,250,251],[2,-1,250,253],[1,1,260,262]"
        ));
        assert!(text.contains("[240,42,8256,0,0],[250,42,8256,10,10],[260,42,8256,20,20]"));
        // Abort after zero was queued but before direction acceptance/completion.
        d.tick(750, start.wrapping_add(750), &t, 1, true);
        d.sent(
            &[ScanCommand::Speed(0)],
            TxProgress([7, 6, 0, start.wrapping_add(750), 0, 0]),
            start,
            750,
            true,
        );
        let mut text = String::new();
        d.records[1].write(&mut text).unwrap();
        assert!(text.contains("[1,0,750,null],null,null"));
        assert_eq!(d.total, 2);
        assert!(core::mem::size_of::<Reversal>() * CAPACITY <= 170_000);
    }

    #[test]
    fn rounded_zero_resume_has_identity_and_does_not_capture_later_ordinary_speed() {
        let mut records = [Reversal::EMPTY; 1];
        let mut d = Diagnostics::new(&mut records);
        d.tick(500, 500, &Telemetry::new(), -1, true);
        for (i, command, crossing) in [
            (1, ScanCommand::Speed(0), true),
            (2, ScanCommand::Direction(-1), true),
            (3, ScanCommand::Speed(0), false),
            (4, ScanCommand::Speed(7), false),
        ] {
            d.sent(
                &[command],
                TxProgress([i, i - 1, 0, 499 + i, 0, 0]),
                0,
                499 + i,
                crossing,
            );
            d.refresh(TxProgress([i, i, 0, 0, 0, 500 + i]), 0);
        }
        assert_eq!(d.records[0].events[5].value, 0);
        assert_eq!(d.records[0].events[5].sequence, 3);
        assert_eq!(d.records[0].events[5].done, 504);
    }

    #[test]
    fn worst_case_reversal_and_binary_payload_fit_console_and_desktop_frames() {
        let r = Reversal {
            direction: -1,
            crossing_ms: u32::MAX,
            events: [Event {
                kind: 2,
                value: i32::MIN,
                queued: u32::MAX,
                done: u32::MAX,
                sequence: u32::MAX,
            }; 6],
            feedback: [Snapshot([u32::MAX; 5]); 3],
            max_error_counts: u32::MAX,
            backlog_polls: u32::MAX,
            busy_polls: u32::MAX,
            stat_or: u32::MAX,
            detail: Detail {
                theoretical_ms: u32::MAX,
                scheduled: [u32::MAX; 3],
                transitions: [u32::MAX; 5],
                window: [[u32::MAX; 11]; 6],
                count: 6,
                dropped: u32::MAX,
                excursion: u32::MAX,
                coalesced: u32::MAX,
                lateness: u32::MAX,
                max_busy: u32::MAX,
                requested: u32::MAX,
                target: i32::MIN,
                stat_changes: u32::MAX,
            },
        };
        let mut samples = [Record::EMPTY; 1];
        let mut records = [r; 2];
        let mut engine = super::super::Engine::new(&mut samples).with_reversals(&mut records);
        engine.reversals.len = 2;
        let mut frame = crate::protocol::FixedFrame::<4096>::new();
        engine
            .chunk_header(
                &mut frame,
                u32::MAX,
                u32::MAX,
                0,
                CHUNK_RECORDS * RECORD_BYTES,
            )
            .unwrap();
        assert!(frame.len + CHUNK_RECORDS * RECORD_BYTES <= 4096);
    }
}
impl Event {
    fn refresh(&mut self, progress: TxProgress, start: u32) {
        let p = progress.0;
        if self.kind != 0 && self.sequence != 0 && self.done == 0 {
            for i in 0..2 {
                if p[1] >= 2 - i as u32 && self.sequence == p[1] - 1 + i as u32 {
                    self.done = p[4 + i].wrapping_sub(start) + 1;
                }
            }
        }
    }
    fn write(&self, out: &mut impl core::fmt::Write) -> core::fmt::Result {
        if self.kind == 0 {
            return write!(out, "null");
        }
        write!(out, "[{},{},{},", self.kind, self.value, self.queued)?;
        if self.done == 0 {
            write!(out, "null")?;
        } else {
            write!(out, "{}", self.done - 1)?;
        }
        write!(out, "]")
    }
}

#[derive(Clone, Copy, Default)]
pub struct Snapshot([u32; 5]); // elapsed, raw EPOS, raw STAT, EPOS/STAT ages
impl Snapshot {
    fn new(elapsed: u32, now: u32, t: &Telemetry) -> Self {
        Self([
            elapsed,
            t.samples[0].value as u32,
            t.samples[2].value as u32,
            now.wrapping_sub(t.samples[0].ms),
            now.wrapping_sub(t.samples[2].ms),
        ])
    }
    fn write(&self, out: &mut impl core::fmt::Write) -> core::fmt::Result {
        let [a, b, c, d, e] = self.0;
        write!(out, "[{a},{},{c},{d},{e}]", b as i32)
    }
}

#[derive(Clone, Copy)]
pub struct Reversal {
    pub direction: i8,
    pub crossing_ms: u32,
    events: [Event; 6], // last 3 speed commands, explicit zero, direction, first nonzero
    feedback: [Snapshot; 3], // preceding scheduler tick, crossing, first nonzero
    pub max_error_counts: u32,
    pub backlog_polls: u32,
    pub busy_polls: u32,
    pub stat_or: u32,
    pub detail: Detail,
}

/// Version 2 extension; raw host observations, never inferred physical timing.
#[derive(Clone, Copy)]
pub struct Detail {
    pub theoretical_ms: u32,
    pub scheduled: [u32; 3],
    pub transitions: [u32; 5], // NeedZero, AwaitZero, AwaitDirection, AwaitResume, Normal; +1
    pub window: [[u32; 11]; 6], // elapsed, EPOS, STAT, ages, generation, TIME, age, RX backlog, RX peak, TX busy
    pub count: usize,
    pub dropped: u32,
    pub excursion: u32,
    pub coalesced: u32,
    pub lateness: u32,
    pub max_busy: u32,
    pub requested: u32,
    pub target: i32,
    pub stat_changes: u32,
}
impl Detail {
    pub const EMPTY: Self = Self {
        theoretical_ms: 0,
        scheduled: [0; 3],
        transitions: [0; 5],
        window: [[0; 11]; 6],
        count: 0,
        dropped: 0,
        excursion: 0,
        coalesced: 0,
        lateness: 0,
        max_busy: 0,
        requested: 0,
        target: 0,
        stat_changes: 0,
    };
}
impl Reversal {
    pub const EMPTY: Self = Self {
        direction: 0,
        crossing_ms: 0,
        events: [Event {
            kind: 0,
            value: 0,
            queued: 0,
            done: 0,
            sequence: 0,
        }; 6],
        feedback: [Snapshot([0; 5]); 3],
        max_error_counts: 0,
        backlog_polls: 0,
        busy_polls: 0,
        stat_or: 0,
        detail: Detail::EMPTY,
    };
    pub fn write(&self, out: &mut impl core::fmt::Write) -> core::fmt::Result {
        write!(
            out,
            "{{\"direction\":{},\"crossing_ms\":{},\"commands\":[",
            self.direction, self.crossing_ms
        )?;
        for (i, event) in self.events.iter().enumerate() {
            if i != 0 {
                write!(out, ",")?;
            }
            event.write(out)?;
        }
        write!(out, "],\"feedback\":[")?;
        for (i, snapshot) in self.feedback.iter().enumerate() {
            if i != 0 {
                write!(out, ",")?;
            }
            if i == 2 && self.events[5].kind == 0 {
                write!(out, "null")?;
            } else {
                snapshot.write(out)?;
            }
        }
        write!(
            out,
            "],\"command_tokens\":{:?}",
            self.events.map(|e| e.sequence)
        )?;
        write!(
            out,
            ",\"detail\":{{\"theoretical_ms\":{},\"scheduled\":{:?},\"transitions\":{:?},\"window\":[",
            self.detail.theoretical_ms, self.detail.scheduled, self.detail.transitions
        )?;
        for (i, sample) in self.detail.window[..self.detail.count].iter().enumerate() {
            if i != 0 {
                write!(out, ",")?;
            }
            write!(out, "{:?}", sample)?;
        }
        write!(
            out,
            "],\"dropped\":{},\"excursion_counts\":{},\"coalesced\":{},\"lateness_ms\":{},\"max_busy_ms\":{},\"requested_sspd\":{},\"target_counts\":{},\"stat_changes\":{}}}",
            self.detail.dropped,
            self.detail.excursion,
            self.detail.coalesced,
            self.detail.lateness,
            self.detail.max_busy,
            self.detail.requested,
            self.detail.target,
            self.detail.stat_changes
        )?;
        write!(
            out,
            ",\"max_error_counts\":{},\"rx_backlog_polls\":{},\"tx_busy_polls\":{},\"stat_or\":{}}}",
            self.max_error_counts, self.backlog_polls, self.busy_polls, self.stat_or
        )
    }
}

pub struct Diagnostics<'a> {
    pub records: &'a mut [Reversal],
    pub len: usize,
    pub total: u32,
    history: [Event; 3],
    previous: Snapshot,
    window: u64,
    error: u32,
    backlog: u32,
    busy: u32,
    stat: u32,
    last_backlog: u32,
    recording: bool,
    error_generation: Option<u32>,
    pre: [[u32; 11]; 2],
    pre_len: usize,
    observed_generation: Option<u32>,
    busy_start: Option<u32>,
    max_busy: u32,
    excursion: u32,
    stat_changes: u32,
    last_stat: Option<i64>,
}
impl<'a> Diagnostics<'a> {
    pub fn new(records: &'a mut [Reversal]) -> Self {
        Self {
            records,
            len: 0,
            total: 0,
            history: [Event::default(); 3],
            previous: Snapshot::default(),
            window: u64::MAX,
            error: 0,
            backlog: 0,
            busy: 0,
            stat: 0,
            last_backlog: 0,
            recording: false,
            error_generation: None,
            pre: [[0; 11]; 2],
            pre_len: 0,
            observed_generation: None,
            busy_start: None,
            max_busy: 0,
            excursion: 0,
            stat_changes: 0,
            last_stat: None,
        }
    }
    pub fn reset(&mut self) {
        self.len = 0;
        self.total = 0;
        self.history = [Event::default(); 3];
        self.previous = Snapshot::default();
        self.window = u64::MAX;
        self.last_backlog = 0;
        self.recording = false;
        self.pre_len = 0;
        self.observed_generation = None;
        self.busy_start = None;
        self.last_stat = None;
    }
    pub fn refresh(&mut self, progress: TxProgress, start: u32) {
        for event in &mut self.history {
            event.refresh(progress, start);
        }
        if let Some(record) = self
            .records
            .get_mut(self.len.wrapping_sub(1))
            .filter(|_| self.recording)
        {
            for event in &mut record.events {
                event.refresh(progress, start);
            }
        }
    }
    pub fn observe(
        &mut self,
        config: ExperimentConfig,
        elapsed: u32,
        now: u32,
        t: &Telemetry,
        busy: bool,
    ) {
        if busy {
            self.busy_start.get_or_insert(elapsed);
        }
        let busy_ms = self
            .busy_start
            .map_or(0, |start| elapsed.wrapping_sub(start));
        if !busy {
            self.busy_start = None;
        }
        let sample = [
            elapsed,
            t.samples[0].value as u32,
            t.samples[2].value as u32,
            now.wrapping_sub(t.samples[0].ms),
            now.wrapping_sub(t.samples[2].ms),
            t.epos_generation,
            t.samples[6].value as u32,
            if t.samples[6].valid {
                now.wrapping_sub(t.samples[6].ms)
            } else {
                u32::MAX
            },
            t.transport.rx_backlog_polls,
            t.transport.rx_peak,
            u32::from(busy),
        ];
        let fresh = t.samples[0].valid && self.observed_generation != Some(t.epos_generation);
        if fresh {
            self.observed_generation = Some(t.epos_generation);
            self.pre.rotate_left(1);
            self.pre[1] = sample;
            self.pre_len = (self.pre_len + 1).min(2);
        }
        let delta = t.transport.rx_backlog_polls.wrapping_sub(self.last_backlog);
        self.last_backlog = t.transport.rx_backlog_polls;
        let ExperimentConfig::Sinusoidal {
            center_nm,
            amplitude_nm,
            frequency_mhz,
            ..
        } = config
        else {
            return;
        };
        let phase = u64::from(elapsed) * u64::from(frequency_mhz);
        // Nearest nonzero half-cycle crossing; startup is not a reversal.
        let crossing = (phase + 250_000) / 500_000;
        if crossing == 0 {
            return;
        }
        let window = crossing - 1;
        let peak = crossing * 500_000;
        if phase.abs_diff(peak) > u64::from(WINDOW_MS * frequency_mhz) {
            return;
        }
        if self.window != window {
            self.window = window;
            self.error_generation = None;
            self.error = 0;
            self.backlog = 0;
            self.busy = 0;
            self.stat = 0;
            self.max_busy = 0;
            self.excursion = 0;
            self.stat_changes = 0;
            self.last_stat = None;
        }
        self.max_busy = self.max_busy.max(busy_ms);
        if t.samples[2].valid {
            self.stat_changes += u32::from(self.last_stat.is_some_and(|v| v != t.samples[2].value));
            self.last_stat = Some(t.samples[2].value);
        }
        // Compare raw EPOS with theoretical position at its HOST receipt time.
        // Transit/mechanical delay is not corrected or known.
        if t.samples[0].valid
            && self.error_generation != Some(t.epos_generation)
            && let Some(receipt) = elapsed.checked_sub(now.wrapping_sub(t.samples[0].ms))
            && (u64::from(receipt) * u64::from(frequency_mhz)).abs_diff(peak)
                <= u64::from(WINDOW_MS * frequency_mhz)
        {
            self.error_generation = Some(t.epos_generation);
            let extremum = counts(
                center_nm
                    + if crossing % 2 == 1 {
                        i64::from(amplitude_nm)
                    } else {
                        -i64::from(amplitude_nm)
                    },
            )
            .unwrap();
            let beyond =
                (t.samples[0].value - i64::from(extremum)) * if crossing % 2 == 1 { 1 } else { -1 };
            self.excursion = self.excursion.max(beyond.max(0) as u32);
            let target = super::sine_target(center_nm, amplitude_nm, frequency_mhz, receipt);
            self.error = self
                .error
                .max(t.samples[0].value.abs_diff(i64::from(target)) as u32);
        }
        self.backlog = self.backlog.saturating_add(delta);
        self.busy = self.busy.saturating_add(u32::from(busy));
        self.stat |= t.samples[2].value as u32;
        if let Some(record) = self
            .records
            .get_mut(window as usize)
            .filter(|_| (window as usize) < self.len)
        {
            record.max_error_counts = self.error;
            record.backlog_polls = self.backlog;
            record.busy_polls = self.busy;
            record.stat_or = self.stat;
            record.detail.excursion = self.excursion;
            record.detail.max_busy = self.max_busy;
            record.detail.stat_changes = self.stat_changes;
            if fresh && elapsed > record.crossing_ms {
                if record.detail.count < record.detail.window.len() {
                    record.detail.window[record.detail.count] = sample;
                    record.detail.count += 1;
                } else {
                    record.detail.dropped += 1;
                }
            }
        }
    }
    pub fn tick(&mut self, elapsed: u32, now: u32, t: &Telemetry, direction: i8, crossing: bool) {
        if crossing {
            self.total += 1;
            self.recording = false;
            if let Some(record) = self.records.get_mut(self.len) {
                *record = Reversal::EMPTY;
                record.direction = direction;
                record.crossing_ms = elapsed;
                record.detail.window[..self.pre_len].copy_from_slice(&self.pre[2 - self.pre_len..]);
                record.detail.count = self.pre_len;
                record.detail.excursion = self.excursion;
                record.detail.max_busy = self.max_busy;
                record.detail.stat_changes = self.stat_changes;
                record.events[..3].copy_from_slice(&self.history);
                record.feedback[0] = self.previous;
                record.feedback[1] = Snapshot::new(elapsed, now, t);
                record.max_error_counts = self.error;
                record.backlog_polls = self.backlog;
                record.busy_polls = self.busy;
                record.stat_or = self.stat;
                self.len += 1;
                self.recording = true;
            }
        }
        self.previous = Snapshot::new(elapsed, now, t);
    }
    pub fn stage(
        &mut self,
        stage: usize,
        elapsed: u32,
        theoretical: u32,
        requested: u32,
        target: i32,
    ) {
        if let Some(r) = self
            .records
            .get_mut(self.len.wrapping_sub(1))
            .filter(|_| self.recording)
        {
            r.detail.transitions[stage] = elapsed + 1;
            r.detail.theoretical_ms = theoretical;
            if stage == 0 {
                r.detail.requested = requested;
                r.detail.target = target;
            }
        }
    }
    pub fn evaluation(&mut self, late: u32, coalesced: bool) {
        if let Some(r) = self
            .records
            .get_mut(self.len.wrapping_sub(1))
            .filter(|r| self.recording && r.detail.transitions[4] == 0)
        {
            r.detail.lateness = r.detail.lateness.max(late);
            r.detail.coalesced += u32::from(coalesced);
        }
    }
    pub fn scheduled(&mut self, command: ScanCommand, scheduled: u32, crossing: bool) {
        if let Some(r) = self
            .records
            .get_mut(self.len.wrapping_sub(1))
            .filter(|_| self.recording)
        {
            let index = match command {
                ScanCommand::Direction(_) => 1,
                _ if crossing => 0,
                _ => 2,
            };
            if index != 2 || r.events[5].kind == 0 {
                r.detail.scheduled[index] = scheduled;
            }
        }
    }
    pub fn sent(
        &mut self,
        commands: &[ScanCommand],
        progress: TxProgress,
        start: u32,
        elapsed: u32,
        crossing: bool,
    ) {
        for (i, &command) in commands.iter().enumerate() {
            let (kind, value) = match command {
                ScanCommand::Speed(v) => (1, v as i32),
                ScanCommand::Direction(v) => (2, i32::from(v)),
            };
            let index = 2 - commands.len() + i;
            let event = Event {
                kind,
                value,
                queued: if progress.0[0] == 0 {
                    elapsed
                } else {
                    progress.0[2 + index].wrapping_sub(start)
                },
                done: 0,
                sequence: progress.0[0].saturating_sub(commands.len() as u32 - 1 - i as u32),
            };
            if let Some(record) = self
                .records
                .get_mut(self.len.wrapping_sub(1))
                .filter(|_| self.recording)
            {
                if crossing {
                    record.events[if kind == 2 { 4 } else { 3 }] = event;
                } else if kind == 1 && record.events[4].kind != 0 && record.events[5].kind == 0 {
                    record.events[5] = event;
                    record.feedback[2] = self.previous;
                }
            }
            if kind == 1 {
                self.history.rotate_left(1);
                self.history[2] = event;
            }
        }
    }
}
