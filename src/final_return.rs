//! Retained final STOP evidence under the legacy final_return wire key. Times are wrapping host milliseconds, not
//! controller execution times. The 50 Hz dataset supplies the EPOS/STAT/TIME
//! history; these snapshots retain the boundary and between-tick completion.
use super::reversal::TxProgress;
use crate::protocol::{Sample, Telemetry};

#[derive(Default)]
pub struct Command {
    pub scheduled: Option<u32>,
    pub queued: Option<u32>,
    pub completed: Option<u32>,
    pub token: u32,
}
impl Command {
    pub fn queued(&mut self, scheduled: u32, progress: TxProgress) {
        self.scheduled = Some(scheduled);
        if progress.0[0] != 0 {
            self.token = progress.0[0];
            self.queued = Some(progress.0[3]);
        }
    }
    pub fn refresh(&mut self, progress: TxProgress) {
        if self.token != 0 && progress.0[1] == self.token {
            self.completed = Some(progress.0[5]);
        }
    }
    fn write(&self, out: &mut impl core::fmt::Write) -> core::fmt::Result {
        write!(out, "{{\"token\":{},\"ms\":[", self.token)?;
        for (i, time) in [self.scheduled, self.queued, self.completed]
            .iter()
            .enumerate()
        {
            if i != 0 {
                write!(out, ",")?;
            }
            match time {
                Some(t) => write!(out, "{t}")?,
                None => write!(out, "null")?,
            }
        }
        write!(out, "]}}")
    }
}

#[derive(Default)]
pub struct Diagnostics {
    pub boundary: Option<u32>,
    pub observed: u32,
    pub theoretical_counts: i32,
    pub target: i32,
    pub speed: u32,
    pub waveform_pending: bool,
    pub intent_pending: bool,
    pub stop: Command,
    pub dpos: Command,
    pub boundary_feedback: [Sample; 4], // EPOS, DPOS, STAT, TIME
    pub last: [Sample; 4],
    pub settings: [Sample; 4],
    pub echo: Sample,
    pub status: [Sample; 8], // STAT transitions, including Position Reached
    pub status_len: usize,
    pub status_dropped: u32,
    pub(super) last_stat: Option<i64>,
    pub finished: Option<u32>,
    pub deadline: u32,
    pub expired: bool,
}
impl Diagnostics {
    pub fn observe(&mut self, t: &Telemetry, progress: TxProgress) {
        if self.boundary.is_none() {
            return;
        }
        self.stop.refresh(progress);
        self.dpos.refresh(progress);
        self.last = [t.samples[0], t.samples[1], t.samples[2], t.samples[6]];
        let stat = t.samples[2];
        if stat.valid && self.last_stat != Some(stat.value) {
            self.last_stat = Some(stat.value);
            if self.status_len < self.status.len() {
                self.status[self.status_len] = stat;
                self.status_len += 1;
            } else {
                self.status_dropped += 1;
            }
        }
        if let Some(sent) = self.dpos.scheduled {
            let echo = t.samples[1];
            let dt = echo.ms.wrapping_sub(sent);
            if !self.echo.valid
                && echo.valid
                && echo.value == i64::from(self.target)
                && dt > 0
                && dt < 0x8000_0000
            {
                self.echo = echo;
            }
        }
    }
    pub fn write(&self, out: &mut impl core::fmt::Write) -> core::fmt::Result {
        let Some(boundary) = self.boundary else {
            return Ok(());
        };
        write!(
            out,
            ",\"final_return\":{{\"version\":1,\"boundary_ms\":{boundary},\"observed_ms\":{},\"theoretical_counts\":{},\"theoretical_velocity_um_s\":0,\"target_counts\":{},\"speed_um_s\":{},\"waveform_tx_pending\":{},\"waveform_intent_pending\":{},\"stop\":",
            self.observed,
            self.theoretical_counts,
            self.target,
            self.speed,
            self.waveform_pending,
            self.intent_pending
        )?;
        self.stop.write(out)?;
        write!(out, ",\"dpos\":")?;
        self.dpos.write(out)?;
        write!(out, ",\"echo\":")?;
        sample(out, self.echo)?;
        for (name, values) in [
            ("boundary_feedback", &self.boundary_feedback[..]),
            ("final_feedback", &self.last[..]),
            ("ptol_pto2_tout_dlay", &self.settings[..]),
            ("stat_transitions", &self.status[..self.status_len]),
        ] {
            write!(out, ",\"{name}\":[")?;
            for (i, &value) in values.iter().enumerate() {
                if i != 0 {
                    write!(out, ",")?;
                }
                sample(out, value)?;
            }
            write!(out, "]")?;
        }
        write!(
            out,
            ",\"stat_transitions_dropped\":{},\"error_counts\":",
            self.status_dropped
        )?;
        if self.last[0].valid {
            write!(
                out,
                "{},\"error_um\":{:.3}",
                self.last[0].value.saturating_sub(i64::from(self.target)),
                (self.last[0].value.saturating_sub(i64::from(self.target))) as f64 * 0.078
            )?;
        } else {
            write!(out, "null,\"error_um\":null")?;
        }
        write!(
            out,
            ",\"deadline_ms\":{},\"expired\":{},\"finished_ms\":",
            self.deadline, self.expired
        )?;
        match self.finished {
            Some(ms) => write!(out, "{ms}")?,
            None => write!(out, "null")?,
        }
        write!(out, "}}")
    }
}
fn sample(out: &mut impl core::fmt::Write, s: Sample) -> core::fmt::Result {
    if s.valid {
        write!(out, "[{},{}]", s.value, s.ms)
    } else {
        write!(out, "null")
    }
}
