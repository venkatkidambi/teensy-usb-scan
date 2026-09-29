//! One pending intent, never a queue of historical waveform values.
use super::{Actuator, Engine, scheduling};
use crate::{experiment_data::*, protocol::Telemetry, xeryon_motion::ScanCommand};

// Engineering bounds, independent of the mathematical evaluation deadline.
// At the maximum 2 Hz these are well below the 250 ms between reversals.
pub const TX_BUSY_LIMIT_MS: u32 = 40;
pub const REVERSAL_LIMIT_MS: u32 = 40;

#[derive(Clone, Copy, Default, PartialEq, Eq)]
enum Phase {
    #[default]
    Normal,
    NeedZero,
    AwaitZero,
    AwaitDirection,
    AwaitResume,
}

#[derive(Default)]
pub struct Transport {
    // Latest *deadline*, only for diagnostics. Velocity is always recomputed.
    pending: Option<u32>,
    phase: Phase,
    reversal_started: u32,
    direction: i8,
    busy_since: Option<u32>,
    consecutive: u32,
}
impl Transport {
    pub fn close_busy(&mut self, elapsed: u32, stats: &mut scheduling::Diagnostics) {
        if let Some(start) = self.busy_since.take() {
            stats.busy_duration.add(elapsed.wrapping_sub(start));
        }
    }
}

impl Engine<'_> {
    fn send_waveform(
        &mut self,
        commands: &[ScanCommand],
        scheduled: u32,
        now: u32,
        crossing: bool,
        io: &mut impl Actuator,
    ) -> Result<(), &'static str> {
        self.scheduling.attempted += commands.len() as u32;
        let before = io.tx_progress().0[0];
        let result = io.scan(commands);
        let progress = io.tx_progress();
        let accepted = if result.is_ok() {
            commands.len()
        } else {
            progress.0[0]
                .wrapping_sub(before)
                .min(commands.len() as u32) as usize
        };
        let commands = &commands[..accepted];
        for &command in commands {
            self.scan.commit(command);
        }
        self.scheduling.sent(
            commands,
            progress,
            self.started_ms.wrapping_add(scheduled),
            now,
            self.elapsed_ms,
            self.scan.direction.unwrap_or(1),
        );
        self.reversals.sent(
            commands,
            progress,
            self.started_ms,
            self.elapsed_ms,
            crossing,
        );
        if accepted != 0 {
            self.waveform_tx.busy_since = Some(if progress.0[0] == 0 {
                self.elapsed_ms
            } else {
                progress.0[4 - accepted].wrapping_sub(self.started_ms)
            });
        }
        if result.is_err() {
            self.scheduling.usb_faults += 1;
        }
        result
    }

    pub(super) fn poll_waveform(
        &mut self,
        now: u32,
        t: &Telemetry,
        amplitude: u32,
        frequency: u32,
        io: &mut impl Actuator,
    ) -> Result<bool, &'static str> {
        let elapsed = self.elapsed_ms;
        let idle = io.tx_idle();
        let progress = io.tx_progress();
        let completed = if progress.0[1] == 0 {
            elapsed
        } else {
            progress.0[5].wrapping_sub(self.started_ms)
        };
        // Inspect completion every foreground poll, including between evaluations.
        // A transfer completing at/after its bound is still a deadline violation.
        if let Some(start) = self.waveform_tx.busy_since {
            let end = if idle { completed } else { elapsed };
            if end.wrapping_sub(start) >= TX_BUSY_LIMIT_MS {
                self.scheduling.backpressure_faults += 1;
                return Err("Xeryon excessive transmit backpressure");
            }
            if idle {
                self.waveform_tx.close_busy(end, &mut self.scheduling);
            }
        } else if !idle {
            self.waveform_tx.busy_since = Some(elapsed);
        }
        let reversal_end = if self.waveform_tx.phase == Phase::AwaitResume && idle {
            completed
        } else {
            elapsed
        };
        if self.waveform_tx.phase != Phase::Normal
            && reversal_end.wrapping_sub(self.waveform_tx.reversal_started) >= REVERSAL_LIMIT_MS
        {
            self.scheduling.backpressure_faults += 1;
            return Err("Xeryon reversal backpressure deadline");
        }

        let due = elapsed >= self.next_target;
        let retry =
            idle && (self.waveform_tx.pending.is_some() || self.waveform_tx.phase != Phase::Normal);
        if !due && !retry {
            return Ok(false);
        }
        if due {
            self.scheduling.lateness.add(elapsed - self.next_target);
            if elapsed - self.next_target >= TRAJECTORY_MS {
                return Err("Trajectory deadline missed");
            }
        }
        // Absolute monotonic phase for *every* evaluation, including retries.
        self.scheduling.evaluations += 1;
        self.scheduling.observe(amplitude, frequency, elapsed);
        let velocity = super::sine_velocity(amplitude, frequency, elapsed);
        let direction = sine_direction(frequency, elapsed);
        let crossing = self.waveform_tx.phase == Phase::Normal
            && self.scan.direction.is_some_and(|last| last != direction);
        if crossing {
            let phase = u64::from(elapsed) * u64::from(frequency);
            let event = ((phase - 250_000) / 500_000 * 500_000 + 250_000)
                .div_ceil(u64::from(frequency)) as u32;
            self.scheduling.reversal_lateness.add(elapsed - event);
            self.waveform_tx.phase = Phase::NeedZero;
            self.waveform_tx.reversal_started = event;
            self.waveform_tx.direction = direction;
        }
        self.reversals.tick(elapsed, now, t, direction, crossing);
        if due {
            let needed = self.scan.needs_speed(velocity)
                || self.scan.direction != Some(direction)
                || self.waveform_tx.phase != Phase::Normal;
            self.waveform_tx.pending = needed.then_some(self.next_target);
            if needed {
                self.scheduling.sspd_requested += 1;
                if !idle {
                    self.scheduling.busy_due += 1;
                    self.waveform_tx.consecutive += 1;
                    self.scheduling.max_coalesced = self
                        .scheduling
                        .max_coalesced
                        .max(self.waveform_tx.consecutive);
                }
            } else {
                self.waveform_tx.consecutive = 0;
            }
            // Always advance useful-evaluation time, even with TX busy.
            self.next_target = scheduling::next(
                self.scheduling.mode,
                amplitude,
                frequency,
                elapsed,
                self.scheduling.held.unwrap_or(0.0),
            );
        }
        if !idle {
            return Ok(false);
        }
        self.waveform_tx.consecutive = 0;
        let reversal = self.waveform_tx.reversal_started;
        match self.waveform_tx.phase {
            Phase::NeedZero => {
                // Mandatory even when the rounded speed is already zero.
                self.send_waveform(&[ScanCommand::Speed(0)], reversal, now, true, io)?;
                self.waveform_tx.phase = Phase::AwaitZero;
                self.next_target =
                    scheduling::next(self.scheduling.mode, amplitude, frequency, elapsed, 0.0);
                return Ok(false);
            }
            Phase::AwaitZero => {
                self.scheduling
                    .reversal_zero
                    .add(completed.wrapping_sub(reversal));
                self.send_waveform(
                    &[ScanCommand::Direction(self.waveform_tx.direction)],
                    reversal,
                    now,
                    true,
                    io,
                )?;
                self.waveform_tx.phase = Phase::AwaitDirection;
                return Ok(false);
            }
            Phase::AwaitDirection => {
                self.scheduling
                    .reversal_direction
                    .add(completed.wrapping_sub(reversal));
                // Force a fresh resume SSPD, including rounded zero. SCAN is complete.
                self.send_waveform(
                    &[ScanCommand::Speed(velocity_speed(velocity))],
                    self.waveform_tx.pending.unwrap_or(elapsed),
                    now,
                    false,
                    io,
                )?;
                self.waveform_tx.phase = Phase::AwaitResume;
                self.waveform_tx.pending = None;
            }
            Phase::AwaitResume => {
                self.scheduling
                    .reversal_resume
                    .add(completed.wrapping_sub(reversal));
                self.waveform_tx.phase = Phase::Normal;
                // A due update can be handled on the next foreground poll.
                return Ok(false);
            }
            Phase::Normal => {
                if let Some(scheduled) = self.waveform_tx.pending.take() {
                    let mut commands = [ScanCommand::Speed(velocity_speed(velocity)); 2];
                    let mut len = usize::from(self.scan.needs_speed(velocity));
                    if self.scan.direction != Some(direction) {
                        commands[len] = ScanCommand::Direction(direction);
                        len += 1;
                    }
                    if len != 0 {
                        self.send_waveform(&commands[..len], scheduled, now, false, io)?;
                    }
                }
            }
        }
        self.next_target = scheduling::next(
            self.scheduling.mode,
            amplitude,
            frequency,
            elapsed,
            self.scheduling.held.unwrap_or(0.0),
        );
        Ok(false)
    }
}
