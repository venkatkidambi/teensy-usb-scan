//! Rust Xeryon command generation and finite positioning (78 nm encoder counts).
use crate::protocol::Telemetry;

pub use crate::experiment_data::MAX_SPEED_UM_S as MAX_SPEED;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScanCommand {
    Speed(u32),
    Direction(i8),
}
impl ScanCommand {
    pub fn write(self, out: &mut impl core::fmt::Write) -> Result<(), &'static str> {
        match self {
            Self::Speed(speed) if speed <= MAX_SPEED => writeln!(out, "SSPD={speed}"),
            Self::Direction(direction) if (-1..=1).contains(&direction) => {
                writeln!(out, "SCAN={direction}")
            }
            _ => return Err("Invalid Xeryon scan command"),
        }
        .map_err(|_| "Xeryon command too long")
    }
}

/// Per-run accepted-command cache. Completion-gated sequencing lives in
/// experiment::waveform_tx; this cache never represents physical feedback.
#[derive(Default)]
pub struct ScanState {
    speed: Option<u32>,
    pub direction: Option<i8>,
}
impl ScanState {
    pub fn needs_speed(&self, velocity: f64) -> bool {
        self.speed != Some(crate::experiment_data::velocity_speed(velocity))
    }
    /// Commit only after the caller has accepted this mandatory reversal line.
    pub fn commit(&mut self, command: ScanCommand) {
        match command {
            ScanCommand::Speed(speed) => self.speed = Some(speed),
            ScanCommand::Direction(direction) => self.direction = Some(direction),
        }
    }
    pub fn stopped(&mut self) {
        self.direction = Some(0);
    }
    #[cfg(test)]
    pub fn set_speed(
        &mut self,
        speed: u32,
        mut send: impl FnMut(&[ScanCommand]) -> Result<(), &'static str>,
    ) -> Result<(), &'static str> {
        if self.speed != Some(speed) {
            send(&[ScanCommand::Speed(speed)])?;
            self.speed = Some(speed);
        }
        Ok(())
    }
    /// Ideal zero-transport-delay model used by mathematical scheduler tests.
    #[cfg(test)]
    pub fn update(
        &mut self,
        velocity: f64,
        direction: i8,
        mut send: impl FnMut(&[ScanCommand]) -> Result<(), &'static str>,
    ) -> Result<(), &'static str> {
        if !matches!(direction, -1 | 1) || !velocity.is_finite() {
            return Err("Invalid scan velocity/direction");
        }
        let speed = crate::experiment_data::velocity_speed(velocity);
        if speed > MAX_SPEED {
            return Err("Invalid scan speed");
        }
        if self.direction.is_some_and(|last| last != direction) {
            // Explicit zero even if rounding already selected zero. No SCAN=0:
            // XD-C manual 4.2 defines it as stop, not a reversal prerequisite.
            send(&[ScanCommand::Speed(0), ScanCommand::Direction(direction)])?;
            self.speed = Some(0);
            self.direction = Some(direction);
        } else {
            let mut commands = [ScanCommand::Speed(speed); 2];
            let mut len = 0;
            if self.speed != Some(speed) {
                len += 1;
            }
            if self.direction != Some(direction) {
                commands[len] = ScanCommand::Direction(direction);
                len += 1;
            }
            if len != 0 {
                send(&commands[..len])?;
                self.speed = Some(speed);
                self.direction = Some(direction);
            }
        }
        Ok(())
    }
}

pub fn write_speed(out: &mut impl core::fmt::Write, speed: u32) -> Result<(), &'static str> {
    if !(1..=MAX_SPEED).contains(&speed) {
        return Err("Xeryon speed must be 1..16777215 um/s");
    }
    writeln!(out, "SSPD={speed}").map_err(|_| "Xeryon command too long")
}

// Compose bounded newline-delimited commands; USB transport flushes each line.
pub fn write_move(
    out: &mut impl core::fmt::Write,
    target: i32,
    speed: Option<u32>,
) -> Result<(), &'static str> {
    if let Some(speed) = speed {
        write_speed(out, speed)?;
    }
    writeln!(out, "DPOS={target}").map_err(|_| "Xeryon command too long")
}

pub use crate::experiment_data::{MAX_POSITION, MAX_TRAVEL_NM, MIN_POSITION};
pub fn controller_fault(stat: i64) -> Option<&'static str> {
    [
        (2, "Xeryon thermal protection 1"),
        (3, "Xeryon thermal protection 2"),
        (12, "Xeryon encoder error"),
        (16, "Xeryon position error limit (ELIM)"),
        (18, "Xeryon safety timeout (TOU2)"),
        (21, "Xeryon positioning timeout (TOU3)"),
    ]
    .into_iter()
    .find_map(|(bit, reason)| (stat & (1 << bit) != 0).then_some(reason))
}

pub fn ready(t: &Telemetry) -> bool {
    t.mounted && t.cdc == 2 && t.vid == 0x04d8 && t.pid == 0xef4a
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn phase_crossings_are_symmetric_once_per_half_cycle_and_zero_before_direction() {
        use crate::experiment_data::{sine_direction, velocity_speed};
        // Exact extrema, between-tick extrema, and fractional-Hz phase drift.
        for frequency in [100, 1000, 1300, 2000] {
            for peak in [0.5, 1.0, 10.0, 6283.0] {
                let mut state = ScanState::default();
                let mut actual_speed = 0;
                let mut actual_direction = 0;
                let mut reversals = 0;
                let mut zero_ms = None;
                let end = 3_000_000u32.div_ceil(frequency);
                for ms in (0..end).step_by(10).chain(core::iter::once(end)) {
                    let direction = sine_direction(frequency, ms);
                    let velocity = peak
                        * (core::f64::consts::TAU * f64::from(frequency) * f64::from(ms)
                            / 1_000_000.0)
                            .sin();
                    let crossing = state.direction.is_some_and(|last| last != direction);
                    let mut commands_this_tick = Vec::new();
                    state
                        .update(velocity, direction, |commands| {
                            commands_this_tick.extend_from_slice(commands);
                            for &command in commands {
                                match command {
                                    ScanCommand::Speed(speed) => {
                                        if speed == 0 {
                                            zero_ms = Some(ms);
                                        }
                                        actual_speed = speed;
                                    }
                                    ScanCommand::Direction(sign) => {
                                        assert_ne!(sign, 0);
                                        assert_ne!(sign, actual_direction);
                                        if actual_direction != 0 {
                                            assert_eq!(actual_speed, 0);
                                            assert_eq!(zero_ms, Some(ms));
                                            reversals += 1;
                                        }
                                        actual_direction = sign;
                                    }
                                }
                            }
                            Ok(())
                        })
                        .unwrap();
                    if crossing {
                        assert_eq!(
                            commands_this_tick,
                            [ScanCommand::Speed(0), ScanCommand::Direction(direction)]
                        );
                        assert_eq!(actual_speed, 0);
                    } else {
                        assert_eq!(actual_speed, velocity_speed(velocity));
                    }
                }
                assert_eq!(reversals, 6, "frequency={frequency}, peak={peak}");
            }
        }
    }

    #[test]
    fn rounded_zero_plateau_does_not_change_direction_or_chatter() {
        let mut state = ScanState::default();
        let mut lines = String::new();
        for (v, direction) in [
            (1.0, 1),
            (0.51, 1),
            (0.49, 1),
            (0.1, 1),
            (0.0, -1),
            (-0.1, -1),
            (-0.49, -1),
            (-0.51, -1),
            (-1.0, -1),
        ] {
            state
                .update(v, direction, |commands| {
                    for command in commands {
                        command.write(&mut lines)?;
                    }
                    Ok(())
                })
                .unwrap();
        }
        assert_eq!(lines, "SSPD=1\nSCAN=1\nSSPD=0\nSSPD=0\nSCAN=-1\nSSPD=1\n");
        // One forced zero at the crossing is intentional, even after rounded zero.
        assert!(state.update(1.0, 1, |_| Err("partial send")).is_err());
        assert_eq!(state.direction, Some(-1));
        state
            .update(1.0, 1, |commands| {
                assert_eq!(commands, [ScanCommand::Speed(0), ScanCommand::Direction(1)]);
                Ok(())
            })
            .unwrap();
    }
    #[test]
    fn scan_commands_round_deduplicate_and_stop_before_reversing() {
        let mut state = ScanState::default();
        let mut lines = String::new();
        let mut update = |state: &mut ScanState, velocity| {
            state
                .update(velocity, if velocity < 0.0 { -1 } else { 1 }, |commands| {
                    for command in commands {
                        command.write(&mut lines)?;
                    }
                    Ok(())
                })
                .unwrap();
        };
        update(&mut state, 10.5);
        update(&mut state, 10.51); // same rounded SSPD and direction
        update(&mut state, 1.0);
        update(&mut state, -1.0); // zero crossing between scheduled updates
        update(&mut state, -2.0); // select direction at zero, then increase speed
        update(&mut state, 0.1); // quantized zero
        update(&mut state, 0.0); // no redundant zero commands
        update(&mut state, 3.0);
        assert_eq!(
            lines,
            "SSPD=11\nSCAN=1\nSSPD=1\nSSPD=0\nSCAN=-1\nSSPD=2\nSSPD=0\nSCAN=1\nSSPD=3\n"
        );
        for command in [ScanCommand::Speed(MAX_SPEED + 1), ScanCommand::Direction(2)] {
            assert!(command.write(&mut String::new()).is_err());
        }
        let mut frame = String::new();
        ScanCommand::Speed(MAX_SPEED).write(&mut frame).unwrap();
        assert_eq!(frame, "SSPD=16777215\n");
        assert!(frame.len() <= 17);
    }
    #[test]
    fn scan_cache_commits_only_after_success_and_preserves_speed_after_stop() {
        let mut state = ScanState::default();
        assert!(state.update(10.0, 1, |_| Err("transmit")).is_err());
        assert_eq!(state.direction, None);
        state
            .update(10.0, 1, |commands| {
                assert_eq!(
                    commands,
                    [ScanCommand::Speed(10), ScanCommand::Direction(1)]
                );
                Ok(())
            })
            .unwrap();
        state.stopped();
        state.set_speed(10, |_| panic!("redundant speed")).unwrap();
        state
            .set_speed(11, |commands| {
                assert_eq!(commands, [ScanCommand::Speed(11)]);
                Ok(())
            })
            .unwrap();
    }
    fn telemetry(now: u32) -> Telemetry {
        let mut t = Telemetry::new();
        t.mounted = true;
        t.cdc = 2;
        t.vid = 0x04d8;
        t.pid = 0xef4a;
        t.record(b"EPOS", 100, now);
        t.record(b"DPOS", 100, now);
        t.record(b"STAT", 1344, now);
        t
    }
    #[test]
    fn speed_precedes_position_and_each_line_fits_transport_limit() {
        for speed in [1, 10000, MAX_SPEED] {
            let mut frame = crate::protocol::Frame::new();
            write_move(&mut frame, MIN_POSITION, Some(speed)).unwrap();
            let text = core::str::from_utf8(frame.pending()).unwrap();
            assert_eq!(text, format!("SSPD={speed}\nDPOS=-33554432\n"));
            assert!(text.lines().all(|line| line.len() + 1 <= 32));
        }
        for speed in [0, MAX_SPEED + 1, u32::MAX] {
            let mut frame = crate::protocol::Frame::new();
            assert!(write_move(&mut frame, 100, Some(speed)).is_err());
            assert!(frame.pending().is_empty());
        }
        let mut frame = crate::protocol::Frame::new();
        write_move(&mut frame, 228, None).unwrap();
        assert_eq!(frame.pending(), b"DPOS=228\n");
    }
    #[test]
    fn relative_target_rounding_range_and_readiness() {
        let mut t = telemetry(100);
        assert_eq!(target(&t, true, 10000, 100), Ok(228));
        assert_eq!(target(&t, false, 10000, 100), Ok(-28));
        assert_eq!(target(&t, true, 78, 100), Ok(101));
        assert_eq!(target(&t, true, 117, 100), Ok(102));
        assert!(target(&t, true, 77, 100).is_err());
        assert!(target(&t, true, MAX_TRAVEL_NM + 1, 100).is_err());
        assert!(target(&t, true, 10000, 3100).is_err());
        t.record(b"EPOS", MAX_POSITION as i64, 100);
        assert!(target(&t, true, 78, 100).is_err());
        t.record(b"EPOS", MIN_POSITION as i64, 100);
        assert!(target(&t, false, 78, 100).is_err());
        for stat in [1 << 2, 1 << 12, 1 << 9, 1 << 13, 1 << 5] {
            t.record(b"STAT", stat, 100);
            assert!(target(&t, true, 78, 100).is_err());
        }
        t = telemetry(100);
        t.pid = 0;
        assert!(target(&t, true, 78, 100).is_err());
        t = telemetry(100);
        t.cdc = 1;
        assert!(target(&t, true, 78, 100).is_err());
    }
    #[test]
    fn completion_requires_echo_then_new_status() {
        let mut t = telemetry(100);
        let mut m = Motion::new();
        m.start(228, 100);
        assert!(!m.poll(&t, 101));
        assert_eq!(m.state, 1);
        t.record(b"STAT", 1344, 101);
        t.record(b"DPOS", 228, 102);
        assert!(!m.poll(&t, 102));
        assert_eq!(m.state, 1);
        t.record(b"STAT", 1344, 103);
        assert!(!m.poll(&t, 103));
        assert_eq!(m.state, 2);
    }
    #[test]
    fn stale_feedback_disconnect_fault_and_timeouts_stop_motion() {
        assert_eq!(controller_fault(1089), None);
        for (bit, expected) in [
            (2, "Xeryon thermal protection 1"),
            (3, "Xeryon thermal protection 2"),
            (12, "Xeryon encoder error"),
            (16, "Xeryon position error limit (ELIM)"),
            (18, "Xeryon safety timeout (TOU2)"),
            (21, "Xeryon positioning timeout (TOU3)"),
        ] {
            assert_eq!(controller_fault(1089 | (1 << bit)), Some(expected));
        }
        let mut m = Motion::new();
        for condition in 0..5 {
            m.start(228, 100);
            let mut t = telemetry(100);
            let now = match condition {
                0 => {
                    t.mounted = false;
                    101
                }
                1 => 3100,
                2 => {
                    t.record(b"STAT", 1 << 16, 101);
                    101
                }
                3 => {
                    t = telemetry(5100);
                    5100
                }
                _ => {
                    t = telemetry(60100);
                    m.heartbeat(60100);
                    60100
                }
            };
            assert!(m.poll(&t, now));
            assert_eq!(m.state, 4);
            assert_eq!(
                m.interruption_reason,
                Some(
                    [
                        "USB2 disconnected",
                        "encoder feedback stale",
                        "Xeryon position error limit (ELIM)",
                        "desktop heartbeat missing",
                        "move deadline",
                    ][condition]
                )
            );
            assert!(!m.poll(&t, now));
        }
        m.start(228, u32::MAX - 100);
        let t = telemetry(10);
        m.heartbeat(10);
        assert!(!m.poll(&t, 10));
        assert_eq!(m.state, 1);
    }
}

pub fn fresh(t: &Telemetry, index: usize, now: u32) -> bool {
    t.samples[index].valid && now.wrapping_sub(t.samples[index].ms) < 3000
}

pub fn target(t: &Telemetry, up: bool, nm: u32, now: u32) -> Result<i32, &'static str> {
    if !ready(t) {
        return Err("Xeryon is not connected");
    }
    if !fresh(t, 0, now) || !fresh(t, 2, now) {
        return Err("Waiting for fresh Xeryon position and status");
    }
    let stat = t.samples[2].value;
    if let Some(reason) = controller_fault(stat) {
        return Err(reason);
    }
    if stat & ((1 << 9) | (1 << 13)) != 0 || (stat & (1 << 5) != 0 && stat & (1 << 10) == 0) {
        return Err("Xeryon is already moving");
    }
    if !(78..=MAX_TRAVEL_NM).contains(&nm) {
        return Err("Invalid Xeryon travel distance");
    }
    let counts = (i64::from(nm) + 39) / 78;
    let position = t.samples[0].value;
    if !(i64::from(MIN_POSITION)..=i64::from(MAX_POSITION)).contains(&position) {
        return Err("Xeryon encoder position is out of range");
    }
    let target = position + if up { counts } else { -counts };
    if !(i64::from(MIN_POSITION)..=i64::from(MAX_POSITION)).contains(&target) {
        return Err("Xeryon target exceeds the signed 26-bit position range");
    }
    Ok(target as i32)
}

#[derive(Clone, Copy)]
pub struct Motion {
    // 0 idle, 1 awaiting target/reached feedback, 2 reached, 3 stopped, 4 failed.
    pub state: u8,
    pub target: Option<i32>,
    started: u32,
    heartbeat: u32,
    echoed: Option<u32>,
    pub interruption_reason: Option<&'static str>,
}
impl Motion {
    pub const fn new() -> Self {
        Self {
            state: 0,
            target: None,
            started: 0,
            heartbeat: 0,
            echoed: None,
            interruption_reason: None,
        }
    }
    pub fn start(&mut self, target: i32, now: u32) {
        self.state = 1;
        self.target = Some(target);
        self.started = now;
        self.heartbeat = now;
        self.echoed = None;
        self.interruption_reason = None;
    }
    pub fn heartbeat(&mut self, now: u32) {
        self.heartbeat = now;
    }
    /// True requests a best-effort STOP; reaching a target requires subsequent feedback.
    pub fn poll(&mut self, t: &Telemetry, now: u32) -> bool {
        self.poll_policy(t, now, false)
    }
    pub fn poll_policy(&mut self, t: &Telemetry, now: u32, autonomous: bool) -> bool {
        if self.state != 1 {
            return false;
        }
        let reason = if !ready(t) {
            Some("USB2 disconnected")
        } else if !fresh(t, 0, now) {
            Some("encoder feedback stale")
        } else if !fresh(t, 2, now) {
            Some("controller status stale")
        } else if let Some(reason) = controller_fault(t.samples[2].value) {
            Some(reason)
        } else if !autonomous && now.wrapping_sub(self.started) >= 60_000 {
            // Autonomous moves have speed-dependent deadlines in Engine.
            Some("move deadline")
        } else if !autonomous && now.wrapping_sub(self.heartbeat) >= 5000 {
            Some("desktop heartbeat missing")
        } else {
            None
        };
        if let Some(reason) = reason {
            self.state = 4;
            self.interruption_reason = Some(reason);
            return true;
        }
        let after = |ms: u32| {
            let age = ms.wrapping_sub(self.started);
            age > 0 && age < 0x8000_0000
        };
        if fresh(t, 1, now)
            && after(t.samples[1].ms)
            && Some(t.samples[1].value) == self.target.map(i64::from)
        {
            let echoed = *self.echoed.get_or_insert(t.samples[1].ms);
            let status_after = t.samples[2].ms.wrapping_sub(echoed);
            if status_after > 0 && status_after < 0x8000_0000 && t.samples[2].value & (1 << 10) != 0
            {
                self.state = 2;
            }
        }
        false
    }
}
