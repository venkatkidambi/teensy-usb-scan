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
