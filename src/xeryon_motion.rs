//! Finite Xeryon positioning, in the existing 78 nm encoder calibration.
use crate::protocol::Telemetry;

pub const MAX_SPEED: u32 = 0x00ff_ffff;

// Queue speed and target together when speed is explicitly requested.
pub fn write_move(
    out: &mut impl core::fmt::Write,
    target: i32,
    speed: Option<u32>,
) -> Result<(), &'static str> {
    match speed {
        Some(speed) if !(1..=MAX_SPEED).contains(&speed) => {
            Err("Xeryon speed must be 1..16777215 um/s")
        }
        Some(speed) => {
            write!(out, "SSPD={speed}\nDPOS={target}\n").map_err(|_| "Xeryon command too long")
        }
        None => write!(out, "DPOS={target}\n").map_err(|_| "Xeryon command too long"),
    }
}

pub const MIN_POSITION: i32 = -(1 << 25);
pub const MAX_POSITION: i32 = (1 << 25) - 1;
pub const MAX_TRAVEL_NM: u32 = 2_617_245_618;
const FAULTS: i64 = (1 << 2) | (1 << 3) | (1 << 12) | (1 << 16) | (1 << 18) | (1 << 21);

pub fn ready(t: &Telemetry) -> bool {
    t.mounted && t.cdc == 2 && t.vid == 0x04d8 && t.pid == 0xef4a
}

#[cfg(test)]
mod tests {
    use super::*;
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
    fn speed_precedes_position_and_fits_one_transport_write() {
        for speed in [1, 10000, MAX_SPEED] {
            let mut frame = crate::protocol::Frame::new();
            write_move(&mut frame, MIN_POSITION, Some(speed)).unwrap();
            let text = core::str::from_utf8(frame.pending()).unwrap();
            assert_eq!(text, format!("SSPD={speed}\nDPOS=-33554432\n"));
            assert!(frame.pending().len() <= 32);
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
    if stat & FAULTS != 0 {
        return Err("Xeryon reports a controller fault");
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
}
impl Motion {
    pub const fn new() -> Self {
        Self {
            state: 0,
            target: None,
            started: 0,
            heartbeat: 0,
            echoed: None,
        }
    }
    pub fn start(&mut self, target: i32, now: u32) {
        self.state = 1;
        self.target = Some(target);
        self.started = now;
        self.heartbeat = now;
        self.echoed = None;
    }
    pub fn heartbeat(&mut self, now: u32) {
        self.heartbeat = now;
    }
    /// True requests a best-effort STOP; reaching a target requires subsequent feedback.
    pub fn poll(&mut self, t: &Telemetry, now: u32) -> bool {
        if self.state != 1 {
            return false;
        }
        if !ready(t)
            || !fresh(t, 0, now)
            || !fresh(t, 2, now)
            || t.samples[2].value & FAULTS != 0
            || now.wrapping_sub(self.started) >= 60_000
            || now.wrapping_sub(self.heartbeat) >= 5000
        {
            self.state = 4;
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
