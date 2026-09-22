//! Version 1 PC protocol: bounded ASCII commands, newline-delimited JSON replies.
use crate::line::LineBuffer;
use core::fmt::{self, Write};

pub const TAGS: [&str; 8] = [
    "EPOS", "DPOS", "STAT", "SRNO", "SOFT", "FREQ", "TIME", "SYNC",
];

#[derive(Clone, Copy, Default)]
pub struct Sample {
    pub value: i64,
    pub ms: u32,
    pub valid: bool,
}

#[derive(Clone, Copy)]
pub struct Telemetry {
    pub epoch: u32,
    pub vid: u16,
    pub pid: u16,
    pub mounted: bool,
    pub cdc: u8,
    pub records: u32,
    pub malformed: u32,
    pub samples: [Sample; TAGS.len()],
    pub plunger_io: Option<crate::plunger::Io>,
    pub pressure: Option<crate::pressure::Reading>,
    pub plunger: Option<(crate::plunger::Motion, Option<u16>)>,
    pub xeryon_motion: Option<crate::xeryon_motion::Motion>,
}

impl Telemetry {
    pub const fn new() -> Self {
        Self {
            xeryon_motion: None,
            plunger_io: None,
            pressure: None,
            plunger: None,
            epoch: 0,
            vid: 0,
            pid: 0,
            mounted: false,
            cdc: 0,
            records: 0,
            malformed: 0,
            samples: [Sample {
                value: 0,
                ms: 0,
                valid: false,
            }; TAGS.len()],
        }
    }

    pub fn reset(&mut self) {
        *self = Self {
            epoch: self.epoch.wrapping_add(1),
            ..Self::new()
        };
    }

    pub fn record(&mut self, tag: &[u8], value: i64, now: u32) {
        self.records = self.records.wrapping_add(1);
        if let Some(i) = TAGS.iter().position(|s| s.as_bytes() == tag) {
            self.samples[i] = Sample {
                value,
                ms: now,
                valid: true,
            };
        }
    }

    pub fn snapshot(&self, out: &mut impl Write, id: u32, now: u32, skipped: u32) -> fmt::Result {
        write!(
            out,
            "{{\"type\":\"snapshot\",\"id\":{id},\"ms\":{now},\"epoch\":{},\"vid\":{},\"pid\":{},\"mounted\":{},\"cdc\":{},\"records\":{},\"malformed\":{},\"skipped\":{skipped},\"values\":{{",
            self.epoch, self.vid, self.pid, self.mounted, self.cdc, self.records, self.malformed
        )?;
        let mut sep = "";
        for (tag, sample) in TAGS.iter().zip(self.samples.iter()) {
            if sample.valid {
                write!(
                    out,
                    "{sep}\"{tag}\":{{\"value\":{},\"ms\":{}}}",
                    sample.value, sample.ms
                )?;
                sep = ",";
            }
        }
        out.write_str("}")?;
        if let Some(m) = self.xeryon_motion {
            write!(
                out,
                ",\"xeryon_motion\":{{\"state\":{},\"target\":",
                m.state
            )?;
            match m.target {
                Some(target) => write!(out, "{target}")?,
                None => out.write_str("null")?,
            }
            out.write_str("}")?;
        }
        if let Some((motor, pot)) = self.plunger {
            write!(
                out,
                ",\"plunger\":{{\"counts\":{},\"counts_per_mm\":{},\"running\":{},\"speed\":{},\"up\":{},\"pot\":",
                motor.position,
                crate::plunger::COUNTS_PER_MM,
                motor.running,
                motor.speed,
                motor.up
            )?;
            match pot {
                Some(value) => write!(out, "{value}")?,
                None => out.write_str("null")?,
            }
            out.write_str("}")?;
        }
        if let Some(io) = self.plunger_io {
            write!(
                out,
                ",\"plunger_io\":{{\"standby\":{},\"ain1\":{},\"ain2\":{},\"encoder\":{},\"pwm_count\":{},\"pwm_compare\":{},\"pwm_running\":{},\"pwm_enabled\":{}}}",
                io.standby,
                io.ain1,
                io.ain2,
                io.encoder,
                io.pwm_count,
                io.pwm_compare,
                io.pwm_running,
                io.pwm_enabled
            )?;
        }
        if let Some(p) = self.pressure {
            write!(
                out,
                ",\"pressure\":{{\"raw\":{},\"average\":{},\"ms\":{},\"samples\":{},\"valid\":{},\"saturated\":{},\"state\":{},\"errors\":{}",
                p.raw, p.average, p.ms, p.samples, p.valid, p.saturated, p.state, p.errors
            )?;
            write!(
                out,
                ",\"sensor\":\"i2c\",\"address\":{},\"temperature\":{},\"who\":",
                crate::pressure::ADDRESS,
                p.temperature
            )?;
            match p.who {
                Some(who) => write!(out, "{who}")?,
                None => out.write_str("null")?,
            }
            out.write_str("}")?;
        }
        out.write_str("}\n")
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum Command {
    Start(u32, crate::experiment_data::ExperimentConfig),
    Abort(u32),
    GetData(u32, u32),
    GetChunk(u32, u32, u32),
    AckData(u32, u32, u32, u32),
    Hello(u32),
    Ping(u32),
    Snapshot(u32),
    Stream(u32, bool),
    Move(u32, bool, u8, u32),
    Stop(u32),
    Zero(u32),
    XeryonMove(u32, bool, u32, Option<u32>),
    XeryonStop(u32),
}

impl Command {
    pub fn id(&self) -> u32 {
        match *self {
            Self::Start(id, _)
            | Self::Abort(id)
            | Self::GetData(id, _)
            | Self::GetChunk(id, _, _)
            | Self::AckData(id, _, _, _)
            | Self::Hello(id)
            | Self::Ping(id)
            | Self::Snapshot(id)
            | Self::Stream(id, _)
            | Self::Move(id, _, _, _)
            | Self::Stop(id)
            | Self::Zero(id)
            | Self::XeryonMove(id, _, _, _)
            | Self::XeryonStop(id) => id,
        }
    }
    pub fn parse(line: &[u8]) -> Result<Self, ()> {
        let text = core::str::from_utf8(line).map_err(|_| ())?;
        let mut words = text.split_ascii_whitespace();
        let name = words.next().ok_or(())?;
        let id = words.next().ok_or(())?.parse::<u32>().map_err(|_| ())?;
        if id == 0 {
            return Err(());
        }
        let command = match name {
            "START" => {
                use crate::experiment_data::ExperimentConfig;
                let kind = words.next().ok_or(())?;
                let start = words.next().ok_or(())?.parse::<i64>().map_err(|_| ())?;
                let increment = words.next().ok_or(())?.parse::<i64>().map_err(|_| ())?;
                let timing = words.next().ok_or(())?.parse::<u32>().map_err(|_| ())?;
                let repeats = words.next().ok_or(())?.parse::<u32>().map_err(|_| ())?;
                Self::Start(
                    id,
                    ExperimentConfig::from_wire(kind, start, increment, timing, repeats)
                        .map_err(|_| ())?,
                )
            }
            "ABORT" => Self::Abort(id),
            "GET_DATA" => Self::GetData(id, words.next().ok_or(())?.parse().map_err(|_| ())?),
            "GET_CHUNK" => Self::GetChunk(
                id,
                words.next().ok_or(())?.parse().map_err(|_| ())?,
                words.next().ok_or(())?.parse().map_err(|_| ())?,
            ),
            "ACK_DATA_RECEIVED" => Self::AckData(
                id,
                words.next().ok_or(())?.parse().map_err(|_| ())?,
                words.next().ok_or(())?.parse().map_err(|_| ())?,
                words.next().ok_or(())?.parse().map_err(|_| ())?,
            ),
            "XERYON_STOP" => Self::XeryonStop(id),
            "XERYON_MOVE" => {
                let up = match words.next() {
                    Some("U") => true,
                    Some("D") => false,
                    _ => return Err(()),
                };
                let nm = words.next().ok_or(())?.parse::<u32>().map_err(|_| ())?;
                if !(78..=crate::xeryon_motion::MAX_TRAVEL_NM).contains(&nm) {
                    return Err(());
                }
                let speed = words
                    .next()
                    .map(|value| value.parse::<u32>())
                    .transpose()
                    .map_err(|_| ())?;
                if speed
                    .is_some_and(|value| !(1..=crate::xeryon_motion::MAX_SPEED).contains(&value))
                {
                    return Err(());
                }
                Self::XeryonMove(id, up, nm, speed)
            }
            "PLUNGER_STOP" => Self::Stop(id),
            "PLUNGER_ZERO" => Self::Zero(id),
            "PLUNGER_MOVE" => {
                let up = match words.next() {
                    Some("U") => true,
                    Some("D") => false,
                    _ => return Err(()),
                };
                let speed = words.next().ok_or(())?.parse::<u8>().map_err(|_| ())?;
                let um = words.next().ok_or(())?.parse::<u32>().map_err(|_| ())?;
                if speed == 0 || crate::plunger::distance_counts(um).is_none() {
                    return Err(());
                }
                Self::Move(id, up, speed, um)
            }
            "HELLO" => Self::Hello(id),
            "PING" => Self::Ping(id),
            "SNAPSHOT" => Self::Snapshot(id),
            "STREAM" => Self::Stream(
                id,
                match words.next() {
                    Some("1") => true,
                    Some("0") => false,
                    _ => return Err(()),
                },
            ),
            _ => return Err(()),
        };
        if words.next().is_some() {
            return Err(());
        }
        Ok(command)
    }
}

pub struct Commands(LineBuffer<128>);
impl Commands {
    pub const fn new() -> Self {
        Self(LineBuffer::new())
    }
    pub fn push(&mut self, byte: u8) -> Option<Result<Command, ()>> {
        match self.0.push(byte)? {
            Ok([]) => None,
            Ok(line) => Some(Command::parse(line)),
            Err(()) => Some(Err(())),
        }
    }
}

// One complete frame at a time. WouldBlock retains the unsent suffix; we never
// overwrite an in-flight line, so backpressure cannot splice JSON messages.
pub struct Frame {
    pub bytes: [u8; 2048],
    pub len: usize,
    pub sent: usize,
}
impl Frame {
    pub const fn new() -> Self {
        Self {
            bytes: [0; 2048],
            len: 0,
            sent: 0,
        }
    }
    pub fn error(&mut self, id: u32, message: &str) -> fmt::Result {
        writeln!(
            self,
            "{{\"type\":\"error\",\"id\":{id},\"message\":\"{message}\"}}"
        )
    }
    pub fn clear(&mut self) {
        self.len = 0;
        self.sent = 0;
    }
    pub fn pending(&self) -> &[u8] {
        &self.bytes[self.sent..self.len]
    }
    pub fn consume(&mut self, n: usize) {
        self.sent += n;
        if self.sent == self.len {
            self.clear();
        }
    }
}
impl Write for Frame {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        if text.len() > self.bytes.len() - self.len {
            return Err(fmt::Error);
        }
        self.bytes[self.len..self.len + text.len()].copy_from_slice(text.as_bytes());
        self.len += text.len();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn experiment_commands_keep_ids_and_reject_extra_arguments() {
        for text in [
            "START 7 Q -1234 78 500 2",
            "START 7 S 0 780 1000 1",
            "ABORT 7",
            "GET_DATA 7 1",
            "GET_CHUNK 7 1 0",
            "ACK_DATA_RECEIVED 7 1 21 12345678",
        ] {
            let command = Command::parse(text.as_bytes()).unwrap();
            assert_eq!(command.id(), 7);
            assert!(Command::parse(format!("{text} extra").as_bytes()).is_err());
        }
        for text in [
            "START 1 X 0 78 500 2",
            "START 1 S 0 -78 500 2",
            "START 0 Q 0 78 500 2",
            "GET_CHUNK 7 1 -1",
            "ACK_DATA_RECEIVED 7 1 21 -1",
        ] {
            assert!(Command::parse(text.as_bytes()).is_err());
        }
    }
    #[test]
    fn commands_reject_motion_and_recover() {
        let mut parser = Commands::new();
        for b in b"PING " {
            assert!(parser.push(*b).is_none());
        }
        assert!(parser.push(b'7').is_none());
        assert_eq!(parser.push(b'\n'), Some(Ok(Command::Ping(7))));
        for bad in [
            "MOVE 1",
            "PING 0",
            "STREAM 1 2",
            "PING 1 extra",
            "HELLO",
            "PING -1",
        ] {
            assert!(Command::parse(bad.as_bytes()).is_err());
        }
        for _ in 0..200 {
            parser.push(b'x');
        }
        assert_eq!(parser.push(b'\n'), Some(Err(())));
        assert_eq!(
            Command::parse(b"STREAM 2 0\r"),
            Ok(Command::Stream(2, false))
        );
    }
    #[test]
    fn plunger_commands_are_bounded() {
        assert_eq!(
            Command::parse(b"PLUNGER_MOVE 1 U 64 10"),
            Ok(Command::Move(1, true, 64, 10))
        );
        assert_eq!(Command::parse(b"PLUNGER_STOP 2"), Ok(Command::Stop(2)));
        assert_eq!(Command::parse(b"PLUNGER_ZERO 3"), Ok(Command::Zero(3)));
        for line in [
            "PLUNGER_MOVE 1 U 0 10",
            "PLUNGER_MOVE 1 D 256 10",
            "PLUNGER_MOVE 1 X 64 10",
            "PLUNGER_MOVE 1 U 64 0",
            "PLUNGER_MOVE 1 U 64 4294967295",
            "PLUNGER_STOP 1 extra",
        ] {
            assert!(Command::parse(line.as_bytes()).is_err(), "{line}");
        }
    }
    #[test]
    fn xeryon_commands_are_bounded() {
        assert_eq!(
            Command::parse(b"XERYON_MOVE 1 U 10000 10000"),
            Ok(Command::XeryonMove(1, true, 10000, Some(10000)))
        );
        assert_eq!(
            Command::parse(b"XERYON_MOVE 2 D 78 10000"),
            Ok(Command::XeryonMove(2, false, 78, Some(10000)))
        );
        assert_eq!(
            Command::parse(b"XERYON_MOVE 3 U 10000"),
            Ok(Command::XeryonMove(3, true, 10000, None))
        );
        assert_eq!(Command::parse(b"XERYON_STOP 3"), Ok(Command::XeryonStop(3)));
        for bad in [
            "XERYON_MOVE 1 U 10000 0",
            "XERYON_MOVE 1 U 10000 16777216",
            "XERYON_MOVE 1 U 10000 -1",
            "XERYON_MOVE 1 U 10000 1.5",
            "XERYON_MOVE 1 U 0 10000",
            "XERYON_MOVE 1 U 77 10000",
            "XERYON_MOVE 1 U 2617245619 10000",
            "XERYON_MOVE 1 X 100 10000",
            "XERYON_MOVE 1 D -100 10000",
            "XERYON_MOVE 1 U 1.5 10000",
            "XERYON_MOVE 1 U 100 extra 10000",
            "XERYON_STOP 0",
            "XERYON_STOP 1 extra",
        ] {
            assert!(Command::parse(bad.as_bytes()).is_err(), "{bad}");
        }
    }
    #[test]
    fn snapshots_reset_and_fit_worst_case() {
        let mut t = Telemetry::new();
        for tag in TAGS {
            t.record(tag.as_bytes(), i64::MIN, u32::MAX);
        }
        let mut motor = crate::plunger::Motion::default();
        motor.position = i32::MIN;
        t.plunger = Some((motor, Some(4095)));
        t.pressure = Some(crate::pressure::Reading {
            who: Some(255),
            temperature: 65535,
            raw: 0xffffff,
            average: 0xffffff,
            ms: u32::MAX,
            samples: u32::MAX,
            valid: true,
            saturated: false,
            state: 2,
            errors: u32::MAX,
        });
        t.plunger_io = Some(crate::plunger::Io::default());
        let mut f = Frame::new();
        let mut motion = crate::xeryon_motion::Motion::new();
        motion.start(crate::xeryon_motion::MIN_POSITION, u32::MAX);
        t.xeryon_motion = Some(motion);
        t.snapshot(&mut f, u32::MAX, u32::MAX, u32::MAX).unwrap();
        assert!(
            core::str::from_utf8(f.pending())
                .unwrap()
                .contains("\"xeryon_motion\":{\"state\":1,\"target\":-33554432}")
        );
        let original = f.pending().to_vec();
        f.consume(512);
        assert_eq!(f.pending(), &original[512..]);
        f.consume(f.pending().len());
        assert!(f.pending().is_empty());
        t.reset();
        assert_eq!(t.epoch, 1);
        assert!(t.samples.iter().all(|s| !s.valid));
        t.snapshot(&mut f, 1, 0, 0).unwrap();
        assert!(
            core::str::from_utf8(f.pending())
                .unwrap()
                .contains("\"values\":{}")
        );
    }
}
