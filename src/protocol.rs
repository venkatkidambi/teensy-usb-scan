//! Version 1 PC protocol: bounded ASCII commands, newline-delimited JSON replies.
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
    pub samples: [Sample; 8],
}

impl Telemetry {
    pub const fn new() -> Self {
        Self {
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
            }; 8],
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
        out.write_str("}}\n")
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum Command {
    Hello(u32),
    Ping(u32),
    Snapshot(u32),
    Stream(u32, bool),
}

impl Command {
    fn parse(line: &[u8]) -> Result<Self, ()> {
        let text = core::str::from_utf8(line).map_err(|_| ())?;
        let mut words = text.split_ascii_whitespace();
        let name = words.next().ok_or(())?;
        let id = words.next().ok_or(())?.parse::<u32>().map_err(|_| ())?;
        if id == 0 {
            return Err(());
        }
        let command = match name {
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

pub struct Commands {
    bytes: [u8; 64],
    len: usize,
    dropping: bool,
}
impl Commands {
    pub const fn new() -> Self {
        Self {
            bytes: [0; 64],
            len: 0,
            dropping: false,
        }
    }
    pub fn push(&mut self, b: u8) -> Option<Result<Command, ()>> {
        if b == b'\n' {
            let len = self.len;
            self.len = 0;
            if self.dropping {
                self.dropping = false;
                return Some(Err(()));
            }
            if len == 0 {
                return None;
            }
            return Some(Command::parse(&self.bytes[..len]));
        }
        if !self.dropping {
            if self.len == self.bytes.len() {
                self.dropping = true;
            } else {
                self.bytes[self.len] = b;
                self.len += 1;
            }
        }
        None
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
        for _ in 0..100 {
            parser.push(b'x');
        }
        assert_eq!(parser.push(b'\n'), Some(Err(())));
        assert_eq!(
            Command::parse(b"STREAM 2 0\r"),
            Ok(Command::Stream(2, false))
        );
    }
    #[test]
    fn snapshots_reset_and_fit_worst_case() {
        let mut t = Telemetry::new();
        for tag in TAGS {
            t.record(tag.as_bytes(), i64::MIN, u32::MAX);
        }
        let mut f = Frame::new();
        t.snapshot(&mut f, u32::MAX, u32::MAX, u32::MAX).unwrap();
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
