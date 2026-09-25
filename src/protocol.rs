//! Version 1 PC protocol: bounded ASCII commands, newline-delimited JSON replies.
use crate::{
    command::{Command, MAX_COMMAND_BYTES},
    line::LineBuffer,
};
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
    pub speed: Sample,
    pub speed_updates: u32,
    pub motor_timeout: Sample,
    pub motor_timeout_updates: u32,
    pub plunger_io: Option<crate::plunger::Io>,
    pub pressure: Option<crate::pressure::Reading>,
    pub plunger: Option<(crate::plunger::Motion, Option<u16>)>,
    pub xeryon_motion: Option<crate::xeryon_motion::Motion>,
}

impl Telemetry {
    pub const fn new() -> Self {
        Self {
            speed: Sample {
                value: 0,
                ms: 0,
                valid: false,
            },
            speed_updates: 0,
            motor_timeout: Sample {
                value: 0,
                ms: 0,
                valid: false,
            },
            motor_timeout_updates: 0,
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
        if tag == b"SSPD" {
            self.speed = Sample {
                value,
                ms: now,
                valid: true,
            };
            self.speed_updates = self.speed_updates.wrapping_add(1);
        }
        if tag == b"TOU2" {
            self.motor_timeout = Sample {
                value,
                ms: now,
                valid: true,
            };
            self.motor_timeout_updates = self.motor_timeout_updates.wrapping_add(1);
        }
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
            write!(
                out,
                ",\"limits\":{{\"min\":{},\"max\":{}}}}}",
                motor.limits.min, motor.limits.max
            )?;
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
                ",\"pressure\":{{\"raw\":{},\"average\":{},\"ms\":{},\"samples\":{},\"valid\":{},\"state\":{},\"errors\":{}",
                p.raw, p.average, p.ms, p.samples, p.valid, p.state, p.errors
            )?;
            write!(
                out,
                ",\"sensor\":\"lps22hh\",\"address\":{},\"odr_hz\":{},\"temperature\":{},\"who\":",
                crate::pressure::ADDRESS,
                crate::pressure::OUTPUT_HZ,
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

pub struct Commands(LineBuffer<MAX_COMMAND_BYTES>);
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
            temperature: i16::MIN,
            raw: -8_388_608,
            average: 8_388_607,
            ms: u32::MAX,
            samples: u32::MAX,
            valid: true,
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
