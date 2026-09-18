//! XD-C ASCII feedback, manual v3.1 section 4.3 (pages 13–14).
//! Fixed storage, no allocation, and no actuator commands.
use core::fmt;

pub struct Decoder {
    bytes: [u8; 64],
    len: usize,
    dropping: bool,
}

#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    TooLong,
    Malformed,
}

#[derive(Debug, PartialEq, Eq)]
pub struct Record {
    tag: [u8; 5],
    tag_len: usize,
    value: i64,
}

impl Decoder {
    pub const fn new() -> Self {
        Self {
            bytes: [0; 64],
            len: 0,
            dropping: false,
        }
    }

    pub fn push(&mut self, byte: u8) -> Option<Result<Record, Error>> {
        if byte == b'\n' {
            let len = self.len;
            self.len = 0;
            if self.dropping {
                self.dropping = false;
                return Some(Err(Error::TooLong));
            }
            let line = self.bytes[..len]
                .strip_suffix(b"\r")
                .unwrap_or(&self.bytes[..len]);
            return if line.is_empty() {
                None
            } else {
                Some(Record::parse(line))
            };
        }
        if !self.dropping {
            if self.len == self.bytes.len() {
                self.dropping = true;
            } else {
                self.bytes[self.len] = byte;
                self.len += 1;
            }
        }
        None
    }
}

impl Record {
    fn parse(line: &[u8]) -> Result<Self, Error> {
        let equal = line
            .iter()
            .position(|&b| b == b'=')
            .ok_or(Error::Malformed)?;
        if !(4..=5).contains(&equal)
            || !line[..equal]
                .iter()
                .all(|b| b.is_ascii_uppercase() || b.is_ascii_digit())
        {
            return Err(Error::Malformed);
        }
        let number = core::str::from_utf8(&line[equal + 1..]).map_err(|_| Error::Malformed)?;
        let digits = number.strip_prefix(['+', '-']).unwrap_or(number);
        if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
            return Err(Error::Malformed);
        }
        let value = number.parse::<i64>().map_err(|_| Error::Malformed)?;
        if &line[..equal] == b"STAT" && !(0..=0xff_ffff).contains(&value) {
            return Err(Error::Malformed);
        }
        let mut tag = [0; 5];
        tag[..equal].copy_from_slice(&line[..equal]);
        Ok(Self {
            tag,
            tag_len: equal,
            value,
        })
    }
}

impl fmt::Display for Record {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let tag = &self.tag[..self.tag_len];
        let name = core::str::from_utf8(tag).map_err(|_| fmt::Error)?;
        write!(f, "{name}={}", self.value)?;
        match tag {
            b"STAT" => write!(
                f,
                " (0x{:06x}) active flags: {}",
                self.value,
                Status(self.value as u32)
            ),
            b"EPOS" => f.write_str(" | encoder position (counts)"),
            b"DPOS" => f.write_str(" | target position (counts)"),
            b"SRNO" => f.write_str(" | controller serial number"),
            b"SOFT" if self.value >= 0 => write!(
                f,
                " | firmware {}.{}.{}",
                self.value / 10000,
                self.value / 100 % 100,
                self.value % 100
            ),
            b"FREQ" => f.write_str(" | excitation frequency (Hz)"),
            b"TIME" => f.write_str(" | controller timestamp (0.1 ms ticks)"),
            b"SYNC" => f.write_str(if self.value == 12345678 {
                " | sync OK"
            } else {
                " | unexpected sync value"
            }),
            b"MODEL" => f.write_str(" | model value"),
            _ => Ok(()), // Retain unfamiliar firmware/stage tags verbatim.
        }
    }
}

struct Status(u32);

// Bit 20 is explicitly unused in this manual; bits 22 and 23 are undefined.
const FLAGS: [&str; 22] = [
    "amplifiers enabled (XRTA only)",
    "end stop",
    "thermal protection 1",
    "thermal protection 2",
    "force zero",
    "motor on",
    "closed loop",
    "encoder at index",
    "encoder valid",
    "searching index",
    "position reached",
    "error compensation",
    "encoder error",
    "scanning",
    "left end stop",
    "right end stop",
    "error limit",
    "searching optimal frequency",
    "safety timeout (TOU2)",
    "EtherCAT acknowledge",
    "unused bit 20",
    "position fail (TOU3)",
];

impl fmt::Display for Status {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut separator = "";
        for (bit, name) in FLAGS.iter().enumerate() {
            if self.0 & (1 << bit) != 0 {
                write!(f, "{separator}{name}")?;
                separator = ", ";
            }
        }
        let unknown = self.0 & !((1 << FLAGS.len()) - 1);
        if unknown != 0 {
            write!(f, "{separator}unknown bits=0x{unknown:06x}")?;
        } else if separator.is_empty() {
            f.write_str("none")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode(bytes: &[u8]) -> Vec<Result<Record, Error>> {
        let mut decoder = Decoder::new();
        bytes.iter().filter_map(|&b| decoder.push(b)).collect()
    }

    #[test]
    fn fragmented_and_multiple_lines() {
        let mut decoder = Decoder::new();
        for b in b"EPOS=-12" {
            assert!(decoder.push(*b).is_none());
        }
        let records: Vec<_> = b"3\r\nSTAT=1344\nDPOS=+100\n\n"
            .iter()
            .filter_map(|&b| decoder.push(b))
            .collect();
        assert_eq!(records.len(), 3);
        assert_eq!(
            records[0].as_ref().unwrap().to_string(),
            "EPOS=-123 | encoder position (counts)"
        );
        assert_eq!(
            records[1].as_ref().unwrap().to_string(),
            "STAT=1344 (0x000540) active flags: closed loop, encoder valid, position reached"
        );
        assert_eq!(records[2].as_ref().unwrap().value, 100);
    }

    #[test]
    fn malformed_and_overlong_lines_recover() {
        for line in [
            "STAT=-1",
            "STAT=16777216",
            "EPOS=",
            "EPOS=+",
            "EPOS=1x",
            "EPOS=9223372036854775808",
            "STAT=1=2",
            "epos=1",
            "EPOS=1\r2",
            "EPOS= 12",
        ] {
            assert_eq!(
                Record::parse(line.as_bytes()),
                Err(Error::Malformed),
                "{line}"
            );
        }
        assert_eq!(decode(b"EPOS=\xff\n")[0], Err(Error::Malformed));
        let mut input = vec![b'X'; 100];
        input.extend_from_slice(b"STAT=32\nSTAT=0\n");
        let records = decode(&input);
        assert_eq!(records.len(), 2);
        assert_eq!(records[0], Err(Error::TooLong));
        assert_eq!(
            records[1].as_ref().unwrap().to_string(),
            "STAT=0 (0x000000) active flags: none"
        );
    }

    #[test]
    fn status_special_and_fault_bits() {
        assert_eq!(
            Status(1 << 12 | 1 << 18 | 1 << 21).to_string(),
            "encoder error, safety timeout (TOU2), position fail (TOU3)"
        );
        assert_eq!(
            Status(1 << 20 | 1 << 22 | 1 << 23).to_string(),
            "unused bit 20, unknown bits=0xc00000"
        );
        assert_eq!(Status(1 << 0).to_string(), "amplifiers enabled (XRTA only)");
        assert_eq!(Status(1 << 5).to_string(), "motor on");
        for bit in 0..24 {
            assert!(!Status(1 << bit).to_string().contains("none"));
        }
    }

    #[test]
    fn other_telemetry_and_reset() {
        assert_eq!(
            Record::parse(b"SOFT=20103").unwrap().to_string(),
            "SOFT=20103 | firmware 2.1.3"
        );
        assert_eq!(Record::parse(b"XLS1=312").unwrap().to_string(), "XLS1=312");
        assert_eq!(
            Record::parse(b"SYNC=12345678").unwrap().to_string(),
            "SYNC=12345678 | sync OK"
        );
        let mut decoder = Decoder::new();
        for &b in b"STAT=12" {
            decoder.push(b);
        }
        decoder = Decoder::new(); // Same reset used on disconnect/mount.
        assert!(decoder.push(b'\n').is_none());
    }
}
