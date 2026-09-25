//! XD-C ASCII feedback, manual v3.1 section 4.3 (pages 13–14).
//! Fixed storage, no allocation, and no actuator commands.
use crate::line::LineBuffer;

pub struct Decoder(LineBuffer<64>);

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
        Self(LineBuffer::new())
    }

    pub fn push(&mut self, byte: u8) -> Option<Result<Record, Error>> {
        match self.0.push(byte)? {
            Err(()) => Some(Err(Error::TooLong)),
            Ok(line) => {
                let line = line.strip_suffix(b"\r").unwrap_or(line);
                if line.is_empty() {
                    None
                } else {
                    Some(Record::parse(line))
                }
            }
        }
    }
}

impl Record {
    pub fn tag(&self) -> &[u8] {
        &self.tag[..self.tag_len]
    }

    pub fn value(&self) -> i64 {
        self.value
    }

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
        assert_eq!(records[0].as_ref().unwrap().tag(), b"EPOS");
        assert_eq!(records[0].as_ref().unwrap().value(), -123);
        assert_eq!(records[1].as_ref().unwrap().tag(), b"STAT");
        assert_eq!(records[1].as_ref().unwrap().value(), 1344);
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
        assert_eq!(records[1].as_ref().unwrap().tag(), b"STAT");
        assert_eq!(records[1].as_ref().unwrap().value(), 0);
    }

    #[test]
    fn other_telemetry_and_reset() {
        for (tag, value) in [("SOFT", 20103), ("XLS1", 312), ("SYNC", 12345678)] {
            let record = Record::parse(format!("{tag}={value}").as_bytes()).unwrap();
            assert_eq!(record.tag(), tag.as_bytes());
            assert_eq!(record.value(), value);
        }
        let mut decoder = Decoder::new();
        for &b in b"STAT=12" {
            decoder.push(b);
        }
        decoder = Decoder::new(); // Same reset used on disconnect/mount.
        assert!(decoder.push(b'\n').is_none());
    }
}
