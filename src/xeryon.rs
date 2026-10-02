//! XD-C ASCII feedback, manual v3.1 section 4.3 (pages 13–14).
//! Fixed storage, no allocation, and bounded setup before motion.
use crate::line::LineBuffer;

// Conservative starting point. Verify actual EPOS cadence on the connected XD-C;
// POLI describes data updates, not a guaranteed per-tag acquisition rate.
pub const EXPERIMENT_POLI_MS: u32 = 20;
pub const EXPERIMENT_INFO: u32 = 2; // preserves DPOS, Dashboard and requested replies

/// Volatile PRO2 reduction for the sinusoidal trial, after INFO setup. Each
/// query must have a fresh reply and each write must drain before verification.
/// A lower existing gain is preserved. No SAVE, reset, or other tuning writes.
pub struct ScanGainSetup {
    pub values: [Option<u32>; 2], // original, verified applied
    step: u8,                     // 0 query original, 1 write cap, 2 query applied, 3 done
    pending: bool,
    started: u32,
    generation: u32,
}
impl ScanGainSetup {
    pub const fn new() -> Self {
        Self {
            values: [None; 2],
            step: 0,
            pending: false,
            started: 0,
            generation: 0,
        }
    }
    pub fn poll(
        &mut self,
        now: u32,
        t: &crate::protocol::Telemetry,
        idle: bool,
        mut send: impl FnMut(&[u8]) -> Result<(), &'static str>,
    ) -> Result<bool, &'static str> {
        use crate::experiment_data::SINE_SCAN_GAIN_CAP;
        use core::fmt::Write;
        if self.step == 3 {
            return Ok(true);
        }
        if self.pending {
            let fresh = t.scan_gain.valid && t.scan_gain_updates != self.generation;
            if idle && (self.step == 1 || fresh) {
                if self.step != 1 {
                    let value = t.scan_gain.value;
                    if !(0..=250).contains(&value) {
                        return Err("Invalid Xeryon PRO2 readback");
                    }
                    let value = value as u32;
                    if self.step == 0 {
                        self.values[0] = Some(value);
                        if value <= SINE_SCAN_GAIN_CAP {
                            self.values[1] = Some(value);
                            self.step = 3;
                            self.pending = false;
                            return Ok(true);
                        }
                    } else if value != SINE_SCAN_GAIN_CAP {
                        return Err("Xeryon PRO2 readback mismatch");
                    } else {
                        self.values[1] = Some(value);
                    }
                }
                self.step += 1;
                self.pending = false;
            } else if now.wrapping_sub(self.started) >= 1500 {
                return Err("Xeryon PRO2 setup timeout");
            }
            return Ok(self.step == 3);
        }
        if !idle {
            return Err("Xeryon previous transmit incomplete");
        }
        let mut frame = crate::protocol::Frame::new();
        if self.step == 1 {
            writeln!(frame, "PRO2={SINE_SCAN_GAIN_CAP}").unwrap();
        } else {
            writeln!(frame, "PRO2=?").unwrap();
        }
        send(frame.pending())?;
        self.generation = t.scan_gain_updates;
        self.started = now;
        self.pending = true;
        Ok(false)
    }
}

/// Sequential, bounded setup before motion. Never SAVE controller settings.
pub struct FeedbackSetup {
    pub before: [Option<i64>; 2],
    step: u8,
    pending: bool,
    started: u32,
    generation: u32,
}
impl FeedbackSetup {
    pub const fn new() -> Self {
        Self {
            before: [None; 2],
            step: 0,
            pending: false,
            started: 0,
            generation: 0,
        }
    }
    pub fn poll(
        &mut self,
        now: u32,
        t: &crate::protocol::Telemetry,
        idle: bool,
        mut send: impl FnMut(&[u8]) -> Result<(), &'static str>,
    ) -> Result<bool, &'static str> {
        use core::fmt::Write;
        if self.step == 10 {
            return Ok(true);
        }
        // Capture the controller's completion policy without changing tuning.
        if self.step >= 6 {
            let i = usize::from(self.step - 6);
            if self.pending {
                if idle && t.positioning_updates[i] != self.generation && t.positioning[i].valid {
                    if !(0..=65535).contains(&t.positioning[i].value) {
                        return Err("Invalid Xeryon positioning setting");
                    }
                    self.step += 1;
                    self.pending = false;
                } else if now.wrapping_sub(self.started) >= 1500 {
                    return Err("Xeryon positioning settings readback timeout");
                }
                return Ok(self.step == 10);
            }
            if !idle {
                return Err("Xeryon previous transmit incomplete");
            }
            send([b"PTOL=?\n", b"PTO2=?\n", b"TOUT=?\n", b"DLAY=?\n"][i])?;
            self.generation = t.positioning_updates[i];
            self.started = now;
            self.pending = true;
            return Ok(false);
        }
        let index = usize::from(self.step % 2);
        let query = self.step < 2 || self.step >= 4;
        if self.pending {
            let fresh = t.setting_updates[index] != self.generation && t.settings[index].valid;
            if idle && (!query || fresh) {
                if query {
                    let value = t.settings[index].value;
                    if self.step < 2 {
                        self.before[index] = Some(value);
                    } else if value
                        != i64::from(if index == 0 {
                            EXPERIMENT_INFO
                        } else {
                            EXPERIMENT_POLI_MS
                        })
                    {
                        return Err("Xeryon INFO/POLI readback mismatch");
                    }
                }
                self.step += 1;
                self.pending = false;
            } else if now.wrapping_sub(self.started) >= 1500 {
                // Inherited INFO modes may omit requested parameters. Report
                // unknown originals, then explicitly establish a known mode.
                if self.step < 2 && idle {
                    self.step += 1;
                    self.pending = false;
                } else {
                    return Err("Xeryon feedback setup timeout");
                }
            }
            return Ok(false);
        }
        if !idle {
            return Err("Xeryon previous transmit incomplete");
        }
        let mut frame = crate::protocol::Frame::new();
        let tag = if index == 0 { "INFO" } else { "POLI" };
        if query {
            writeln!(frame, "{tag}=?").unwrap();
        } else {
            writeln!(
                frame,
                "{tag}={}",
                if index == 0 {
                    EXPERIMENT_INFO
                } else {
                    EXPERIMENT_POLI_MS
                }
            )
            .unwrap();
        }
        send(frame.pending())?;
        self.started = now;
        self.generation = t.setting_updates[index];
        self.pending = true;
        Ok(false)
    }
}

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

    #[test]
    fn scan_gain_reduction_serializes_read_write_verify_and_never_increases_gain() {
        for original in [0, 20, 30, 40, 250] {
            let mut setup = ScanGainSetup::new();
            let mut t = crate::protocol::Telemetry::new();
            t.record(b"PRO2", original, 0); // cached data must not authorize motion
            let mut commands = Vec::new();
            assert!(
                !setup
                    .poll(10, &t, true, |b| {
                        commands.push(b.to_vec());
                        Ok(())
                    })
                    .unwrap()
            );
            assert!(!setup.poll(11, &t, true, |_| panic!("stale reply")).unwrap());
            t.record(b"PRO2", original, 12);
            assert!(!setup.poll(12, &t, false, |_| panic!("TX busy")).unwrap());
            assert_eq!(
                setup.poll(13, &t, true, |_| panic!()).unwrap(),
                original <= 30
            );
            if original > 30 {
                assert!(
                    !setup
                        .poll(14, &t, true, |b| {
                            commands.push(b.to_vec());
                            Ok(())
                        })
                        .unwrap()
                );
                assert!(
                    !setup
                        .poll(15, &t, false, |_| panic!("write still pending"))
                        .unwrap()
                );
                // A write echo cannot satisfy the subsequent explicit query.
                t.record(b"PRO2", 30, 16);
                assert!(!setup.poll(16, &t, true, |_| panic!()).unwrap());
                assert!(
                    !setup
                        .poll(17, &t, true, |b| {
                            commands.push(b.to_vec());
                            Ok(())
                        })
                        .unwrap()
                );
                assert!(
                    !setup
                        .poll(18, &t, true, |_| panic!("old write echo"))
                        .unwrap()
                );
                t.record(b"PRO2", 30, 19);
                assert!(setup.poll(19, &t, true, |_| panic!()).unwrap());
                assert_eq!(
                    commands,
                    [
                        b"PRO2=?\n".to_vec(),
                        b"PRO2=30\n".to_vec(),
                        b"PRO2=?\n".to_vec()
                    ]
                );
            } else {
                assert_eq!(commands, [b"PRO2=?\n".to_vec()]);
            }
            assert_eq!(
                setup.values,
                [Some(original as u32), Some(original.min(30) as u32)]
            );
            assert!(
                setup
                    .poll(20, &t, true, |_| panic!("already verified"))
                    .unwrap()
            );
        }
    }

    #[test]
    fn scan_gain_faults_on_invalid_mismatched_missing_or_failed_readback() {
        for step in [0, 2] {
            for value in [-1, 251, 40] {
                let mut setup = ScanGainSetup::new();
                setup.step = step;
                let mut t = crate::protocol::Telemetry::new();
                setup.poll(0, &t, true, |_| Ok(())).unwrap();
                t.record(b"PRO2", value, 1);
                let result = setup.poll(1, &t, true, |_| panic!());
                if value == 40 && step == 0 {
                    assert_eq!(result, Ok(false));
                } else {
                    assert!(result.is_err());
                }
                assert_eq!(setup.values[1], None);
            }
        }
        for step in 0..3 {
            let mut setup = ScanGainSetup::new();
            setup.step = step;
            let t = crate::protocol::Telemetry::new();
            assert_eq!(
                setup.poll(0, &t, false, |_| panic!()),
                Err("Xeryon previous transmit incomplete")
            );
            assert_eq!(
                setup.poll(0, &t, true, |_| Err("USB failed")),
                Err("USB failed")
            );
            setup.poll(u32::MAX - 100, &t, true, |_| Ok(())).unwrap();
            assert_eq!(
                setup.poll(1399, &t, step != 1, |_| panic!()),
                Err("Xeryon PRO2 setup timeout")
            );
            assert_eq!(setup.values[1], None);
        }
    }

    #[test]
    fn feedback_setup_serializes_queries_settings_and_requires_fresh_readback() {
        let mut setup = FeedbackSetup::new();
        let mut t = crate::protocol::Telemetry::new();
        let mut commands = Vec::new();
        let expected = [
            "INFO=?\n",
            "POLI=?\n",
            "INFO=2\n",
            "POLI=20\n",
            "INFO=?\n",
            "POLI=?\n",
            "PTOL=?\n",
            "PTO2=?\n",
            "TOUT=?\n",
            "DLAY=?\n",
        ];
        for (step, command) in expected.iter().enumerate() {
            let now = step as u32 * 20;
            assert!(
                !setup
                    .poll(now, &t, true, |b| {
                        commands.push(String::from_utf8(b.to_vec()).unwrap());
                        Ok(())
                    })
                    .unwrap()
            );
            assert_eq!(commands.last().unwrap(), command);
            // Neither a busy transfer nor cached parameter values advances it.
            assert!(
                !setup
                    .poll(now + 1, &t, false, |_| panic!("overlapping send"))
                    .unwrap()
            );
            if step < 2 || step >= 4 {
                assert!(
                    !setup
                        .poll(now + 2, &t, true, |_| panic!("overlapping query"))
                        .unwrap()
                );
                let i = step % 2;
                let value = if i == 0 {
                    2
                } else if step == 1 {
                    97
                } else {
                    20
                };
                if step < 6 {
                    t.record(if i == 0 { b"INFO" } else { b"POLI" }, value, now + 3);
                } else {
                    t.record([b"PTOL", b"PTO2", b"TOUT", b"DLAY"][step - 6], 2, now + 3);
                }
            }
            assert_eq!(
                setup
                    .poll(now + 4, &t, true, |_| panic!("send before drain"))
                    .unwrap(),
                step == 9
            );
        }
        assert_eq!(setup.before, [Some(2), Some(97)]);
        assert!(
            setup
                .poll(999, &t, true, |_| panic!("already ready"))
                .unwrap()
        );
    }

    #[test]
    fn positioning_readback_rejects_stale_invalid_and_missing_values() {
        for index in 0..4 {
            let tag = [b"PTOL", b"PTO2", b"TOUT", b"DLAY"][index];
            let mut t = crate::protocol::Telemetry::new();
            t.record(tag, 2, 1);
            let mut s = FeedbackSetup::new();
            s.step = 6 + index as u8;
            s.poll(10, &t, true, |_| Ok(())).unwrap();
            assert!(!s.poll(11, &t, true, |_| panic!()).unwrap());
            assert_eq!(
                s.poll(1510, &t, true, |_| panic!()),
                Err("Xeryon positioning settings readback timeout")
            );
            t.record(tag, 65536, 1511);
            assert_eq!(
                s.poll(1511, &t, true, |_| panic!()),
                Err("Invalid Xeryon positioning setting")
            );
        }
    }

    #[test]
    fn feedback_setup_unknown_originals_timeout_mismatch_and_send_failure() {
        let t = crate::protocol::Telemetry::new();
        let mut s = FeedbackSetup::new();
        s.poll(u32::MAX - 100, &t, true, |_| Ok(())).unwrap();
        assert!(!s.poll(1399, &t, true, |_| panic!()).unwrap());
        assert_eq!(s.step, 1);
        assert_eq!(s.before[0], None);
        s.step = 4;
        s.poll(1400, &t, true, |_| Ok(())).unwrap();
        let mut t = t;
        t.record(b"INFO", 7, 1401);
        assert_eq!(
            s.poll(1401, &t, true, |_| panic!()),
            Err("Xeryon INFO/POLI readback mismatch")
        );
        t.setting_updates[0] = 0;
        assert_eq!(
            s.poll(2900, &t, true, |_| panic!()),
            Err("Xeryon feedback setup timeout")
        );
        assert_eq!(
            FeedbackSetup::new().poll(0, &t, true, |_| Err("TX failed")),
            Err("TX failed")
        );
        assert_eq!(
            FeedbackSetup::new().poll(0, &t, false, |_| panic!()),
            Err("Xeryon previous transmit incomplete")
        );
    }

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
