//! XD-C ASCII feedback, manual v3.1 section 4.3 (pages 13–14).
//! Fixed storage, no allocation, and no actuator commands.
use crate::line::LineBuffer;

// Conservative starting point. Verify actual EPOS cadence on the connected XD-C;
// POLI describes data updates, not a guaranteed per-tag acquisition rate.
pub const EXPERIMENT_POLI_MS: u32 = 20;
// INFO=4 also includes TIME but omits requested parameter readbacks needed for
// safety/setup. Neither mode promises atomic EPOS/TIME sampling or a clock origin.
pub const EXPERIMENT_INFO: u32 = 2; // preserves DPOS, Dashboard and requested replies

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

    #[test]
    fn gpio_setup_requires_ordered_fresh_matching_readbacks_and_times_out() {
        let mut t = crate::protocol::Telemetry::new();
        let mut s = GpioSetup::new();
        for (i, expected) in [
            "STPS=1\n", "GPIO=2\n", "STPS=?\n", "GPIO=?\n", "LLIM=?\n", "HLIM=?\n",
        ]
        .iter()
        .enumerate()
        {
            let now = i as u32 * 10;
            assert!(
                !s.poll(now, &t, true, |b| {
                    assert_eq!(b, expected.as_bytes());
                    Ok(())
                })
                .unwrap()
            );
            assert!(!s.poll(now + 1, &t, false, |_| panic!()).unwrap());
            if i >= 2 {
                assert!(!s.poll(now + 2, &t, true, |_| panic!()).unwrap());
                t.record(
                    [b"STPS", b"GPIO", b"LLIM", b"HLIM"][i - 2],
                    if i == 2 { 1 } else { 2 },
                    now + 3,
                );
            }
            assert_eq!(s.poll(now + 4, &t, true, |_| panic!()).unwrap(), i == 5);
        }
        for mismatch in [false, true] {
            let mut s = GpioSetup::new();
            s.step = 2;
            s.poll(0, &t, true, |_| Ok(())).unwrap();
            if mismatch {
                t.record(b"STPS", 2, 1);
            }
            assert!(s.poll(1500, &t, true, |_| panic!()).is_err());
        }
    }
    #[test]
    fn serial_setup_stops_disables_gpio_and_requires_fresh_zero_readback() {
        let mut t = crate::protocol::Telemetry::new();
        t.record(b"GPIO", 0, 0); // cached zero must not count as verification
        for reply in [None, Some(2), Some(0)] {
            let mut setup = GpioSetup::new();
            for (i, expected) in [b"STOP\n".as_slice(), b"GPIO=0\n", b"GPIO=?\n"]
                .into_iter()
                .enumerate()
            {
                let now = i as u32 * 10;
                assert!(
                    !setup
                        .poll_serial(now, &t, true, |b| {
                            assert_eq!(b, expected);
                            Ok(())
                        })
                        .unwrap()
                );
                assert!(!setup.poll_serial(now + 1, &t, false, |_| panic!()).unwrap());
                if i < 2 {
                    assert!(!setup.poll_serial(now + 2, &t, true, |_| panic!()).unwrap());
                }
            }
            assert!(!setup.poll_serial(23, &t, true, |_| panic!()).unwrap());
            if let Some(value) = reply {
                t.record(b"GPIO", value, 24);
            }
            let result = setup.poll_serial(1520, &t, true, |_| panic!());
            assert_eq!(result.is_ok(), reply == Some(0));
            if reply == Some(0) {
                assert_eq!(result, Ok(true));
            }
        }
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

/// GPIO configuration is volatile and verified before ENABLE goes high.
/// USB feedback remains active in GPIO=2. Never SAVE these settings.
pub struct GpioSetup {
    step: usize,
    pending: bool,
    started: u32,
    generation: u32,
}
impl GpioSetup {
    pub const fn new() -> Self {
        Self {
            step: 0,
            pending: false,
            started: 0,
            generation: 0,
        }
    }
    /// Explicitly leave any retained GPIO mode before serial motion. Never SAVE.
    pub fn poll_serial(
        &mut self,
        now: u32,
        t: &crate::protocol::Telemetry,
        idle: bool,
        mut send: impl FnMut(&[u8]) -> Result<(), &'static str>,
    ) -> Result<bool, &'static str> {
        if self.step == 3 {
            return Ok(true);
        }
        if self.pending {
            let fresh = t.gpio[0].valid && t.gpio_updates[0] != self.generation;
            if idle && (self.step < 2 || fresh) {
                if self.step == 2 && t.gpio[0].value != 0 {
                    return Err("Serial GPIO=0 readback mismatch");
                }
                self.step += 1;
                self.pending = false;
            } else if now.wrapping_sub(self.started) >= 1500 {
                return Err("Serial mode setup timeout");
            }
        } else if idle {
            send([b"STOP\n".as_slice(), b"GPIO=0\n", b"GPIO=?\n"][self.step])?;
            self.started = now;
            self.generation = t.gpio_updates[0];
            self.pending = true;
        } else {
            return Err("Serial mode setup transmit busy");
        }
        Ok(self.step == 3)
    }
    pub fn poll(
        &mut self,
        now: u32,
        t: &crate::protocol::Telemetry,
        idle: bool,
        mut send: impl FnMut(&[u8]) -> Result<(), &'static str>,
    ) -> Result<bool, &'static str> {
        const COMMANDS: [&[u8]; 6] = [
            b"STPS=1\n",
            b"GPIO=2\n",
            b"STPS=?\n",
            b"GPIO=?\n",
            b"LLIM=?\n",
            b"HLIM=?\n",
        ];
        if self.step == 6 {
            return Ok(true);
        }
        let index = match self.step {
            0 | 2 => 1,
            1 | 3 => 0,
            n => n - 2,
        };
        if self.pending {
            let fresh = t.gpio[index].valid && t.gpio_updates[index] != self.generation;
            if idle && (self.step < 2 || fresh) {
                if (2..4).contains(&self.step)
                    && t.gpio[index].value != if index == 0 { 2 } else { 1 }
                {
                    return Err("GPIO/STPS readback mismatch");
                }
                self.step += 1;
                self.pending = false;
            } else if now.wrapping_sub(self.started) >= 1500 {
                return Err("GPIO setup readback timeout");
            }
        } else if idle {
            send(COMMANDS[self.step])?;
            self.started = now;
            self.generation = t.gpio_updates[index];
            self.pending = true;
        } else {
            return Err("GPIO setup transmit busy");
        }
        Ok(self.step == 6)
    }
}
