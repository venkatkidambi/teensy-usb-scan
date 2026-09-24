//! Nonblocking XD-C scan setup, velocity sequencing and settings restoration.
//! All USB calls stay in the foreground. No controller settings are saved to flash.
use crate::{
    experiment::Actuator,
    experiment_data::*,
    protocol::{SETTING_TAGS, Telemetry},
};

const INFO: usize = 0;
const POLI: usize = 1;
const SSPD: usize = 2;
const LLIM: usize = 3;
const HLIM: usize = 4;
const SETUP_TIMEOUT_MS: u32 = 5_000;
const REPLY_TIMEOUT_MS: u32 = 1_500;
const RESTORE_TIMEOUT_MS: u32 = 2_000;
// USB completion acknowledges transport, not controller command execution.
// These conservative setup/restore gaps are host policy, not specified XD-C
// timing guarantees. Keep them out of the 100 Hz waveform command path.
const SETTING_SETTLE_MS: u32 = 20;
const INFO_SETTLE_MS: u32 = 100;
const QUERY_RETRY_MS: u32 = 250;
const QUERY_ATTEMPTS: u8 = 3;
pub const TRACKING_LIMIT: &str = "Scan tracking error exceeded guard";
pub const TRAVEL_LIMIT: &str = "Scan position reached travel bound";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Command {
    pub tag: &'static str,
    pub value: Option<i32>, // None is a query
}
impl Command {
    pub const fn set(tag: &'static str, value: i32) -> Self {
        Self {
            tag,
            value: Some(value),
        }
    }
    pub fn encode(self, bytes: &mut [u8; 20]) -> &[u8] {
        bytes[..4].copy_from_slice(self.tag.as_bytes());
        bytes[4] = b'=';
        let mut end = 5;
        if let Some(value) = self.value {
            if value < 0 {
                bytes[end] = b'-';
                end += 1;
            }
            let mut digits = [0; 10];
            let mut pos = digits.len();
            let mut n = value.unsigned_abs();
            loop {
                pos -= 1;
                digits[pos] = b'0' + (n % 10) as u8;
                n /= 10;
                if n == 0 {
                    break;
                }
            }
            let len = digits.len() - pos;
            bytes[end..end + len].copy_from_slice(&digits[pos..]);
            end += len;
        } else {
            bytes[end] = b'?';
            end += 1;
        }
        bytes[end] = b'\n';
        &bytes[..end + 1]
    }
}

#[derive(Clone, Copy)]
enum Setup {
    Read(usize),
    Write(usize, i32),
    Verify(usize, i32),
    Feedback,
}

pub struct Session {
    saved: [Option<i32>; 5],
    changed: u8,
    setup_step: usize,
    setup_started: Option<u32>,
    sent: Option<u32>,
    tx_completed: Option<u32>,
    query_last_sent: u32,
    query_attempts: u8,
    revision: u32,
    feedback_after: u32,
    feedback_first: Option<u32>,
    plan: [Command; 3],
    plan_len: usize,
    plan_pos: usize,
    plan_started: u32,
    speed: u32,
    direction: i32,
    pub scan_started: bool,
    center_step: u8,
    restore_started: u32,
    restore_step: usize,
    stop_confirmed: bool,
}
impl Session {
    pub fn new() -> Self {
        Self {
            // INFO modes 3/4/7 have no requested-parameter slot, so querying
            // INFO before disabling the stream can deadlock setup. Own the
            // telemetry mode explicitly and return to EPOS/STAT/DPOS/TIME.
            saved: [Some(4), None, None, None, None],
            changed: 0,
            setup_step: 0,
            setup_started: None,
            sent: None,
            tx_completed: None,
            query_last_sent: 0,
            query_attempts: 0,
            revision: 0,
            feedback_after: 0,
            feedback_first: None,
            plan: [Command::set("SCAN", 0); 3],
            plan_len: 0,
            plan_pos: 0,
            plan_started: 0,
            speed: 0,
            direction: 0,
            scan_started: false,
            center_step: 0,
            restore_started: 0,
            restore_step: 0,
            stop_confirmed: false,
        }
    }
    pub fn changed(&self) -> bool {
        self.changed != 0
    }

    pub fn prepare(
        &mut self,
        config: ExperimentConfig,
        t: &Telemetry,
        now: u32,
        io: &mut impl Actuator,
    ) -> Result<bool, &'static str> {
        let ExperimentConfig::Sinusoidal {
            center_nm,
            amplitude_nm,
            frequency_mhz,
            ..
        } = config
        else {
            return Ok(true);
        };
        let start = *self.setup_started.get_or_insert(now);
        if now.wrapping_sub(start) >= SETUP_TIMEOUT_MS {
            return Err("Scan setup deadline");
        }
        let (low, high) = scan_bounds(center_nm, amplitude_nm)?;
        if self.setup_step >= 5 {
            // Never widen existing controller limits or exceed its existing speed setting.
            if self.saved[LLIM].unwrap() > low || self.saved[HLIM].unwrap() < high {
                return Err("Sine plus guard exceeds controller travel limits");
            }
            if peak_speed(amplitude_nm, frequency_mhz) * (1.0 + SCAN_CORRECTION)
                > f64::from(self.saved[SSPD].unwrap())
            {
                return Err("Controller SSPD too low for sine plus correction");
            }
        }
        let steps = [
            Setup::Write(INFO, 0),
            Setup::Read(SSPD),
            Setup::Read(POLI),
            Setup::Read(LLIM),
            Setup::Read(HLIM),
            Setup::Write(LLIM, low),
            Setup::Verify(LLIM, low),
            Setup::Write(HLIM, high),
            Setup::Verify(HLIM, high),
            Setup::Write(POLI, 2),
            Setup::Verify(POLI, 2),
            // Confirm the starting speed while queries are still enabled.
            // USB OUT completion alone cannot confirm that SSPD took effect.
            // A sub-resolution waveform still stops via SCAN=0, never SSPD=0.
            Setup::Write(SSPD, scan_start_speed(amplitude_nm, frequency_mhz).max(1)),
            Setup::Verify(SSPD, scan_start_speed(amplitude_nm, frequency_mhz).max(1)),
            Setup::Write(INFO, 4),
            Setup::Feedback,
        ];
        let step = steps[self.setup_step];
        if matches!(step, Setup::Feedback) {
            // Require two distinct fresh EPOS publications after enabling fast feedback.
            // Receive timestamps do not establish sensor time or true 100 Hz feedback.
            if [0, 1, 2].iter().all(|&i| {
                let s = t.samples[i];
                s.valid
                    && now.wrapping_sub(s.ms) < SCAN_FEEDBACK_MS
                    && s.ms.wrapping_sub(self.feedback_after) < 0x8000_0000
            }) {
                if self.feedback_first.is_some_and(|ms| ms != t.samples[0].ms) {
                    return Ok(true);
                }
                self.feedback_first = Some(t.samples[0].ms);
            }
            return Ok(false);
        }
        if let Some(sent) = self.sent {
            if now.wrapping_sub(sent) >= REPLY_TIMEOUT_MS {
                return Err(if !io.tx_idle() {
                    "Scan setting USB transmit timeout"
                } else {
                    match step {
                        Setup::Read(SSPD) => "Scan SSPD reply timeout",
                        Setup::Read(POLI) => "Scan POLI reply timeout",
                        Setup::Read(LLIM) => "Scan LLIM reply timeout",
                        Setup::Read(HLIM) => "Scan HLIM reply timeout",
                        Setup::Verify(LLIM, _) => "Scan LLIM readback timeout",
                        Setup::Verify(HLIM, _) => "Scan HLIM readback timeout",
                        Setup::Verify(POLI, _) => "Scan POLI readback timeout",
                        Setup::Verify(SSPD, _) => "Scan initial SSPD readback timeout",
                        _ => "Scan setting completion timeout",
                    }
                });
            }
            if !io.tx_idle() {
                return Ok(false);
            }
            let completed = *self.tx_completed.get_or_insert(now);
            let settle = if matches!(step, Setup::Write(INFO, 0)) {
                INFO_SETTLE_MS
            } else {
                SETTING_SETTLE_MS
            };
            if now.wrapping_sub(completed) < settle {
                return Ok(false);
            }
            match step {
                Setup::Read(i) | Setup::Verify(i, _) => {
                    if t.setting_revisions[i] == self.revision {
                        // Retry only this read. Preserve the original revision
                        // baseline and deadline; never retry motion or advance
                        // to another setting without a fresh matching reply.
                        if self.query_attempts < QUERY_ATTEMPTS
                            && now.wrapping_sub(self.query_last_sent) >= QUERY_RETRY_MS
                            && io.command(Command {
                                tag: SETTING_TAGS[i],
                                value: None,
                            })?
                        {
                            self.query_attempts += 1;
                            self.query_last_sent = now;
                            self.tx_completed = None;
                        }
                        return Ok(false);
                    }
                    let value = i32::try_from(t.settings[i].value)
                        .map_err(|_| "Invalid controller setting")?;
                    let valid = match i {
                        INFO => (0..=7).contains(&value),
                        POLI => (1..=65535).contains(&value),
                        SSPD => (1..=16_777_215).contains(&value),
                        _ => (MIN_POSITION..=MAX_POSITION).contains(&value),
                    };
                    if !valid {
                        return Err("Invalid controller setting");
                    }
                    if let Setup::Verify(_, expected) = step {
                        if value != expected {
                            return Err(if i == SSPD {
                                "Controller initial SSPD mismatch"
                            } else {
                                "Controller scan setting mismatch"
                            });
                        }
                    } else {
                        self.saved[i] = Some(value);
                    }
                    if i == SSPD {
                        self.speed = value as u32;
                    }
                }
                _ => {}
            }
            self.sent = None;
            self.setup_step += 1;
            self.feedback_after = now;
            return Ok(false);
        }
        let (i, value) = match step {
            Setup::Read(i) | Setup::Verify(i, _) => (i, None),
            Setup::Write(i, value) => (i, Some(value)),
            Setup::Feedback => unreachable!(),
        };
        if io.command(Command {
            tag: SETTING_TAGS[i],
            value,
        })? {
            if value.is_some() {
                self.changed |= 1 << i;
            }
            self.revision = t.setting_revisions[i];
            self.sent = Some(now);
            self.tx_completed = None;
            self.query_last_sent = now;
            self.query_attempts = u8::from(value.is_none());
        }
        Ok(false)
    }

    /// Drain at most one command per poll, with a USB completion between lines.
    pub fn service(&mut self, now: u32, io: &mut impl Actuator) -> Result<(), &'static str> {
        if self.plan_len == 0 {
            return Ok(());
        }
        if self.plan_pos == self.plan_len && io.tx_idle() {
            self.plan_len = 0;
            return Ok(());
        }
        if now.wrapping_sub(self.plan_started) >= TRAJECTORY_MS {
            return Err("Scan command deadline");
        }
        if self.plan_pos < self.plan_len && io.command(self.plan[self.plan_pos])? {
            self.plan_pos += 1;
        }
        Ok(())
    }
    pub fn velocity(
        &mut self,
        velocity: i32,
        now: u32,
        io: &mut impl Actuator,
    ) -> Result<(), &'static str> {
        self.service(now, io)?;
        if self.plan_len != 0 || !io.tx_idle() {
            return Err("Scan previous update incomplete");
        }
        let speed = velocity.unsigned_abs();
        if speed > 16_777_215 {
            return Err("Scan speed out of range");
        }
        let direction = velocity.signum();
        let mut len = 0;
        // Reversal first stops scanning; never apply the new speed in the old direction.
        if self.direction != 0 && direction != self.direction {
            self.plan[len] = Command::set("SCAN", 0);
            len += 1;
        }
        if direction != 0 {
            if speed != self.speed {
                self.plan[len] = Command::set("SSPD", speed as i32);
                len += 1;
                self.changed |= 1 << SSPD;
            }
            if direction != self.direction {
                self.plan[len] = Command::set("SCAN", direction);
                len += 1;
            }
            self.speed = speed;
            self.scan_started = true;
        }
        self.direction = direction;
        self.plan_len = len;
        self.plan_pos = 0;
        self.plan_started = now;
        self.service(now, io)
    }
    pub fn cancel(&mut self) {
        self.plan_len = 0;
        self.direction = 0;
    }

    /// Called after STOP, before final DPOS, so a near-zero sine speed cannot
    /// strand the return-to-center move. Initial SSPD is restored first.
    pub fn center(
        &mut self,
        target: i32,
        now: u32,
        io: &mut impl Actuator,
    ) -> Result<bool, &'static str> {
        if !io.tx_idle() {
            return Ok(false);
        }
        if self.center_step == 0 {
            if io.command(Command::set(
                "SSPD",
                self.saved[SSPD].ok_or("Missing original speed")?,
            ))? {
                self.center_step = 1;
            }
            return Ok(false);
        }
        io.target(target, now, true)?;
        Ok(true)
    }
    pub fn begin_restore(&mut self, now: u32) {
        self.cancel();
        self.sent = None;
        self.tx_completed = None;
        self.restore_started = now;
        self.restore_step = 0;
    }
    /// Restore only after STOP has completed and fresh status says scanning is off.
    /// Failure leaves remaining narrow controller limits in place; no SAVE is sent.
    pub fn restore(
        &mut self,
        t: &Telemetry,
        now: u32,
        io: &mut impl Actuator,
    ) -> Result<bool, &'static str> {
        if !self.changed() {
            return Ok(true);
        }
        if now.wrapping_sub(self.restore_started) >= RESTORE_TIMEOUT_MS {
            return Err(if !io.tx_idle() {
                "Restore USB transmit timeout"
            } else if self.scan_started && !self.stop_confirmed {
                let s = t.samples[2];
                if !s.valid
                    || now.wrapping_sub(s.ms) >= SCAN_FEEDBACK_MS
                    || s.ms.wrapping_sub(self.restore_started) >= 0x8000_0000
                    || s.ms == self.restore_started
                {
                    "Restore needs fresh post-STOP status"
                } else if s.value & (1 << 13) != 0 {
                    "Restore blocked: controller still scanning"
                } else {
                    "Restore blocked: motor on without position reached"
                }
            } else {
                "Controller settings restore timeout"
            });
        }
        if !crate::xeryon_motion::ready(t) {
            return Err("Controller disconnected during restore");
        }
        if !io.tx_idle() {
            return Ok(false);
        }
        if self.scan_started && !self.stop_confirmed {
            let s = t.samples[2];
            if !s.valid
                || now.wrapping_sub(s.ms) >= SCAN_FEEDBACK_MS
                || s.ms.wrapping_sub(self.restore_started) >= 0x8000_0000
                || s.ms == self.restore_started
                || s.value & (1 << 13) != 0
                || (s.value & (1 << 5) != 0 && s.value & (1 << 10) == 0)
            {
                return Ok(false);
            }
            self.stop_confirmed = true;
        }
        let order = [SSPD, POLI, LLIM, HLIM, INFO];
        if self.sent.is_some() {
            let completed = *self.tx_completed.get_or_insert(now);
            if now.wrapping_sub(completed) < SETTING_SETTLE_MS {
                return Ok(false);
            }
            self.sent = None;
            self.changed &= !(1 << order[self.restore_step]);
            self.restore_step += 1;
        }
        while self.restore_step < order.len() && self.changed & (1 << order[self.restore_step]) == 0
        {
            self.restore_step += 1;
        }
        if self.restore_step == order.len() {
            return Ok(true);
        }
        let i = order[self.restore_step];
        if io.command(Command::set(
            SETTING_TAGS[i],
            self.saved[i].ok_or("Missing original setting")?,
        ))? {
            self.sent = Some(now);
            self.tx_completed = None;
        }
        Ok(false)
    }
}

pub fn reference(center_nm: i64, amplitude_nm: u32, frequency_mhz: u32, elapsed: u32) -> f64 {
    center_nm as f64 / 1000.0 + f64::from(amplitude_nm) / 1000.0 * wave(frequency_mhz, elapsed, 0)
}
fn wave(frequency_mhz: u32, elapsed: u32, offset: u64) -> f64 {
    let phase = (u64::from(elapsed) * u64::from(frequency_mhz) + offset) % 1_000_000;
    crate::experiment::sine(phase as f64 / 1_000_000.0).clamp(-1.0, 1.0)
}
pub fn velocity(
    config: ExperimentConfig,
    elapsed: u32,
    now: u32,
    t: &Telemetry,
) -> Result<i32, &'static str> {
    let ExperimentConfig::Sinusoidal {
        center_nm,
        amplitude_nm,
        frequency_mhz,
        ..
    } = config
    else {
        return Err("Not a sine run");
    };
    let encoder = t.samples[0];
    let status = t.samples[2];
    let age = now.wrapping_sub(encoder.ms);
    if !encoder.valid
        || !status.valid
        || age >= SCAN_FEEDBACK_MS
        || now.wrapping_sub(status.ms) >= SCAN_FEEDBACK_MS
    {
        return Err("Scan feedback stale");
    }
    // Compare with the reference at reception time, not with the current phase.
    // This avoids repeatedly treating a held sample as a new lag measurement.
    let error = reference(
        center_nm,
        amplitude_nm,
        frequency_mhz,
        elapsed.saturating_sub(age),
    ) - encoder.value as f64 * 0.078;
    let guard = scan_guard_nm(amplitude_nm) as f64 / 1000.0;
    let (low, high) = scan_bounds(center_nm, amplitude_nm)?;
    if encoder.value <= i64::from(low) || encoder.value >= i64::from(high) {
        return Err(TRAVEL_LIMIT);
    }
    if error.abs() > guard {
        return Err(TRACKING_LIMIT);
    }
    let peak = peak_speed(amplitude_nm, frequency_mhz);
    let correction = if error.abs() <= 0.078 {
        0.0
    } else {
        (SCAN_KP * error).clamp(-peak * SCAN_CORRECTION, peak * SCAN_CORRECTION)
    };
    let speed = peak * wave(frequency_mhz, elapsed, 250_000) + correction;
    // Explicit zero is SCAN=0; SSPD=0 is never used. No integral windup.
    Ok((speed + if speed < 0.0 { -0.5 } else { 0.5 }) as i32)
}

#[cfg(test)]
pub mod test_support {
    use super::*;
    pub struct Io {
        pub targets: Vec<(i32, u32, bool)>,
        pub commands: Vec<Command>,
        pub stops: usize,
        pub fail: bool,
        pub busy: bool,
        pub settings: [i32; 5],
        pub revisions: [u32; 5],
        pub drop_queries: usize,
    }
    impl Default for Io {
        fn default() -> Self {
            Self {
                targets: vec![],
                commands: vec![],
                stops: 0,
                fail: false,
                busy: false,
                settings: [4, 97, 10000, MIN_POSITION, MAX_POSITION],
                revisions: [0; 5],
                drop_queries: 0,
            }
        }
    }
    impl Io {
        pub fn feedback(&self, t: &mut Telemetry, now: u32) {
            for i in 0..5 {
                t.record(SETTING_TAGS[i].as_bytes(), i64::from(self.settings[i]), now);
                t.setting_revisions[i] = self.revisions[i];
            }
        }
    }
    impl Actuator for Io {
        fn tx_idle(&self) -> bool {
            !self.busy
        }
        fn command(&mut self, command: Command) -> Result<bool, &'static str> {
            if self.fail {
                return Err("transmit failure");
            }
            if self.busy {
                return Ok(false);
            }
            if let Some(i) = SETTING_TAGS.iter().position(|&tag| tag == command.tag) {
                if let Some(value) = command.value {
                    self.settings[i] = value;
                } else if matches!(self.settings[INFO], 0 | 2 | 5 | 6) {
                    // Mirror the controller's requested-parameter telemetry slot.
                    if self.drop_queries > 0 {
                        self.drop_queries -= 1;
                    } else {
                        self.revisions[i] = self.revisions[i].wrapping_add(1);
                    }
                }
            }
            self.commands.push(command);
            Ok(true)
        }
        fn target(&mut self, target: i32, now: u32, track: bool) -> Result<(), &'static str> {
            if self.fail {
                return Err("transmit failure");
            }
            self.targets.push((target, now, track));
            Ok(())
        }
        fn stop(&mut self) -> Result<(), &'static str> {
            self.stops += 1;
            if self.fail {
                Err("stop failure")
            } else {
                Ok(())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_support::Io;
    fn config(amplitude_nm: u32, frequency_mhz: u32) -> ExperimentConfig {
        ExperimentConfig::Sinusoidal {
            center_nm: 0,
            amplitude_nm,
            frequency_mhz,
            cycles: 1,
        }
    }
    fn feedback(now: u32, position_um: f64) -> Telemetry {
        let mut t = Telemetry::new();
        t.mounted = true;
        t.cdc = 2;
        t.vid = 0x04d8;
        t.pid = 0xef4a;
        t.record(b"EPOS", (position_um / 0.078).round() as i64, now);
        t.record(b"DPOS", 0, now);
        t.record(b"STAT", 1344, now);
        t
    }
    fn prepare(session: &mut Session, io: &mut Io) -> Result<u32, &'static str> {
        for now in 0..1_000 {
            let mut t = feedback(now, 0.0);
            io.feedback(&mut t, now);
            if session.prepare(config(10_000, 1000), &t, now, io)? {
                return Ok(now);
            }
        }
        panic!("setup did not finish");
    }
    #[test]
    fn commands_fit_controller_lines_and_encode_queries_and_signed_values() {
        for (cmd, expected) in [
            (Command::set("SCAN", -1), "SCAN=-1\n"),
            (Command::set("SSPD", 16_777_215), "SSPD=16777215\n"),
            (Command::set("LLIM", MIN_POSITION), "LLIM=-33554432\n"),
            (
                Command {
                    tag: "INFO",
                    value: None,
                },
                "INFO=?\n",
            ),
        ] {
            let mut buffer = [0; 20];
            let bytes = cmd.encode(&mut buffer);
            assert_eq!(bytes, expected.as_bytes());
            assert!(bytes.len() <= 17);
        }
    }
    #[test]
    fn reported_5um_run_requests_positive_scan_and_rejects_backward_feedback() {
        let c = ExperimentConfig::Sinusoidal {
            center_nm: 374_556,
            amplitude_nm: 5_000,
            frequency_mhz: 1000,
            cycles: 5,
        };
        let start = 62_612;
        let mut t = feedback(start, 0.0);
        t.record(b"EPOS", 4801, start);
        t.record(b"STAT", 1345, start - 4);
        let speed = velocity(c, 0, start, &t).unwrap();
        assert_eq!(speed, 31);
        let mut session = Session::new();
        let mut io = Io::default();
        session.velocity(speed, start, &mut io).unwrap();
        session.service(start + 1, &mut io).unwrap();
        session.service(start + 2, &mut io).unwrap();
        let lines: Vec<_> = io
            .commands
            .iter()
            .map(|command| {
                let mut bytes = [0; 20];
                command.encode(&mut bytes).to_vec()
            })
            .collect();
        assert_eq!(lines, [b"SSPD=31\n".to_vec(), b"SCAN=1\n".to_vec()]);
        // The 10 ms sample was not recorded. Check every encoder value that
        // could pass its guard; none may request a negative direction.
        let (low, high) = scan_bounds(374_556, 5_000).unwrap();
        for count in low..=high {
            t.record(b"EPOS", i64::from(count), start + 10);
            t.record(b"STAT", 8545, start + 10);
            if let Ok(speed) = velocity(c, 10, start + 10, &t) {
                assert!(speed > 0);
            }
        }
        t.record(b"EPOS", 4737, start + 18);
        t.record(b"STAT", 8545, start + 18);
        assert_eq!(velocity(c, 20, start + 20, &t), Err(TRACKING_LIMIT));
    }
    #[test]
    fn setup_verifies_initial_speed_and_preserves_the_original_for_restoration() {
        let mut session = Session::new();
        let mut io = Io::default();
        let original_speed = io.settings[SSPD];
        let now = prepare(&mut session, &mut io).unwrap();
        let speed = scan_start_speed(10_000, 1000);
        assert_eq!(io.settings[SSPD], speed);
        assert_eq!(session.saved[SSPD], Some(original_speed));
        let write = io
            .commands
            .iter()
            .position(|c| *c == Command::set("SSPD", speed))
            .unwrap();
        assert_eq!(
            io.commands[write + 1],
            Command {
                tag: "SSPD",
                value: None
            }
        );
        assert_eq!(io.commands[write + 2], Command::set("INFO", 4));
        assert!(!io.commands.iter().any(|c| c.tag == "SCAN"));
        let before = io.commands.len();
        session.velocity(speed, now + 1, &mut io).unwrap();
        assert_eq!(&io.commands[before..], [Command::set("SCAN", 1)]);
        session.begin_restore(now + 2);
        for tick in now + 3..now + 300 {
            if session
                .restore(&feedback(tick, 0.0), tick, &mut io)
                .unwrap()
            {
                break;
            }
        }
        assert!(!session.changed());
        assert_eq!(io.settings[SSPD], original_speed);
    }
    #[test]
    fn initial_speed_must_have_a_fresh_matching_readback_before_scanning() {
        for lost in [false, true] {
            let mut session = Session::new();
            let mut io = Io::default();
            let speed = scan_start_speed(10_000, 1000);
            let mut fault = None;
            for now in 0..3_000 {
                if io.commands.contains(&Command::set("SSPD", speed)) {
                    if lost {
                        io.drop_queries = usize::MAX;
                    } else {
                        io.settings[SSPD] = 10_000;
                    } // write was not applied
                }
                let mut t = feedback(now, 0.0);
                io.feedback(&mut t, now);
                match session.prepare(config(10_000, 1000), &t, now, &mut io) {
                    Err(e) => {
                        fault = Some(e);
                        break;
                    }
                    Ok(true) => panic!("unverified initial speed permitted scanning"),
                    Ok(false) => {}
                }
            }
            assert_eq!(
                fault,
                Some(if lost {
                    "Scan initial SSPD readback timeout"
                } else {
                    "Controller initial SSPD mismatch"
                })
            );
            assert_eq!(session.saved[SSPD], Some(10_000));
            assert!(!io.commands.iter().any(|c| c.tag == "SCAN"));
        }
    }
    #[test]
    fn speed_updates_are_deduplicated_and_reversals_wait_for_each_transfer() {
        let mut session = Session::new();
        let mut io = Io::default();
        session.velocity(100, 0, &mut io).unwrap();
        assert_eq!(io.commands, [Command::set("SSPD", 100)]);
        io.busy = true;
        session.service(1, &mut io).unwrap();
        assert_eq!(io.commands.len(), 1);
        io.busy = false;
        session.service(2, &mut io).unwrap();
        session.service(3, &mut io).unwrap();
        assert_eq!(io.commands[1], Command::set("SCAN", 1));
        session.velocity(100, 10, &mut io).unwrap();
        assert_eq!(io.commands.len(), 2);
        session.velocity(-50, 20, &mut io).unwrap();
        session.service(21, &mut io).unwrap();
        session.service(22, &mut io).unwrap();
        session.service(23, &mut io).unwrap();
        assert_eq!(
            &io.commands[2..],
            [
                Command::set("SCAN", 0),
                Command::set("SSPD", 50),
                Command::set("SCAN", -1)
            ]
        );
        session.velocity(0, 30, &mut io).unwrap();
        session.service(31, &mut io).unwrap();
        assert_eq!(io.commands.last(), Some(&Command::set("SCAN", 0)));
        assert!(!io.commands.contains(&Command::set("SSPD", 0)));
    }
    #[test]
    fn stalled_or_failed_speed_transfer_cannot_start_scanning() {
        let mut session = Session::new();
        let mut io = Io::default();
        session.velocity(100, u32::MAX - 5, &mut io).unwrap();
        io.busy = true;
        assert_eq!(session.service(4, &mut io), Err("Scan command deadline"));
        assert!(!io.commands.iter().any(|c| c.tag == "SCAN"));
        session.cancel();
        io.busy = false;
        io.fail = true;
        assert_eq!(session.velocity(50, 10, &mut io), Err("transmit failure"));
        assert!(!io.commands.iter().any(|c| c.tag == "SCAN"));
    }
    #[test]
    fn setup_waits_after_usb_completion_and_recovers_a_lost_query_across_clock_wrap() {
        let mut session = Session::new();
        let mut io = Io::default();
        io.drop_queries = 1;
        let start = u32::MAX - 200;
        let mut sent = Vec::new();
        let mut ready = false;
        for offset in 0..1_000 {
            let now = start.wrapping_add(offset);
            // Delay the initial INFO transfer itself, then require its full
            // settling interval. Time spent in flight must not count as settling.
            io.busy = (1..75).contains(&offset);
            let mut t = feedback(now, 0.0);
            io.feedback(&mut t, now);
            let before = io.commands.len();
            ready = session
                .prepare(config(10_000, 1000), &t, now, &mut io)
                .unwrap();
            assert!(io.commands.len() <= before + 1);
            if io.commands.len() > before {
                sent.push((offset, io.commands[before]));
            }
            if ready {
                break;
            }
        }
        assert!(ready);
        assert_eq!(sent[0], (0, Command::set("INFO", 0)));
        assert!(sent[1].0 >= 75 + INFO_SETTLE_MS);
        assert_eq!(
            sent[1].1,
            Command {
                tag: "SSPD",
                value: None
            }
        );
        assert_eq!(sent[2].1, sent[1].1); // retry only the unanswered query
        assert_eq!(sent[2].0 - sent[1].0, QUERY_RETRY_MS);
        for pair in sent.windows(2) {
            assert!(pair[1].0 - pair[0].0 >= SETTING_SETTLE_MS);
        }
        assert_eq!(session.saved[SSPD], Some(10_000));
        assert!(!io.commands.iter().any(|c| c.tag == "SCAN"));
    }
    #[test]
    fn unanswered_query_retries_are_bounded_and_do_not_extend_the_deadline() {
        let mut session = Session::new();
        let mut io = Io::default();
        io.drop_queries = usize::MAX;
        let mut first_query = None;
        let mut fault = None;
        for now in 0..2_000 {
            let mut t = feedback(now, 0.0);
            io.feedback(&mut t, now);
            let result = session.prepare(config(10_000, 1000), &t, now, &mut io);
            if io.commands.len() == 2 && first_query.is_none() {
                first_query = Some(now);
            }
            if let Err(e) = result {
                fault = Some((now, e));
                break;
            }
        }
        assert_eq!(
            fault,
            Some((
                first_query.unwrap() + REPLY_TIMEOUT_MS,
                "Scan SSPD reply timeout"
            ))
        );
        assert_eq!(io.commands.len(), 1 + usize::from(QUERY_ATTEMPTS));
        assert!(io.commands[1..].iter().all(|c| *c
            == Command {
                tag: "SSPD",
                value: None
            }));
        assert_eq!(session.changed, 1 << INFO);
        assert_eq!(session.saved[SSPD], None);
    }
    #[test]
    fn setup_enters_query_mode_from_every_stream_mode_and_restores_telemetry() {
        for mode in 0..=7 {
            let mut session = Session::new();
            let mut io = Io::default();
            io.settings[INFO] = mode;
            let original = io.settings;
            prepare(&mut session, &mut io).unwrap();
            assert_eq!(io.commands[0], Command::set("INFO", 0));
            assert!(!io.commands.contains(&Command {
                tag: "INFO",
                value: None,
            }));
            session.begin_restore(100);
            for now in 100..300 {
                if session.restore(&feedback(now, 0.0), now, &mut io).unwrap() {
                    break;
                }
            }
            assert!(!session.changed());
            assert_eq!(io.settings[INFO], 4);
            assert_eq!(io.settings[1..], original[1..]);
        }
    }
    #[test]
    fn stalled_query_mode_write_faults_before_queries_and_restores_telemetry() {
        let mut session = Session::new();
        let mut io = Io::default();
        session
            .prepare(config(10_000, 1000), &feedback(0, 0.0), 0, &mut io)
            .unwrap();
        io.busy = true;
        assert_eq!(
            session.prepare(
                config(10_000, 1000),
                &feedback(REPLY_TIMEOUT_MS, 0.0),
                REPLY_TIMEOUT_MS,
                &mut io,
            ),
            Err("Scan setting USB transmit timeout")
        );
        assert_eq!(io.commands, [Command::set("INFO", 0)]);
        session.begin_restore(REPLY_TIMEOUT_MS);
        io.busy = false;
        assert_eq!(
            session.restore(&feedback(1501, 0.0), 1501, &mut io),
            Ok(false)
        );
        assert_eq!(
            session.restore(&feedback(1502, 0.0), 1502, &mut io),
            Ok(false)
        );
        assert_eq!(
            session.restore(&feedback(1522, 0.0), 1522, &mut io),
            Ok(true)
        );
        assert_eq!(io.settings[INFO], 4);
    }
    #[test]
    fn setup_verifies_narrow_limits_and_restores_original_settings_after_stop() {
        let mut session = Session::new();
        let mut io = Io::default();
        let original = io.settings;
        prepare(&mut session, &mut io).unwrap();
        assert_eq!(io.settings[LLIM], -160);
        assert_eq!(io.settings[HLIM], 160);
        assert_eq!(io.settings[POLI], 2);
        assert_eq!(io.settings[INFO], 4);
        for tag in ["LLIM", "HLIM", "POLI"] {
            assert!(io.commands.contains(&Command { tag, value: None }));
        }
        session.velocity(100, 100, &mut io).unwrap();
        session.service(101, &mut io).unwrap();
        session.service(102, &mut io).unwrap();
        io.stop().unwrap();
        session.begin_restore(110);
        let writes = io.commands.len();
        let mut t = feedback(111, 0.0);
        t.record(b"STAT", 352 | (1 << 13), 111);
        assert_eq!(session.restore(&t, 111, &mut io), Ok(false));
        assert_eq!(io.commands.len(), writes); // no wider limits while still scanning
        for now in 112..300 {
            if session.restore(&feedback(now, 0.0), now, &mut io).unwrap() {
                break;
            }
        }
        assert!(!session.changed());
        assert_eq!(io.settings, original);
    }
    #[test]
    fn setup_refuses_widening_limits_insufficient_speed_and_unverified_readback() {
        for field in [LLIM, HLIM, SSPD] {
            let mut session = Session::new();
            let mut io = Io::default();
            io.settings[field] = if field == LLIM { -100 } else { 100 };
            if field == SSPD {
                io.settings[field] = 1;
            }
            assert!(prepare(&mut session, &mut io).is_err());
            assert!(!io.commands.iter().any(
                |c| c.tag == "SCAN" || (matches!(c.tag, "LLIM" | "HLIM") && c.value.is_some())
            ));
        }
        let mut session = Session::new();
        let mut io = Io::default();
        let mut error = None;
        for now in 0..1_000 {
            let mut t = feedback(now, 0.0);
            io.feedback(&mut t, now);
            if session.setup_step == 6 {
                t.settings[LLIM].value = -999;
            }
            if let Err(e) = session.prepare(config(10_000, 1000), &t, now, &mut io) {
                error = Some(e);
                break;
            }
        }
        assert_eq!(error, Some("Controller scan setting mismatch"));
        assert!(!io.commands.iter().any(|c| c.tag == "SCAN"));
    }
    #[test]
    fn cached_setting_cannot_satisfy_query_and_restore_timeout_keeps_limits() {
        let mut session = Session::new();
        let mut io = Io::default();
        let mut t = feedback(0, 0.0);
        io.feedback(&mut t, 0);
        session
            .prepare(config(10_000, 1000), &t, 0, &mut io)
            .unwrap();
        session
            .prepare(config(10_000, 1000), &t, 1, &mut io)
            .unwrap();
        session
            .prepare(config(10_000, 1000), &t, 101, &mut io)
            .unwrap();
        session
            .prepare(config(10_000, 1000), &t, 102, &mut io)
            .unwrap();
        assert_eq!(
            session.prepare(config(10_000, 1000), &t, REPLY_TIMEOUT_MS + 102, &mut io),
            Err("Scan SSPD reply timeout")
        );
        assert_eq!(session.changed, 1 << INFO);
        assert!(!io.commands.iter().any(|c| c.tag == "SCAN"));
        let mut session = Session::new();
        prepare(&mut session, &mut io).unwrap();
        session.scan_started = true;
        session.begin_restore(100);
        let narrowed = io.settings;
        assert_eq!(
            session.restore(&feedback(0, 0.0), 2100, &mut io),
            Err("Restore needs fresh post-STOP status")
        );
        assert_eq!(io.settings, narrowed);
    }
    #[test]
    fn restore_reports_the_blocking_condition_without_widening_limits() {
        for (busy, stat, error) in [
            (true, 1344, "Restore USB transmit timeout"),
            (
                false,
                352 | (1 << 13),
                "Restore blocked: controller still scanning",
            ),
            (
                false,
                352,
                "Restore blocked: motor on without position reached",
            ),
        ] {
            let mut session = Session::new();
            let mut io = Io::default();
            let now = prepare(&mut session, &mut io).unwrap();
            session.scan_started = true;
            session.begin_restore(now);
            let commands = io.commands.len();
            let narrowed = io.settings;
            io.busy = busy;
            for elapsed in 1..=RESTORE_TIMEOUT_MS {
                let tick = now + elapsed;
                let mut t = feedback(tick, 0.0);
                t.record(b"STAT", stat, tick);
                assert_eq!(
                    session.restore(&t, tick, &mut io),
                    if elapsed == RESTORE_TIMEOUT_MS {
                        Err(error)
                    } else {
                        Ok(false)
                    }
                );
            }
            assert_eq!(io.settings, narrowed);
            assert_eq!(io.commands.len(), commands);
        }
    }
    #[test]
    fn correction_has_correct_sign_cap_and_sample_time_alignment() {
        let c = config(100_000, 100);
        let nominal = velocity(c, 0, 0, &feedback(0, 0.0)).unwrap();
        let behind = velocity(c, 0, 0, &feedback(0, -1.0)).unwrap();
        let ahead = velocity(c, 0, 0, &feedback(0, 1.0)).unwrap();
        assert!(behind > nominal && ahead < nominal);
        let saturated = velocity(c, 0, 0, &feedback(0, -20.0)).unwrap();
        let expected = (peak_speed(100_000, 100) * 1.25).round() as i32;
        assert_eq!(saturated, expected);
        let elapsed = 150;
        let now: u32 = 15; // reception timestamp wraps, waveform time does not
        let age = 20;
        let reference_then = reference(0, 100_000, 100, elapsed - age);
        let delayed = feedback(now.wrapping_sub(age), reference_then);
        let ideal =
            peak_speed(100_000, 100) * (elapsed as f64 * core::f64::consts::TAU / 10000.0).cos();
        assert_eq!(
            velocity(c, elapsed, now, &delayed),
            Ok(ideal.round() as i32)
        );
    }
    #[test]
    fn stale_feedback_travel_escape_and_large_error_stop_the_controller() {
        let c = config(10_000, 1000);
        assert_eq!(
            velocity(c, 50, 50, &feedback(0, 0.0)),
            Err("Scan feedback stale")
        );
        let mut t = feedback(50, 0.0);
        t.samples[2].ms = 0;
        assert_eq!(velocity(c, 50, 50, &t), Err("Scan feedback stale"));
        assert_eq!(velocity(c, 0, 0, &feedback(0, -3.0)), Err(TRACKING_LIMIT));
        assert_eq!(
            velocity(c, 250, 250, &feedback(250, 13.0)),
            Err(TRAVEL_LIMIT)
        );
        assert_eq!(velocity(c, 250, 250, &feedback(250, 10.0)), Ok(0));
    }
    #[test]
    fn proportional_correction_reduces_error_in_a_biased_velocity_model() {
        // Illustrative plant: actual velocity = 80% of command. This is not a
        // validation of the real stage's dynamics, latency, or tuning.
        let c = config(100_000, 100);
        let (mut corrected, mut uncorrected) = (0.0, 0.0);
        let (mut corrected_error, mut uncorrected_error) = (0.0, 0.0);
        for now in (0..40_000).step_by(10) {
            let command = velocity(c, now, now, &feedback(now, corrected)).unwrap();
            let ideal = 100.0 * (f64::from(now) * core::f64::consts::TAU / 10000.0).sin();
            if now >= 30_000 {
                corrected_error += (corrected - ideal).powi(2);
                uncorrected_error += (uncorrected - ideal).powi(2);
            }
            corrected += f64::from(command) * 0.8 * 0.01;
            uncorrected += (peak_speed(100_000, 100)
                * (f64::from(now) * core::f64::consts::TAU / 10000.0).cos())
            .round()
                * 0.8
                * 0.01;
        }
        assert!(corrected_error < uncorrected_error * 0.25);
    }
}
