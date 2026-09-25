//! LPS22HH: 200 Hz conversion, data-ready polling and a 16-sample average.
//! Register definitions: ST LPS22HH datasheet, sections 7.2 and 9.
use core::task::Poll;
const WINDOW: usize = 16;
pub const ADDRESS: u8 = 0x5c;
pub const WHO_AM_I: u8 = 0xb3;
pub const OUTPUT_HZ: u32 = 200;
const READY_POLL_MS: u32 = 1;
const SAMPLE_TIMEOUT_MS: u32 = 100;
const RETRY_MS: u32 = 1000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Operation {
    Identify,
    Interface,
    Control,
    Configure,
    Verify,
    Status,
    Sample,
}
impl Operation {
    /// (register, optional write value, read length). All reads use repeated START.
    pub fn transaction(self) -> (u8, Option<u8>, usize) {
        match self {
            Self::Identify => (0x0f, None, 1),
            Self::Interface => (0x0e, Some(0x02), 0), // I3C disabled, I2C enabled
            Self::Control => (0x11, Some(0x10), 0),   // automatic address increment
            Self::Configure => (0x10, Some(0x72), 0), // 200 Hz, BDU, no extra low-pass
            Self::Verify => (0x10, None, 2),
            Self::Status => (0x27, None, 1),
            // XL, L, H pressure then L, H temperature: MSB last for each BDU group.
            Self::Sample => (0x28, None, 5),
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Timeout,
    Bus,
}
pub trait Bus {
    /// Advance one register transaction without waiting for hardware.
    fn poll(&mut self, op: Operation, now: u32) -> Poll<Result<[u8; 5], Error>>;
}
#[derive(Clone, Copy, Default)]
pub struct Reading {
    pub raw: i32, // sign-extended 24-bit pressure, 4096 counts/hPa
    pub average: i32,
    pub ms: u32,
    pub samples: u32,
    pub valid: bool,
    // 0 initializing, 1 waiting, 2 receiving, 3 transaction timeout, 4 bus error,
    // 5 wrong device, 6 no fresh conversion, 7 configuration readback mismatch.
    pub state: u8,
    pub errors: u32,
    pub who: Option<u8>,
    pub temperature: i16, // signed 16-bit, 100 counts/degree C
}
pub struct Pressure {
    pub reading: Reading,
    values: [i32; WINDOW],
    sum: i32,
    next: usize,
    phase: Option<Operation>,
    last_attempt: u32,
    last_sample: u32,
    retry: bool,
}
impl Pressure {
    pub fn new() -> Self {
        Self {
            reading: Reading::default(),
            values: [0; WINDOW],
            sum: 0,
            next: 0,
            phase: Some(Operation::Identify),
            last_attempt: 0,
            last_sample: 0,
            retry: false,
        }
    }
    /// Keep servicing the foreground while a transaction or setup step is pending.
    /// Sleeping until SysTick between bytes would throttle the four-entry I2C FIFO.
    pub fn busy(&self) -> bool {
        self.phase.is_some()
    }
    fn fail(&mut self, state: u8, now: u32) {
        self.reading.valid = false;
        self.reading.state = state;
        self.reading.errors = self.reading.errors.wrapping_add(1);
        self.values.fill(0);
        self.sum = 0;
        self.next = 0;
        self.phase = None;
        self.retry = true;
        self.last_attempt = now;
    }
    pub fn poll(&mut self, now: u32, bus: &mut impl Bus) {
        use Operation::*;
        if self.phase.is_none() {
            let interval = if self.retry { RETRY_MS } else { READY_POLL_MS };
            if now.wrapping_sub(self.last_attempt) < interval {
                return;
            }
            self.last_attempt = now;
            self.phase = Some(if self.retry {
                self.reading.who = None;
                Identify
            } else {
                Status
            });
        }
        let op = self.phase.unwrap();
        let bytes = match bus.poll(op, now) {
            Poll::Pending => return,
            Poll::Ready(Ok(bytes)) => bytes,
            Poll::Ready(Err(error)) => {
                self.reading.who = None;
                self.fail(if error == Error::Timeout { 3 } else { 4 }, now);
                return;
            }
        };
        self.phase = match op {
            Identify => {
                self.reading.who = Some(bytes[0]);
                if bytes[0] != WHO_AM_I {
                    self.fail(5, now);
                    return;
                }
                Some(Interface)
            }
            Interface => Some(Control),
            Control => Some(Configure),
            Configure => Some(Verify),
            Verify => {
                if bytes[..2] != [0x72, 0x10] {
                    self.fail(7, now);
                    return;
                }
                self.reading.state = 1;
                self.retry = false;
                self.last_attempt = now;
                self.last_sample = now;
                None
            }
            Status => {
                if now.wrapping_sub(self.last_sample) >= SAMPLE_TIMEOUT_MS {
                    self.fail(6, now);
                    return;
                }
                // Never read/clear the output registers or refresh timestamps until
                // both conversions are ready. Identical *fresh* values still count.
                (bytes[0] & 0x03 == 0x03).then_some(Sample)
            }
            Sample => {
                let raw = i32::from_le_bytes([0, bytes[0], bytes[1], bytes[2]]) >> 8;
                self.sum -= self.values[self.next];
                self.values[self.next] = raw;
                self.sum += raw;
                self.next = (self.next + 1) % WINDOW;
                self.reading.raw = raw;
                self.reading.average = self.sum / WINDOW as i32;
                self.reading.temperature = i16::from_le_bytes([bytes[3], bytes[4]]);
                self.reading.ms = now;
                self.last_sample = now;
                self.reading.valid = true;
                self.reading.state = 2;
                self.reading.samples = self.reading.samples.wrapping_add(1);
                None
            }
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    struct Fake(VecDeque<(Operation, Poll<Result<[u8; 5], Error>>)>);
    impl Bus for Fake {
        fn poll(&mut self, op: Operation, _: u32) -> Poll<Result<[u8; 5], Error>> {
            let (expected, result) = self.0.pop_front().expect("unexpected bus operation");
            assert_eq!(op, expected);
            result
        }
    }
    fn complete(p: &mut Pressure, op: Operation, bytes: [u8; 5], now: u32) {
        let mut bus = Fake([(op, Poll::Pending), (op, Poll::Ready(Ok(bytes)))].into());
        p.poll(now, &mut bus);
        assert!(p.busy());
        p.poll(now, &mut bus);
        assert!(bus.0.is_empty());
    }
    fn initialize(p: &mut Pressure, now: u32) {
        complete(p, Operation::Identify, [WHO_AM_I, 0, 0, 0, 0], now);
        for op in [
            Operation::Interface,
            Operation::Control,
            Operation::Configure,
        ] {
            complete(p, op, [0; 5], now);
        }
        complete(p, Operation::Verify, [0x72, 0x10, 0, 0, 0], now);
        assert_eq!(p.reading.who, Some(WHO_AM_I));
        assert!(!p.busy());
    }
    fn sample(p: &mut Pressure, now: u32, bytes: [u8; 5]) {
        complete(p, Operation::Status, [3, 0, 0, 0, 0], now);
        complete(p, Operation::Sample, bytes, now);
    }
    #[test]
    fn acquires_200_fresh_samples_per_second_without_duplicate_publication() {
        let mut p = Pressure::new();
        initialize(&mut p, 0);
        for now in 1..=1000 {
            if now % (1000 / OUTPUT_HZ) == 0 {
                sample(&mut p, now, [0x00, 0x80, 0x3e, 0xc4, 0x09]); // 1000 hPa, 25 C
            } else {
                complete(&mut p, Operation::Status, [0; 5], now);
            }
            assert_eq!(p.reading.samples, now / 5);
            assert_eq!(p.reading.ms, now / 5 * 5);
            assert!(!p.busy());
            p.poll(now, &mut Fake(VecDeque::new())); // no extra poll within the same ms
        }
        assert_eq!(p.reading.raw, 1000 * 4096);
        assert_eq!(p.reading.average, p.reading.raw);
        assert_eq!(p.reading.temperature, 2500);
    }
    #[test]
    fn signed_registers_and_zero_filled_average() {
        let mut p = Pressure::new();
        initialize(&mut p, 0);
        sample(&mut p, 5, [0xff, 0xff, 0x7f, 0xff, 0x7f]);
        assert_eq!(p.reading.raw, 8_388_607);
        assert_eq!(p.reading.average, 8_388_607 / 16);
        assert_eq!(p.reading.temperature, i16::MAX);
        sample(&mut p, 10, [0, 0, 0x80, 0, 0x80]);
        assert_eq!(p.reading.raw, -8_388_608);
        assert_eq!(p.reading.temperature, i16::MIN);
        for now in (15..=90).step_by(5) {
            sample(&mut p, now, [0xf0, 0xff, 0xff, 0x3c, 0xf6]);
        }
        assert_eq!(p.reading.raw, -16);
        assert_eq!(p.reading.average, -16);
        assert_eq!(p.reading.temperature, -2500);
    }
    #[test]
    fn wrong_identity_and_bad_configuration_prevent_sampling() {
        let mut p = Pressure::new();
        complete(&mut p, Operation::Identify, [0xb1, 0, 0, 0, 0], 0);
        assert_eq!(p.reading.who, Some(0xb1));
        assert_eq!(p.reading.state, 5);
        assert!(!p.reading.valid);
        p.poll(999, &mut Fake(VecDeque::new()));
        initialize(&mut p, 1000);
        let mut p = Pressure::new();
        complete(&mut p, Operation::Identify, [WHO_AM_I, 0, 0, 0, 0], 0);
        for op in [
            Operation::Interface,
            Operation::Control,
            Operation::Configure,
        ] {
            complete(&mut p, op, [0; 5], 0);
        }
        complete(&mut p, Operation::Verify, [0x70, 0x10, 0, 0, 0], 0);
        assert_eq!(p.reading.state, 7);
        assert!(!p.reading.valid);
        assert!(!p.busy());
    }
    #[test]
    fn missing_conversions_do_not_refresh_the_last_sample() {
        let mut p = Pressure::new();
        initialize(&mut p, 0);
        sample(&mut p, 5, [16, 0, 0, 0, 0]);
        for (now, status) in [(6, 0), (7, 1), (8, 2), (104, 0), (105, 0)] {
            complete(&mut p, Operation::Status, [status, 0, 0, 0, 0], now);
        }
        assert_eq!(p.reading.ms, 5);
        assert_eq!(p.reading.samples, 1);
        assert_eq!(p.reading.state, 6);
        assert!(!p.reading.valid);
        assert!(!p.busy());
    }
    #[test]
    fn failures_invalidate_retry_and_reinitialize_across_timer_wrap() {
        for error in [Error::Timeout, Error::Bus] {
            let mut p = Pressure::new();
            let start = u32::MAX - 10;
            initialize(&mut p, start);
            sample(&mut p, start + 5, [16, 0, 0, 0, 0]);
            complete(&mut p, Operation::Status, [3, 0, 0, 0, 0], start + 10);
            p.poll(
                start + 10,
                &mut Fake([(Operation::Sample, Poll::Ready(Err(error)))].into()),
            );
            assert!(!p.reading.valid);
            assert_eq!(p.reading.who, None);
            assert_eq!(p.reading.state, if error == Error::Timeout { 3 } else { 4 });
            assert_eq!(p.reading.errors, 1);
            p.poll(998, &mut Fake(VecDeque::new()));
            initialize(&mut p, 999);
            sample(&mut p, 1004, [32, 0, 0, 0, 0]);
            assert_eq!(p.reading.average, 2);
            assert!(p.reading.valid);
        }
    }
}
