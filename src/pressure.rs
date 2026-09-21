//! NanoManipulator Pressure.cpp register protocol and zero-filled 16-sample average.
use core::task::Poll;
const WINDOW: usize = 16;
pub const ADDRESS: u8 = 0x5c;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Operation {
    Configure,
    WhoPointer,
    WhoRead,
    PressurePointer,
    PressureRead,
    TemperaturePointer,
    TemperatureRead,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Timeout,
    Bus,
}
pub trait Bus {
    /// Poll one STOP-terminated transaction. Never wait for hardware in this call.
    fn poll(&mut self, op: Operation, now: u32) -> Poll<Result<[u8; 3], Error>>;
}
#[derive(Clone, Copy, Default)]
pub struct Reading {
    pub raw: i32,
    pub average: i32,
    pub ms: u32,
    pub samples: u32,
    pub valid: bool,
    pub saturated: bool, // Retained for protocol compatibility; unused for this sensor.
    // 0 initializing, 1 waiting for first sample, 2 receiving, 3 timeout, 4 bus error.
    pub state: u8,
    pub errors: u32,
    pub who: Option<u8>,
    pub temperature: u16, // Preserve Arduino's unsigned two-byte raw value.
}
pub struct Pressure {
    pub reading: Reading,
    values: [i32; WINDOW],
    sum: i32,
    next: usize,
    phase: Option<Operation>,
    last_attempt: u32,
    retry: bool,
    raw: i32,
}
impl Pressure {
    pub fn new() -> Self {
        Self {
            reading: Reading::default(),
            values: [0; WINDOW],
            sum: 0,
            next: 0,
            phase: Some(Operation::Configure),
            last_attempt: 0,
            retry: false,
            raw: 0,
        }
    }
    pub fn poll(&mut self, now: u32, bus: &mut impl Bus) {
        use Operation::*;
        if self.phase.is_none() {
            let interval = if self.retry { 1000 } else { 50 };
            if now.wrapping_sub(self.last_attempt) < interval {
                return;
            }
            self.last_attempt = now;
            self.phase = Some(if self.retry {
                Configure
            } else {
                PressurePointer
            });
        }
        let op = self.phase.unwrap();
        let bytes = match bus.poll(op, now) {
            Poll::Pending => return,
            Poll::Ready(Ok(bytes)) => bytes,
            Poll::Ready(Err(error)) => {
                self.reading.valid = false;
                self.reading.who = None;
                self.reading.state = if error == Error::Timeout { 3 } else { 4 };
                self.reading.errors = self.reading.errors.wrapping_add(1);
                self.values.fill(0);
                self.sum = 0;
                self.next = 0;
                self.phase = None;
                self.retry = true;
                self.last_attempt = now;
                return;
            }
        };
        self.phase = match op {
            Configure => Some(WhoPointer),
            WhoPointer => Some(WhoRead),
            WhoRead => {
                self.reading.who = Some(bytes[0]);
                self.reading.state = 1;
                self.retry = false;
                self.last_attempt = now;
                None
            }
            PressurePointer => Some(PressureRead),
            PressureRead => {
                self.raw = i32::from_le_bytes([bytes[0], bytes[1], bytes[2], 0]);
                Some(TemperaturePointer)
            }
            TemperaturePointer => Some(TemperatureRead),
            TemperatureRead => {
                self.sum -= self.values[self.next];
                self.values[self.next] = self.raw;
                self.sum += self.raw;
                self.next = (self.next + 1) % WINDOW;
                self.reading.raw = self.raw;
                self.reading.average = self.sum / WINDOW as i32;
                self.reading.temperature = u16::from_le_bytes([bytes[0], bytes[1]]);
                self.reading.ms = now;
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
    struct Fake(VecDeque<(Operation, Poll<Result<[u8; 3], Error>>)>);
    impl Bus for Fake {
        fn poll(&mut self, op: Operation, _: u32) -> Poll<Result<[u8; 3], Error>> {
            let (expected, result) = self.0.pop_front().expect("unexpected bus operation");
            assert_eq!(op, expected);
            result
        }
    }
    fn complete(p: &mut Pressure, op: Operation, bytes: [u8; 3], now: u32) {
        let mut bus = Fake([(op, Poll::Pending), (op, Poll::Ready(Ok(bytes)))].into());
        p.poll(now, &mut bus);
        p.poll(now, &mut bus);
        assert!(bus.0.is_empty());
    }
    fn initialize(p: &mut Pressure, now: u32) {
        complete(p, Operation::Configure, [0; 3], now);
        complete(p, Operation::WhoPointer, [0; 3], now);
        complete(p, Operation::WhoRead, [0xb1, 0, 0], now);
        assert_eq!(p.reading.who, Some(0xb1));
    }
    fn sample(p: &mut Pressure, now: u32, bytes: [u8; 3]) {
        complete(p, Operation::PressurePointer, [0; 3], now);
        complete(p, Operation::PressureRead, bytes, now);
        complete(p, Operation::TemperaturePointer, [0; 3], now);
        complete(p, Operation::TemperatureRead, [0x34, 0x92, 0], now);
    }
    #[test]
    fn matches_arduino_unsigned_bytes_zero_filled_average_and_cadence() {
        let mut p = Pressure::new();
        initialize(&mut p, 0);
        p.poll(49, &mut Fake(VecDeque::new()));
        sample(&mut p, 50, [0x56, 0x34, 0x92]);
        assert_eq!(p.reading.raw, 0x923456);
        assert_eq!(p.reading.average, 0x923456 / 16);
        assert_eq!(p.reading.temperature, 0x9234);
        for now in (100..=800).step_by(50) {
            sample(&mut p, now, [0x56, 0x34, 0x92]);
        }
        assert_eq!(p.reading.average, 0x923456);
        sample(&mut p, 850, [0xff; 3]);
        assert_eq!(p.reading.average, (15 * 0x923456 + 0xffffff) / 16);
        assert!(p.reading.valid);
        assert_eq!(p.reading.samples, 17);
    }
    #[test]
    fn failures_invalidate_retry_and_reinitialize_across_timer_wrap() {
        for error in [Error::Timeout, Error::Bus] {
            let mut p = Pressure::new();
            let start = u32::MAX - 100;
            initialize(&mut p, start);
            sample(&mut p, start + 50, [16, 0, 0]);
            p.poll(
                start + 100,
                &mut Fake([(Operation::PressurePointer, Poll::Ready(Err(error)))].into()),
            );
            assert!(!p.reading.valid);
            assert_eq!(p.reading.who, None);
            assert_eq!(p.reading.state, if error == Error::Timeout { 3 } else { 4 });
            assert_eq!(p.reading.errors, 1);
            p.poll(998, &mut Fake(VecDeque::new()));
            initialize(&mut p, 999);
            sample(&mut p, 1049, [32, 0, 0]);
            assert_eq!(p.reading.average, 2);
            assert!(p.reading.valid);
        }
    }
}
