//! HX711 acquisition state and a constant-time, 16-sample moving average.
//! Readings retain the sign inversion from HX711_meas_1.ino; units are ADC counts.
const WINDOW: usize = 16;
const TIMEOUT_MS: u32 = 1000;
const SETTLE_MS: u32 = 400; // Worst-case channel-change settling at RATE=0.

pub trait Bus {
    fn ready(&self) -> bool;
    /// Exactly one available conversion; None indicates a framing/wiring fault.
    fn read(&mut self) -> Option<u32>;
}

/// Shift one MSB-first conversion and select B/32 for the next one.
pub fn read_channel_b(mut pulse: impl FnMut() -> bool) -> u32 {
    let mut word = 0;
    for _ in 0..24 {
        word = (word << 1) | u32::from(pulse());
    }
    pulse();
    pulse(); // 26 clocks total, channel B / gain 32.
    word
}

pub fn signed_inverted(word: u32) -> i32 {
    -(((word << 8) as i32) >> 8)
}

#[derive(Clone, Copy, Default)]
pub struct Reading {
    pub raw: i32,
    pub average: i32,
    pub ms: u32,
    pub samples: u32,
    pub valid: bool,
    pub saturated: bool,
}

pub struct Pressure {
    pub reading: Reading,
    values: [i32; WINDOW],
    sum: i32,
    next: usize,
    count: usize,
    configured: bool,
    selected_at: u32,
    last_rx: u32,
}
impl Pressure {
    pub fn new() -> Self {
        Self {
            reading: Reading::default(),
            values: [0; WINDOW],
            sum: 0,
            next: 0,
            count: 0,
            configured: false,
            selected_at: 0,
            last_rx: 0,
        }
    }
    fn invalidate(&mut self) {
        self.reading.valid = false;
        self.configured = false;
        self.values.fill(0);
        self.sum = 0;
        self.next = 0;
        self.count = 0;
    }
    pub fn poll(&mut self, now: u32, bus: &mut impl Bus) {
        if self.configured && now.wrapping_sub(self.last_rx) >= TIMEOUT_MS {
            self.invalidate();
        }
        if !bus.ready() {
            return;
        }
        let Some(word) = bus.read() else {
            self.invalidate();
            return;
        };
        self.last_rx = now;
        if !self.configured {
            // The first result may be channel A after power-up. Reading with
            // 26 clocks selects B/32 for the NEXT conversion, so discard this one.
            self.configured = true;
            self.selected_at = now;
            return;
        }
        if now.wrapping_sub(self.selected_at) < SETTLE_MS {
            return;
        }
        let raw = signed_inverted(word);
        self.sum -= self.values[self.next];
        self.values[self.next] = raw;
        self.sum += raw;
        self.next = (self.next + 1) % WINDOW;
        self.count = (self.count + 1).min(WINDOW);
        self.reading = Reading {
            raw,
            average: self.sum / self.count as i32,
            ms: now,
            samples: self.reading.samples.wrapping_add(1),
            valid: true,
            saturated: matches!(word & 0xffffff, 0x800000 | 0x7fffff),
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    struct Fake {
        ready: bool,
        word: Option<u32>,
        reads: usize,
    }
    impl Bus for Fake {
        fn ready(&self) -> bool {
            self.ready
        }
        fn read(&mut self) -> Option<u32> {
            self.reads += 1;
            self.word
        }
    }
    #[test]
    fn wire_bit_order_and_gain_selection() {
        for expected in [0, 1, 0x800000, 0x7fffff, 0xffffff, 0x123456] {
            let mut pulses = 0;
            let word = read_channel_b(|| {
                let bit = if pulses < 24 {
                    expected & (1 << (23 - pulses)) != 0
                } else {
                    true
                };
                pulses += 1;
                bit
            });
            assert_eq!(word, expected);
            assert_eq!(pulses, 26);
        }
    }
    #[test]
    fn sign_and_range() {
        assert_eq!(signed_inverted(0), 0);
        assert_eq!(signed_inverted(1), -1);
        assert_eq!(signed_inverted(0xffffff), 1);
        assert_eq!(signed_inverted(0x800000), 8388608);
        assert_eq!(signed_inverted(0x7fffff), -8388607);
    }
    #[test]
    fn ready_settling_average_and_recovery() {
        let mut p = Pressure::new();
        let mut bus = Fake {
            ready: false,
            word: Some(100),
            reads: 0,
        };
        p.poll(0, &mut bus);
        assert_eq!(bus.reads, 0);
        bus.ready = true;
        p.poll(1, &mut bus);
        assert!(!p.reading.valid);
        p.poll(400, &mut bus);
        assert!(!p.reading.valid);
        p.poll(401, &mut bus);
        assert_eq!(p.reading.average, -100);
        bus.word = Some(200);
        p.poll(501, &mut bus);
        assert_eq!(p.reading.average, -150);
        for now in (601..=2101).step_by(100) {
            p.poll(now, &mut bus);
        }
        assert_eq!(p.reading.average, -200);
        bus.ready = false;
        p.poll(3101, &mut bus);
        assert!(!p.reading.valid);
        bus.ready = true;
        bus.word = Some(0xffffff);
        p.poll(3201, &mut bus);
        assert!(!p.reading.valid);
        p.poll(3601, &mut bus);
        assert_eq!(p.reading.average, 1);
        bus.word = None;
        p.poll(3701, &mut bus);
        assert!(!p.reading.valid);
    }
    #[test]
    fn rollover_and_saturation() {
        let mut p = Pressure::new();
        let mut bus = Fake {
            ready: true,
            word: Some(0x800000),
            reads: 0,
        };
        p.poll(u32::MAX - 200, &mut bus);
        p.poll(200, &mut bus);
        assert!(p.reading.valid && p.reading.saturated);
        assert_eq!(p.reading.average, 8388608);
        for now in (300..=2000).step_by(100) {
            p.poll(now, &mut bus);
        }
        assert_eq!(p.reading.average, 8388608); // Sum fits i32 even at full scale.
    }
}
