//! Bounded manual XD-OEM pulse tests, independent of the sinusoidal trajectory.
//! Each compare is scheduled at least HALF_US after the physical output write.
//! Thus IRQ latency can only slow the train; it never causes catch-up edges.
use crate::trajectory::Status;

pub const HALF_US: u32 = 500; // <= 1 kHz, below the manual's recommended 20 kHz
pub const MAX_STEPS: u32 = 100; // 7.8 um with this setup's 78 nm encoder

pub struct Burst {
    remaining: u32,
    pub high: bool,
    pub status: Status,
}

impl Burst {
    pub fn new(steps: u32) -> Result<Self, &'static str> {
        if !(1..=MAX_STEPS).contains(&steps) {
            return Err("GPIO steps must be 1..100");
        }
        Ok(Self {
            remaining: steps,
            high: false,
            status: Status::Running,
        })
    }

    pub fn tick(&mut self) {
        if self.status != Status::Running {
            return;
        }
        if self.high {
            self.high = false;
        } else if self.remaining == 0 {
            // One final low half-period before disabling ENABLE / changing DIR.
            self.status = Status::Complete;
        } else {
            self.remaining -= 1;
            self.high = true;
        }
    }

    pub fn stop(&mut self, status: Status) {
        self.high = false;
        self.status = status;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounded_bursts_have_exact_edges_and_a_final_hold_interval() {
        assert!(Burst::new(0).is_err());
        assert!(Burst::new(MAX_STEPS + 1).is_err());
        assert!(2 * HALF_US >= 50);
        for count in 1..=MAX_STEPS {
            let mut b = Burst::new(count).unwrap();
            let mut rises = 0;
            for _ in 0..count {
                b.tick();
                assert!(b.high);
                rises += 1;
                b.tick();
                assert!(!b.high);
                assert_eq!(b.status, Status::Running);
            }
            b.tick();
            assert_eq!(b.status, Status::Complete);
            b.tick();
            assert!(!b.high);
            assert_eq!(rises, count);
        }
    }

    #[test]
    fn stop_or_fault_cannot_emit_another_edge() {
        for status in [Status::Stopped, Status::Fault] {
            let mut b = Burst::new(100).unwrap();
            b.tick();
            b.stop(status);
            b.tick();
            assert!(!b.high);
            assert_eq!(b.status, status);
        }
    }
}
