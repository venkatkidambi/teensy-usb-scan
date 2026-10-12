//! Bounded manual XD-OEM pulse tests, independent of the sinusoidal trajectory.
//! Each compare is scheduled at least HALF_US after the physical output write.
//! Thus IRQ latency can only slow the train; it never causes catch-up edges.
use crate::trajectory::Status;

pub const HALF_US: u32 = 500; // <= 1 kHz, below the manual's recommended 20 kHz
pub const MAX_STEPS: u32 = 100; // 7.8 um with this setup's 78 nm encoder

/// GPIO increments the existing target. A settled encoder need not equal it:
/// XD-OEM PTOL / PTO2 define the controller's accepted positioning error.
/// Keep an independent 100-count offset ceiling even with permissive tuning.
pub fn target(
    t: &crate::protocol::Telemetry,
    positive: bool,
    steps: u32,
    now: u32,
    since: u32,
    generations: [u32; 2],
) -> Result<i32, &'static str> {
    use crate::xeryon_motion as motion;
    if !(1..=MAX_STEPS).contains(&steps) {
        return Err("GPIO steps must be 1..100");
    }
    motion::target(t, positive, steps * 78, now)?;
    let after = |s: crate::protocol::Sample| {
        let age = s.ms.wrapping_sub(since);
        s.valid && age > 0 && age < 0x8000_0000 && now.wrapping_sub(s.ms) < 3000
    };
    if !t.samples[..3].iter().copied().all(after) {
        return Err("GPIO baseline feedback stale");
    }
    let mut tolerance = 0;
    for (i, generation) in generations.into_iter().enumerate() {
        let sample = t.positioning[i];
        if !after(sample) || t.positioning_updates[i] == generation {
            return Err("GPIO positioning tolerance readback missing or stale");
        }
        if !(0..=65535).contains(&sample.value) {
            return Err("Invalid GPIO positioning tolerance");
        }
        tolerance = tolerance.max(sample.value);
    }
    let dpos = t.samples[1].value;
    if !(i64::from(motion::MIN_POSITION)..=i64::from(motion::MAX_POSITION)).contains(&dpos) {
        return Err("GPIO DPOS out of range");
    }
    let gap = (dpos - t.samples[0].value).abs();
    if gap > i64::from(MAX_STEPS) {
        return Err("GPIO target offset exceeds 100 counts; check position using USB control");
    }
    if gap > tolerance {
        return Err("GPIO target offset exceeds PTOL/PTO2; settle using USB control first");
    }
    let target = dpos
        + if positive {
            i64::from(steps)
        } else {
            -i64::from(steps)
        };
    if !(i64::from(motion::MIN_POSITION)..=i64::from(motion::MAX_POSITION)).contains(&target) {
        return Err("GPIO target out of range");
    }
    Ok(target as i32)
}

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

    fn baseline() -> crate::protocol::Telemetry {
        let mut t = crate::protocol::Telemetry::new();
        t.mounted = true;
        t.cdc = 2;
        t.vid = 0x04d8;
        t.pid = 0xef4a;
        // Reported hardware case: EPOS 335.010 um, DPOS 334.776 um.
        for (tag, value) in [
            (b"EPOS", 4295),
            (b"DPOS", 4292),
            (b"STAT", 8513),
            (b"PTOL", 2),
            (b"PTO2", 10),
        ] {
            t.record(tag, value, 101);
        }
        t
    }

    #[test]
    fn three_count_settled_offset_uses_reported_tolerance_and_preserves_target() {
        let mut t = baseline();
        assert_eq!(target(&t, true, 10, 102, 100, [0; 2]), Ok(4302));
        assert_eq!(target(&t, false, 10, 102, 100, [0; 2]), Ok(4282));
        // PTOL may be the larger setting; both values are controller readbacks.
        t.record(b"PTOL", 3, 101);
        t.record(b"PTO2", 2, 101);
        assert!(target(&t, true, 10, 102, 100, [0; 2]).is_ok());
        t.record(b"PTOL", 2, 101);
        assert!(target(&t, true, 10, 102, 100, [0; 2]).is_err());
    }

    #[test]
    fn tolerance_preflight_rejects_stale_invalid_and_distant_baselines() {
        let t = baseline();
        for i in 0..2 {
            let mut invalid = t;
            invalid.positioning[i].valid = false;
            assert!(target(&invalid, true, 10, 102, 100, [0; 2]).is_err());
            invalid = t;
            invalid.positioning[i].ms = 100;
            assert!(target(&invalid, true, 10, 102, 100, [0; 2]).is_err());
            invalid = t;
            for value in [-1, 65536] {
                invalid.positioning[i].value = value;
                assert!(target(&invalid, true, 10, 102, 100, [0; 2]).is_err());
            }
            let mut generations = [0; 2];
            generations[i] = t.positioning_updates[i];
            assert!(target(&t, true, 10, 102, 100, generations).is_err());
        }
        let mut distant = t;
        distant.record(b"PTO2", 65535, 101);
        distant.record(b"DPOS", 4396, 101); // 101 counts: independent safety cap
        assert!(target(&distant, true, 10, 102, 100, [0; 2]).is_err());
        distant.record(b"DPOS", i64::MIN, 101);
        assert!(target(&distant, true, 10, 102, 100, [0; 2]).is_err());
        let mut stale = t;
        stale.samples[1].ms = 100;
        assert!(target(&stale, true, 10, 102, 100, [0; 2]).is_err());
        assert!(target(&t, true, 10, 4000, 100, [0; 2]).is_err());
        let mut wrapped = t;
        for sample in &mut wrapped.samples[..3] {
            sample.ms = 1;
        }
        for sample in &mut wrapped.positioning[..2] {
            sample.ms = 1;
        }
        assert_eq!(target(&wrapped, true, 10, 2, u32::MAX, [0; 2]), Ok(4302));
    }

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
