//! Bounded manual XD-OEM pulse tests, independent of the sinusoidal trajectory.
//! Each compare is scheduled at least HALF_US after the physical output write.
//! Thus IRQ latency can only slow the train; it never causes catch-up edges.
use crate::trajectory::Status;

pub const HALF_US: u32 = 500; // <= 1 kHz, below the manual's recommended 20 kHz
pub const MAX_STEPS: u32 = 100; // 7.8 um with this setup's 78 nm encoder

/// A stopped controller can retain a target that differs from the encoder.
/// Capture a fresh, stationary encoder reference before setting DPOS to it with
/// GPIO disabled. This is preparation only; the requested increment uses pulses.
pub fn position(t: &crate::protocol::Telemetry, now: u32, since: u32) -> Result<i32, &'static str> {
    use crate::xeryon_motion as motion;
    if !motion::ready(t) {
        return Err("Xeryon is not connected");
    }
    let after = |s: crate::protocol::Sample| {
        let age = s.ms.wrapping_sub(since);
        s.valid && age > 0 && age < 0x8000_0000 && now.wrapping_sub(s.ms) < 250
    };
    if !t.samples[..3].iter().copied().all(after) {
        return Err("GPIO baseline feedback stale");
    }
    if let Some(reason) = motion::controller_fault(t.samples[2].value) {
        return Err(reason);
    }
    if motion::moving(t.samples[2].value) {
        return Err("Xeryon is already moving");
    }
    for sample in &t.samples[..2] {
        if !(i64::from(motion::MIN_POSITION)..=i64::from(motion::MAX_POSITION))
            .contains(&sample.value)
        {
            return Err("GPIO position out of range");
        }
    }
    Ok(t.samples[0].value as i32)
}

/// Verify the new reference before ENABLE is asserted. Tuning tolerances are
/// completion criteria, not a prerequisite that the old target already match.
/// Bound the entire target path, including preparation drift, to 100 counts
/// from the latest encoder reading (7.8 um with this setup's 78 nm encoder).
pub fn target(
    t: &crate::protocol::Telemetry,
    positive: bool,
    steps: u32,
    now: u32,
    since: u32,
    origin: i32,
) -> Result<i32, &'static str> {
    use crate::xeryon_motion as motion;
    if !(1..=MAX_STEPS).contains(&steps) {
        return Err("GPIO steps must be 1..100");
    }
    let epos = i64::from(position(t, now, since)?);
    if t.samples[1].value != i64::from(origin) {
        return Err("GPIO reference DPOS readback mismatch");
    }
    if t.samples[2].value & (1 << 10) == 0 {
        return Err("GPIO reference has not reached position");
    }
    let target = i64::from(origin)
        + if positive {
            i64::from(steps)
        } else {
            -i64::from(steps)
        };
    if !(i64::from(motion::MIN_POSITION)..=i64::from(motion::MAX_POSITION)).contains(&target) {
        return Err("GPIO target out of range");
    }
    if (i64::from(origin) - epos).abs() > i64::from(MAX_STEPS)
        || (target - epos).abs() > i64::from(MAX_STEPS)
    {
        return Err("GPIO reference drift exceeds test travel limit; retry with fewer pulses");
    }
    if (positive && target <= epos) || (!positive && target >= epos) {
        return Err("GPIO reference drift opposes requested direction; check encoder feedback");
    }
    Ok(target as i32)
}

/// A DPOS sample can still describe the previous target after the last pulse.
/// Keep polling until fresh feedback confirms the requested target and the
/// controller's position-reached flag. The coordinator bounds this wait.
pub fn completed(t: &crate::protocol::Telemetry, expected: i32, now: u32, since: u32) -> bool {
    position(t, now, since).is_ok()
        && t.samples[1].value == i64::from(expected)
        && t.samples[2].value & (1 << 10) != 0
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
            // One final low half-period before reporting pulse completion.
            // The driver then holds ENABLE / DIR until feedback settles or Stop.
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
    fn stopped_offset_is_synchronized_before_pulses_even_with_tight_tuning() {
        let mut t = baseline();
        // Both settings smaller than the reported three-count gap. Neither
        // setting is changed or used to waive the preparation readback.
        t.record(b"PTOL", 1, 101);
        t.record(b"PTO2", 2, 101);
        let origin = position(&t, 102, 100).unwrap();
        assert_eq!(origin, 4295); // measured position, not retained DPOS 4292
        assert!(target(&t, true, 10, 102, 100, origin).is_err());
        // Post-command EPOS / DPOS / STAT must all arrive after preparation.
        t.record(b"DPOS", i64::from(origin), 104);
        assert!(target(&t, true, 10, 105, 102, origin).is_err());
        t.record(b"EPOS", i64::from(origin), 104);
        t.record(b"STAT", 1344, 104);
        assert_eq!(target(&t, true, 10, 105, 102, origin), Ok(4305));
        assert_eq!(target(&t, false, 10, 105, 102, origin), Ok(4285));
        // Preparation must settle; motor-on without reached blocks ENABLE.
        t.record(b"STAT", 1 << 5, 104);
        assert!(target(&t, true, 10, 105, 102, origin).is_err());
    }

    #[test]
    fn preparation_rejects_stale_faulted_invalid_or_drifting_feedback() {
        let mut t = baseline();
        t.record(b"DPOS", 4295, 101);
        t.record(b"STAT", 1344, 101);
        for index in 0..3 {
            let mut stale = t;
            stale.samples[index].ms = 100;
            assert!(position(&stale, 102, 100).is_err());
            stale.samples[index].valid = false;
            assert!(position(&stale, 102, 100).is_err());
        }
        assert!(position(&t, 351, 100).is_err());
        let mut invalid = t;
        invalid.record(b"DPOS", i64::MIN, 101);
        assert!(position(&invalid, 102, 100).is_err());
        invalid = t;
        invalid.record(b"STAT", 1 << 12, 101);
        assert!(position(&invalid, 102, 100).is_err());
        // Range, direction, and total travel include the measured drift.
        invalid = t;
        invalid.record(b"EPOS", 4306, 101);
        assert!(target(&invalid, true, 10, 102, 100, 4295).is_err());
        invalid.record(b"EPOS", 4396, 101);
        assert!(target(&invalid, true, 100, 102, 100, 4295).is_err());
        invalid.record(b"EPOS", 4296, 101);
        assert!(target(&invalid, false, 100, 102, 100, 4295).is_err());
        assert_eq!(target(&invalid, false, 99, 102, 100, 4295), Ok(4196));
        invalid.record(b"EPOS", i64::from(crate::xeryon_motion::MAX_POSITION), 101);
        invalid.record(b"DPOS", i64::from(crate::xeryon_motion::MAX_POSITION), 101);
        assert!(
            target(
                &invalid,
                true,
                1,
                102,
                100,
                crate::xeryon_motion::MAX_POSITION
            )
            .is_err()
        );
        let mut wrapped = t;
        for sample in &mut wrapped.samples[..3] {
            sample.ms = 1;
        }
        assert_eq!(target(&wrapped, true, 10, 2, u32::MAX, 4295), Ok(4305));
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
    fn completion_waits_for_target_feedback_and_stationary_reached_status() {
        let mut t = baseline();
        // Encoder has moved 100 counts, but USB still reports the old target.
        t.record(b"EPOS", 4395, 201);
        t.record(b"DPOS", 4295, 201);
        t.record(b"STAT", 1344, 201);
        assert!(!completed(&t, 4395, 202, 200));
        t.record(b"DPOS", 4395, 203);
        assert!(completed(&t, 4395, 204, 200));
        // A new query round cannot reuse an old position-reached indication.
        assert!(!completed(&t, 4395, 204, 202));
        t.record(b"EPOS", 4394, 205);
        t.record(b"STAT", 1 << 5, 205);
        assert!(!completed(&t, 4395, 206, 200));
        t.record(b"STAT", 1344, 207);
        assert!(completed(&t, 4395, 208, 200));
        assert!(!completed(&t, 4395, 500, 200));
        t.record(b"STAT", 1344 | (1 << 12), 208);
        assert!(!completed(&t, 4395, 209, 200));
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
        // The settling watchdog also stops a completed train without another edge.
        let mut b = Burst::new(1).unwrap();
        b.tick();
        b.tick();
        b.tick();
        assert_eq!(b.status, Status::Complete);
        b.stop(Status::Fault);
        b.tick();
        assert!(!b.high);
        assert_eq!(b.status, Status::Fault);
    }
}
