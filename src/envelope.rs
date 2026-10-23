//! Fault-only guard. No actuator commands or trajectory correction here.
use crate::{experiment_data::ExperimentConfig, protocol::Telemetry};

#[derive(Clone, Copy, Default)]
pub struct Envelope {
    pub margin_nm: Option<u32>,
    pub max_receipt_age_ms: u32,
    generation: Option<u32>,
    pub violation: Option<(i64, u32, u32)>, // raw counts, host receipt time, generation
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn disabled_quasi_static_invalid_and_encoder_rounding_do_not_fault() {
        let sine = ExperimentConfig::Sinusoidal {
            center_nm: 0,
            amplitude_nm: 7800,
            frequency_mhz: 1000,
            cycles: 1,
            control: crate::experiment_data::SineControl::Gpio,
        };
        let mut guard = Envelope::default();
        let mut t = Telemetry::new();
        t.record(b"EPOS", 1000, 1);
        assert!(!guard.exceeded(sine, &t, 1));
        guard.margin_nm = Some(78);
        guard.max_receipt_age_ms = 20;
        let quasi = ExperimentConfig::QuasiStatic {
            start_nm: 0,
            step_nm: 78,
            hold_ms: 10,
            steps: 1,
            speed_um_s: 10,
            cycle_wait_ms: 0,
            predelay_ms: 0,
            cycles: 1,
            return_to_start: false,
        };
        assert!(!guard.exceeded(quasi, &t, 1));
        t.samples[0].valid = false;
        assert!(!guard.exceeded(sine, &t, 1));
        t.record(b"EPOS", 101, 2);
        assert!(!guard.exceeded(sine, &t, 2)); // exactly on boundary
        t.record(b"EPOS", -102, 3);
        assert!(guard.exceeded(sine, &t, 3)); // either direction
    }
}
impl Envelope {
    pub fn configured() -> Self {
        Self {
            margin_nm: option_env!("SINE_ENVELOPE_MARGIN_NM").and_then(|s| s.parse().ok()),
            max_receipt_age_ms: option_env!("SINE_ENVELOPE_MAX_AGE_MS")
                .and_then(|s| s.parse().ok())
                .unwrap_or(100),
            ..Self::default()
        }
    }
    pub fn reset(&mut self, generation: u32) {
        self.generation = Some(generation);
        self.violation = None;
    }
    pub fn exceeded(&mut self, config: ExperimentConfig, t: &Telemetry, now: u32) -> bool {
        let ExperimentConfig::Sinusoidal {
            center_nm,
            amplitude_nm,
            ..
        } = config
        else {
            return false;
        };
        let Some(margin) = self.margin_nm else {
            return false;
        };
        let epos = t.samples[0];
        if !epos.valid || self.generation == Some(t.epos_generation) {
            return false;
        }
        self.generation = Some(t.epos_generation);
        if now.wrapping_sub(epos.ms) > self.max_receipt_age_ms {
            return false;
        }
        // Half a count covers endpoint rounding, independent of the chosen margin.
        let radius = u64::from(amplitude_nm) + u64::from(margin) + 39;
        if epos.value.saturating_mul(78).abs_diff(center_nm) > radius {
            self.violation = Some((epos.value, epos.ms, t.epos_generation));
            return true;
        }
        false
    }
    pub fn write(&self, out: &mut impl core::fmt::Write) -> core::fmt::Result {
        write!(
            out,
            ",\"envelope\":{{\"enabled\":{},\"margin_nm\":{},\"max_receipt_age_ms\":{},\"violation\":",
            self.margin_nm.is_some(),
            self.margin_nm.unwrap_or(0),
            self.max_receipt_age_ms
        )?;
        if let Some((epos, ms, generation)) = self.violation {
            write!(out, "[{epos},{ms},{generation}]")?;
        } else {
            write!(out, "null")?;
        }
        write!(out, "}}")
    }
}
