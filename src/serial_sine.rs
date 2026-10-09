//! Bounded USB setpoint streaming. Never accumulate or replay overdue commands.
use super::Actuator;
use crate::{
    experiment_data::{ExperimentConfig, SineControl},
    protocol::Sample,
    trajectory::Trajectory,
};

pub const UPDATE_MS: u32 = 5;
pub const STALL_MS: u32 = 25;
pub const POSITION_GAIN_S: f64 = 2.0;
pub const CORRECTION_FRACTION: f64 = 0.2;
pub const CORRECTION_RAMP_MS: u32 = 100;
pub const FEEDBACK_MAX_AGE_MS: u32 = 100;
pub const VELOCITY_LIMIT_UM_S: f64 = 780.0;

#[derive(Default)]
struct PositionCorrection {
    velocity: f64,
    last: u32,
}
impl PositionCorrection {
    fn update(
        &mut self,
        trajectory: &Trajectory,
        elapsed: u32,
        now: u32,
        epos: Sample,
        peak_speed: f64,
    ) -> Result<f64, &'static str> {
        let age = now.wrapping_sub(epos.ms);
        if !epos.valid || age > FEEDBACK_MAX_AGE_MS {
            return Err("Position correction feedback stale");
        }
        if !(i64::from(crate::experiment_data::MIN_POSITION)
            ..=i64::from(crate::experiment_data::MAX_POSITION))
            .contains(&epos.value)
        {
            return Err("Invalid encoder position");
        }
        // Match the reference to the receipt time, then hold that error until
        // another EPOS arrives. Comparing old EPOS with today's target would
        // turn telemetry cadence into a spurious position lag. Receipt time
        // cannot compensate for unknown controller/USB acquisition latency.
        let target = trajectory.position(elapsed.saturating_sub(age) * 1000);
        let error = i64::from(target) - epos.value;
        // A continuous one-count deadband avoids chasing encoder rounding.
        let error_um =
            (error.unsigned_abs().saturating_sub(1) as f64) * error.signum() as f64 * 0.078;
        let limit = peak_speed * CORRECTION_FRACTION;
        let desired = (POSITION_GAIN_S * error_um).clamp(-limit, limit);
        let step =
            limit * f64::from(elapsed.saturating_sub(self.last)) / f64::from(CORRECTION_RAMP_MS);
        self.velocity += (desired - self.velocity).clamp(-step, step);
        self.last = elapsed;
        Ok(self.velocity)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Speed(u32),
    Position(i32),
    Direction(i8),
}
impl Command {
    pub fn write(self, out: &mut impl core::fmt::Write) -> Result<(), &'static str> {
        match self {
            Self::Speed(v) if v <= crate::experiment_data::MAX_SPEED_UM_S => {
                writeln!(out, "SSPD={v}")
            }
            Self::Position(v)
                if (crate::experiment_data::MIN_POSITION
                    ..=crate::experiment_data::MAX_POSITION)
                    .contains(&v) =>
            {
                writeln!(out, "DPOS={v}")
            }
            Self::Direction(v) if (-1..=1).contains(&v) => writeln!(out, "SCAN={v}"),
            _ => return Err("Invalid serial waveform command"),
        }
        .map_err(|_| "Serial waveform command too long")
    }
}

#[derive(Default)]
pub struct Stream {
    next: u32,
    last: u32,
    speed: Option<u32>,
    position: Option<i32>,
    direction: Option<i8>,
    terminal: bool,
    changed: u32,
    correction: PositionCorrection,
    pub updates: u32,
    pub skipped: u32,
    pub max_gap_ms: u32,
}
impl Stream {
    pub fn mode_transition(&self, elapsed: u32) -> bool {
        self.direction.is_none_or(|d| d == 0) || elapsed.saturating_sub(self.changed) < 100
    }
    fn due(&mut self, elapsed: u32, io: &impl Actuator) -> Result<bool, &'static str> {
        if elapsed.saturating_sub(self.last) > STALL_MS {
            return Err("Serial waveform update deadline missed");
        }
        if elapsed < self.next || !io.tx_idle() {
            return Ok(false);
        }
        self.max_gap_ms = self.max_gap_ms.max(elapsed - self.last);
        self.skipped += elapsed.saturating_sub(self.next) / UPDATE_MS;
        self.last = elapsed;
        self.next = (elapsed / UPDATE_MS + 1) * UPDATE_MS;
        Ok(true)
    }
    fn position(
        &mut self,
        position: i32,
        speed: u32,
        io: &mut impl Actuator,
    ) -> Result<(), &'static str> {
        // Do not repeatedly restart positioning at the same encoder count.
        // Keep updating speed while approaching a turning point, even if the
        // rounded target has not changed yet.
        let mut commands = [Command::Speed(speed); 2];
        let mut n = 0;
        if self.speed != Some(speed) {
            commands[n] = Command::Speed(speed);
            n += 1;
        }
        if self.position != Some(position) {
            commands[n] = Command::Position(position);
            n += 1;
        }
        if n > 0 {
            io.serial_commands(&commands[..n])?;
            self.updates += 1;
        }
        self.speed = Some(speed);
        self.position = Some(position);
        Ok(())
    }
    pub fn tick(
        &mut self,
        config: ExperimentConfig,
        elapsed: u32,
        now: u32,
        epos: Sample,
        io: &mut impl Actuator,
    ) -> Result<bool, &'static str> {
        if elapsed.saturating_sub(self.last) > STALL_MS {
            return Err("Serial waveform update deadline missed");
        }
        let trajectory = Trajectory::new(config)?;
        let us = elapsed * 1000;
        if us >= trajectory.duration_us {
            if config.control().is_some_and(SineControl::velocity) {
                return Ok(true);
            }
            if self.terminal {
                if elapsed.saturating_sub(self.last) > STALL_MS {
                    return Err("Serial waveform drain deadline missed");
                }
                return Ok(io.tx_idle());
            }
            if !self.due(elapsed, io)? {
                return Ok(false);
            }
            self.position(trajectory.start, 1, io)?;
            self.terminal = true;
            return Ok(false);
        }
        if !self.due(elapsed, io)? {
            return Ok(false);
        }
        if config.control() == Some(SineControl::Displacement) {
            // Follow local trajectory speed, rather than driving each tiny DPOS
            // step at a large fixed speed. Look one tick ahead to avoid a zero
            // speed dead zone; +1 covers integer um/s and encoder quantization.
            let speed = trajectory
                .velocity(us)
                .abs()
                .max(trajectory.velocity(us + UPDATE_MS * 1000).abs())
                as u32
                + 1;
            self.position(trajectory.position(us), speed, io)?;
        } else {
            // Midpoint speed reduces zero-order-hold integration error.
            let mut velocity = trajectory.velocity(us + UPDATE_MS * 500);
            if let ExperimentConfig::Sinusoidal {
                amplitude_nm,
                frequency_mhz,
                control: SineControl::VelocityPosition,
                ..
            } = config
            {
                let peak = crate::experiment_data::sine_max_velocity(amplitude_nm, frequency_mhz);
                velocity += self
                    .correction
                    .update(&trajectory, elapsed, now, epos, peak)?;
                velocity = velocity.clamp(-VELOCITY_LIMIT_UM_S, VELOCITY_LIMIT_UM_S);
            }
            let speed = (velocity.abs() + 0.5) as u32;
            let direction = if speed == 0 {
                0
            } else if velocity > 0.0 {
                1
            } else {
                -1
            };
            if self.direction != Some(direction) {
                // Drain zero speed before reversing direction. The next poll
                // recomputes current phase, never sends an obsolete speed.
                if self.speed != Some(0) {
                    io.serial_commands(&[Command::Speed(0)])?;
                    self.speed = Some(0);
                } else {
                    io.serial_commands(&[Command::Direction(direction)])?;
                    self.direction = Some(direction);
                }
                self.changed = elapsed;
                self.updates += 1;
                self.next = elapsed; // continue reversal as soon as this USB line drains
            } else if self.speed != Some(speed) {
                io.serial_commands(&[Command::Speed(speed)])?;
                self.speed = Some(speed);
                self.updates += 1;
            }
        }
        Ok(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn trajectory() -> Trajectory {
        Trajectory::new(ExperimentConfig::Sinusoidal {
            center_nm: 0,
            amplitude_nm: 7800,
            frequency_mhz: 1000,
            cycles: 2,
            control: SineControl::VelocityPosition,
        })
        .unwrap()
    }
    #[test]
    fn correction_uses_receipt_phase_across_clock_wrap_and_encoder_deadband() {
        let trajectory = trajectory();
        let mut correction = PositionCorrection::default();
        let epoch = u32::MAX - 100;
        for elapsed in (0u32..500).step_by(5) {
            let receipt = elapsed / 20 * 20;
            for quantization in -1..=1 {
                let epos = Sample {
                    valid: true,
                    ms: epoch.wrapping_add(receipt),
                    value: i64::from(trajectory.position(receipt * 1000)) + quantization,
                };
                assert_eq!(
                    correction
                        .update(
                            &trajectory,
                            elapsed,
                            epoch.wrapping_add(elapsed),
                            epos,
                            50.0
                        )
                        .unwrap(),
                    0.0
                );
            }
        }
    }
    #[test]
    fn correction_is_bounded_slew_limited_and_rejects_bad_feedback() {
        let trajectory = trajectory();
        let mut correction = PositionCorrection::default();
        let mut last = 0.0;
        for elapsed in (0..=400).step_by(5) {
            let offset = if elapsed < 200 { -10000 } else { 10000 };
            let epos = Sample {
                valid: true,
                ms: elapsed,
                value: i64::from(trajectory.position(elapsed * 1000)) + offset,
            };
            let velocity = correction
                .update(&trajectory, elapsed, elapsed, epos, 50.0)
                .unwrap();
            assert!(velocity.abs() <= 10.0);
            assert!((velocity - last).abs() <= 0.500001);
            if elapsed == 100 {
                assert_eq!(velocity, 10.0);
            }
            if elapsed == 400 {
                assert_eq!(velocity, -10.0);
            }
            last = velocity;
        }
        let mut epos = Sample {
            valid: true,
            ms: 400,
            value: 0,
        };
        assert!(correction.update(&trajectory, 500, 500, epos, 50.0).is_ok());
        assert_eq!(
            correction.update(&trajectory, 501, 501, epos, 50.0),
            Err("Position correction feedback stale")
        );
        epos.valid = false;
        assert!(
            correction
                .update(&trajectory, 400, 400, epos, 50.0)
                .is_err()
        );
        epos.valid = true;
        epos.value = i64::MAX;
        assert_eq!(
            correction.update(&trajectory, 400, 400, epos, 50.0),
            Err("Invalid encoder position")
        );
    }
}

/// Rest-to-rest quintic positioning for approach and return, with zero endpoint
/// velocity and acceleration. Controller dynamics and serial quantization still
/// determine physical tracking; this is not a guarantee of zero physical jerk.
pub struct Ramp {
    from: i32,
    to: i32,
    duration: u32,
    started: Option<u32>,
    stream: Stream,
}
impl Ramp {
    pub fn new(from: i32, to: i32, speed: u32) -> Self {
        let distance_um = (i64::from(to) - i64::from(from)).unsigned_abs() as f64 * 0.078;
        Self {
            from,
            to,
            duration: (distance_um * 1875.0 / f64::from(speed) + 1.0) as u32,
            started: None,
            stream: Stream::default(),
        }
    }
    pub fn target(&self) -> i32 {
        self.stream.position.unwrap_or(self.from)
    }
    pub fn tick(&mut self, now: u32, io: &mut impl Actuator) -> Result<bool, &'static str> {
        let elapsed = now.wrapping_sub(*self.started.get_or_insert(now));
        let duration = self.duration.max(1000);
        if self.stream.terminal {
            if elapsed > duration + STALL_MS {
                return Err("Serial ramp drain deadline missed");
            }
            return Ok(io.tx_idle());
        }
        if !self.stream.due(elapsed, io)? {
            return Ok(false);
        }
        let u = (f64::from(elapsed) / f64::from(duration)).min(1.0);
        let blend = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
        let delta = f64::from(self.to) - f64::from(self.from);
        let position = f64::from(self.from) + delta * blend;
        let position = (position + if position < 0.0 { -0.5 } else { 0.5 }) as i32;
        let speed =
            (delta.abs() * 78.0 / f64::from(duration) * 30.0 * u * u * (1.0 - u) * (1.0 - u))
                as u32
                + 1;
        self.stream.position(position, speed, io)?;
        self.stream.terminal = elapsed >= duration;
        Ok(false)
    }
}
