//! Shared, allocation-free experiment units, validation and wire record format.
pub const CAPACITY: usize = 4096;
pub const RECORD_FIELDS: usize = 12;
pub const DATA_VERSION: u32 = 2;
pub const RECORD_BYTES: usize = RECORD_FIELDS * 4;
pub const CHUNK_RECORDS: usize = 16;
pub const SAMPLE_MS: u32 = 20; // 50 Hz experiment recording
pub const TRAJECTORY_MS: u32 = 10; // 100 Hz sine velocity updates
pub const MOVE_TIMEOUT_MS: u32 = 10_000;
pub const MAX_SPEED_UM_S: u32 = 0x00ff_ffff;
pub const MAX_DURATION_MS: u32 = (CAPACITY as u32 - 1) * SAMPLE_MS;
pub const MIN_POSITION: i32 = -(1 << 25);
pub const MAX_POSITION: i32 = (1 << 25) - 1;
pub const MAX_TRAVEL_NM: u32 = MAX_POSITION as u32 * 78;

/// nm and millihertz -> micrometers/second. Also used by desktop validation.
pub fn sine_max_velocity(amplitude_nm: u32, frequency_mhz: u32) -> f64 {
    core::f64::consts::TAU * f64::from(frequency_mhz) * f64::from(amplitude_nm) / 1_000_000.0
}

/// Exact phase direction, with the outgoing sign at both extrema. Integer
/// phase avoids cosine approximation/rounding deciding which tick reverses.
pub fn sine_direction(frequency_mhz: u32, elapsed_ms: u32) -> i8 {
    let phase = u64::from(frequency_mhz) * u64::from(elapsed_ms) % 1_000_000;
    if (250_000..750_000).contains(&phase) {
        -1
    } else {
        1
    }
}

/// SSPD is an unsigned 24-bit integer; round magnitude to nearest, ties up.
pub fn velocity_speed(velocity: f64) -> u32 {
    (velocity.abs() + 0.5) as u32
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExperimentConfig {
    QuasiStatic {
        start_nm: i64,
        step_nm: i64,
        hold_ms: u32,
        steps: u32,
        speed_um_s: u32,
    },
    Sinusoidal {
        center_nm: i64,
        amplitude_nm: u32,
        frequency_mhz: u32,
        cycles: u32,
    },
}

pub fn counts(nm: i64) -> Result<i32, &'static str> {
    let rounded = nm
        .checked_add(if nm < 0 { -39 } else { 39 })
        .ok_or("Position overflow")?
        / 78;
    if !(i64::from(MIN_POSITION)..=i64::from(MAX_POSITION)).contains(&rounded) {
        return Err("Position exceeds signed 26-bit encoder range");
    }
    Ok(rounded as i32)
}

impl ExperimentConfig {
    /// Shared wire units for firmware parsing and native desktop validation.
    pub fn from_wire(
        kind: &str,
        start: i64,
        increment: i64,
        timing: u32,
        repeats: u32,
        speed_um_s: Option<u32>,
    ) -> Result<Self, &'static str> {
        match kind {
            "Q" => Ok(Self::QuasiStatic {
                start_nm: start,
                step_nm: increment,
                hold_ms: timing,
                steps: repeats,
                speed_um_s: speed_um_s
                    .filter(|v| (1..=MAX_SPEED_UM_S).contains(v))
                    .ok_or("Quasi-static speed must be 1..16777215 um/s")?,
            }),
            "S" if speed_um_s.is_none() => Ok(Self::Sinusoidal {
                center_nm: start,
                amplitude_nm: u32::try_from(increment).map_err(|_| "Amplitude out of range")?,
                frequency_mhz: timing,
                cycles: repeats,
            }),
            _ => Err("Unknown experiment mode"),
        }
    }
    pub fn start(self) -> Result<i32, &'static str> {
        counts(match self {
            Self::QuasiStatic { start_nm, .. } => start_nm,
            Self::Sinusoidal { center_nm, .. } => center_nm,
        })
    }
    pub fn speed(self) -> Option<u32> {
        match self {
            Self::QuasiStatic { speed_um_s, .. } => Some(speed_um_s),
            Self::Sinusoidal { .. } => None,
        }
    }
    /// Twice nominal travel time plus 1 s settling, with the existing 10 s floor.
    /// nm / (um/s) gives milliseconds. Bound even initial positioning.
    pub fn move_timeout(self, distance_nm: u64) -> Result<u32, &'static str> {
        let Some(speed) = self.speed() else {
            return Ok(MOVE_TIMEOUT_MS);
        };
        if !(1..=MAX_SPEED_UM_S).contains(&speed) {
            return Err("Quasi-static speed must be 1..16777215 um/s");
        }
        let timeout = distance_nm
            .div_ceil(u64::from(speed))
            .checked_mul(2)
            .and_then(|v| v.checked_add(1000))
            .ok_or("Move duration overflow")?
            .max(u64::from(MOVE_TIMEOUT_MS));
        if timeout > u64::from(MAX_DURATION_MS) {
            return Err("Move too long at selected speed; increase speed or reduce travel");
        }
        Ok(timeout as u32)
    }
    pub fn step_timeout(self) -> Result<u32, &'static str> {
        match self {
            // Account for rounding each endpoint to the encoder grid.
            Self::QuasiStatic { step_nm, .. } => self.move_timeout(
                step_nm
                    .unsigned_abs()
                    .checked_add(78)
                    .ok_or("Trajectory overflow")?,
            ),
            Self::Sinusoidal { .. } => Ok(MOVE_TIMEOUT_MS),
        }
    }
    /// Worst-case acquisition duration. Initial positioning is outside acquisition.
    pub fn validate(self) -> Result<u32, &'static str> {
        self.start()?;
        let duration = match self {
            Self::QuasiStatic {
                start_nm,
                step_nm,
                hold_ms,
                steps,
                ..
            } => {
                if step_nm.unsigned_abs() < 78 || hold_ms == 0 || steps == 0 {
                    return Err("Step must be at least one count; hold and steps must be positive");
                }
                let end = step_nm
                    .checked_mul(i64::from(steps))
                    .and_then(|d| start_nm.checked_add(d))
                    .ok_or("Trajectory overflow")?;
                counts(end)?;
                u64::from(steps) * (u64::from(hold_ms) + u64::from(self.step_timeout()?))
            }
            Self::Sinusoidal {
                center_nm,
                amplitude_nm,
                frequency_mhz,
                cycles,
            } => {
                if amplitude_nm < 78 || !(100..=2000).contains(&frequency_mhz) || cycles == 0 {
                    return Err("Amplitude must be positive; frequency 0.1..2 Hz; cycles positive");
                }
                counts(
                    center_nm
                        .checked_sub(i64::from(amplitude_nm))
                        .ok_or("Trajectory overflow")?,
                )?;
                counts(
                    center_nm
                        .checked_add(i64::from(amplitude_nm))
                        .ok_or("Trajectory overflow")?,
                )?;
                let vmax = sine_max_velocity(amplitude_nm, frequency_mhz);
                if !vmax.is_finite()
                    || vmax <= 0.0
                    || vmax > f64::from(MAX_SPEED_UM_S)
                    || !(1..=MAX_SPEED_UM_S).contains(&velocity_speed(vmax))
                {
                    return Err("Sinusoidal peak speed is outside representable SSPD range");
                }
                (u64::from(cycles) * 1_000_000).div_ceil(u64::from(frequency_mhz))
                    + u64::from(MOVE_TIMEOUT_MS)
            }
        };
        if duration > u64::from(MAX_DURATION_MS) {
            return Err("Experiment exceeds fixed sample buffer (81.9 seconds at 50 Hz)");
        }
        Ok(duration as u32)
    }
    pub fn write_json(self, out: &mut impl core::fmt::Write) -> core::fmt::Result {
        match self {
            Self::QuasiStatic {
                start_nm,
                step_nm,
                hold_ms,
                steps,
                speed_um_s,
            } => write!(
                out,
                "{{\"kind\":\"quasi_static\",\"start_nm\":{start_nm},\"step_nm\":{step_nm},\"hold_ms\":{hold_ms},\"steps\":{steps},\"speed_um_s\":{speed_um_s}}}"
            ),
            Self::Sinusoidal {
                center_nm,
                amplitude_nm,
                frequency_mhz,
                cycles,
            } => write!(
                out,
                "{{\"kind\":\"sinusoidal\",\"center_nm\":{center_nm},\"amplitude_nm\":{amplitude_nm},\"frequency_mhz\":{frequency_mhz},\"cycles\":{cycles}}}"
            ),
        }
    }
}

/// Explicit little-endian wire encoding; no casting native struct memory.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Record {
    pub elapsed_ms: u32,
    pub epos: i32,
    pub pressure_raw: i32,
    pub stat: u32,
    pub epos_age_ms: u32,
    pub pressure_age_ms: u32,
    pub stat_age_ms: u32,
    pub target: i32,
    pub controller_ticks: u32,
    pub controller_age_ms: u32,
    pub epos_generation: u32,
    pub pressure_generation: u32,
}
impl Record {
    pub const EMPTY: Self = Self {
        elapsed_ms: 0,
        epos: 0,
        pressure_raw: 0,
        stat: 0,
        epos_age_ms: 0,
        pressure_age_ms: 0,
        stat_age_ms: 0,
        target: 0,
        controller_ticks: 0,
        controller_age_ms: 0,
        epos_generation: 0,
        pressure_generation: 0,
    };
    pub fn encode(self) -> [u8; RECORD_BYTES] {
        let words = [
            self.elapsed_ms,
            self.epos as u32,
            self.pressure_raw as u32,
            self.stat,
            self.epos_age_ms,
            self.pressure_age_ms,
            self.stat_age_ms,
            self.target as u32,
            self.controller_ticks,
            self.controller_age_ms,
            self.epos_generation,
            self.pressure_generation,
        ];
        let mut bytes = [0; RECORD_BYTES];
        for (chunk, word) in bytes.chunks_exact_mut(4).zip(words) {
            chunk.copy_from_slice(&word.to_le_bytes());
        }
        bytes
    }
}
/// Desktop representation of the same wire layout, retaining signed fields.
#[allow(dead_code)] // Reception is desktop-only; firmware sends the same layout.
pub fn decode_fields(bytes: &[u8; RECORD_BYTES]) -> [i64; RECORD_FIELDS] {
    let mut fields = [0; RECORD_FIELDS];
    for (i, word) in bytes.chunks_exact(4).enumerate() {
        let word: [u8; 4] = word.try_into().unwrap();
        fields[i] = if matches!(i, 1 | 2 | 7) {
            i64::from(i32::from_le_bytes(word))
        } else {
            i64::from(u32::from_le_bytes(word))
        };
    }
    fields
}
const _: () = assert!(core::mem::size_of::<Record>() == RECORD_BYTES);

/// CRC-32/ISO-HDLC; initialize to !0 and complement the final state.
pub fn crc32(mut crc: u32, bytes: &[u8]) -> u32 {
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320 & (0u32.wrapping_sub(crc & 1)));
        }
    }
    crc
}
