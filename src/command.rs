//! Shared, allocation-free command grammar for USB1 and the desktop allowlist.
pub const MAX_COMMAND_BYTES: usize = 128;

#[derive(Debug, PartialEq, Eq)]
pub enum Command {
    Start(u32, crate::experiment_data::ExperimentConfig),
    Abort(u32),
    GetData(u32, u32),
    GetChunk(u32, u32, u32),
    AckData(u32, u32, u32, u32),
    Hello(u32),
    Ping(u32),
    Snapshot(u32),
    Stream(u32, bool),
    Move(u32, bool, u8, u32),
    Limits(u32, u8, u8),
    Stop(u32),
    Zero(u32),
    XeryonMove(u32, bool, u32, Option<u32>),
    XeryonStop(u32),
}

impl Command {
    pub fn id(&self) -> u32 {
        match *self {
            Self::Start(id, _)
            | Self::Abort(id)
            | Self::GetData(id, _)
            | Self::GetChunk(id, _, _)
            | Self::AckData(id, _, _, _)
            | Self::Hello(id)
            | Self::Ping(id)
            | Self::Snapshot(id)
            | Self::Stream(id, _)
            | Self::Move(id, _, _, _)
            | Self::Limits(id, _, _)
            | Self::Stop(id)
            | Self::Zero(id)
            | Self::XeryonMove(id, _, _, _)
            | Self::XeryonStop(id) => id,
        }
    }
    pub fn parse(line: &[u8]) -> Result<Self, ()> {
        let text = core::str::from_utf8(line).map_err(|_| ())?;
        let mut words = text.split_ascii_whitespace();
        let name = words.next().ok_or(())?;
        let id = words.next().ok_or(())?.parse::<u32>().map_err(|_| ())?;
        if id == 0 {
            return Err(());
        }
        let command = match name {
            "START" => {
                use crate::experiment_data::ExperimentConfig;
                let kind = words.next().ok_or(())?;
                let start = words.next().ok_or(())?.parse::<i64>().map_err(|_| ())?;
                let increment = words.next().ok_or(())?.parse::<i64>().map_err(|_| ())?;
                let timing = words.next().ok_or(())?.parse::<u32>().map_err(|_| ())?;
                let repeats = words.next().ok_or(())?.parse::<u32>().map_err(|_| ())?;
                let optional = words.next();
                let speed = if kind == "Q" {
                    optional
                        .map(str::parse::<u32>)
                        .transpose()
                        .map_err(|_| ())?
                } else {
                    None
                };
                let mut config =
                    ExperimentConfig::from_wire(kind, start, increment, timing, repeats, speed)
                        .map_err(|_| ())?;
                if let ExperimentConfig::Sinusoidal {
                    ref mut control, ..
                } = config
                {
                    *control = match optional {
                        None | Some("D") => crate::experiment_data::SineControl::Displacement,
                        Some("V") => crate::experiment_data::SineControl::Velocity,
                        Some("G") => crate::experiment_data::SineControl::Gpio,
                        Some("VP") => crate::experiment_data::SineControl::VelocityPosition,
                        _ => return Err(()),
                    };
                }
                Self::Start(id, config)
            }
            "ABORT" => Self::Abort(id),
            "GET_DATA" => Self::GetData(id, words.next().ok_or(())?.parse().map_err(|_| ())?),
            "GET_CHUNK" => Self::GetChunk(
                id,
                words.next().ok_or(())?.parse().map_err(|_| ())?,
                words.next().ok_or(())?.parse().map_err(|_| ())?,
            ),
            "ACK_DATA_RECEIVED" => Self::AckData(
                id,
                words.next().ok_or(())?.parse().map_err(|_| ())?,
                words.next().ok_or(())?.parse().map_err(|_| ())?,
                words.next().ok_or(())?.parse().map_err(|_| ())?,
            ),
            "XERYON_STOP" => Self::XeryonStop(id),
            "XERYON_MOVE" => {
                let up = match words.next() {
                    Some("U") => true,
                    Some("D") => false,
                    _ => return Err(()),
                };
                let nm = words.next().ok_or(())?.parse::<u32>().map_err(|_| ())?;
                if !(78..=crate::experiment_data::MAX_TRAVEL_NM).contains(&nm) {
                    return Err(());
                }
                let speed = words
                    .next()
                    .map(|value| value.parse::<u32>())
                    .transpose()
                    .map_err(|_| ())?;
                if speed.is_some_and(|value| {
                    !(1..=crate::experiment_data::MAX_SPEED_UM_S).contains(&value)
                }) {
                    return Err(());
                }
                Self::XeryonMove(id, up, nm, speed)
            }
            "PLUNGER_LIMITS" => {
                let min = words.next().ok_or(())?.parse::<u8>().map_err(|_| ())?;
                let max = words.next().ok_or(())?.parse::<u8>().map_err(|_| ())?;
                let limits = crate::plunger_config::Limits::new(min, max).map_err(|_| ())?;
                Self::Limits(id, limits.min, limits.max)
            }
            "PLUNGER_STOP" => Self::Stop(id),
            "PLUNGER_ZERO" => Self::Zero(id),
            "PLUNGER_MOVE" => {
                let up = match words.next() {
                    Some("U") => true,
                    Some("D") => false,
                    _ => return Err(()),
                };
                let speed = words.next().ok_or(())?.parse::<u8>().map_err(|_| ())?;
                let um = words.next().ok_or(())?.parse::<u32>().map_err(|_| ())?;
                if speed == 0 || crate::plunger_config::distance_counts(um).is_none() {
                    return Err(());
                }
                Self::Move(id, up, speed, um)
            }
            "HELLO" => Self::Hello(id),
            "PING" => Self::Ping(id),
            "SNAPSHOT" => Self::Snapshot(id),
            "STREAM" => Self::Stream(
                id,
                match words.next() {
                    Some("1") => true,
                    Some("0") => false,
                    _ => return Err(()),
                },
            ),
            _ => return Err(()),
        };
        if words.next().is_some() {
            return Err(());
        }
        Ok(command)
    }
}
