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
                let speed = words
                    .next()
                    .map(str::parse::<u32>)
                    .transpose()
                    .map_err(|_| ())?;
                Self::Start(
                    id,
                    ExperimentConfig::from_wire(kind, start, increment, timing, repeats, speed)
                        .map_err(|_| ())?,
                )
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

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn experiment_commands_keep_ids_and_reject_extra_arguments() {
        for text in [
            "START 7 Q -1234 78 500 2 10",
            "START 7 S 0 780 1000 1",
            "ABORT 7",
            "GET_DATA 7 1",
            "GET_CHUNK 7 1 0",
            "ACK_DATA_RECEIVED 7 1 21 12345678",
        ] {
            let command = Command::parse(text.as_bytes()).unwrap();
            assert_eq!(command.id(), 7);
            assert!(Command::parse(format!("{text} extra").as_bytes()).is_err());
        }
        for text in [
            "START 1 X 0 78 500 2",
            "START 1 S 0 -78 500 2",
            "START 0 Q 0 78 500 2",
            "START 1 Q 0 78 500 2",
            "START 1 Q 0 78 500 2 0",
            "START 1 Q 0 78 500 2 16777216",
            "START 1 Q 0 78 500 2 -1",
            "START 1 Q 0 78 500 2 1.5",
            "START 1 S 0 78 500 2 10",
            "GET_CHUNK 7 1 -1",
            "ACK_DATA_RECEIVED 7 1 21 -1",
        ] {
            assert!(Command::parse(text.as_bytes()).is_err());
        }
    }
    #[test]
    fn plunger_commands_are_bounded() {
        assert_eq!(
            Command::parse(b"PLUNGER_MOVE 1 U 64 10"),
            Ok(Command::Move(1, true, 64, 10))
        );
        assert_eq!(
            Command::parse(b"PLUNGER_LIMITS 4 30 75"),
            Ok(Command::Limits(4, 30, 75))
        );
        assert_eq!(Command::parse(b"PLUNGER_STOP 2"), Ok(Command::Stop(2)));
        assert_eq!(Command::parse(b"PLUNGER_ZERO 3"), Ok(Command::Zero(3)));
        for line in [
            "PLUNGER_MOVE 1 U 0 10",
            "PLUNGER_MOVE 1 D 256 10",
            "PLUNGER_MOVE 1 X 64 10",
            "PLUNGER_MOVE 1 U 64 0",
            "PLUNGER_MOVE 1 U 64 4294967295",
            "PLUNGER_STOP 1 extra",
            "PLUNGER_LIMITS 1 75 30",
            "PLUNGER_LIMITS 1 30 30",
            "PLUNGER_LIMITS 1 0 101",
            "PLUNGER_LIMITS 1 -1 75",
            "PLUNGER_LIMITS 1 30 75 extra",
        ] {
            assert!(Command::parse(line.as_bytes()).is_err(), "{line}");
        }
    }
    #[test]
    fn xeryon_commands_are_bounded() {
        assert_eq!(
            Command::parse(b"XERYON_MOVE 1 U 10000 10000"),
            Ok(Command::XeryonMove(1, true, 10000, Some(10000)))
        );
        assert_eq!(
            Command::parse(b"XERYON_MOVE 2 D 78 10000"),
            Ok(Command::XeryonMove(2, false, 78, Some(10000)))
        );
        assert_eq!(
            Command::parse(b"XERYON_MOVE 3 U 10000"),
            Ok(Command::XeryonMove(3, true, 10000, None))
        );
        assert_eq!(Command::parse(b"XERYON_STOP 3"), Ok(Command::XeryonStop(3)));
        for bad in [
            "XERYON_MOVE 1 U 10000 0",
            "XERYON_MOVE 1 U 10000 16777216",
            "XERYON_MOVE 1 U 10000 -1",
            "XERYON_MOVE 1 U 10000 1.5",
            "XERYON_MOVE 1 U 0 10000",
            "XERYON_MOVE 1 U 77 10000",
            "XERYON_MOVE 1 U 2617245619 10000",
            "XERYON_MOVE 1 X 100 10000",
            "XERYON_MOVE 1 D -100 10000",
            "XERYON_MOVE 1 U 1.5 10000",
            "XERYON_MOVE 1 U 100 extra 10000",
            "XERYON_STOP 0",
            "XERYON_STOP 1 extra",
        ] {
            assert!(Command::parse(bad.as_bytes()).is_err(), "{bad}");
        }
    }
}
