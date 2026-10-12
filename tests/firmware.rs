// Native tests for hardware-independent firmware modules.
#[path = "../src/command.rs"]
mod command;
#[path = "../src/line.rs"]
mod line;
#[path = "../src/plunger_config.rs"]
mod plunger_config;
#[path = "../src/protocol.rs"]
mod protocol;
#[path = "../src/xeryon.rs"]
mod xeryon;
#[path = "../src/xeryon_motion.rs"]
mod xeryon_motion;

#[test]
fn decoded_feedback_reaches_snapshot() {
    let mut decoder = xeryon::Decoder::new();
    let mut telemetry = protocol::Telemetry::new();
    for byte in b"EPOS=-123\r\nSTAT=352\n" {
        if let Some(record) = decoder.push(*byte) {
            let record = record.unwrap();
            telemetry.record(record.tag(), record.value(), 42);
        }
    }
    let mut frame = protocol::Frame::new();
    telemetry.snapshot(&mut frame, 7, 50, 0).unwrap();
    let json = core::str::from_utf8(frame.pending()).unwrap();
    assert!(json.contains("\"EPOS\":{\"value\":-123,\"ms\":42}"));
    assert!(json.contains("\"STAT\":{\"value\":352,\"ms\":42}"));
    assert_eq!(telemetry.records, 2);
}

#[path = "../src/plunger.rs"]
mod plunger;

#[path = "../src/pressure.rs"]
mod pressure;
#[path = "../src/pressure_i2c.rs"]
mod pressure_i2c;

#[test]
fn pressure_snapshot_preserves_i2c_values_and_identity() {
    let mut telemetry = protocol::Telemetry::new();
    telemetry.pressure = Some(pressure::Reading {
        raw: -8_388_608,
        average: 8_388_607,
        ms: 42,
        samples: 16,
        valid: true,
        state: 2,
        who: Some(pressure::WHO_AM_I),
        temperature: -2500,
        ..Default::default()
    });
    let mut frame = protocol::Frame::new();
    telemetry.snapshot(&mut frame, 7, 50, 0).unwrap();
    let json = core::str::from_utf8(frame.pending()).unwrap();
    assert!(json.contains("\"raw\":-8388608,\"average\":8388607"));
    assert!(json.contains(
        "\"sensor\":\"lps22hh\",\"address\":92,\"odr_hz\":200,\"temperature\":-2500,\"who\":179"
    ));
}

#[path = "experiment.rs"]
mod experiment;
#[path = "../src/experiment_data.rs"]
mod experiment_data;

#[test]
fn error_reply_keeps_request_id_and_partial_write_suffix() {
    let mut frame = protocol::Frame::new();
    frame.error(42, "Invalid command").unwrap();
    let expected = b"{\"type\":\"error\",\"id\":42,\"message\":\"Invalid command\"}\n";
    assert_eq!(frame.pending(), expected);
    frame.consume(7);
    assert_eq!(frame.pending(), &expected[7..]);
}

#[test]
fn record_layout_preserves_signed_fields_and_unsigned_sentinels() {
    let record = experiment_data::Record {
        elapsed_ms: 50,
        epos: -1,
        pressure_raw: -2,
        stat: 0x80000000,
        epos_age_ms: 1,
        pressure_age_ms: 2,
        stat_age_ms: 3,
        target: i32::MIN,
        controller_ticks: u32::MAX,
        controller_age_ms: u32::MAX,
        epos_generation: 123,
        pressure_generation: 456,
    };
    let bytes = [
        50, 0, 0, 0, 255, 255, 255, 255, 254, 255, 255, 255, 0, 0, 0, 128, 1, 0, 0, 0, 2, 0, 0, 0,
        3, 0, 0, 0, 0, 0, 0, 128, 255, 255, 255, 255, 255, 255, 255, 255, 123, 0, 0, 0, 200, 1, 0,
        0,
    ];
    assert_eq!(record.encode(), bytes);
    assert_eq!(
        experiment_data::decode_fields(&bytes),
        [
            50,
            -1,
            -2,
            2147483648,
            1,
            2,
            3,
            -2147483648,
            4294967295,
            4294967295,
            123,
            456
        ]
    );
}

mod command_tests {
    use crate::command::*;
    #[test]
    fn experiment_commands_keep_ids_and_reject_extra_arguments() {
        for text in [
            "START 7 Q -1234 78 500 2 10",
            "START 7 S 0 780 1000 1",
            "START 7 S 0 780 1000 1 D",
            "START 7 S 0 780 1000 1 V",
            "START 7 S 0 780 1000 1 G",
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
        for removed in ["GPIO_MOVE 4 P 1", "GPIO_STATUS 6"] {
            assert!(Command::parse(removed.as_bytes()).is_err());
        }
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

mod line_tests {
    use crate::line::*;

    #[test]
    fn capacity_overflow_and_recovery() {
        let mut lines = LineBuffer::<3>::new();
        for byte in b"abc" {
            assert!(lines.push(*byte).is_none());
        }
        assert_eq!(lines.push(b'\n'), Some(Ok(&b"abc"[..])));
        for byte in b"abcdignored" {
            assert!(lines.push(*byte).is_none());
        }
        assert_eq!(lines.push(b'\n'), Some(Err(())));
        assert_eq!(lines.push(b'\n'), Some(Ok(&b""[..])));
        assert!(lines.push(b'\r').is_none());
        assert_eq!(lines.push(b'\n'), Some(Ok(&b"\r"[..])));
    }
}

mod plunger_tests {
    use crate::plunger::*;
    #[test]
    fn potentiometer_limits_and_recovery() {
        let limits = Limits::default();
        assert_eq!((limits.min, limits.max), (30, 75));
        assert!(limits.allows(true, Some(3070)));
        assert!(!limits.allows(true, Some(3071)));
        assert!(!limits.allows(false, Some(1229)));
        assert!(limits.allows(false, Some(1230)));
        assert!(limits.allows(true, Some(0)));
        assert!(!limits.allows(false, Some(0)));
        assert!(limits.allows(false, Some(4095)));
        assert!(!limits.allows(true, Some(4095)));
        for up in [true, false] {
            assert!(!limits.allows(up, None));
            assert!(!limits.allows(up, Some(4096)));
        }
        assert!(Limits::new(0, 100).is_ok());
        for (min, max) in [(30, 30), (75, 30), (0, 101)] {
            assert!(Limits::new(min, max).is_err());
        }
        let custom = Limits::new(40, 60).unwrap();
        assert!(!custom.allows(true, Some(2457)));
        assert!(!custom.allows(false, Some(1638)));
    }
    #[test]
    fn calibration_and_limits() {
        assert_eq!(distance_counts(1), Some(17));
        assert_eq!(distance_counts(1000), Some(16688));
        assert_eq!(distance_counts(0), None);
        assert_eq!(distance_counts(u32::MAX), None);
    }
    #[test]
    fn quadrature_and_target_stop() {
        let mut motion = Motion::default();
        motion.start(true, 100, 1).unwrap();
        assert!(motion.zero().is_err());
        assert!(motion.start(false, 100, 1).is_err());
        let states = [0, 1, 3, 2, 0];
        for pair in states.windows(2) {
            assert_eq!(encoder_delta(pair[0], pair[1]), 1);
        }
        for pair in states.windows(2) {
            assert_eq!(encoder_delta(pair[1], pair[0]), -1);
        }
        assert_eq!(encoder_delta(0, 3), 0);
        for _ in 0..16 {
            assert!(!motion.step(1));
        }
        assert!(motion.step(1));
        assert!(!motion.running);
        assert_eq!(motion.speed, 0);
        motion.zero().unwrap();
        motion.start(false, 255, 1).unwrap();
        for _ in 0..16 {
            motion.step(-1);
        }
        assert!(motion.step(-1));
    }
}

mod xeryon_motion_tests {
    use crate::protocol::Telemetry;
    use crate::xeryon_motion::*;

    fn telemetry(now: u32) -> Telemetry {
        let mut t = Telemetry::new();
        t.mounted = true;
        t.cdc = 2;
        t.vid = 0x04d8;
        t.pid = 0xef4a;
        t.record(b"EPOS", 100, now);
        t.record(b"DPOS", 100, now);
        t.record(b"STAT", 1344, now);
        t
    }
    #[test]
    fn speed_precedes_position_and_each_line_fits_transport_limit() {
        for speed in [1, 10000, MAX_SPEED] {
            let mut frame = crate::protocol::Frame::new();
            write_move(&mut frame, MIN_POSITION, Some(speed)).unwrap();
            let text = core::str::from_utf8(frame.pending()).unwrap();
            assert_eq!(text, format!("SSPD={speed}\nDPOS=-33554432\n"));
            assert!(text.lines().all(|line| line.len() + 1 <= 32));
        }
        for speed in [0, MAX_SPEED + 1, u32::MAX] {
            let mut frame = crate::protocol::Frame::new();
            assert!(write_move(&mut frame, 100, Some(speed)).is_err());
            assert!(frame.pending().is_empty());
        }
        let mut frame = crate::protocol::Frame::new();
        write_move(&mut frame, 228, None).unwrap();
        assert_eq!(frame.pending(), b"DPOS=228\n");
    }
    #[test]
    fn relative_target_rounding_range_and_readiness() {
        let mut t = telemetry(100);
        assert_eq!(target(&t, true, 10000, 100), Ok(228));
        assert_eq!(target(&t, false, 10000, 100), Ok(-28));
        assert_eq!(target(&t, true, 78, 100), Ok(101));
        assert_eq!(target(&t, true, 117, 100), Ok(102));
        assert!(target(&t, true, 77, 100).is_err());
        assert!(target(&t, true, MAX_TRAVEL_NM + 1, 100).is_err());
        assert!(target(&t, true, 10000, 3100).is_err());
        t.record(b"EPOS", MAX_POSITION as i64, 100);
        assert!(target(&t, true, 78, 100).is_err());
        t.record(b"EPOS", MIN_POSITION as i64, 100);
        assert!(target(&t, false, 78, 100).is_err());
        for stat in [1 << 2, 1 << 12, 1 << 9, (1 << 13) | (1 << 5), 1 << 5] {
            t.record(b"STAT", stat, 100);
            assert!(target(&t, true, 78, 100).is_err());
        }
        t = telemetry(100);
        t.pid = 0;
        assert!(target(&t, true, 78, 100).is_err());
        t = telemetry(100);
        t.cdc = 1;
        assert!(target(&t, true, 78, 100).is_err());
    }
    #[test]
    fn stopped_scan_allows_manual_moves_but_active_scan_and_faults_still_block() {
        let mut t = telemetry(100);
        t.record(b"STAT", 8513, 100);
        assert_eq!(target(&t, true, 78, 100), Ok(101));
        assert_eq!(target(&t, false, 78, 100), Ok(99));
        for extra in [1 << 5, (1 << 5) | (1 << 10), 1 << 9, 1 << 12, 1 << 18] {
            t.record(b"STAT", 8513 | extra, 100);
            assert!(target(&t, true, 78, 100).is_err());
        }
    }
    #[test]
    fn completion_requires_echo_then_new_status() {
        let mut t = telemetry(100);
        let mut m = Motion::new();
        m.start(228, 100);
        assert!(!m.poll(&t, 101));
        assert_eq!(m.state, 1);
        t.record(b"STAT", 1344, 101);
        t.record(b"DPOS", 228, 102);
        assert!(!m.poll(&t, 102));
        assert_eq!(m.state, 1);
        t.record(b"STAT", 1344, 103);
        assert!(!m.poll(&t, 103));
        assert_eq!(m.state, 2);
    }
    #[test]
    fn stale_feedback_disconnect_fault_and_timeouts_stop_motion() {
        assert_eq!(controller_fault(1089), None);
        for (bit, expected) in [
            (2, "Xeryon thermal protection 1"),
            (3, "Xeryon thermal protection 2"),
            (12, "Xeryon encoder error"),
            (16, "Xeryon position error limit (ELIM)"),
            (18, "Xeryon safety timeout (TOU2)"),
            (21, "Xeryon positioning timeout (TOU3)"),
        ] {
            assert_eq!(controller_fault(1089 | (1 << bit)), Some(expected));
        }
        let mut m = Motion::new();
        for condition in 0..5 {
            m.start(228, 100);
            let mut t = telemetry(100);
            let now = match condition {
                0 => {
                    t.mounted = false;
                    101
                }
                1 => 3100,
                2 => {
                    t.record(b"STAT", 1 << 16, 101);
                    101
                }
                3 => {
                    t = telemetry(5100);
                    5100
                }
                _ => {
                    t = telemetry(60100);
                    m.heartbeat(60100);
                    60100
                }
            };
            assert!(m.poll(&t, now));
            assert_eq!(m.state, 4);
            assert_eq!(
                m.interruption_reason,
                Some(
                    [
                        "USB2 disconnected",
                        "encoder feedback stale",
                        "Xeryon position error limit (ELIM)",
                        "desktop heartbeat missing",
                        "move deadline",
                    ][condition]
                )
            );
            assert!(!m.poll(&t, now));
        }
        m.start(228, u32::MAX - 100);
        let t = telemetry(10);
        m.heartbeat(10);
        assert!(!m.poll(&t, 10));
        assert_eq!(m.state, 1);
    }
}

#[path = "../src/trajectory.rs"]
mod trajectory;
