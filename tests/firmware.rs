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

#[path = "../src/experiment.rs"]
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
    };
    let bytes = [
        50, 0, 0, 0, 255, 255, 255, 255, 254, 255, 255, 255, 0, 0, 0, 128, 1, 0, 0, 0, 2, 0, 0, 0,
        3, 0, 0, 0, 0, 0, 0, 128, 255, 255, 255, 255, 255, 255, 255, 255,
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
            4294967295
        ]
    );
}
