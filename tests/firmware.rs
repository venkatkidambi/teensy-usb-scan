// Native tests for hardware-independent firmware modules.
#[path = "../src/line.rs"]
mod line;
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

#[test]
fn pressure_snapshot_preserves_i2c_values_and_identity() {
    let mut telemetry = protocol::Telemetry::new();
    telemetry.pressure = Some(pressure::Reading {
        raw: 0xffffff,
        average: 0xffffff,
        ms: 42,
        samples: 16,
        valid: true,
        state: 2,
        who: Some(0xb1),
        temperature: 65535,
        ..Default::default()
    });
    let mut frame = protocol::Frame::new();
    telemetry.snapshot(&mut frame, 7, 50, 0).unwrap();
    let json = core::str::from_utf8(frame.pending()).unwrap();
    assert!(json.contains("\"raw\":16777215,\"average\":16777215"));
    assert!(json.contains("\"sensor\":\"i2c\",\"address\":92,\"temperature\":65535,\"who\":177"));
}

#[path = "../src/experiment.rs"]
mod experiment;
#[path = "../src/experiment_data.rs"]
mod experiment_data;
