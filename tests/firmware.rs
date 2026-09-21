// Native tests for hardware-independent firmware modules.
#[path = "../src/line.rs"]
mod line;
#[path = "../src/protocol.rs"]
mod protocol;
#[path = "../src/xeryon.rs"]
mod xeryon;

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
