//! Firmware engine -> actual binary wire framing -> native receiver interoperability.
#![allow(dead_code)]
#[path = "../../src/command.rs"]
mod command;
#[path = "../../src/experiment.rs"]
mod engine;
#[path = "../../src/experiment_data.rs"]
mod experiment_data;
#[path = "../../src/line.rs"]
mod line;
#[path = "../../src/plunger.rs"]
mod plunger;
#[path = "../../src/plunger_config.rs"]
mod plunger_config;
#[path = "../../src/pressure.rs"]
mod pressure;
#[path = "../../src/protocol.rs"]
mod protocol;
#[path = "../src/experiment.rs"]
mod receiver;
#[path = "../../src/xeryon_motion.rs"]
mod xeryon_motion;
use experiment_data::*;
use serde_json::{Value, json};

struct Io;
impl engine::Actuator for Io {
    fn speed(&mut self, _: u32) -> Result<(), &'static str> {
        Ok(())
    }
    fn query_motor_timeout(&mut self) -> Result<(), &'static str> {
        Ok(())
    }
    fn tx_idle(&self) -> bool {
        true
    }
    fn target(&mut self, _: i32, _: u32, _: bool) -> Result<(), &'static str> {
        Ok(())
    }
    fn stop(&mut self) -> Result<(), &'static str> {
        Ok(())
    }
}
fn telemetry(now: u32) -> protocol::Telemetry {
    let mut t = protocol::Telemetry::new();
    t.mounted = true;
    t.vid = 0x04d8;
    t.pid = 0xef4a;
    t.cdc = 2;
    for (tag, value) in [(b"EPOS", -123), (b"DPOS", 0), (b"STAT", 1344)] {
        t.record(tag, value, now);
    }
    let mut m = xeryon_motion::Motion::new();
    m.start(0, now);
    m.state = 2;
    t.xeryon_motion = Some(m);
    t
}
fn pressure(now: u32) -> pressure::Reading {
    pressure::Reading {
        raw: 0x123456,
        valid: true,
        state: 2,
        ms: now,
        ..Default::default()
    }
}
fn meta(run: u32, bytes: &[u8]) -> Value {
    json!({"type":"data_meta","id":receiver::DOWNLOAD_ID,"version":1,"run":run,"record_bytes":RECORD_BYTES,"count":bytes.len()/RECORD_BYTES,"bytes":bytes.len(),"crc":!crc32(!0,bytes),"sample_ms":SAMPLE_MS,"outcome":"completed","reason":"none","config":{"kind":"sinusoidal"}})
}
#[test]
fn firmware_download_is_complete_before_ack_and_csv_preserves_raw_values() {
    let mut records = [Record::EMPTY; 256];
    let mut e = engine::Engine::new(&mut records);
    let config = ExperimentConfig::Sinusoidal {
        center_nm: 0,
        amplitude_nm: 780,
        frequency_mhz: 1000,
        cycles: 1,
    };
    e.accept(config, &telemetry(0), pressure(0), false, 0)
        .unwrap();
    e.begin(0, &mut Io);
    e.poll(1, &telemetry(1), pressure(1), &mut Io);
    for now in 2..=512 {
        e.poll(now, &telemetry(now), pressure(now), &mut Io);
    }
    e.abort(&mut Io);
    assert_eq!(e.len, 11);
    let mut d = receiver::Download::default();
    assert!(d.begin(e.run).starts_with("GET_DATA "));
    e.transfer(e.run).unwrap();
    let mut text = String::new();
    e.metadata(&mut text, receiver::DOWNLOAD_ID).unwrap();
    d.metadata(serde_json::from_str(&text).unwrap()).unwrap();
    let mut decoder = receiver::Decoder::default();
    let mut dataset = None;
    while dataset.is_none() {
        let offset = d
            .next()
            .unwrap()
            .0
            .split_ascii_whitespace()
            .last()
            .unwrap()
            .parse::<usize>()
            .unwrap();
        let chunk = e.chunk(e.run, offset).unwrap();
        let payload: Vec<_> = chunk.iter().flat_map(|r| r.encode()).collect();
        let header = json!({"type":"data_chunk","id":receiver::DOWNLOAD_ID,"run":e.run,"offset":offset,"bytes":payload.len()});
        let wire: Vec<_> = format!("{header}\n").bytes().chain(payload).collect();
        // Every split including arbitrary LF, CR, NUL and non-UTF8 in binary is opaque.
        for fragment in wire.chunks(7) {
            for &byte in fragment {
                if let Some(event) = decoder.push(byte).unwrap() {
                    match event {
                        receiver::Event::Chunk(h, b) => d.chunk(h, &b).unwrap(),
                        _ => panic!("binary leaked into text"),
                    }
                }
            }
        }
        let (ack, result) = d.next().unwrap();
        dataset = result;
        if dataset.is_some() {
            assert!(e.owns_hardware());
            assert!(ack.starts_with("ACK_DATA_RECEIVED "));
            e.acknowledge(e.run, e.len, e.checksum()).unwrap();
        }
    }
    let dataset = dataset.unwrap();
    assert_eq!(dataset.samples.len(), 11);
    assert_eq!(dataset.samples[0][1], -123);
    assert!(dataset.csv().contains("-9.594,1193046"));
    assert_eq!(dataset.metadata["outcome"], "aborted");
    assert!(!e.owns_hardware());
}
#[test]
fn corruption_and_wrong_count_offset_or_run_never_produce_ack() {
    let bytes = Record {
        epos: -100,
        pressure_raw: 0x00ff_0a0d,
        ..Record::EMPTY
    }
    .encode();
    for bad in 0..5 {
        let mut d = receiver::Download::default();
        d.begin(2);
        let mut metadata = meta(2, &bytes);
        if bad == 0 {
            metadata["bytes"] = json!(41);
            assert!(d.metadata(metadata).is_err());
            continue;
        }
        d.metadata(metadata).unwrap();
        let mut header = json!({"id":receiver::DOWNLOAD_ID,"run":2,"offset":0,"bytes":40});
        if bad == 1 {
            header["offset"] = json!(1);
        }
        if bad == 2 {
            header["run"] = json!(3);
        }
        let mut corrupt = bytes;
        if bad == 3 {
            corrupt[3] ^= 1;
        }
        if bad == 4 {
            header["bytes"] = json!(39);
        }
        if bad == 3 {
            d.chunk(header, &corrupt).unwrap();
            assert!(d.next().is_err());
        } else {
            assert!(d.chunk(header, &bytes).is_err());
        }
    }
}
#[test]
fn empty_fault_dataset_and_maximum_payload_transfer() {
    for count in [0, CAPACITY] {
        let bytes: Vec<_> = (0..count)
            .flat_map(|i| {
                Record {
                    elapsed_ms: i as u32 * SAMPLE_MS,
                    ..Record::EMPTY
                }
                .encode()
            })
            .collect();
        let mut d = receiver::Download::default();
        d.begin(3);
        d.metadata(meta(3, &bytes)).unwrap();
        for (i, chunk) in bytes.chunks(RECORD_BYTES * 16).enumerate() {
            assert!(d.next().unwrap().1.is_none());
            d.chunk(
                json!({"id":receiver::DOWNLOAD_ID,"run":3,"offset":i*16,"bytes":chunk.len()}),
                chunk,
            )
            .unwrap();
        }
        assert_eq!(d.next().unwrap().1.unwrap().samples.len(), count);
    }
}
#[test]
fn truncated_binary_can_only_recover_with_a_fresh_decoder_and_download() {
    let mut decoder = receiver::Decoder::default();
    let header = json!({"type":"data_chunk","bytes":40});
    for b in format!("{header}\n").bytes().chain([0; 39]) {
        assert!(decoder.push(b).unwrap().is_none());
    }
    let mut fresh = receiver::Decoder::default();
    let events: Vec<_> = b"# reconnect\n"
        .iter()
        .filter_map(|&b| fresh.push(b).unwrap())
        .collect();
    assert_eq!(events.len(), 1);
    for bytes in [0, 39, 641, 1_000_000] {
        let mut decoder = receiver::Decoder::default();
        let mut error = false;
        for b in format!("{{\"type\":\"data_chunk\",\"bytes\":{bytes}}}\n").bytes() {
            error |= decoder.push(b).is_err();
        }
        assert!(error);
    }
}
#[test]
fn native_configuration_is_validated_in_rust() {
    let command = receiver::Input {
        kind: "quasi_static".into(),
        speed_um_s: Some(10),
        start_um: -1.234,
        increment_um: 0.078,
        timing: 0.5,
        repeats: 2,
    }
    .command(5)
    .unwrap();
    assert_eq!(command, "START 5 Q -1234 78 500 2 10");
    let command::Command::Start(id, config) = command::Command::parse(command.as_bytes()).unwrap()
    else {
        panic!("Expected START");
    };
    assert_eq!(id, 5);
    assert!(config.validate().is_ok());
    for speed in [1, 10, MAX_SPEED_UM_S] {
        let config: receiver::Input = serde_json::from_value(json!({
            "kind":"quasi_static", "start_um":0, "increment_um":1,
            "timing":0.5, "repeats":2, "speed_um_s":speed,
        }))
        .unwrap();
        assert_eq!(
            config.command(1).unwrap(),
            format!("START 1 Q 0 1000 500 2 {speed}")
        );
    }
    for speed in [
        json!(null),
        json!(0),
        json!(16777216),
        json!(-1),
        json!(1.5),
    ] {
        let result = serde_json::from_value::<receiver::Input>(json!({
            "kind":"quasi_static", "start_um":0, "increment_um":1,
            "timing":0.5, "repeats":2, "speed_um_s":speed,
        }));
        assert!(
            result
                .map(|input| input.command(1).is_err())
                .unwrap_or(true)
        );
    }
    for command in [
        "START 1 Q 0 78 500 2",
        "START 1 Q 0 78 500 2 0",
        "START 1 Q 0 78 500 2 16777216",
        "START 1 Q 0 78 500 2 1.5",
        "START 1 S 0 78 500 2 10",
        "START 1 Q 0 78 500 2 10 extra",
    ] {
        assert!(command::Command::parse(command.as_bytes()).is_err());
    }
    for timing in [0.0, 2.001, f64::NAN, f64::INFINITY] {
        assert!(
            receiver::Input {
                kind: "sinusoidal".into(),
                speed_um_s: None,
                start_um: 0.0,
                increment_um: 1.0,
                timing,
                repeats: 1
            }
            .command(1)
            .is_err()
        );
    }
}
