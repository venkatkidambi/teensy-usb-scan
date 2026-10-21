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
#[path = "../../src/trajectory.rs"]
mod trajectory;
#[path = "../../src/xeryon_motion.rs"]
mod xeryon_motion;
use experiment_data::*;
use serde_json::{Value, json};

struct Io {
    complete: bool,
}

#[derive(Default)]
struct TrackingIo {
    target: i32,
    stops: usize,
}
impl engine::Actuator for TrackingIo {
    fn prepare_serial(&mut self, _: u32) -> Result<bool, &'static str> {
        Ok(true)
    }
    fn serial_commands(
        &mut self,
        commands: &[engine::serial_sine::Command],
    ) -> Result<(), &'static str> {
        for command in commands {
            if let engine::serial_sine::Command::Position(target) = command {
                self.target = *target;
            }
        }
        Ok(())
    }
    fn prepare_gpio(&mut self, _: u32) -> Result<bool, &'static str> {
        unreachable!()
    }
    fn position_gpio(&mut self, _: ExperimentConfig, _: u32) -> Result<(), &'static str> {
        unreachable!()
    }
    fn start_gpio(&mut self, _: ExperimentConfig) -> Result<(), &'static str> {
        unreachable!()
    }
    fn gpio_status(&self) -> trajectory::Status {
        trajectory::Status::Stopped
    }
    fn stop_gpio(&mut self) {}
    fn speed(&mut self, _: u32) -> Result<(), &'static str> {
        Ok(())
    }
    fn query_motor_timeout(&mut self) -> Result<(), &'static str> {
        Ok(())
    }
    fn tx_idle(&self) -> bool {
        true
    }
    fn target(&mut self, target: i32, _: u32, _: bool) -> Result<(), &'static str> {
        self.target = target;
        Ok(())
    }
    fn stop(&mut self) -> Result<(), &'static str> {
        self.stops += 1;
        Ok(())
    }
}
fn tracking_poll(e: &mut engine::Engine<'_>, io: &mut TrackingIo, now: u32) {
    let mut t = telemetry(now);
    t.record(b"EPOS", i64::from(io.target), now);
    t.record(b"DPOS", i64::from(io.target), now);
    t.record(b"SSPD", i64::from(e.config.unwrap().speed().unwrap()), now);
    t.record(b"TOU2", 60, now);
    t.speed_updates = now.wrapping_add(1);
    t.motor_timeout_updates = now.wrapping_add(1);
    t.xeryon_motion.as_mut().unwrap().target = Some(io.target);
    e.poll(now, &t, pressure(now), io);
}
fn begin_batch(e: &mut engine::Engine<'_>, archive: &receiver::Archive) -> receiver::Download {
    let mut download = receiver::Download::default();
    download.begin(e.run);
    e.transfer(e.run).unwrap();
    let mut metadata = String::new();
    e.metadata(&mut metadata, receiver::DOWNLOAD_ID).unwrap();
    download
        .metadata_from_archive(serde_json::from_str(&metadata).unwrap(), archive)
        .unwrap();
    download
}
fn transfer_step(
    e: &mut engine::Engine<'_>,
    d: &mut receiver::Download,
) -> Option<(String, receiver::Dataset)> {
    let (command, dataset) = d.next().unwrap();
    if let Some(dataset) = dataset {
        return Some((command, dataset));
    }
    let command::Command::GetChunk(_, run, offset) =
        command::Command::parse(command.as_bytes()).unwrap()
    else {
        panic!("Expected chunk request")
    };
    let bytes: Vec<_> = e
        .chunk(run, offset as usize)
        .unwrap()
        .iter()
        .flat_map(|r| r.encode())
        .collect();
    d.chunk(
        json!({"id":receiver::DOWNLOAD_ID,"run":run,"offset":offset,"bytes":bytes.len()}),
        &bytes,
    )
    .unwrap();
    None
}
fn ack_batch(e: &mut engine::Engine<'_>, ack: &str) {
    let command::Command::AckData(_, run, count, crc) =
        command::Command::parse(ack.as_bytes()).unwrap()
    else {
        panic!("Expected ACK")
    };
    e.acknowledge(run, count as usize, crc).unwrap();
}

#[test]
fn long_quasi_static_and_sine_runs_reuse_storage_and_publish_every_sample_once() {
    for config in [
        ExperimentConfig::QuasiStatic {
            start_nm: 0,
            step_nm: 78,
            hold_ms: 130_000,
            steps: 1,
            speed_um_s: 10,
        },
        ExperimentConfig::Sinusoidal {
            center_nm: 0,
            amplitude_nm: 780,
            frequency_mhz: 1000,
            cycles: 130,
            control: SineControl::Displacement,
        },
    ] {
        let mut records = [Record::EMPTY; CAPACITY];
        let mut e = engine::Engine::new(&mut records);
        let mut io = TrackingIo::default();
        e.accept(config, &telemetry(0), pressure(0), false, 0)
            .unwrap();
        e.begin(0, &mut io);
        let mut archive = receiver::Archive::default();
        archive.begin(false);
        let mut download = None;
        let mut batches = 0;
        for now in (0..150_000).step_by(5) {
            tracking_poll(&mut e, &mut io, now);
            if e.batch_ready() && download.is_none() {
                download = Some(begin_batch(&mut e, &archive));
            }
            if let Some(d) = &mut download {
                // Acquisition advances between every chunk, including across ring wraps.
                if let Some((ack, batch)) = transfer_step(&mut e, d) {
                    let final_batch = batch.metadata["final"] == true;
                    assert_eq!(archive.receive(batch).unwrap().is_some(), final_batch);
                    ack_batch(&mut e, &ack);
                    batches += 1;
                    download = None;
                    if final_batch {
                        break;
                    }
                    assert!(e.owns_hardware());
                    assert!(archive.datasets.is_empty());
                }
            }
        }
        assert_eq!(e.state, engine::State::Dashboard);
        assert_eq!(e.outcome, "completed", "{}", e.reason);
        assert!(batches >= 6);
        assert_eq!(archive.datasets.len(), 1);
        let dataset = &archive.datasets[0];
        assert!(dataset.samples.len() > CAPACITY * 3);
        assert_eq!(dataset.samples.len(), e.len);
        assert_eq!(dataset.samples[0][0], 0);
        assert!(
            dataset
                .samples
                .windows(2)
                .all(|pair| pair[1][0] - pair[0][0] == i64::from(SAMPLE_MS))
        );
        assert_eq!(dataset.metadata["count"], e.len);
        assert_eq!(dataset.metadata["bytes"], e.len * RECORD_BYTES);
        assert_eq!(dataset.metadata["crc"], e.checksum());
        assert_eq!(dataset.csv().lines().count(), e.len + 2);
    }
}

#[test]
fn abort_during_batch_and_lost_ack_reconnect_preserve_prefix_and_terminal_tail() {
    let mut records = [Record::EMPTY; 32];
    let mut e = engine::Engine::new(&mut records);
    let mut io = TrackingIo::default();
    let config = ExperimentConfig::QuasiStatic {
        start_nm: 0,
        step_nm: 78,
        hold_ms: 10_000,
        steps: 1,
        speed_um_s: 10,
    };
    e.accept(config, &telemetry(0), pressure(0), false, 0)
        .unwrap();
    e.begin(0, &mut io);
    let mut archive = receiver::Archive::default();
    let mut now = 0;
    while !e.batch_ready() {
        tracking_poll(&mut e, &mut io, now);
        now += 5;
    }
    let mut d = begin_batch(&mut e, &archive);
    // Freeze the batch, then acquire more data and abort while it is downloading.
    for _ in 0..8 {
        tracking_poll(&mut e, &mut io, now);
        now += 5;
    }
    e.abort(&mut io);
    let (lost_ack, batch) = loop {
        if let Some(result) = transfer_step(&mut e, &mut d) {
            break result;
        }
    };
    assert_eq!(batch.metadata["outcome"], "running");
    assert!(archive.receive(batch).unwrap().is_none());
    assert!(e.owns_hardware());
    // New serial worker, same archive: retry a verified batch whose ACK was lost.
    let mut retry = begin_batch(&mut e, &archive);
    let (ack, batch) = loop {
        if let Some(result) = transfer_step(&mut e, &mut retry) {
            break result;
        }
    };
    assert_eq!(ack, lost_ack);
    assert!(archive.receive(batch).unwrap().is_none());
    ack_batch(&mut e, &ack);
    assert!(e.owns_hardware());
    let mut d = begin_batch(&mut e, &archive);
    let (ack, batch) = loop {
        if let Some(result) = transfer_step(&mut e, &mut d) {
            break result;
        }
    };
    assert_eq!(batch.metadata["outcome"], "aborted");
    let completed = archive.receive(batch).unwrap().unwrap();
    assert_eq!(completed.samples.len(), e.len);
    assert_eq!(completed.samples[0][0], 0);
    ack_batch(&mut e, &ack);
    assert!(!e.owns_hardware());
}

#[test]
fn stalled_receiver_faults_without_overwriting_samples() {
    let mut records = [Record::EMPTY; 32];
    let mut e = engine::Engine::new(&mut records);
    let mut io = TrackingIo::default();
    let config = ExperimentConfig::QuasiStatic {
        start_nm: 0,
        step_nm: 78,
        hold_ms: 10_000,
        steps: 1,
        speed_um_s: 10,
    };
    e.accept(config, &telemetry(0), pressure(0), false, 0)
        .unwrap();
    e.begin(0, &mut io);
    for now in (0..1000).step_by(5) {
        tracking_poll(&mut e, &mut io, now);
    }
    assert_eq!(e.outcome, "faulted");
    assert_eq!(e.reason, "PC transfer stalled; sample buffer full");
    assert_eq!(e.len, 32);
    assert!(io.stops > 0);
    let mut archive = receiver::Archive::default();
    let mut d = begin_batch(&mut e, &archive);
    let (ack, batch) = loop {
        if let Some(result) = transfer_step(&mut e, &mut d) {
            break result;
        }
    };
    let dataset = archive.receive(batch).unwrap().unwrap();
    assert_eq!(dataset.samples[0][0], 0);
    assert_eq!(dataset.samples.last().unwrap()[0], 310);
    ack_batch(&mut e, &ack);
}

#[test]
fn batch_prefix_integrity_retries_and_empty_final_batch() {
    let bytes: Vec<_> = (0..4)
        .flat_map(|i| {
            Record {
                elapsed_ms: i * SAMPLE_MS,
                ..Record::EMPTY
            }
            .encode()
        })
        .collect();
    let metadata = |start: usize, end: usize, final_batch: bool| {
        let mut m = meta(7, &bytes[..end * RECORD_BYTES]);
        m["offset"] = start.into();
        m["bytes"] = ((end - start) * RECORD_BYTES).into();
        m["final"] = final_batch.into();
        m["outcome"] = if final_batch { "completed" } else { "running" }.into();
        m
    };
    let mut archive = receiver::Archive::default();
    // A second batch can be retried with a fresh worker after losing its ACK.
    for (start, end) in [(0, 2), (2, 4), (2, 4)] {
        let mut d = receiver::Download::default();
        d.begin(7);
        d.metadata_from_archive(metadata(start, end, false), &archive)
            .unwrap();
        let chunk = &bytes[start * RECORD_BYTES..end * RECORD_BYTES];
        d.chunk(
            json!({"id":receiver::DOWNLOAD_ID,"run":7,"offset":start,"bytes":chunk.len()}),
            chunk,
        )
        .unwrap();
        let (_, batch) = d.next().unwrap();
        assert!(archive.receive(batch.unwrap()).unwrap().is_none());
    }
    for bad in 0..5 {
        let mut d = receiver::Download::default();
        d.begin(7);
        let mut m = metadata(2, 4, false);
        match bad {
            0 => {
                // Correct suffix is insufficient without its acknowledged prefix.
                assert!(d.metadata(m).is_err());
                continue;
            }
            1 => {
                m["offset"] = 3.into();
                m["bytes"] = RECORD_BYTES.into();
            }
            2 => m["config"] = json!({"kind":"quasi_static"}),
            _ => {}
        }
        if bad < 3 {
            assert!(d.metadata_from_archive(m, &archive).is_err());
            continue;
        }
        let mut chunk = bytes[2 * RECORD_BYTES..].to_vec();
        chunk[0] = 0; // Timestamp crosses backwards over the batch boundary.
        if bad == 4 {
            // Even a valid cumulative CRC cannot excuse a backwards timestamp.
            m["crc"] = (!crc32(crc32(!0, &bytes[..2 * RECORD_BYTES]), &chunk)).into();
        }
        d.metadata_from_archive(m, &archive).unwrap();
        d.chunk(
            json!({"id":receiver::DOWNLOAD_ID,"run":7,"offset":2,"bytes":chunk.len()}),
            &chunk,
        )
        .unwrap();
        let error = d.next().err().unwrap();
        assert!(error.contains(if bad == 3 { "CRC" } else { "timestamps" }));
    }
    // Completion exactly after an ACK still needs a final outcome and hardware release.
    for attempt in 0..2 {
        let mut d = receiver::Download::default();
        d.begin(7);
        d.metadata_from_archive(metadata(4, 4, true), &archive)
            .unwrap();
        let (ack, batch) = d.next().unwrap();
        assert!(ack.starts_with("ACK_DATA_RECEIVED"));
        assert_eq!(
            archive.receive(batch.unwrap()).unwrap().is_some(),
            attempt == 0
        );
    }
    assert_eq!(archive.datasets.len(), 1);
    assert_eq!(archive.datasets[0].samples.len(), 4);
}

#[test]
fn csv_exports_direct_force_with_raw_counts_ranges_and_calibration_provenance() {
    let counts = [
        4_200_000, 4_300_000, 4_163_833, 4_163_832, 4_281_744, 4_281_745, 0,
    ];
    let dataset = receiver::Dataset {
        archive_index: None,
        metadata: json!({"run": 1, "outcome": "completed"}),
        samples: counts
            .iter()
            .map(|&raw| {
                let mut sample = [0; RECORD_FIELDS];
                sample[2] = raw;
                sample
            })
            .collect(),
    };
    let csv = dataset.csv();
    let lines: Vec<_> = csv.lines().collect();
    assert_eq!(lines.len(), counts.len() + 2);
    let metadata: Value = serde_json::from_str(
        &lines[0]
            .strip_prefix("metadata,\"")
            .unwrap()
            .strip_suffix('"')
            .unwrap()
            .replace("\"\"", "\""),
    )
    .unwrap();
    assert_eq!(
        metadata["force_calibration"]["id"],
        "lps-probe-force-2026-08-27-v1"
    );
    assert_eq!(
        metadata["force_calibration"]["observations"]["lps_counts_probe_counts"]
            .as_array()
            .unwrap()
            .len(),
        50
    );
    assert!(dataset.metadata.get("force_calibration").is_none());
    let header: Vec<_> = lines[1].split(',').collect();
    assert_eq!(
        &header[17..20],
        &["force_n", "force_mn", "force_extrapolated"]
    );
    for (i, expected) in [false, true, false, true, false, true, true]
        .iter()
        .enumerate()
    {
        let row: Vec<_> = lines[i + 2].split(',').collect();
        assert_eq!(row.len(), header.len());
        assert_eq!(row[2].parse::<i64>().unwrap(), counts[i]);
        assert_eq!(row[19], expected.to_string());
        let force_n = row[17].parse::<f64>().unwrap();
        let force_mn = row[18].parse::<f64>().unwrap();
        assert!((force_n * 1000.0 - force_mn).abs() < 0.000001);
    }
    assert!(lines[2].contains(",0.012405235,12.405235,false,"));
    assert!(lines[3].contains(",0.021054229,21.054229,true,"));
    assert!(
        lines
            .last()
            .unwrap()
            .contains(",-0.350852500,-350.852500,true,")
    );
}

#[test]
fn final_return_metadata_above_old_line_limit_survives_download_and_csv() {
    let record = Record {
        elapsed_ms: 20,
        ..Record::EMPTY
    };
    let bytes = record.encode();
    let mut metadata = meta(8, &bytes);
    metadata["final_return"] = json!({
        "version": 1, "error_counts": -2, "error_um": -0.156,
        "echo": [17666, 55940], "final_feedback": [[17664, 56789], [17666, 56770], [1345, 56789], null],
        "expired": false
    });
    // Additional diagnostics already carried by firmware can fill the old limit.
    metadata["padding"] = json!("x".repeat(4500));
    let wire = format!("{metadata}\n");
    assert!(wire.len() > 4096 && wire.len() < 8192);
    let mut decoder = receiver::Decoder::default();
    let mut parsed = None;
    for byte in wire.bytes() {
        if let Some(receiver::Event::Line(value)) = decoder.push(byte).unwrap() {
            parsed = Some(serde_json::from_str(&value).unwrap());
        }
    }
    let mut download = receiver::Download::default();
    download.begin(8);
    download.metadata(parsed.unwrap()).unwrap();
    download
        .chunk(
            json!({"id":receiver::DOWNLOAD_ID,"run":8,"offset":0,"bytes":bytes.len()}),
            &bytes,
        )
        .unwrap();
    let dataset = download.next().unwrap().1.unwrap();
    assert_eq!(dataset.metadata["final_return"], metadata["final_return"]);
    assert!(dataset.csv().contains("\"\"error_counts\"\":-2"));
    // Oversize frames still fail boundedly and recover at the next newline.
    for byte in "x".repeat(8193).bytes() {
        decoder.push(byte).unwrap();
    }
    assert!(
        matches!(decoder.push(b'\n').unwrap(), Some(receiver::Event::Line(line)) if line.contains("Oversized"))
    );
}

impl engine::Actuator for Io {
    fn prepare_serial(&mut self, _: u32) -> Result<bool, &'static str> {
        Ok(true)
    }
    fn serial_commands(&mut self, _: &[engine::serial_sine::Command]) -> Result<(), &'static str> {
        Ok(())
    }

    fn prepare_gpio(&mut self, _: u32) -> Result<bool, &'static str> {
        Ok(true)
    }
    fn position_gpio(&mut self, _: ExperimentConfig, _: u32) -> Result<(), &'static str> {
        self.complete = true;
        Ok(())
    }
    fn start_gpio(&mut self, _: ExperimentConfig) -> Result<(), &'static str> {
        self.complete = false;
        Ok(())
    }
    fn gpio_status(&self) -> trajectory::Status {
        if self.complete {
            trajectory::Status::Complete
        } else {
            trajectory::Status::Running
        }
    }
    fn stop_gpio(&mut self) {}
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
    m.start(-10, now);
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
    json!({"type":"data_meta","id":receiver::DOWNLOAD_ID,"version":DATA_VERSION,"run":run,"record_bytes":RECORD_BYTES,"count":bytes.len()/RECORD_BYTES,"bytes":bytes.len(),"crc":!crc32(!0,bytes),"sample_ms":SAMPLE_MS,"outcome":"completed","reason":"none","config":{"kind":"sinusoidal"}})
}
#[test]
fn archive_retains_only_the_current_experiment_and_keeps_previous_data_until_verified() {
    let mut archive = receiver::Archive::default();
    let mut dataset = receiver::Dataset {
        archive_index: None,
        metadata: json!({"run":1}),
        samples: vec![],
    };
    archive.begin(false);
    archive.retain(&mut dataset);
    assert_eq!(dataset.archive_index, Some(0));
    archive.begin(true);
    dataset.metadata["run"] = json!(2);
    archive.retain(&mut dataset);
    assert_eq!(dataset.archive_index, Some(1));
    assert_eq!(archive.datasets.len(), 2);

    // START rejection or a disconnected transfer does not erase verified data.
    archive.begin(false);
    assert_eq!(archive.datasets.len(), 2);
    dataset.metadata["run"] = json!(3);
    archive.retain(&mut dataset);
    assert_eq!(dataset.archive_index, Some(0));
    assert_eq!(archive.datasets.len(), 1);
    assert_eq!(archive.datasets[0].metadata["run"], 3);
    archive.begin(true);
    dataset.metadata["run"] = json!(4);
    archive.retain(&mut dataset);
    assert_eq!(archive.datasets.len(), 2);
    assert_eq!(dataset.archive_index, Some(1));

    archive.begin(false);
    dataset.metadata["run"] = json!(5);
    archive.retain(&mut dataset);
    assert_eq!(archive.datasets.len(), 1);
    assert_eq!(archive.datasets[0].metadata["run"], 5);
}

#[test]
fn independent_signal_times_and_identities_round_trip_into_csv() {
    let records = [
        Record {
            elapsed_ms: 0,
            epos: 10,
            pressure_raw: 100,
            epos_age_ms: 17,
            controller_age_ms: u32::MAX,
            pressure_age_ms: 3,
            epos_generation: u32::MAX,
            pressure_generation: 1,
            ..Record::EMPTY
        },
        Record {
            elapsed_ms: 50,
            epos: 10,
            pressure_raw: 100,
            epos_age_ms: 67,
            controller_age_ms: 7,
            pressure_age_ms: 2,
            epos_generation: u32::MAX,
            pressure_generation: 2,
            ..Record::EMPTY
        },
        Record {
            elapsed_ms: 100,
            epos: 10,
            pressure_raw: 100,
            epos_age_ms: 5,
            pressure_age_ms: 1,
            epos_generation: 0,
            pressure_generation: 3,
            ..Record::EMPTY
        },
    ];
    let bytes: Vec<_> = records.iter().flat_map(|r| r.encode()).collect();
    let mut d = receiver::Download::default();
    d.begin(8);
    d.metadata(meta(8, &bytes)).unwrap();
    d.chunk(
        json!({"id":receiver::DOWNLOAD_ID,"run":8,"offset":0,"bytes":bytes.len()}),
        &bytes,
    )
    .unwrap();
    let dataset = d.next().unwrap().1.unwrap();
    assert_eq!(dataset.samples[0][10], i64::from(u32::MAX));
    let csv = dataset.csv();
    let lines: Vec<_> = csv.lines().collect();
    assert!(lines[1].contains("epos_host_receipt_s,epos_generation"));
    assert!(!lines[1].contains("pressure_i2c_completion_s"));
    assert!(
        lines[1]
            .ends_with("controller_host_receipt_s,epos_timestamp_source,epos_controller_time_s")
    );
    assert!(lines[2].ends_with(",,host_receipt_ms,"));
    assert!(lines[3].ends_with(",0.043,host_receipt_ms,"));
    assert_eq!(
        &lines[2].split(',').collect::<Vec<_>>()[12..17],
        &["-0.017", "4294967295", "1", "1", "1"]
    );
    assert_eq!(
        &lines[3].split(',').collect::<Vec<_>>()[12..17],
        &["-0.017", "4294967295", "2", "0", "1"]
    );
    assert_eq!(
        &lines[4].split(',').collect::<Vec<_>>()[12..17],
        &["0.095", "0", "3", "1", "1"]
    );
    let mut legacy = meta(8, &bytes);
    legacy["version"] = json!(1);
    assert!(d.metadata(legacy).is_err());
}
#[test]
fn firmware_download_is_complete_before_ack_and_csv_preserves_raw_values() {
    let mut records = [Record::EMPTY; CAPACITY];
    let mut e = engine::Engine::new(&mut records);
    let config = ExperimentConfig::Sinusoidal {
        center_nm: 0,
        amplitude_nm: 780,
        frequency_mhz: 1000,
        cycles: 1,
        control: crate::experiment_data::SineControl::Gpio,
    };
    e.accept(config, &telemetry(0), pressure(0), false, 0)
        .unwrap();
    let mut io = Io { complete: false };
    e.begin(0, &mut io);
    e.poll(0, &telemetry(0), pressure(0), &mut io);
    let endpoint = |now| {
        let mut t = telemetry(now);
        t.record(b"EPOS", -10, now);
        t
    };
    e.poll(1, &endpoint(1), pressure(1), &mut io);
    for now in 2..=1012 {
        if now > 2 {
            io.complete = now >= 1002;
        }
        let t = if now == 2 || now > 1002 {
            endpoint(now)
        } else {
            telemetry(now)
        };
        e.poll(now, &t, pressure(now), &mut io);
    }
    e.abort(&mut io); // A late ABORT cannot overwrite a completed run.
    assert_eq!(e.len, 101);
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
        let mut text = String::new();
        e.chunk_header(
            &mut text,
            receiver::DOWNLOAD_ID,
            e.run,
            offset,
            payload.len(),
        )
        .unwrap();
        let header: Value = serde_json::from_str(&text).unwrap();
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
    assert_eq!(dataset.samples.len(), 101);
    assert_eq!(dataset.samples[0][1], -10);
    assert_eq!(dataset.samples[0][7], -10); // negative extremum at rest
    assert_eq!(dataset.samples[24][7], -1); // approaching center at t=240 ms
    assert_eq!(dataset.samples[50][7], 10); // positive extremum at half-cycle
    assert!(dataset.csv().contains("-9.594,1193046"));
    let csv = dataset.csv();
    assert_eq!(
        csv.lines().nth(2).unwrap().split(',').nth(7),
        Some("-0.780")
    );
    assert_eq!(dataset.metadata["outcome"], "completed");
    assert_eq!(dataset.metadata["final_return"]["error_counts"], 0);
    assert_eq!(dataset.metadata["final_return"]["error_um"], 0.0);
    assert_eq!(dataset.metadata["final_return"]["finished_ms"], 1003);
    assert_eq!(dataset.metadata["final_return"]["expired"], false);
    assert!(dataset.csv().contains("final_feedback"));
    assert_eq!(dataset.metadata["sine_control"], "gpio_pulse_direction");
    assert_eq!(dataset.metadata["record_bytes"], RECORD_BYTES);
    assert_eq!(dataset.metadata["sample_ms"], 10);
    assert!(
        dataset
            .samples
            .windows(2)
            .all(|pair| pair[1][0] - pair[0][0] == 10)
    );
    let csv = dataset.csv();
    let times: Vec<_> = csv
        .lines()
        .skip(2)
        .map(|row| row.split(',').next().unwrap())
        .collect();
    assert_eq!(&times[..3], &["0.000", "0.010", "0.020"]);
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
            metadata["bytes"] = json!(RECORD_BYTES + 1);
            assert!(d.metadata(metadata).is_err());
            continue;
        }
        d.metadata(metadata).unwrap();
        let mut header =
            json!({"id":receiver::DOWNLOAD_ID,"run":2,"offset":0,"bytes":RECORD_BYTES});
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
            header["bytes"] = json!(RECORD_BYTES - 1);
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
    let header = json!({"type":"data_chunk","bytes":RECORD_BYTES});
    for b in format!("{header}\n").bytes().chain([0; RECORD_BYTES - 1]) {
        assert!(decoder.push(b).unwrap().is_none());
    }
    let mut fresh = receiver::Decoder::default();
    let events: Vec<_> = b"# reconnect\n"
        .iter()
        .filter_map(|&b| fresh.push(b).unwrap())
        .collect();
    assert_eq!(events.len(), 1);
    for bytes in [
        0,
        RECORD_BYTES - 1,
        RECORD_BYTES * CHUNK_RECORDS + 1,
        1_000_000,
    ] {
        let mut decoder = receiver::Decoder::default();
        let mut error = false;
        for b in format!("{{\"type\":\"data_chunk\",\"bytes\":{bytes}}}\n").bytes() {
            error |= decoder.push(b).is_err();
        }
        assert!(error);
    }
}
#[test]
fn native_quasi_static_start_uses_step_travel_budget() {
    for repeats in [4, 10, 18, 19] {
        let input: receiver::Input = serde_json::from_value(json!({
            "kind":"quasi_static", "start_um":0, "increment_um":1,
            "timing":1, "repeats":repeats, "speed_um_s":10,
        }))
        .unwrap();
        let result = input.command(1);
        let wire = result.unwrap();
        assert_eq!(wire, format!("START 1 Q 0 1000 1000 {repeats} 10"));
        let command::Command::Start(_, config) = command::Command::parse(wire.as_bytes()).unwrap()
        else {
            panic!("Expected START");
        };
        assert_eq!(config.validate(), Ok(repeats * 2216));
    }
}
#[test]
fn native_configuration_is_validated_in_rust() {
    for timing in [0.1, 2.001, 5.0, 10.0] {
        let input: receiver::Input = serde_json::from_value(json!({
            "kind":"sinusoidal", "start_um":0, "increment_um":1,
            "timing":timing, "repeats":1,
        }))
        .unwrap();
        assert!(input.command(1).is_ok());
    }
    let command = receiver::Input {
        sine_control: None,
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
    for timing in [0.0, 10.001, f64::NAN, f64::INFINITY] {
        assert!(
            receiver::Input {
                sine_control: None,
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
#[test]
fn serial_selection_round_trips_from_ui_to_firmware() {
    for (mode, valid) in [
        ("velocity", false),
        ("velocity_position", false),
        ("displacement", true),
    ] {
        let input: receiver::Input = serde_json::from_value(json!({"kind":"sinusoidal", "start_um":0, "increment_um":0.078, "timing":0.1, "repeats":1, "sine_control":mode})).unwrap();
        assert_eq!(input.command(1).is_ok(), valid);
    }
    for (selection, suffix, control) in [
        (None, "D", experiment_data::SineControl::Displacement),
        (Some("gpio"), "G", experiment_data::SineControl::Gpio),
        (
            Some("displacement"),
            "D",
            experiment_data::SineControl::Displacement,
        ),
        (
            Some("velocity"),
            "V",
            experiment_data::SineControl::Velocity,
        ),
        (
            Some("velocity_position"),
            "VP",
            experiment_data::SineControl::VelocityPosition,
        ),
    ] {
        let input: receiver::Input = serde_json::from_value(json!({"kind":"sinusoidal", "start_um":1.5, "increment_um":2, "timing":1.3, "repeats":3, "sine_control":selection})).unwrap();
        let line = input.command(1).unwrap();
        assert_eq!(line, format!("START 1 S 1500 2000 1300 3 {suffix}"));
        let command::Command::Start(_, config) = command::Command::parse(line.as_bytes()).unwrap()
        else {
            panic!()
        };
        assert_eq!(config.control(), Some(control));
        assert!(config.validate().is_ok());
    }
    let input: receiver::Input = serde_json::from_value(json!({"kind":"sinusoidal", "start_um":0, "increment_um":1, "timing":1, "repeats":1, "sine_control":"unknown"})).unwrap();
    assert!(input.command(1).is_err());
}
#[path = "../../src/xeryon.rs"]
mod xeryon;
