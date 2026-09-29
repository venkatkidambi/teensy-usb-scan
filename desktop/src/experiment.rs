//! Native experiment configuration, binary reception, integrity checking and CSV.
use crate::experiment_data::{
    self as data, CHUNK_RECORDS, ExperimentConfig, RECORD_BYTES, RECORD_FIELDS,
};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::fmt::Write;

// IDs reserved for native download requests; frontend IDs stay below this range.
pub const DOWNLOAD_ID: u32 = 0xf000_0000;

#[derive(Deserialize)]
pub struct Input {
    pub kind: String,
    pub start_um: f64,
    pub increment_um: f64,
    pub timing: f64,
    pub repeats: u32,
    pub speed_um_s: Option<u32>,
}
fn scaled(value: f64, scale: f64) -> Result<i64, String> {
    let n = value * scale;
    if !n.is_finite() || n.abs() > 9_000_000_000_000_000.0 || (n - n.round()).abs() > 0.00001 {
        return Err("Use values with at most three decimal places".into());
    }
    Ok(n.round() as i64)
}
impl Input {
    pub fn command(self, id: u32) -> Result<String, String> {
        if id == 0 || id >= DOWNLOAD_ID {
            return Err("Invalid request ID".into());
        }
        let start = scaled(self.start_um, 1000.0)?;
        let increment = scaled(self.increment_um, 1000.0)?;
        let timing =
            u32::try_from(scaled(self.timing, 1000.0)?).map_err(|_| "Timing out of range")?;
        let kind = match self.kind.as_str() {
            "quasi_static" => "Q",
            "sinusoidal" => "S",
            _ => return Err("Unknown experiment mode".into()),
        };
        let config = ExperimentConfig::from_wire(
            kind,
            start,
            increment,
            timing,
            self.repeats,
            self.speed_um_s,
        )?;
        config.validate()?;
        let mut command = format!(
            "START {id} {kind} {start} {increment} {timing} {}",
            self.repeats
        );
        if let Some(speed) = config.speed() {
            write!(command, " {speed}").unwrap();
        }
        Ok(command)
    }
}

pub enum Event {
    Line(String),
    Chunk(Value, Vec<u8>),
}
#[derive(Default)]
pub struct Decoder {
    line: Vec<u8>,
    binary: Vec<u8>,
    header: Option<Value>,
    remaining: usize,
    dropping: bool,
}
impl Decoder {
    pub fn push(&mut self, byte: u8) -> Result<Option<Event>, String> {
        if self.remaining != 0 {
            self.binary.push(byte);
            self.remaining -= 1;
            return Ok((self.remaining == 0).then(|| {
                Event::Chunk(
                    self.header.take().unwrap(),
                    std::mem::take(&mut self.binary),
                )
            }));
        }
        if byte != b'\n' {
            if !self.dropping {
                if self.line.len() == 4096 {
                    self.line.clear();
                    self.dropping = true;
                } else {
                    self.line.push(byte);
                }
            }
            return Ok(None);
        }
        if core::mem::take(&mut self.dropping) {
            return Ok(Some(Event::Line(
                "# Oversized serial line discarded".into(),
            )));
        }
        let mut bytes = std::mem::take(&mut self.line);
        if bytes.last() == Some(&b'\r') {
            bytes.pop();
        }
        if bytes.is_empty() {
            return Ok(None);
        }
        let line = match String::from_utf8(bytes) {
            Ok(line) => line,
            Err(_) => {
                return Ok(Some(Event::Line(
                    "# Invalid UTF-8 serial line discarded".into(),
                )));
            }
        };
        let value = serde_json::from_str::<Value>(&line).ok();
        if let Some(header) = value.filter(|v| v["type"] == "data_chunk") {
            let bytes = uint(&header, "bytes")? as usize;
            if bytes == 0 || bytes > CHUNK_RECORDS * RECORD_BYTES || bytes % RECORD_BYTES != 0 {
                return Err("Invalid binary chunk length".into());
            }
            self.remaining = bytes;
            self.binary = Vec::with_capacity(bytes);
            self.header = Some(header);
            Ok(None)
        } else {
            Ok(Some(Event::Line(line)))
        }
    }
}
pub fn uint(value: &Value, key: &str) -> Result<u32, String> {
    value[key]
        .as_u64()
        .and_then(|n| u32::try_from(n).ok())
        .ok_or_else(|| format!("Invalid dataset {key}"))
}

#[derive(Clone, Serialize)]
pub struct Dataset {
    pub metadata: Value,
    /// Wire field order; i64 accommodates all signed and unsigned 32-bit words.
    pub samples: Vec<[i64; RECORD_FIELDS]>,
}
impl Dataset {
    pub fn csv(&self) -> String {
        let mut csv = String::new();
        // Metadata is quoted as one CSV field, including fault/abort information.
        let _ = writeln!(
            csv,
            "metadata,\"{}\"",
            self.metadata.to_string().replace('"', "\"\"")
        );
        csv.push_str("time_s,position_um,pressure_raw,stat,epos_age_ms,pressure_age_ms,stat_age_ms,target_um,controller_ticks,controller_age_ms,epos_counts,target_counts,epos_host_receipt_s,epos_generation,pressure_generation,epos_new,pressure_new\n");
        let mut previous: Option<&[i64; RECORD_FIELDS]> = None;
        for s in &self.samples {
            let _ = writeln!(
                csv,
                "{:.3},{:.3},{},{},{},{},{},{:.3},{},{},{},{},{:.3},{},{},{},{}",
                s[0] as f64 / 1000.0,
                s[1] as f64 * 0.078,
                s[2],
                s[3],
                s[4],
                s[5],
                s[6],
                s[7] as f64 * 0.078,
                s[8],
                s[9],
                s[1],
                s[7],
                (s[0] - s[4]) as f64 / 1000.0,
                s[10],
                s[11],
                u8::from(previous.is_none_or(|p| p[10] != s[10])),
                u8::from(previous.is_none_or(|p| p[11] != s[11])),
            );
            previous = Some(s);
        }
        csv
    }
}
#[derive(Default)]
pub struct Download {
    pub run: Option<u32>,
    metadata: Option<Value>,
    bytes: Vec<u8>,
}
impl Download {
    pub fn begin(&mut self, run: u32) -> String {
        self.run = Some(run);
        self.metadata = None;
        self.bytes.clear();
        format!("GET_DATA {DOWNLOAD_ID} {run}")
    }
    pub fn metadata(&mut self, metadata: Value) -> Result<(), String> {
        if uint(&metadata, "id")? != DOWNLOAD_ID
            || Some(uint(&metadata, "run")?) != self.run
            || uint(&metadata, "version")? != data::DATA_VERSION
            || uint(&metadata, "record_bytes")? as usize != RECORD_BYTES
            || uint(&metadata, "count")? as usize > data::CAPACITY
            || uint(&metadata, "bytes")? != uint(&metadata, "count")? * RECORD_BYTES as u32
            || uint(&metadata, "sample_ms")? != data::SAMPLE_MS
        {
            return Err("Invalid dataset metadata".into());
        }
        uint(&metadata, "crc")?;
        if !matches!(
            metadata["outcome"].as_str(),
            Some("completed" | "aborted" | "faulted")
        ) || !metadata["reason"].is_string()
            || !metadata["config"].is_object()
        {
            return Err("Invalid completion metadata".into());
        }
        self.bytes = Vec::with_capacity(uint(&metadata, "bytes")? as usize);
        self.metadata = Some(metadata);
        Ok(())
    }
    pub fn chunk(&mut self, header: Value, bytes: &[u8]) -> Result<(), String> {
        let meta = self
            .metadata
            .as_ref()
            .ok_or("Binary data before metadata")?;
        if uint(&header, "id")? != DOWNLOAD_ID
            || Some(uint(&header, "run")?) != self.run
            || (uint(&header, "offset")? as usize).checked_mul(RECORD_BYTES)
                != Some(self.bytes.len())
            || uint(&header, "bytes")? as usize != bytes.len()
            || self.bytes.len() + bytes.len() > uint(meta, "bytes")? as usize
        {
            return Err("Dataset offset, run or length mismatch; reconnect to retry".into());
        }
        if let Some(expected) = meta.get("reversal_count").and_then(Value::as_u64) {
            let received = meta["reversals"]
                .as_array()
                .ok_or("Missing reversal diagnostics")?
                .len();
            let diagnostics = header["reversals"]
                .as_array()
                .ok_or("Missing chunk diagnostics")?;
            if uint(&header, "reversal_offset")? as usize != received
                || diagnostics.len() > 2
                || received + diagnostics.len() > expected as usize
            {
                return Err("Invalid reversal diagnostic offset or count".into());
            }
            self.metadata.as_mut().unwrap()["reversals"]
                .as_array_mut()
                .unwrap()
                .extend(diagnostics.iter().cloned());
        }
        self.bytes.extend_from_slice(bytes);
        Ok(())
    }
    /// The caller stores a verified dataset before transmitting its ACK.
    pub fn next(&self) -> Result<(String, Option<Dataset>), String> {
        let meta = self.metadata.as_ref().ok_or("Missing dataset metadata")?;
        let run = self.run.ok_or("Missing run ID")?;
        if self.bytes.len() != uint(meta, "bytes")? as usize {
            return Ok((
                format!(
                    "GET_CHUNK {DOWNLOAD_ID} {run} {}",
                    self.bytes.len() / RECORD_BYTES
                ),
                None,
            ));
        }
        if let Some(expected) = meta.get("reversal_count").and_then(Value::as_u64) {
            if meta["reversals"].as_array().map(Vec::len) != Some(expected as usize) {
                return Err("Incomplete reversal diagnostics; data retained on Teensy".into());
            }
        }
        if !data::crc32(!0, &self.bytes) != uint(meta, "crc")? {
            return Err("Dataset CRC mismatch; data retained on Teensy, reconnect to retry".into());
        }
        let mut samples = Vec::with_capacity(self.bytes.len() / RECORD_BYTES);
        for bytes in self.bytes.chunks_exact(RECORD_BYTES) {
            let fields = data::decode_fields(bytes.try_into().unwrap());
            if fields[0] > i64::from(data::MAX_DURATION_MS)
                || samples
                    .last()
                    .is_some_and(|previous: &[i64; RECORD_FIELDS]| fields[0] <= previous[0])
            {
                return Err("Dataset timestamps invalid".into());
            }
            samples.push(fields);
        }
        Ok((
            format!(
                "ACK_DATA_RECEIVED {DOWNLOAD_ID} {run} {} {}",
                uint(meta, "count")?,
                uint(meta, "crc")?
            ),
            Some(Dataset {
                metadata: meta.clone(),
                samples,
            }),
        ))
    }
}
