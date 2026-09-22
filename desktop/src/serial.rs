use serde::Serialize;
use std::{
    collections::VecDeque,
    io::{self, Read, Write},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
        mpsc::{self, SyncSender},
    },
    thread::{self, JoinHandle},
    time::Duration,
};

#[derive(Serialize)]
pub struct Port {
    pub name: String,
    pub label: String,
    pub teensy: bool,
}

pub fn ports() -> Result<Vec<Port>, String> {
    let mut ports: Vec<_> = serialport::available_ports()
        .map_err(|e| e.to_string())?
        .into_iter()
        .map(|p| {
            let (label, teensy) = match p.port_type {
                serialport::SerialPortType::UsbPort(usb) => (
                    format!(
                        "{} · {} ({:04x}:{:04x})",
                        p.port_name,
                        usb.product.as_deref().unwrap_or("USB serial"),
                        usb.vid,
                        usb.pid
                    ),
                    usb.vid == 0x5824 && usb.pid == 0x27dd,
                ),
                _ => (p.port_name.clone(), false),
            };
            Port {
                name: p.port_name,
                label,
                teensy,
            }
        })
        .collect();
    ports.sort_by(|a, b| b.teensy.cmp(&a.teensy).then(a.name.cmp(&b.name)));
    Ok(ports)
}

// This allowlist mirrors firmware protocol v1. No arbitrary serial writes.
pub fn validate_command(command: &str) -> bool {
    if command.len() > 64 || command.contains(['\r', '\n']) {
        return false;
    }
    let parts: Vec<_> = command.split_ascii_whitespace().collect();
    if parts.len() < 2
        || !parts[1].bytes().all(|b| b.is_ascii_digit())
        || !parts[1].parse::<u32>().is_ok_and(|id| id > 0)
    {
        return false;
    }
    match parts.as_slice() {
        [
            "HELLO" | "PING" | "SNAPSHOT" | "PLUNGER_STOP" | "PLUNGER_ZERO" | "XERYON_STOP",
            _,
        ]
        | ["STREAM", _, "0" | "1"] => true,
        ["XERYON_MOVE", _, "U" | "D", nm, speed @ ..] if speed.len() <= 1 => {
            let speed_valid = speed.is_empty()
                || speed[0].bytes().all(|b| b.is_ascii_digit())
                    && speed[0]
                        .parse::<u32>()
                        .is_ok_and(|s| (1..=16777215).contains(&s));
            speed_valid
                && nm.bytes().all(|b| b.is_ascii_digit())
                && nm
                    .parse::<u32>()
                    .is_ok_and(|n| (78..=2_617_245_618).contains(&n))
        }
        ["PLUNGER_MOVE", _, "U" | "D", speed, um] => {
            speed.bytes().all(|b| b.is_ascii_digit())
                && speed.parse::<u8>().is_ok_and(|s| s > 0)
                && um.bytes().all(|b| b.is_ascii_digit())
                && um
                    .parse::<u32>()
                    .is_ok_and(|d| d > 0 && (u64::from(d) * 16688 + 500) / 1000 <= i32::MAX as u64)
        }
        _ => false,
    }
}

#[derive(Default)]
pub struct Inbox {
    replies: VecDeque<(u64, String)>,
    logs: VecDeque<(u64, String)>,
    latest: Option<(u64, String)>,
    sequence: u64,
    error: Option<String>,
    coalesced: u64,
}

impl Inbox {
    fn push(&mut self, line: String) {
        self.sequence += 1;
        let sequence = self.sequence;
        if line.starts_with('#') {
            if self.logs.len() == 32 {
                self.logs.pop_front();
            }
            self.logs.push_back((sequence, line));
        } else if serde_json::from_str::<serde_json::Value>(&line)
            .is_ok_and(|m| m["type"] == "snapshot" && m["id"] == 0)
        {
            if self.latest.replace((sequence, line)).is_some() {
                self.coalesced += 1;
            }
        } else if self.replies.len() < 32 {
            self.replies.push_back((sequence, line));
        } else {
            self.error = Some("Serial response queue overflow; reconnect to retry".into());
        }
    }
    fn batch(&mut self, session: u32, closed: bool) -> Batch {
        let mut messages: Vec<_> = self.replies.drain(..).collect();
        if let Some(snapshot) = self.latest.take() {
            messages.push(snapshot);
        }
        messages.extend(self.logs.drain(..));
        // Keep temporal order across replies and coalesced unsolicited snapshots.
        // Otherwise an older broadcast could overwrite a newer requested snapshot.
        messages.sort_unstable_by_key(|(sequence, _)| *sequence);
        Batch {
            session,
            messages: messages.into_iter().map(|(_, line)| line).collect(),
            closed,
            error: self.error.clone(),
            coalesced: self.coalesced,
        }
    }
}

#[derive(Serialize)]
pub struct Batch {
    pub session: u32,
    pub messages: Vec<String>,
    pub closed: bool,
    pub error: Option<String>,
    pub coalesced: u64,
}

#[derive(Default)]
struct Lines {
    bytes: Vec<u8>,
    dropping: bool,
}
impl Lines {
    fn feed(&mut self, bytes: &[u8], mut line: impl FnMut(String)) {
        for &b in bytes {
            if b == b'\n' {
                if self.dropping {
                    line("# Oversized serial line discarded".into());
                } else if !self.bytes.is_empty() {
                    if self.bytes.last() == Some(&b'\r') {
                        self.bytes.pop();
                    }
                    match String::from_utf8(self.bytes.clone()) {
                        Ok(s) => line(s),
                        Err(_) => line("# Invalid UTF-8 serial line discarded".into()),
                    }
                }
                self.bytes.clear();
                self.dropping = false;
            } else if !self.dropping {
                if self.bytes.len() == 4096 {
                    self.bytes.clear();
                    self.dropping = true;
                } else {
                    self.bytes.push(b);
                }
            }
        }
    }
}

pub struct Connection {
    pub id: u32,
    tx: SyncSender<String>,
    inbox: Arc<Mutex<Inbox>>,
    stop: Arc<AtomicBool>,
    finished: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl Connection {
    pub fn open(name: &str, id: u32) -> Result<Self, String> {
        if !ports()?.iter().any(|p| p.name == name) {
            return Err("Port is no longer available; refresh the list".into());
        }
        let mut port = serialport::new(name, 115200)
            .flow_control(serialport::FlowControl::None)
            .data_bits(serialport::DataBits::Eight)
            .stop_bits(serialport::StopBits::One)
            .parity(serialport::Parity::None)
            .timeout(Duration::from_millis(20))
            .open()
            .map_err(|e| format!("Cannot open {name}: {e}. Close other serial terminals."))?;
        port.write_data_terminal_ready(true)
            .map_err(|e| e.to_string())?;
        port.write_request_to_send(false)
            .map_err(|e| e.to_string())?;
        let (tx, rx) = mpsc::sync_channel::<String>(8);
        let inbox = Arc::new(Mutex::new(Inbox::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let finished = Arc::new(AtomicBool::new(false));
        let (queue, cancel, done) = (inbox.clone(), stop.clone(), finished.clone());
        let worker = thread::Builder::new()
            .name("teensy-serial".into())
            .spawn(move || {
                let mut buffer = [0; 1024];
                let mut lines = Lines::default();
                let result = (|| -> io::Result<()> {
                    while !cancel.load(Ordering::Relaxed) {
                        if queue.lock().unwrap().error.is_some() {
                            break;
                        }
                        // Bounded writes before returning to reception.
                        for command in rx.try_iter().take(8) {
                            port.write_all(command.as_bytes())?;
                            port.write_all(b"\n")?;
                        }
                        match port.read(&mut buffer) {
                            Ok(0) => {
                                return Err(io::Error::new(
                                    io::ErrorKind::UnexpectedEof,
                                    "Serial device disconnected",
                                ));
                            }
                            Ok(n) => {
                                lines.feed(&buffer[..n], |line| queue.lock().unwrap().push(line))
                            }
                            Err(e)
                                if matches!(
                                    e.kind(),
                                    io::ErrorKind::TimedOut
                                        | io::ErrorKind::WouldBlock
                                        | io::ErrorKind::Interrupted
                                ) => {}
                            Err(e) => return Err(e),
                        }
                    }
                    Ok(())
                })();
                if let Err(e) = result {
                    queue.lock().unwrap().error = Some(e.to_string());
                }
                let _ = port.write_data_terminal_ready(false);
                done.store(true, Ordering::Release);
            })
            .map_err(|e| e.to_string())?;
        Ok(Self {
            id,
            tx,
            inbox,
            stop,
            finished,
            worker: Some(worker),
        })
    }
    pub fn send(&self, command: String) -> Result<(), String> {
        if !validate_command(&command) {
            return Err("Unsupported command".into());
        }
        if self.finished.load(Ordering::Acquire) {
            return Err("Serial connection closed".into());
        }
        self.tx
            .try_send(command)
            .map_err(|_| "Serial command queue busy or disconnected".into())
    }
    pub fn poll(&self) -> Batch {
        self.inbox
            .lock()
            .unwrap()
            .batch(self.id, self.finished.load(Ordering::Acquire))
    }
}
impl Drop for Connection {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn only_supported_requests_are_allowed() {
        for command in [
            "XERYON_MOVE 1 U 10000",
            "XERYON_MOVE 2 D 78",
            "XERYON_MOVE 1 U 10000 1",
            "XERYON_MOVE 1 U 10000 16777215",
            "XERYON_MOVE 2 D 78 10000",
            "XERYON_MOVE 3 U 2617245618 10000",
            "XERYON_STOP 4",
            "PLUNGER_MOVE 1 U 64 10",
            "PLUNGER_MOVE 2 D 255 128684302",
            "PLUNGER_STOP 3",
            "PLUNGER_ZERO 4",
            "HELLO 1",
            "PING 2",
            "SNAPSHOT 4294967295",
            "STREAM 3 0",
            "STREAM 4 1",
        ] {
            assert!(validate_command(command));
        }
        for command in [
            "XERYON_MOVE 1 U 77",
            "XERYON_MOVE 1 U 2617245619",
            "XERYON_MOVE 1 U 10000 10000 extra",
            "XERYON_MOVE 1 U 10000 0",
            "XERYON_MOVE 1 U 10000 16777216",
            "XERYON_MOVE 1 U 10000 -1",
            "XERYON_MOVE 1 U 10000 1.5",
            "XERYON_MOVE 1 U 77 10000",
            "XERYON_MOVE 1 U 2617245619 10000",
            "XERYON_MOVE 1 X 100 10000",
            "XERYON_MOVE 1 D -100 10000",
            "XERYON_MOVE 1 U 1.5 10000",
            "XERYON_STOP 1 extra",
            "PLUNGER_MOVE 1 U 0 10",
            "PLUNGER_MOVE 1 D 256 10",
            "PLUNGER_MOVE 1 D 64 128684303",
            "PLUNGER_MOVE 1 X 64 10",
            "PLUNGER_MOVE 1 U 64 0",
            "PLUNGER_MOVE 1 U 64 -1",
            "PLUNGER_STOP 1 extra",
            "MOVE 1",
            "DPOS=100",
            "PING 0",
            "PING -1",
            "PING 4294967296",
            "HELLO 1\nMOVE=1",
            "STREAM 1 2",
            "PING 1 extra",
        ] {
            assert!(!validate_command(command), "{command}");
        }
    }
    #[test]
    fn line_fragmentation_overflow_and_recovery() {
        let mut parser = Lines::default();
        let mut lines = Vec::new();
        parser.feed(b"one\r", |s| lines.push(s));
        parser.feed(b"\ntwo\n", |s| lines.push(s));
        assert_eq!(lines, ["one", "two"]);
        parser.feed(&[b'x'; 5000], |s| lines.push(s));
        parser.feed(b"\nthree\n", |s| lines.push(s));
        assert!(lines[2].contains("Oversized"));
        assert_eq!(lines[3], "three");
    }
    #[test]
    fn slow_ui_coalesces_snapshots_but_keeps_replies() {
        let mut q = Inbox::default();
        for i in 0..100 {
            q.push(format!("{{\"type\":\"snapshot\",\"id\":0,\"ms\":{i}}}"));
            q.push(format!("# log {i}"));
        }
        q.push("{\"type\":\"pong\",\"id\":1}".into());
        let batch = q.batch(7, false);
        assert_eq!(batch.coalesced, 99);
        assert_eq!(batch.messages.len(), 34);
        assert!(batch.messages.last().unwrap().contains("pong"));
        assert!(batch.messages.iter().any(|s| s.contains(":99")));
        assert!(q.batch(7, false).messages.is_empty());
        for _ in 0..33 {
            q.push("bad reply".into());
        }
        assert!(q.error.is_some());
    }
}
