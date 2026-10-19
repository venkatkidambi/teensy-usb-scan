//! Packaged firmware and signed app updates. No user-supplied paths or commands.
use crate::Backend;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    io::{BufRead, BufReader, Read},
    path::{Path, PathBuf},
    process::{Command, Stdio},
    sync::{Arc, Mutex, atomic::Ordering},
    time::{Duration, Instant},
};
use tauri::{Manager, ipc::Channel};
use tauri_plugin_updater::{Update, UpdaterExt};

#[derive(Deserialize, Serialize)]
pub struct ReleaseInfo {
    version: String,
    channel: String,
    commit: String,
    firmware_sha256: Option<String>,
    updater: bool,
    #[serde(default)]
    repository: Option<String>,
}

impl ReleaseInfo {
    pub fn updater_enabled(&self) -> bool {
        self.updater
    }
}

fn release_info() -> ReleaseInfo {
    serde_json::from_str(include_str!(concat!(env!("OUT_DIR"), "/release.json")))
        .expect("embedded release metadata")
}

#[tauri::command]
pub fn app_release_info() -> ReleaseInfo {
    release_info()
}

#[derive(Default)]
pub struct PendingUpdate(Mutex<Option<Update>>);

pub struct MaintenanceGuard(Arc<Backend>);

impl MaintenanceGuard {
    pub fn begin(state: Arc<Backend>) -> Result<Self, String> {
        // This same mutex protects serial_connect: the check and transition must
        // be atomic with respect to opening a port, including concurrent IPC.
        {
            let connection = state.connection.lock().map_err(|e| e.to_string())?;
            if connection.is_some() {
                return Err("Disconnect the Teensy before updating".into());
            }
            if state.experiment_owned.load(Ordering::Acquire) {
                return Err("Reconnect and resolve the experiment before updating".into());
            }
            if state.maintenance.swap(true, Ordering::AcqRel) {
                return Err("An update is already in progress".into());
            }
        }
        Ok(Self(state))
    }
}

impl Drop for MaintenanceGuard {
    fn drop(&mut self) {
        self.0.maintenance.store(false, Ordering::Release);
    }
}

fn verify_firmware(path: &Path, expected: &str) -> Result<(), String> {
    let bytes = std::fs::read(path).map_err(|e| format!("Cannot read bundled firmware: {e}"))?;
    if bytes.is_empty() || format!("{:x}", Sha256::digest(&bytes)) != expected {
        return Err("Bundled firmware failed its integrity check. Reinstall the app.".into());
    }
    Ok(())
}

fn loader_path() -> Result<PathBuf, String> {
    let executable = std::env::current_exe().map_err(|e| e.to_string())?;
    let name = if cfg!(windows) {
        "teensy_loader_cli.exe"
    } else {
        "teensy_loader_cli"
    };
    Ok(executable
        .parent()
        .ok_or("Cannot locate the installed app")?
        .join(name))
}

fn run_loader(
    loader: &Path,
    firmware: &Path,
    progress: Channel<String>,
    timeout: Duration,
) -> Result<(), String> {
    let mut command = Command::new(loader);
    command
        .args(["--mcu=TEENSY41", "-w", "-v"])
        .arg(firmware)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(0x08000000); // CREATE_NO_WINDOW
    }
    let mut child = command
        .spawn()
        .map_err(|e| format!("Cannot start bundled Teensy flasher: {e}"))?;
    let _ = progress.send(
        "Press the Teensy Program button. Keep USB connected until flashing finishes.".into(),
    );
    // Drain both pipes while waiting so verbose output cannot deadlock the child.
    let stdout = child.stdout.take().unwrap();
    let stderr = child.stderr.take().unwrap();
    let readers: Vec<_> = [Box::new(stdout) as Box<dyn Read + Send>, Box::new(stderr)]
        .into_iter()
        .map(|pipe| {
            let progress = progress.clone();
            std::thread::spawn(move || {
                for line in BufReader::new(pipe.take(64 * 1024))
                    .lines()
                    .map_while(Result::ok)
                {
                    let _ = progress.send(line);
                }
            })
        })
        .collect();
    let start = Instant::now();
    let result = loop {
        match child.try_wait() {
            Ok(Some(status)) => {
                break if status.success() {
                    Ok(())
                } else {
                    Err(format!(
                        "Flashing failed ({status}). Close other loaders, then retry using the Program button."
                    ))
                };
            }
            Ok(None) if start.elapsed() < timeout => std::thread::sleep(Duration::from_millis(100)),
            other => {
                let _ = child.kill();
                let _ = child.wait();
                break Err(match other {
                    Err(error) => format!("Could not monitor the Teensy flasher: {error}"),
                    _ => "Flashing timed out. Reconnect USB and retry, pressing the Program button when prompted.".into(),
                });
            }
        }
    };
    for reader in readers {
        let _ = reader.join();
    }
    result
}

#[tauri::command]
pub async fn firmware_install(
    app: tauri::AppHandle,
    state: tauri::State<'_, Arc<Backend>>,
    progress: Channel<String>,
) -> Result<(), String> {
    let info = release_info();
    let expected = info
        .firmware_sha256
        .ok_or("Firmware installation is available in packaged releases")?;
    let firmware = app
        .path()
        .resource_dir()
        .map_err(|e| e.to_string())?
        .join("release/firmware.hex");
    let loader = loader_path()?;
    let guard = MaintenanceGuard::begin(state.inner().clone())?;
    tauri::async_runtime::spawn_blocking(move || {
        let _guard = guard;
        verify_firmware(&firmware, &expected)?;
        run_loader(&loader, &firmware, progress, Duration::from_secs(180))
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
pub async fn app_update_check(
    app: tauri::AppHandle,
    pending: tauri::State<'_, PendingUpdate>,
) -> Result<Option<String>, String> {
    if !release_info().updater {
        return Err(
            "This build has no update signing key. Install a newer release manually.".into(),
        );
    }
    // Clear an old candidate before a new check, including when the request fails.
    *pending.0.lock().map_err(|e| e.to_string())? = None;
    let update = app
        .updater_builder()
        .timeout(Duration::from_secs(30))
        .build()
        .map_err(|e| e.to_string())?
        .check()
        .await
        .map_err(|e| e.to_string())?;
    let version = update.as_ref().map(|update| update.version.clone());
    *pending.0.lock().map_err(|e| e.to_string())? = update;
    Ok(version)
}

#[tauri::command]
pub async fn app_update_install(
    app: tauri::AppHandle,
    state: tauri::State<'_, Arc<Backend>>,
    pending: tauri::State<'_, PendingUpdate>,
    progress: Channel<String>,
) -> Result<(), String> {
    let _guard = MaintenanceGuard::begin(state.inner().clone())?;
    let update = pending
        .0
        .lock()
        .map_err(|e| e.to_string())?
        .take()
        .ok_or("Check for updates first")?;
    let mut downloaded = 0_u64;
    update
        .download_and_install(
            |bytes, total| {
                downloaded += bytes as u64;
                let message = match total.filter(|n| *n > 0) {
                    Some(total) => format!(
                        "Downloading update… {}%",
                        downloaded.saturating_mul(100) / total
                    ),
                    None => format!("Downloading update… {} KiB", downloaded / 1024),
                };
                let _ = progress.send(message);
            },
            || {
                let _ = progress.send("Verifying and installing update…".into());
            },
        )
        .await
        .map_err(|e| e.to_string())?;
    app.restart();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn updates_are_exclusive_and_block_unresolved_experiments() {
        let backend = Arc::new(Backend::default());
        backend.experiment_owned.store(true, Ordering::Release);
        assert!(MaintenanceGuard::begin(backend.clone()).is_err());
        assert!(!backend.maintenance.load(Ordering::Acquire));
        backend.experiment_owned.store(false, Ordering::Release);
        let guard = MaintenanceGuard::begin(backend.clone()).unwrap();
        assert!(MaintenanceGuard::begin(backend.clone()).is_err());
        drop(guard);
        assert!(!backend.maintenance.load(Ordering::Acquire));
        assert!(MaintenanceGuard::begin(backend).is_ok());
    }

    #[test]
    fn firmware_hash_must_match_before_a_loader_can_run() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("calibration/lps-force.json");
        let expected = format!("{:x}", Sha256::digest(std::fs::read(&path).unwrap()));
        assert!(verify_firmware(&path, &expected).is_ok());
        assert!(verify_firmware(&path, &"0".repeat(64)).is_err());
        assert!(verify_firmware(&path.with_extension("missing"), &expected).is_err());
    }

    #[cfg(unix)]
    #[test]
    fn flasher_handles_paths_with_spaces_failure_and_timeout_without_hardware() {
        use std::os::unix::fs::PermissionsExt;
        let dir = std::env::temp_dir().join(format!("teensy loader test {}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let loader = dir.join("fake loader");
        let write = |script| {
            std::fs::write(&loader, script).unwrap();
            std::fs::set_permissions(&loader, std::fs::Permissions::from_mode(0o700)).unwrap();
        };
        write(
            "#!/bin/sh\n[ \"$1\" = '--mcu=TEENSY41' ] && [ \"$2\" = '-w' ] && [ \"$3\" = '-v' ] && [ \"$4\" = 'firmware with spaces.hex' ]\n",
        );
        let firmware = Path::new("firmware with spaces.hex");
        assert!(
            run_loader(
                &loader,
                firmware,
                Channel::new(|_| Ok(())),
                Duration::from_secs(2)
            )
            .is_ok()
        );
        write("#!/bin/sh\necho 'device unavailable' >&2\nexit 7\n");
        assert!(
            run_loader(
                &loader,
                firmware,
                Channel::new(|_| Ok(())),
                Duration::from_secs(2)
            )
            .unwrap_err()
            .contains("Flashing failed")
        );
        write("#!/bin/sh\nexec sleep 5\n");
        assert!(
            run_loader(
                &loader,
                firmware,
                Channel::new(|_| Ok(())),
                Duration::from_millis(50)
            )
            .unwrap_err()
            .contains("timed out")
        );
        std::fs::remove_dir_all(dir).unwrap();
    }
}
