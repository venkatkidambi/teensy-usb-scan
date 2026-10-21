#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

#[path = "../../src/command.rs"]
mod command;
mod experiment;
#[allow(dead_code)]
#[path = "../../src/experiment_data.rs"]
mod experiment_data;
mod maintenance;
#[path = "../../src/plunger_config.rs"]
mod plunger_config;
mod serial;
#[cfg(feature = "smoke-test")]
mod smoke;
mod zeiss;
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, AtomicU32, Ordering},
};
use tauri::Manager;
use tauri_plugin_dialog::DialogExt;

#[derive(serde::Deserialize)]
#[serde(rename_all = "snake_case")]
enum ExperimentPlot {
    Position,
    Pressure,
    SweepPhase,
}

impl ExperimentPlot {
    fn filename_suffix(&self) -> &'static str {
        match self {
            Self::Position => "position",
            Self::Pressure => "pressure",
            Self::SweepPhase => "sweep-phase",
        }
    }
}

#[derive(Default)]
struct Backend {
    connection: Mutex<Option<serial::Connection>>,
    next: AtomicU32,
    archive: Arc<Mutex<experiment::Archive>>,
    experiment_owned: Arc<AtomicBool>,
    maintenance: AtomicBool,
}

impl Backend {
    fn with_session<T>(
        &self,
        session: u32,
        action: impl FnOnce(&serial::Connection) -> Result<T, String>,
    ) -> Result<T, String> {
        let connection = self.connection.lock().map_err(|e| e.to_string())?;
        let connection = connection
            .as_ref()
            .filter(|c| c.id == session)
            .ok_or("Serial session expired")?;
        action(connection)
    }

    fn ensure_export_allowed(&self) -> Result<(), String> {
        if self.experiment_owned.load(Ordering::Acquire) {
            Err("Export is disabled until autonomous ownership is resolved".into())
        } else {
            Ok(())
        }
    }

    /// Snapshot the selected run before opening the dialog; never hold the serial
    /// mutex while waiting for the user. Recheck ownership before writing the file.
    fn save_csv(
        &self,
        index: Option<usize>,
        choose_path: impl FnOnce(&str) -> Result<Option<std::path::PathBuf>, String>,
    ) -> Result<Option<String>, String> {
        let (filename, csv) = {
            let _connection = self.connection.lock().map_err(|e| e.to_string())?;
            self.ensure_export_allowed()?;
            let archive = self.archive.lock().map_err(|e| e.to_string())?;
            let dataset = match index {
                Some(index) => archive.datasets.get(index),
                None => archive.datasets.last(),
            }
            .ok_or("No verified experiment dataset")?;
            (
                format!("experiment-{}.csv", dataset.metadata["run"]),
                dataset.csv(),
            )
        };
        let Some(path) = choose_path(&filename)? else {
            return Ok(None);
        };
        let _connection = self.connection.lock().map_err(|e| e.to_string())?;
        self.ensure_export_allowed()?;
        std::fs::write(&path, csv)
            .map_err(|e| format!("Could not save {}: {e}", path.display()))?;
        Ok(Some(path.to_string_lossy().into_owned()))
    }

    fn save_png(
        &self,
        index: Option<usize>,
        plot: ExperimentPlot,
        png: &[u8],
        choose_path: impl FnOnce(&str) -> Result<Option<std::path::PathBuf>, String>,
    ) -> Result<Option<String>, String> {
        // Only accept a bounded PNG snapshot, never an arbitrary file payload/path.
        if png.len() > 16 * 1024 * 1024
            || png.len() < 45
            || !png.starts_with(b"\x89PNG\r\n\x1a\n\0\0\0\rIHDR")
            || !png.ends_with(b"\0\0\0\0IEND\xaeB\x60\x82")
        {
            return Err("Invalid or oversized PNG image".into());
        }
        let filename = {
            let _connection = self.connection.lock().map_err(|e| e.to_string())?;
            self.ensure_export_allowed()?;
            let archive = self.archive.lock().map_err(|e| e.to_string())?;
            let dataset = match index {
                Some(index) => archive.datasets.get(index),
                None => archive.datasets.last(),
            }
            .ok_or("No verified experiment dataset")?;
            let run = if matches!(plot, ExperimentPlot::SweepPhase) {
                &archive.datasets.first().unwrap().metadata["run"]
            } else {
                &dataset.metadata["run"]
            };
            format!("experiment-{run}-{}.png", plot.filename_suffix())
        };
        let Some(path) = choose_path(&filename)? else {
            return Ok(None);
        };
        let _connection = self.connection.lock().map_err(|e| e.to_string())?;
        self.ensure_export_allowed()?;
        std::fs::write(&path, png)
            .map_err(|e| format!("Could not save {}: {e}", path.display()))?;
        Ok(Some(path.to_string_lossy().into_owned()))
    }

    fn disconnect(&self) {
        if let Ok(mut connection) = self.connection.lock() {
            connection.take();
        }
    }
}

#[tauri::command]
async fn serial_ports() -> Result<Vec<serial::Port>, String> {
    tauri::async_runtime::spawn_blocking(serial::ports)
        .await
        .map_err(|e| e.to_string())?
}

#[tauri::command]
async fn serial_connect(
    port: String,
    state: tauri::State<'_, Arc<Backend>>,
) -> Result<u32, String> {
    let state = state.inner().clone();
    tauri::async_runtime::spawn_blocking(move || {
        let mut connection = state.connection.lock().map_err(|e| e.to_string())?;
        if state.maintenance.load(Ordering::Acquire) {
            return Err("Wait for the update to finish before connecting".into());
        }
        if connection.is_some() {
            return Err("Disconnect the current port first".into());
        }
        let id = state.next.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
        *connection = Some(serial::Connection::open(
            &port,
            id,
            state.archive.clone(),
            state.experiment_owned.clone(),
        )?);
        Ok(id)
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
async fn serial_send(
    session: u32,
    command: String,
    state: tauri::State<'_, Arc<Backend>>,
) -> Result<(), String> {
    let state = state.inner().clone();
    tauri::async_runtime::spawn_blocking(move || {
        state.with_session(session, |connection| connection.send(command))
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
async fn serial_poll(
    session: u32,
    state: tauri::State<'_, Arc<Backend>>,
) -> Result<serial::Batch, String> {
    let state = state.inner().clone();
    tauri::async_runtime::spawn_blocking(move || {
        state.with_session(session, |connection| Ok(connection.poll()))
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
async fn serial_disconnect(
    session: u32,
    state: tauri::State<'_, Arc<Backend>>,
) -> Result<(), String> {
    let state = state.inner().clone();
    tauri::async_runtime::spawn_blocking(move || {
        let mut connection = state.connection.lock().map_err(|e| e.to_string())?;
        if connection.as_ref().is_some_and(|c| c.id == session) {
            connection.take();
        }
        Ok(())
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
async fn experiment_start(
    session: u32,
    id: u32,
    config: experiment::Input,
    append_results: Option<bool>,
    state: tauri::State<'_, Arc<Backend>>,
) -> Result<(), String> {
    let command = config.command(id)?;
    state.with_session(session, |connection| {
        let mut archive = state.archive.lock().map_err(|e| e.to_string())?;
        connection.send(command)?;
        archive.begin(append_results.unwrap_or(false));
        Ok(())
    })
}

#[tauri::command]
fn experiment_result(
    state: tauri::State<'_, Arc<Backend>>,
) -> Result<Option<experiment::Dataset>, String> {
    Ok(state
        .archive
        .lock()
        .map_err(|e| e.to_string())?
        .datasets
        .last()
        .cloned())
}

#[tauri::command]
fn experiment_results(
    state: tauri::State<'_, Arc<Backend>>,
) -> Result<Vec<experiment::Dataset>, String> {
    Ok(state
        .archive
        .lock()
        .map_err(|e| e.to_string())?
        .datasets
        .clone())
}

#[tauri::command]
fn experiment_validate(configs: Vec<experiment::Input>) -> Result<(), String> {
    if configs.is_empty() || configs.len() > 100 {
        return Err("Specify 1 to 100 frequencies".into());
    }
    for (index, config) in configs.into_iter().enumerate() {
        config
            .command(1)
            .map_err(|error| format!("Run {}: {error}", index + 1))?;
    }
    Ok(())
}

#[tauri::command]
async fn experiment_save_csv(
    index: Option<usize>,
    window: tauri::WebviewWindow,
    state: tauri::State<'_, Arc<Backend>>,
) -> Result<Option<String>, String> {
    let state = state.inner().clone();
    tauri::async_runtime::spawn_blocking(move || {
        state.save_csv(index, |filename| {
            window
                .dialog()
                .file()
                .set_parent(&window)
                .set_title("Save experiment CSV")
                .add_filter("CSV files", &["csv"])
                .set_file_name(filename)
                .blocking_save_file()
                .map(|path| path.into_path().map_err(|e| e.to_string()))
                .transpose()
        })
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
async fn experiment_save_png(
    index: Option<usize>,
    plot: ExperimentPlot,
    png: Vec<u8>,
    window: tauri::WebviewWindow,
    state: tauri::State<'_, Arc<Backend>>,
) -> Result<Option<String>, String> {
    let state = state.inner().clone();
    tauri::async_runtime::spawn_blocking(move || {
        state.save_png(index, plot, &png, |filename| {
            window
                .dialog()
                .file()
                .set_parent(&window)
                .set_title("Save experiment plot PNG")
                .add_filter("PNG images", &["png"])
                .set_file_name(filename)
                .blocking_save_file()
                .map(|path| path.into_path().map_err(|e| e.to_string()))
                .transpose()
        })
    })
    .await
    .map_err(|e| e.to_string())?
}

// Keep normal and smoke builds on the same application command surface.
macro_rules! command_handler {
    ($($extra:path),* $(,)?) => {
        tauri::generate_handler![serial_ports, serial_connect, serial_send, serial_poll,
            serial_disconnect, experiment_start, experiment_result, experiment_results, experiment_validate, experiment_save_csv, experiment_save_png,
            zeiss::zeiss_czi_files, zeiss::zeiss_choose_folder,
            maintenance::app_release_info, maintenance::firmware_install, maintenance::app_update_check, maintenance::app_update_install, $($extra),*]
    };
}

fn main() {
    let builder = tauri::Builder::default()
        .manage(Arc::new(Backend::default()))
        .manage(maintenance::PendingUpdate::default())
        .plugin(tauri_plugin_dialog::init())
        .setup(|app| {
            if maintenance::app_release_info().updater_enabled() {
                app.handle()
                    .plugin(tauri_plugin_updater::Builder::new().build())?;
            }
            Ok(())
        })
        .invoke_handler(command_handler![])
        .on_page_load(|window, payload| {
            if payload.event() == tauri::webview::PageLoadEvent::Started {
                // A full Vite reload discards the JS session that owned this port.
                window.state::<Arc<Backend>>().disconnect();
            }
            #[cfg(feature = "smoke-test")]
            if payload.event() == tauri::webview::PageLoadEvent::Finished {
                let _ = window.eval(smoke::SCRIPT);
            }
        })
        .on_window_event(|window, event| {
            if let tauri::WindowEvent::CloseRequested { api, .. } = event {
                if window
                    .state::<Arc<Backend>>()
                    .maintenance
                    .load(Ordering::Acquire)
                {
                    api.prevent_close();
                }
            }
            if matches!(event, tauri::WindowEvent::Destroyed) {
                window.state::<Arc<Backend>>().disconnect();
            }
        });
    #[cfg(feature = "smoke-test")]
    let builder = builder
        .invoke_handler(command_handler![smoke::smoke_report])
        .setup(|app| {
            let handle = app.handle().clone();
            std::thread::spawn(move || {
                std::thread::sleep(std::time::Duration::from_secs(20));
                eprintln!("NATIVE_SMOKE timed out");
                handle.exit(2);
            });
            Ok(())
        });
    builder
        .run(tauri::generate_context!())
        .expect("Could not start Teensy Panel");
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn backend() -> Backend {
        let backend = Backend::default();
        backend
            .archive
            .lock()
            .unwrap()
            .datasets
            .push(experiment::Dataset {
                archive_index: Some(0),
                metadata: serde_json::json!({"run": 7}),
                samples: vec![[0, -10, 123, 0, 1, 2, 3, -20, 42, 4, 1, 2]],
            });
        backend
    }

    #[test]
    fn history_export_selects_earlier_run_without_overwriting_latest() {
        let backend = backend();
        let mut later = backend.archive.lock().unwrap().datasets[0].clone();
        later.metadata["run"] = serde_json::json!(8);
        backend.archive.lock().unwrap().datasets.push(later);
        backend
            .save_csv(Some(0), |name| {
                assert_eq!(name, "experiment-7.csv");
                Ok(None)
            })
            .unwrap();
        backend
            .save_csv(None, |name| {
                assert_eq!(name, "experiment-8.csv");
                Ok(None)
            })
            .unwrap();
        assert!(
            backend
                .save_csv(Some(2), |_| panic!("invalid index"))
                .is_err()
        );
        assert_eq!(backend.archive.lock().unwrap().datasets.len(), 2);
    }

    struct Output(PathBuf);
    impl Output {
        fn new() -> Self {
            static NEXT: AtomicU32 = AtomicU32::new(0);
            let directory = std::env::temp_dir().join(format!(
                "teensy-csv-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir(&directory).unwrap();
            Self(directory)
        }
        fn path(&self) -> PathBuf {
            self.0.join("chosen name.csv")
        }
    }
    impl Drop for Output {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn save_uses_chosen_path_and_releases_locks_during_dialog() {
        let backend = backend();
        let output = Output::new();
        let path = output.path();
        let expected = backend
            .archive
            .lock()
            .unwrap()
            .datasets
            .last()
            .unwrap()
            .csv();
        let result = backend
            .save_csv(None, |filename| {
                assert_eq!(filename, "experiment-7.csv");
                assert!(backend.connection.try_lock().is_ok());
                assert!(backend.archive.try_lock().is_ok());
                Ok(Some(path.clone()))
            })
            .unwrap();
        assert_eq!(result, Some(path.to_string_lossy().into_owned()));
        assert_eq!(std::fs::read_to_string(path).unwrap(), expected);
        assert!(!backend.archive.lock().unwrap().datasets.is_empty());
    }

    #[test]
    fn cancellation_and_errors_keep_the_dataset_available() {
        let backend = backend();
        assert_eq!(backend.save_csv(None, |_| Ok(None)).unwrap(), None);
        assert_eq!(
            backend
                .save_csv(None, |_| Err("dialog failed".into()))
                .unwrap_err(),
            "dialog failed"
        );
        let output = Output::new();
        let missing_parent = output.0.join("missing").join("run.csv");
        assert!(
            backend
                .save_csv(None, |_| Ok(Some(missing_parent.clone())))
                .unwrap_err()
                .contains("Could not save")
        );
        assert!(!missing_parent.exists());
        assert!(!backend.archive.lock().unwrap().datasets.is_empty());
    }

    #[test]
    fn absent_dataset_or_owned_experiment_never_opens_dialog() {
        assert!(
            Backend::default()
                .save_csv(None, |_| panic!("no dataset"))
                .is_err()
        );
        let backend = backend();
        backend.experiment_owned.store(true, Ordering::Release);
        assert!(
            backend
                .save_csv(None, |_| panic!("experiment active"))
                .is_err()
        );
    }

    #[test]
    fn ownership_acquired_while_dialog_is_open_prevents_file_write() {
        let backend = backend();
        let output = Output::new();
        let path = output.path();
        std::fs::write(&path, "existing CSV").unwrap();
        let result = backend.save_csv(None, |_| {
            backend.experiment_owned.store(true, Ordering::Release);
            Ok(Some(path.clone()))
        });
        assert!(result.unwrap_err().contains("ownership"));
        assert_eq!(std::fs::read_to_string(path).unwrap(), "existing CSV");
        assert!(!backend.archive.lock().unwrap().datasets.is_empty());
    }

    #[test]
    fn png_export_writes_exact_snapshot_and_releases_locks_for_dialog() {
        let backend = backend();
        let output = Output::new();
        let path = output.0.join("chosen plot.png");
        let png = include_bytes!("../icons/icon.png");
        let result = backend
            .save_png(Some(0), ExperimentPlot::Position, png, |name| {
                assert_eq!(name, "experiment-7-position.png");
                assert!(backend.connection.try_lock().is_ok());
                assert!(backend.archive.try_lock().is_ok());
                Ok(Some(path.clone()))
            })
            .unwrap();
        assert_eq!(result, Some(path.to_string_lossy().into_owned()));
        assert_eq!(std::fs::read(&path).unwrap(), png);
        assert_eq!(backend.archive.lock().unwrap().datasets.len(), 1);
    }

    #[test]
    fn png_export_cancel_invalid_payload_and_ownership_never_overwrite() {
        let backend = backend();
        let png = include_bytes!("../icons/icon.png");
        assert_eq!(
            backend
                .save_png(None, ExperimentPlot::Pressure, png, |name| {
                    assert_eq!(name, "experiment-7-pressure.png");
                    Ok(None)
                })
                .unwrap(),
            None
        );
        assert!(
            backend
                .save_png(None, ExperimentPlot::Position, b"not PNG", |_| panic!(
                    "invalid PNG"
                ))
                .is_err()
        );
        assert!(
            backend
                .save_png(
                    None,
                    ExperimentPlot::Position,
                    &vec![0; 16 * 1024 * 1024 + 1],
                    |_| panic!("oversized PNG")
                )
                .is_err()
        );
        assert!(
            Backend::default()
                .save_png(None, ExperimentPlot::Position, png, |_| panic!(
                    "no dataset"
                ))
                .is_err()
        );
        assert!(
            backend
                .save_png(Some(9), ExperimentPlot::Position, png, |_| panic!(
                    "invalid index"
                ))
                .is_err()
        );
        let output = Output::new();
        let path = output.0.join("plot.png");
        std::fs::write(&path, "existing image").unwrap();
        assert!(
            backend
                .save_png(None, ExperimentPlot::Position, png, |_| {
                    backend.experiment_owned.store(true, Ordering::Release);
                    Ok(Some(path.clone()))
                })
                .unwrap_err()
                .contains("ownership")
        );
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "existing image");
        assert!(
            backend
                .save_png(None, ExperimentPlot::Position, png, |_| panic!(
                    "owned experiment"
                ))
                .is_err()
        );
    }

    #[test]
    fn sweep_png_uses_first_run_name_and_reports_write_failures() {
        let backend = backend();
        let mut later = backend.archive.lock().unwrap().datasets[0].clone();
        later.metadata["run"] = serde_json::json!(8);
        backend.archive.lock().unwrap().datasets.push(later);
        let png = include_bytes!("../icons/icon.png");
        let output = Output::new();
        let result = backend.save_png(Some(1), ExperimentPlot::SweepPhase, png, |name| {
            assert_eq!(name, "experiment-7-sweep-phase.png");
            Ok(Some(output.0.join("missing").join("plot.png")))
        });
        assert!(result.unwrap_err().contains("Could not save"));
        assert_eq!(backend.archive.lock().unwrap().datasets.len(), 2);
    }
}
