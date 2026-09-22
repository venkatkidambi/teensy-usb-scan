#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod experiment;
#[allow(dead_code)]
#[path = "../../src/experiment_data.rs"]
mod experiment_data;
mod serial;
#[cfg(feature = "smoke-test")]
mod smoke;
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, AtomicU32, Ordering},
};
use tauri::Manager;

#[derive(Default)]
struct Backend {
    connection: Mutex<Option<serial::Connection>>,
    next: AtomicU32,
    archive: Arc<Mutex<Option<experiment::Dataset>>>,
    experiment_owned: Arc<AtomicBool>,
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
    state: tauri::State<'_, Arc<Backend>>,
) -> Result<(), String> {
    let command = config.command(id)?;
    state.with_session(session, |connection| connection.send(command))
}

#[tauri::command]
fn experiment_result(
    state: tauri::State<'_, Arc<Backend>>,
) -> Result<Option<experiment::Dataset>, String> {
    Ok(state.archive.lock().map_err(|e| e.to_string())?.clone())
}

#[tauri::command]
fn experiment_csv(state: tauri::State<'_, Arc<Backend>>) -> Result<String, String> {
    // Serialize export with START submission and preserve ownership across disconnect.
    let _connection = state.connection.lock().map_err(|e| e.to_string())?;
    if state.experiment_owned.load(Ordering::Acquire) {
        return Err("CSV export is disabled until autonomous ownership is resolved".into());
    }
    // UI offers an explicit Save; CSV is constructed only from a verified result.
    state
        .archive
        .lock()
        .map_err(|e| e.to_string())?
        .as_ref()
        .map(|d| d.csv())
        .ok_or("No verified experiment dataset".into())
}

fn main() {
    let builder = tauri::Builder::default()
        .manage(Arc::new(Backend::default()))
        .invoke_handler(tauri::generate_handler![
            serial_ports,
            serial_connect,
            serial_send,
            serial_poll,
            serial_disconnect,
            experiment_start,
            experiment_result,
            experiment_csv
        ])
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
            if matches!(event, tauri::WindowEvent::Destroyed) {
                window.state::<Arc<Backend>>().disconnect();
            }
        });
    #[cfg(feature = "smoke-test")]
    let builder = builder
        .invoke_handler(tauri::generate_handler![
            serial_ports,
            serial_connect,
            serial_send,
            serial_poll,
            serial_disconnect,
            experiment_start,
            experiment_result,
            experiment_csv,
            smoke::smoke_report
        ])
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
