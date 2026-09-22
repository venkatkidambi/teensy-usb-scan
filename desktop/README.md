# Teensy Panel desktop application

Three modes are available: **Dashboard**, **Quasi-static**, and **Sinusoidal**.
Experiment configuration is validated in Rust before START; the Teensy performs
all motion scheduling and acquisition. Use current copies fresh encoder position.
Normal PC traffic pauses until the retained dataset is downloaded and verified.
ABORT stays available. Results show both plots and the completed/aborted/faulted
outcome; Save CSV exports the verified native result on demand. Reconnecting
recovers an active run or downloads its retained data without sending a heartbeat.
See [the experiment design and limits](../EXPERIMENTS.md).

Tauri 2 + plain HTML/CSS/JavaScript, with serial IO in Rust. The release app
embeds its UI and needs no Python, Node, browser tab, or HTTP server.

## Local development

Install Node 22.12+ and the platform Rust/Tauri prerequisites, then from the
repository root:

```sh
npm install
npm run dev
```

This opens the native desktop window with Vite on `127.0.0.1:1420`. CSS changes
update in place; HTML/JavaScript edits reload the UI (reconnect the serial port
after reload). Rust changes rebuild and restart the app. No browser tab opens.
The launcher detects the native Rust host target on Windows, macOS, and Linux.
On Windows, run **`.\dev.cmd`** from the repository root to install dependencies,
build and reflash the Teensy, then open the frontend in development mode. Close
any existing app or serial terminal first. The upload uses `upload.config.json`;
press the Teensy Program button if the loader waits. Installation or upload
failures stop the launcher before development starts. Firmware upload prerequisites
are listed in the root README's Windows upload section.
The launcher uses `npm.cmd`, avoiding PowerShell's blocked `npm.ps1`, and scopes
the upload execution-policy bypass to its PowerShell process. Extra arguments are
forwarded only to the development command. For frontend-only development, use
`npm.cmd install` and `npm.cmd run dev` directly. The bare `npm` command may still
be blocked by PowerShell's policy.

`npm run build` embeds the optimized Vite assets in the release executable.
`npm test` runs the JavaScript protocol tests. Vite and the Tauri CLI are dev-only
dependencies; there is no frontend framework or additional runtime.

## Xeryon manual movement

Flash current firmware with `.\dev.cmd`, connect Teensy, then use **Xeryon movement**:
enter travel in µm, optionally enter speed in µm/s, then click **Up (+)** or **Down (−)**. Travel is relative to the
latest encoder position and rounded to 0.078 µm/count. Up increases the encoder
value; Down decreases it. If entered, speed is a positive whole number and is
sent before the target. If left blank, the XD-C keeps its current speed setting and existing tuning
to reach and hold the target. **Stop** requests an interruption.

The panel shows the requested target and waits for controller feedback before
reporting completion. Controls require fresh position/status and Xeryon-capable
firmware. With older firmware that lacks `xeryon_speed: true`, the speed input is
disabled and moves use the controller's existing speed setting.
The plunger controls operate the separate wired motor. See the root README for
motion timeout behavior and hardware-validation limits.

## Cargo-only launch

Run `desktop\start.cmd` on Windows or `./desktop/start.sh` on macOS/Linux.
These select the Rust **host target**, overriding the parent firmware's target.
The initial dependency download/build needs network access. Subsequent direct
launches of the release executable work offline and do not require Rust installed.
The Windows executable requires the system WebView2 runtime, not Edge running.
Platform development prerequisites: https://v2.tauri.app/start/prerequisites/

Select the primary Teensy USB serial port (COM6 on the user's Windows machine),
close other terminals using it, then connect. Matching `5824:27dd` ports sort first.
The native port list includes other serial devices in case USB metadata is missing;
the handshake must still identify protocol 1 / Teensy 4.1 before controls enable.
The XD-C's `04d8:ef4a` is not the PC-facing serial port.

Encoder and target positions, including the plot and demo, display microns (µm)
using 0.078 µm per encoder count, with three decimal places.

## Build

```text
cargo build --manifest-path desktop/Cargo.toml --target HOST --release --locked
```

Replace HOST with `rustc -vV`'s host, usually `x86_64-pc-windows-msvc` on Windows
or `aarch64-apple-darwin` on Apple Silicon. The binary is in
`desktop/target/HOST/release/teensy-panel[.exe]`. A Windows binary must be built
on Windows with the MSVC prerequisites; the macOS build is not a Windows executable.
The supplied launchers handle the target selection automatically.

Installer bundling is disabled; the initial deliverable is a standalone executable
with embedded assets, using the OS webview runtime. No Tauri CLI is required.
The separate desktop Cargo.lock pins its dependencies and does not alter firmware.

## Architecture

- `src/main.rs`: local serial/experiment Tauri commands, retained result state, and port cleanup on exit.
- `src/serial.rs`: native port discovery, exclusive connection ownership, bounded
  command validation, and a worker with a 20 ms read timeout. Eight queued commands,
  32 replies, 32 logs, one latest unsolicited snapshot; lines capped at 4096 bytes.
  Reply overflow closes the connection explicitly instead of dropping acknowledgments.
- `ui/protocol.mjs`: schema validation, session/request IDs, expected reply types,
  2.5-second request timeout, and one in-flight request. IPC polling is serial and
  scheduled every 50 ms after the previous call finishes, preventing overlapping polls.
- `src/experiment.rs`: configuration units/validation, bounded binary framing, dataset CRC and CSV generation.
- `ui/experiment.js`: configuration tabs, experiment status, result plots and Save CSV.
- `ui/app.js`: local port picker, telemetry and flags, bounded Canvas history,
  heartbeat, stale indicators, an explicitly simulated demo, and wired-plunger controls.

The normal app only grants access to its own commands; no filesystem, shell,
network, or third-party Tauri plugins are exposed to the UI. CSP permits embedded
assets and Tauri IPC. The smoke-test command is compiled only with its test feature.

The desktop process uses normal OS threads and allocations. These are entirely
separate from the Teensy, which remains no_std/no_main with static buffers and
TinyUSB OPT_OS_NONE. This app never flashes or modifies the Teensy automatically.

## Validation

```text
cargo test --manifest-path desktop/Cargo.toml --target HOST --locked
node --test desktop/tests/protocol.test.mjs
cargo run --manifest-path desktop/Cargo.toml --target HOST --features smoke-test --locked
```

Node 22.12+ is needed for npm development and JavaScript tests. The native smoke test
opens the real desktop window, exercises native IPC/port listing and demo rendering,
then exits with a success/failure status. It never opens a physical serial device.
Normal launchers do not enable it.

Plunger controls require firmware advertising `motion: true`. They use the
separate 16,688 counts/mm calibration, integer travel in µm, and PWM drive 1–255.
Stop can interrupt a pending request and closes the serial session if its reply
fails. Zero and new moves are disabled while moving or when telemetry is stale.
The HX711 panel shows raw ADC counts, a 16-sample average, sample age, saturation,
and a 60-second signal plot. It uses A1/p15 DOUT and A0/p14 SCK, channel B/gain 32,
with inverted polarity matching the supplied measurement sketch. Physical pressure
units need calibration; no scale is assumed. Demo readings are explicitly simulated.
Pressure collection continues independently of Xeryon enumeration or stream pause.
