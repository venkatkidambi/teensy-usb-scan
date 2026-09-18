# Teensy Panel desktop application

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
If PowerShell blocks npm.ps1, use `npm.cmd install` and `npm.cmd run dev`.

`npm run build` embeds the optimized Vite assets in the release executable.
`npm test` runs the JavaScript protocol tests. Vite and the Tauri CLI are dev-only
dependencies; there is no frontend framework or additional runtime.

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

- `src/main.rs`: five local Tauri commands, app state, and port cleanup on exit.
- `src/serial.rs`: native port discovery, exclusive connection ownership, read-only
  command validation, and a worker with a 20 ms read timeout. Eight queued commands,
  32 replies, 32 logs, one latest unsolicited snapshot; lines capped at 4096 bytes.
  Reply overflow closes the connection explicitly instead of dropping acknowledgments.
- `ui/protocol.mjs`: schema validation, session/request IDs, expected reply types,
  2.5-second request timeout, and one in-flight request. IPC polling is serial and
  scheduled every 50 ms after the previous call finishes, preventing overlapping polls.
- `ui/app.js`: local port picker, telemetry and flags, bounded Canvas history,
  heartbeat, stale indicators, and an explicitly simulated demo.

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
