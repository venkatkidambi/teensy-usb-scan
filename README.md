# Teensy 4.1 USB host scanner

Bare-metal Rust (`no_std`, `no_main`) plus the TinyUSB 0.21.0 C host stack.
Cargo builds everything; no Arduino, PlatformIO, or operating system is used.
The Rust `teensy4-bsp` supplies startup, clocks, pins, and the linker script.

The primary **USB1 connector** provides bidirectional CDC communication with a
local Tauri desktop application. The separate **USB2 host header** receives feedback
from the Xeryon XD-C (`04d8:ef4a`). The onboard LED still toggles every 500 ms.

## Desktop app quick start

1. Flash the firmware with `.\upload.cmd` on Windows (`./upload.sh` on macOS).
2. Close any serial terminal using the Teensy's port.
3. Run **`.\desktop\start.cmd`** on Windows. This builds and opens a desktop
   window. On macOS/Linux use **`./desktop/start.sh`**.
4. Select the Teensy's primary USB port (COM6 on your setup) and click **Connect Teensy**.
5. Connect the XD-C to the USB2 host header to see its live feedback.

This is a **Tauri 2 desktop app** with native Rust serial access. There is no Python,
browser tab, Web Serial, or Node runtime dependency in the release app.
For development, install Node 22.12+ and run `npm install`, then `npm run dev`
from this repository root. Vite serves the UI on loopback inside the Tauri window;
CSS updates live, while JavaScript/HTML edits reload the UI and require reconnecting.
Rust edits rebuild and restart the app. `npm run build` produces a release executable.
Plain JavaScript/CSS assets are embedded in the executable. Tauri uses the system
webview rather than bundling a browser engine; the UI still uses web technologies
inside its native desktop window. [Tauri architecture](https://v2.tauri.app/start/)

First-build prerequisites on Windows: Rust's MSVC toolchain, Microsoft C++ Build
Tools with Desktop development with C++, and the WebView2 runtime. These are
[Tauri's platform prerequisites](https://v2.tauri.app/start/prerequisites/).
Rust/C++ may already be installed for firmware development. The first build
fetches Cargo dependencies and takes longer; the pinned desktop Cargo.lock is
included. Once built, you can launch the executable directly without Cargo:

`desktop\target\x86_64-pc-windows-msvc\release\teensy-panel.exe`

That path is for x64 Windows; ARM64 Windows uses its native target directory.
The launchers explicitly select the host target so this app is never compiled
for the repository's default Teensy ARM target. The desktop and firmware crates
have separate lockfiles and dependencies. Build-time desktop dependencies never
run on or link into the Teensy firmware.

The dashboard includes positions, frequency, decoded status flags, a 60-second
plot, sample ages, a bounded activity log, and Ping / Read snapshot / Pause stream.
Pause only stops USB1 snapshots; actuator reception continues. Pressure sensing
is labeled **Not configured**. No motion commands are enabled. **Preview demo**
renders simulated readings without opening a serial port.

The desktop serial worker has a 20 ms read timeout, bounded command/response/log
queues, and coalesces unsolicited snapshots if the UI is slow. The UI polls local
IPC at up to 20 Hz. Request IDs, reply types, protocol version, and timeouts are
checked; device disconnects close the session and clear displayed values. Closing
the app releases the port. A host-side command allowlist rejects arbitrary serial
writes. Hardware throughput and latency have not been benchmarked.

### Bare-metal data path

The firmware remains `no_std` / `no_main`, with TinyUSB `OPT_OS_NONE`: no RTOS,
Arduino runtime, or heap allocation. The main loop services both USB ports and
uses SysTick for its 1 ms wakeup; the USB2 interrupt only services host events.
USB1 uses the existing i.MX RT USB driver directly instead of the output-only
logger backend. The USB bus operates in high-speed mode with 512-byte packets.

XD-C feedback is parsed into a fixed-size latest-value cache. Compact JSON
snapshots are generated at 10 Hz, while reception continues independently.
Readable flag labels and plotting run on the PC. USB writes never wait: an
unsent frame retains its offset, and later snapshots are coalesced when the PC
is slow. Command responses are processed before starting another snapshot.
Buffers and work per foreground iteration are bounded. These choices reduce
formatting and USB overhead; real-board throughput/latency has not been benchmarked.
This dashboard samples telemetry; it is not a lossless recorder or a hard-real-time
control loop.

### XD-C feedback

The decoder follows the [XD-C manual v3.1, section 4.3, pages 13–14](https://xeryon.com/download-files/Xeryon%20XD-C%20manual%20v3.1%20-%20JPL.pdf).
The dashboard names active `STAT` bits, preserving the numeric word. Bit 0 is
XRTA-only, bit 20 unused, and bits 22–23 undefined. No flags set is not a health
verdict. Position remains in encoder counts; physical units require stage resolution.

The fixed 64-byte parser handles LF/CRLF lines across USB packets. Malformed or
oversized lines are counted and discarded through newline. Disconnects clear
partial lines and cached values. The snapshot exports EPOS, DPOS, STAT, SRNO,
SOFT, FREQ, TIME, and SYNC; other numeric tags count as records but are not retained.
Values older than three seconds are marked stale. Even fresh samples are not an
atomic measurement of every actuator property: each has its own reception time.

USB2 supports one standard CDC ACM interface, opened at 115200 8-N-1 with DTR.
Other USB protocols (including CANopen/SLCAN) are not implemented. A three-second
CDC setup timeout or failed setup requires unplugging/reconnecting to retry.
No motion, configuration, or query commands are sent to the XD-C. The PC command
protocol is documented in [desktop/PROTOCOL.md](desktop/PROTOCOL.md).

## Tools and build

Tested with Rust 1.95.0 and Arm GNU Toolchain 15.3.Rel1.

```sh
rustup target add thumbv7em-none-eabihf
rustup component add llvm-tools-preview
cargo install cargo-binutils --locked
brew install teensy_loader_cli
brew install --cask gcc-arm-embedded
```

The C compiler installer may require your administrator password in a terminal.
Alternatively, unpack the official [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
and set `ARM_GNU_TOOLCHAIN` to its root directory. This workspace already has a
portable toolchain in `.tools/arm-gnu-toolchain-15.3.rel1-darwin-arm64-arm-none-eabi`,
which `build.rs` detects automatically. `.tools/` is ignored and is not part of
the source distribution. A normal `arm-none-eabi-gcc`/`arm-none-eabi-ar` on PATH
also works. No downloads happen in the build script.

```sh
cargo check --locked
cargo build --release
```

Cargo defaults to `thumbv7em-none-eabihf`. Keep `Cargo.lock` and `vendor/` in
version control. The normal ELF path is
`target/thumbv7em-none-eabihf/release/teensy-usb-scan`.

## Upload and scan

1. Connect the primary USB connector to the Mac.
2. Run `./upload.sh`, then press the Teensy's Program button when the loader waits.
3. Start the dashboard using the quick-start steps above. For manual inspection,
   a serial terminal at 115200 can read newline-delimited JSON and `#` diagnostics.
   Use either the terminal or the desktop app, not both at once.
4. Attach the actuator to the separate five-pin host header using the correct
   Teensy 4.1 USB host cable. For this first scanner, connect directly, without a hub.

The dashboard handshake verifies firmware protocol version 1. Structured snapshots
report device identity, configuration, CDC state, and available feedback. Optional
`#` diagnostics are best-effort and can be dropped when another log is pending.

The upload script builds, converts the ELF with `cargo objcopy` to
`target/firmware/firmware.hex`, and runs
`teensy_loader_cli --mcu=TEENSY41 -w -v` with that file. It discovers the Cargo
artifact rather than assuming a binary filename. USB software reboot is not implemented.

### Windows upload

Use Windows PowerShell 5.1 or PowerShell 7 with `upload.ps1`. Install Rust
(including its Visual Studio C++ build prerequisites), the ARM Rust target,
`llvm-tools-preview`, and `cargo-binutils` using the commands above. Install the
Windows **arm-none-eabi** Arm GNU Toolchain and put its `bin` directory on PATH.
Do not copy the macOS `.tools` directory to Windows.

The script requires **teensy_loader_cli.exe**, not the graphical `teensy.exe`.
PJRC provides the [command-line loader source and build instructions](https://www.pjrc.com/teensy/loader_cli.html)
(Windows builds use MinGW). `upload.config.json` stores the loader path and is
already set to `C:\msys64\home\vek19004\teensy_loader_cli\teensy_loader_cli.exe`.
Edit its `TeensyLoader` value if the executable moves (JSON requires doubled
backslashes). For normal uploads from PowerShell or Command Prompt, run:

```powershell
.\upload.cmd
```

The launcher invokes `upload.ps1` with an execution-policy bypass scoped to that
PowerShell process, so there is no repeated policy setup or persistent policy
change. Organization-enforced Group Policy still takes precedence.

An explicit loader argument overrides the config. If the config file is absent,
the script searches PATH for `teensy_loader_cli.exe`:

```powershell
.\upload.cmd -TeensyLoader 'C:\Tools\Teensy\teensy_loader_cli.exe'
```

You can also invoke PowerShell directly (equivalent to the launcher):

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\upload.ps1
```

The script builds, creates `target/firmware/firmware.hex`, then flashes using the
same options as `upload.sh`. It stops on any failed command, handles paths with
spaces, works from another directory, and restores the caller's working directory.
After upload, find the Teensy's COM port in Device Manager and open it in a serial
terminal at 115200 baud, 8-N-1, with no flow control. Windows upload and actuator
recognition have been confirmed by the user; the new config and `.cmd` launcher
still need a Windows run.

## Validation and limits

The user confirmed the LED heartbeat, USB host enumeration, and XD-C EPOS/STAT
reception on hardware. Desktop Rust/JavaScript tests cover framing, bounded queues,
command allowlisting, reply matching, timeouts, and disconnect cleanup. A native
macOS Tauri smoke test verifies embedded UI rendering, native port discovery,
invalid-port rejection, demo telemetry, status flags, and clearing on disconnect.
Windows packaging and actual native COM6 communication still need a Windows/Teensy run.

Firmware tests and build:

```sh
./tests/run.sh
cargo fmt --check
cargo build --release --locked
```

Desktop tests (replace HOST with the host from `rustc -vV`, such as
`x86_64-pc-windows-msvc` or `aarch64-apple-darwin`):

```sh
cargo test --manifest-path desktop/Cargo.toml --target HOST --locked
node --test desktop/tests/protocol.test.mjs
```

Node is used for the npm/Vite development workflow and JavaScript tests.
The Cargo-only launchers and release executable do not require Node. Native UI smoke test (opens a temporary window, tests, then exits):

```sh
cargo run --manifest-path desktop/Cargo.toml --target HOST --features smoke-test --locked
```

The smoke-test feature is off in the normal launcher/release app. It does not
open any physical serial port. See [desktop/README.md](desktop/README.md).

On hardware, verify Connect/Ping/Read snapshot, pause/resume, actuator unplug and
reconnect, and primary USB unplug/reconnect. Check that displayed data clears on
disconnect and the LED continues blinking when the desktop app is closed. If the
handshake times out, verify the correct port and current firmware. The earlier
logging-only firmware cannot answer these requests.

Only one directly attached device and one CDC ACM interface are supported, with
a configuration descriptor up to 1024 bytes. Host control-transfer timeout support
is incomplete; an unresponsive device may require unplugging or resetting the board.
Motion and I²C pressure sensing are not implemented in this telemetry phase.
