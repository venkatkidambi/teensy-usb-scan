# Teensy 4.1 USB host scanner

Bare-metal Rust (`no_std`, `no_main`) plus the TinyUSB 0.21.0 C host stack.
Cargo builds everything; no Arduino, PlatformIO, or operating system is used.
The Rust `teensy4-bsp` supplies startup, clocks, pins, and the linker script.

The primary **USB1 connector** provides bidirectional CDC communication with a
local browser dashboard. The separate **USB2 host header** receives feedback
from the Xeryon XD-C (`04d8:ef4a`). The onboard LED still toggles every 500 ms.

## Dashboard quick start

1. Flash the updated firmware: `.\upload.cmd` on Windows, or `./upload.sh` on macOS.
2. Close your serial terminal so the browser can open the port.
3. On Windows, run `.\frontend\start.cmd` (requires Python 3). On macOS/Linux,
   run `python3 frontend/serve.py`. No pip/npm packages or frontend build are needed.
4. Open **http://127.0.0.1:8000** in desktop Chrome or Edge.
5. Click **Connect Teensy** and select its primary USB serial port (previously COM6;
   Windows may assign another number after the USB product name changes).
6. Connect the XD-C to the host header. Position, frequency, status flags, and
   controller identity populate as its existing broadcasts arrive.

The interface includes a 60-second position plot, sample ages, decoded flags,
Ping/Read snapshot/Pause stream requests, and a bounded activity log. Pause
only stops USB1 snapshots; actuator reception keeps running. Pressure is shown
as **Not configured**, and motion commands are not implemented. A clearly labeled
**demo preview** works without a board and sends no USB traffic.

The frontend is plain HTML/CSS/JavaScript with system fonts, local assets, and
Canvas plotting (about 33 KB uncompressed). Python only serves static files on
127.0.0.1; it never opens the serial port. There is no backend API, framework,
package install, analytics, CDN, or network connection required after loading.
[Web Serial](https://developer.chrome.com/docs/capabilities/serial) requires a
supported browser and a secure context such as localhost. Do not open index.html
as a file; JavaScript modules need the local HTTP server. Ctrl+C stops the server.

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
protocol is documented in [frontend/PROTOCOL.md](frontend/PROTOCOL.md).

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
   Use either the terminal or the browser, not both at once.
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

The user confirmed the LED heartbeat, enumeration, and USB2 reception of XD-C
EPOS/STAT on hardware. The new USB1 command transport and dashboard have passed
release compilation, native C/Rust tests, frontend protocol tests, and desktop/mobile
Chrome smoke tests using simulated serial IO. Actual bidirectional USB1 behavior
still requires a Teensy/Windows run. An upstream CDC-only unused-function warning
remains in TinyUSB; the build succeeds.

```sh
./tests/run.sh                   # native C bridge, decoder, protocol tests
node --test tests/frontend.test.mjs   # Node 22+; development tests only
cargo fmt --check
cargo build --release --locked
```

Optional browser smoke tests: run the local server and start an isolated Chrome
with `--headless --remote-debugging-port=9222 --user-data-dir=<temporary-directory>`,
then run `node tests/browser-smoke.mjs`. Screenshots go to `target/tests/`.
The test simulates Web Serial; it does not access a physical actuator.

On the board, verify Connect/Ping/Read snapshot, pause/resume, and actuator unplug/
reconnect. Check that positions and flags clear on unplug and repopulate on reconnect,
and that the heartbeat continues with the browser closed. Repeat after unplugging
and reconnecting the Teensy's primary USB cable. If a terminal works but Connect
fails, close the terminal and check that the updated firmware is flashed. Firmware
from the earlier logging-only stage cannot answer the dashboard handshake.

Only one directly attached device and one CDC ACM interface are supported, with
a configuration descriptor up to 1024 bytes. Strings are optional and best-effort.
Host control-transfer timeout support is incomplete; a nonresponsive device may
require unplugging/reconnecting or resetting the board to retry. No motion or I²C
pressure-sensor driver is implemented in this telemetry phase.
