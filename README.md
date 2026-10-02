# Teensy experiment controller

Bare-metal Teensy 4.1 / i.MX RT1062 firmware and a Tauri 2 desktop dashboard for
Xeryon motion, LPS22HHTR pressure acquisition, and a wired plunger. Firmware is
Rust `#![no_std]` / `#![no_main]` with fixed buffers and TinyUSB C FFI; it uses
no allocator, Arduino runtime, or RTOS. The desktop uses Vite and vanilla
JavaScript/HTML/CSS with native Rust serial access.

## Structure

- `src/`: firmware application, experiment engine, safety, acquisition, and protocol.
- `c/`, `vendor/tinyusb/`: USB2 host bridge and pinned TinyUSB 0.21.0 source subset.
- `desktop/src/`, `desktop/ui/`: native backend and dashboard/experiment views.
- `desktop/calibration/`: shared force calibration and its source observations.
- `tests/`, `desktop/tests/`: native firmware/C and desktop regression suites.
- `tools/calibrate_force.py`: regenerate calibration from the original workbooks.
- `upload.*`, `desktop/start.*`: build/flash and native app launchers.

Keep both Cargo lockfiles, `package-lock.json`, and the vendored sources in version
control. Generated firmware lives in `target/firmware/`; frontend output lives in
`desktop/dist/`. [Protocol and dataset reference](desktop/PROTOCOL.md) covers
units, framing, ownership, diagnostics, and CRC/ACK semantics.

## Toolchains and firmware build

Use Rust with edition 2024 support (tested with 1.95.0), Arm GNU `arm-none-eabi-gcc`
and `arm-none-eabi-ar` (tested with 15.3.Rel1), and cargo-binutils:

```sh
rustup target add thumbv7em-none-eabihf
rustup component add llvm-tools-preview
cargo install cargo-binutils --locked
cargo build --release --locked
mkdir -p target/firmware
cargo objcopy --release --locked -- -O ihex target/firmware/firmware.hex
```

These commands build and generate HEX without flashing. Cargo defaults to the
Teensy target through `.cargo/config.toml`. Put the ARM tools on PATH or set
`ARM_GNU_TOOLCHAIN` to their installation root. `build.rs` also detects the local
macOS toolchain under `.tools/arm-gnu-toolchain-15.3.rel1-darwin-arm64-arm-none-eabi`.
It does not download tools. The ELF is
`target/thumbv7em-none-eabihf/release/teensy-usb-scan`.

## Flashing

Install `teensy_loader_cli` separately; the graphical Teensy Loader is a different
program. Close apps or terminals holding the primary Teensy serial port.

On Windows, install the Rust MSVC/C++ build tools and Windows ARM GNU tools, then:

```powershell
.\upload.cmd
# If the command-line loader is not on PATH:
.\upload.cmd -TeensyLoader 'C:\Tools\Teensy\teensy_loader_cli.exe'
```

The version-controlled `upload.config.json` points to
`C:\msys64\home\vek19004\teensy_loader_cli\teensy_loader_cli.exe`.
Edit that file if the loader moves, or pass `-TeensyLoader` to override it.
If the config is absent, the script searches PATH.

Both `upload.cmd` and `dev.cmd` run the upload through
`powershell.exe -NoProfile -ExecutionPolicy Bypass`, so no persistent execution-policy
change is needed. Use these `.cmd` launchers when PowerShell blocks `.ps1` files.
To invoke the PowerShell script directly with the same process-scoped bypass:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\upload.ps1
```

On macOS/Linux with Bash and the tools on PATH:

```sh
./upload.sh
```

Both upload scripts build, generate `target/firmware/firmware.hex`, and invoke
`teensy_loader_cli --mcu=TEENSY41 -w -v`. Press the Teensy Program button when
the loader waits. On macOS, the loader is available through Homebrew; Linux may
require USB device permissions configured for the loader.

## Desktop build and use

Development needs Node 22.12+, Rust, and the platform Tauri build prerequisites:
Windows MSVC C++ tools/WebView2, macOS Xcode command-line tools, or Linux GTK/WebKit
development libraries. From the repository root:

```sh
npm ci
npm run dev       # native app with Vite development server
npm run build     # Vite assets and native release executable
```

The scripts select the native Rust host target, overriding the Teensy default.
The executable is `desktop/target/HOST/release/teensy-panel[.exe]`; installer
bundling is disabled. Release apps embed their assets and need no Node or server.
`desktop\start.cmd` or `./desktop/start.sh` builds/runs with Cargo alone, embedding
the source UI. Windows `dev.cmd` additionally installs npm dependencies and
**flashes the firmware** before opening the development app.

Select the primary Teensy port, click **Connect Teensy**, and attach the XD-C to
the separate USB2 host header. Only one app/terminal can own the serial port.
**Preview demo** supplies simulated readings without opening hardware.

- **Dashboard:** feedback/status, pressure/force plots, manual Xeryon and plunger
  movement, Stop, and potentiometer limits. Xeryon travel is relative to fresh
  EPOS at 0.078 µm/count; optional integer speed is sent before DPOS.
- **Quasi-static:** validated start, increment, hold, step count, and required
  speed. Firmware verifies SSPD and the controller TOU2 budget before motion;
  holds begin after confirmed arrival.
- **Sinusoidal:** starts at `center − amplitude` with zero velocity, then uses
  `x = center − A cos(2πft)` and `v = 2πfA sin(2πft)`. Adaptive 4–10 ms SSPD/SCAN
  commands use elapsed Teensy time and coalesce under backpressure. Reversals
  wait for zero-speed, direction, and resume transfer completion in order. The
  final boundary sends STOP at the original starting extremum (`center − amplitude`),
  with no subsequent positioning move.
- **Experiment Data:** verified results, position/force plots, phase estimates,
  CSV and PNG exports. Sweeps run frequencies in order, downloading each result
  before the next START. ABORT, faults, or disconnect cancel remaining runs.
  The waveform envelope excursion summary checks every unique logged EPOS
  receipt during the sinusoid against its rounded encoder endpoints. Reversal
  window excursions are labelled separately; spikes outside those windows are
  still included in the waveform summary. Raw readings are never smoothed or
  removed. These sampled maxima cannot bound motion between recorded readings.

The current sinusoidal gain trial first reads `PRO2`, reduces it to **30** only
when the current value is higher, and requires a fresh matching readback before
any positioning or waveform motion. A gain already at or below 30 is preserved.
Setup failure prevents the run and requests STOP. Results/CSV metadata record
`scan_gain.before`, `scan_gain.applied`, and `scan_gain.cap`; an unverified value
is null. This writes no `SAVE`: the volatile controller-wide gain stays in effect
after completion or abort, including subsequent manual/quasi-static moves, until
the controller is reset/power-cycled or the original gain is explicitly restored.
Each sinusoidal START reapplies the cap, defined by `SINE_SCAN_GAIN_CAP` in
`src/experiment_data.rs`. For the first comparison, rerun the experiment-7 settings
(1 Hz, 10 µm amplitude, five cycles, same center/load, compensation off).

Optional **Feed-forward delay compensation** defaults off with a 17 ms entry.
Whole milliseconds from 0 through `MAX_COMPENSATION_MS` (25 ms) are accepted.
Only actuator velocity advances: `v_ff(t) = Aω sin(ω(t + τ s(u)))`, where
`τ = compensation_ms / 1000`, `u = min(t / 0.250 s, 1)`, and `s(u) = 3u² − 2u³`.
After 250 ms this is exactly `Aω sin(ω(t + τ))`. MoveToStart stays at `C−A`;
the smooth activation preserves zero initial velocity without an abrupt jump.
This experimental time advance is not a measurement of mechanical delay.
The unadvanced reference `C−A cos(ωt)`, duration `N/f`, all recorded timestamps,
and pressure timing retain real elapsed time. There is no EPOS feedback or
waveform DPOS correction. Reversals and scheduler error bounds use the advanced
velocity, retaining ordered `SSPD=0 → SCAN=new direction → current speed`.
The boundary STOP remains at `N/f`, even if advanced velocity is nonzero there;
compensation does not guarantee physical return to the start position. Metadata
and CSV retain the settings, configured/applied phase advance, and existing
command latency and feedback age statistics. Toggle the checkbox for A/B runs
without changing firmware; sweep runs retain the same settings.

Experiments record at 50 Hz into 4,096 fixed 48-byte records (192 KiB). The total
acquisition bound is 81.9 s; sinusoidal runs reserve 10 s for final STOP transmission,
accept 0.1–10 Hz, and are limited to 144 cycles by diagnostic capacity. USB1 PC
disconnect does **not** stop an autonomous run while Teensy power remains. Manual
moves instead stop on disconnect, stale feedback, heartbeat loss, or deadline.
ABORT/faults request STOP and retain partial results. USB STOP is best effort if
the actuator link fails. Command acceptance is not physical motion confirmation.
Controller travel limits and tuning remain the operator's responsibility.

Results stay owned by firmware until count/CRC/ACK verification. Reconnect recovers
an active run or retained dataset. Desktop results survive reconnect/reload in the
same process; the first verified result of a new experiment replaces the previous
experiment. Save CSV before replacement, closing the app, or removing power.

## Hardware and diagnostics

USB1 is a Rust CDC device; USB2 uses TinyUSB `OPT_OS_NONE`, one directly attached
CDC ACM XD-C (`04d8:ef4a`), 115200 8-N-1. SysTick supplies the wrapping 1 ms clock;
foreground work services pressure, USB, motion, and acquisition without blocking.
USB2 control-transfer timeout support is limited; failed enumeration may require
unplugging/reconnecting. No homing or saved controller settings change automatically.

The plunger uses pin 2 PWM, 4/5 direction, 6 standby, 11/12 quadrature, and 20
potentiometer; calibration is 16,688 counts/mm. Zero requires stopped motion.
Potentiometer bounds default to 30–75% at boot and permit inward recovery. Drive
stops at a bound, invalid/stale feedback, or watchdog timeout; mechanical coast
can overshoot. Up increases the reading. Limits survive reconnect, not power loss.

LPS22HHTR uses SDA 18, SCL 19, address 0x5C, 400 kHz I²C, and 200 Hz conversions.
Use external pull-ups, SA0 low, CS high, and INT_DRDY unconnected/low at power-up.
WHO_AM_I must be 0xB3. Nonblocking reads retain signed pressure/temperature and a
zero-initialized 16-sample average. Errors invalidate readings and retry setup.

The UI and CSV use `desktop/calibration/lps-force.json`:
`force_N = 8.648993703536255e-8 × raw − 0.35085250017057007`.
The supported range is 4,163,833–4,281,744.298 raw counts; extrapolation is labelled,
without tare or clamping. Wire data stays raw. To regenerate from the original
workbooks using Python's standard library:

```sh
python3 tools/calibrate_force.py results_calibration.xlsx 'Probe sensitivity.xlsx'
```

Retained timing, reversal, final-return, and fault metadata support post-run
diagnosis. USB completion/host receipt timestamps do not establish physical
actuation or sensor conversion times. `SINE_ENVELOPE_MARGIN_NM` optionally enables
a fresh-EPOS fault/STOP guard; it is disabled by default. The positive margin and
`SINE_ENVELOPE_MAX_AGE_MS` (1–1000, default 100) are build environment settings.
The `fixed-100hz` Cargo feature remains available for timing comparisons;
normal firmware uses adaptive scheduling.

Fatal pin-13 LED diagnostics repeat a long pulse, a fault group, then a phase group.
Fault counts: 2 panic, 3 HardFault, 4 unexpected exception, 5 stalled foreground.
Phase counts: 1 dashboard/startup, 2 positioning, 3 armed, 4 waveform/stopping,
5 cleanup. An immediate **3 / 1** can indicate startup stack overflow.
`.cargo/config.toml` reserves **64 KiB** because the main frame plus startup logging
exceeds the BSP's 16 KiB default. Fatal handling uses a separate 1 KiB emergency
stack, disables the wired plunger, and halts; it cannot deliver USB STOP to the
independently powered Xeryon. `FATAL_REPORT` retains registers/phase in debugger
memory until power loss. Normal experiment faults use the existing STOP path.

## Validation

```sh
cargo fmt --check
cargo fmt --manifest-path desktop/Cargo.toml --check
./tests/run.sh
cargo test --manifest-path desktop/Cargo.toml --target HOST --locked
npm test
npm run build:ui
cargo build --release --locked
npm run build
```

Replace `HOST` with the host reported by `rustc -vV` (for example
`aarch64-apple-darwin` or `x86_64-pc-windows-msvc`). Native C tests require `cc` or
`HOST_CC`; the firmware tests use host `rustc`. Optional webview/IPC validation:
`cargo run --manifest-path desktop/Cargo.toml --target HOST --features smoke-test --locked`.
It opens a test window and uses demo data without opening a physical serial port.
Software checks do not replace loaded-stage timing, tracking, and stop validation.
