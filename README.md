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
- **Sinusoidal:** Teensy generates `x = center − A cos(2πft)` using GPIO pulses.
  Initial DPOS moves to `center − amplitude` and waits for Position Reached.
  The waveform starts at rest, completes exactly N cycles, disables pulses,
  sends STOP, then returns to center by DPOS and waits for Position Reached.
- **Experiment Data:** verified results, position/force plots, phase estimates,
  CSV and PNG exports. Sweeps run frequencies in order, downloading each result
  before the next START. ABORT, faults, or disconnect cancel remaining runs.
  The waveform envelope excursion summary checks every unique logged EPOS
  receipt during the sinusoid against its rounded encoder endpoints. Raw readings are never smoothed or
  removed. These sampled maxima cannot bound motion between recorded readings.

Sinusoidal motion uses XD-C `GPIO=2` and `STPS=1`, verified by USB readback before
ENABLE goes high. No settings are saved to controller flash. USB EPOS/STAT feedback,
pressure acquisition, and logging continue normally; no position-feedback correction
is applied. GPT1 runs at 1 MHz; bounded compare interrupts sample the Q30 trajectory
at intervals of at least 25 µs, emitting at most one edge per interrupt. Direction
changes get a full low interval before a rising edge. Pulses are separated by at
least 75 µs in the timer model (extra margin below the controller's 20 kHz limit).
Phase derives from the actual wrapping hardware counter, never loop/interrupt count.
A gap over 75 µs, a missed encoder step, or a foreground service gap over 10 ms
stops pulses and faults; there are no catch-up bursts. Completion clamps the exact
N-cycle endpoint; a final quantized edge, if needed, drains before disabling ENABLE
(microsecond clock resolution, bounded interrupt latency).

Peak requested rate is `2π × amplitude_µm × frequency_Hz / 0.078`. Validation caps
it at **10,000 pulses/s** (780 µm/s peak velocity), conservatively below Xeryon's
recommended 20 kHz. For example, 1 µm at 1 Hz needs 80.55 pulses/s; the largest
integer-nm amplitude at 1 Hz is 124.140 µm (9,999.93 pulses/s). GPIO timing and
physical direction still need oscilloscope/unloaded verification on the wired rig;
software builds/tests do not establish physical tracking accuracy.

Experiments record at 50 Hz into 4,096 fixed 48-byte records (192 KiB). The total
acquisition bound is 81.9 s; sinusoidal runs reserve 10 s for STOP and confirmed
center return and accept 0.1–10 Hz within the pulse-rate limit. USB1 PC disconnect
stops sinusoidal runs and retains partial data; quasi-static runs retain their
autonomous disconnect behavior. USB2 disconnect always stops trajectory pulses. Manual
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

Xeryon GPIO wiring (in addition to the USB2 feedback connection):

| Teensy 4.1 | XD-C GPIO bank 1 | Signal |
| --- | --- | --- |
| 7 | 23 / DI0 | PULSE |
| 8 | 25 / DI1 | DIRECTION (high increments) |
| 9 | 27 / DI2 | ENABLE (high enables) |
| GND | 19 / GND | Common ground |

The [XD-C manual v3.0, sections 5.1 and 7.2](https://xeryon.com/download-files/Xeryon%20XD-C%20manual%20v3.0.pdf)
specifies 3.3 V for these digital inputs: no level shifting is required for that
bank/revision. Verify the actual controller connector before hardware use; other
voltage interfaces require an appropriate buffer. Teensy pins are
[not 5 V tolerant](https://www.pjrc.com/teensy/card11b_rev4_web.pdf).
Do not connect the nearby 5 V or 48 V supply/piezo pins. Fit external 10 kΩ
pulldowns on PULSE and ENABLE to hold them low during reset/unpowered Teensy;
hold INDEX (pin 30) low so it cannot initiate an index search. Use short signal
wires with a common ground. Verify direction unloaded before attaching the load.

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

Retained timing, final-return, and fault metadata support post-run
diagnosis. USB completion/host receipt timestamps do not establish physical
actuation or sensor conversion times. `SINE_ENVELOPE_MARGIN_NM` optionally enables
a fresh-EPOS fault/STOP guard; it is disabled by default. The positive margin and
`SINE_ENVELOPE_MAX_AGE_MS` (1–1000, default 100) are build environment settings.

Fatal pin-13 LED diagnostics repeat a long pulse, a fault group, then a phase group.
Fault counts: 2 panic, 3 HardFault, 4 unexpected exception, 5 stalled foreground.
Phase counts: 1 dashboard/startup, 2 positioning, 3 armed, 4 waveform/stopping,
5 cleanup. An immediate **3 / 1** can indicate startup stack overflow.
`.cargo/config.toml` reserves **64 KiB** because the main frame plus startup logging
exceeds the BSP's 16 KiB default. Fatal handling uses a separate 1 KiB emergency
stack, disables the wired plunger and GPIO ENABLE/PULSE, and halts; it cannot deliver USB STOP to the
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
