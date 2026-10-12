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
- **Sinusoidal:** choose **Displacement** (default), **Velocity**, or
  **Velocity + position correction** serial control
  for `x = center − A cos(2πft)`. No Teensy-to-Xeryon GPIO wiring is needed.
  All modes approach `center − amplitude` gradually and wait for Position Reached,
  execute the requested cycles, then STOP at the last cycle endpoint without returning to center.
- **Experiment Data:** verified results, position/force plots, phase estimates,
  CSV and PNG exports. Sweeps run frequencies in order, downloading each result
  before the next START. ABORT, faults, or disconnect cancel remaining runs.
  The waveform envelope excursion summary checks every unique logged EPOS
  receipt during the sinusoid against its rounded encoder endpoints. Raw readings are never smoothed or
  removed. These sampled maxima cannot bound motion between recorded readings.

Serial sine setup sends STOP, disables GPIO with `GPIO=0`, and verifies a fresh
readback before moving. It also verifies the approach speed and TOU2 budget.
No settings are saved to controller flash, and controller gains are unchanged.
The [XD-C command reference](https://xeryon.com/download-files/Xeryon%20XD-C%20manual%20v3.0.pdf)
defines SSPD as both scan speed and the speed towards a DPOS target.

- **Displacement:** streams encoder-rounded DPOS targets, with SSPD following
  local sine speed. Duplicate targets are suppressed; speed decreases near
  reversals, reducing the abrupt tiny moves caused by a large fixed speed.
- **Velocity:** streams `v = 2πf A sin(2πft)` through SSPD and SCAN. It samples
  speed at the middle of each nominal interval to reduce integration error,
  drains a zero-speed command before changing direction, and avoids repeated
  SCAN commands within each direction. This controls speed, so position can drift;
  no abrupt position correction is injected during the waveform. SSPD resolves
  whole µm/s; velocity runs with peak speed below 1 µm/s are rejected. Use
  displacement mode for small, slow trajectories.

- **Velocity + position correction:** keeps SSPD/SCAN velocity control and adds
  `v_correction = 2 s⁻¹ × position_error` to the sine velocity. The absolute-time
  position target anchors each cycle, reducing accumulated drift without issuing
  DPOS during the waveform. Error uses the target at EPOS receipt time, with a
  one-count (0.078 µm) deadband. Correction is capped at 20% of sine peak speed
  and takes at least 100 ms to ramp from zero to that cap; total commanded speed
  is capped at 780 µm/s. Missing feedback or EPOS older than 100 ms faults with
  STOP. This mode shares velocity's 1 µm/s minimum peak and speed quantization.
  Receipt timestamps cannot remove unknown sensor/USB latency; loaded-rig testing
  is still needed to assess tracking and smoothness. Dataset metadata records the
  correction settings. Update both firmware and desktop to enable this mode.

Setpoints are scheduled every **5 ms** (up to 200 updates/s), separately from
50 Hz recording. Phase uses absolute elapsed time; busy transfers coalesce to the
current setpoint instead of replaying stale commands. A control service gap over
25 ms faults and requests STOP. Actual cadence depends on USB/controller latency;
`serial_timing` metadata records update batches, skipped ticks, and maximum gap.
The approach uses a quintic position ramp with zero endpoint velocity and
acceleration, capped approximately at the configured waveform peak speed. Its
minimum duration is 1 s; it waits for confirmed arrival before acquisition.
The sine itself retains its cosine shape and zero endpoint velocity; its acceleration
changes at start/end. Serial quantization and actuator tuning still affect physical
jerk and tracking. Software tests do not establish smoothness on the loaded rig.

Serial sine validation retains the **780 µm/s peak speed** bound:
`2π × amplitude_µm × frequency_Hz <= 780`. For example, the largest integer-nm
amplitude at 1 Hz is 124.140 µm. Accepted frequencies are 0.1–10 Hz; lower
frequencies provide more serial updates per cycle (200 at 1 Hz, 20 at 10 Hz).

GPIO sine accepts up to **20,000 pulses/s** peak (1560 µm/s at 0.078 µm/count,
`STPS=1`). It retains the same position equation and validates both extrema against
controller travel limits before enabling. Pin 7 uses FlexPWM1 SM3/B: selected
50 µs slots contain a hardware edge at 20 µs and a falling edge at 45 µs. Only
required count changes select a pulse. Thus high/low times are at least 25 µs;
direction setup is at least 10 µs and rising-edge hold at least 25 µs. DMA0 clears
the output enable every slot, so missed CPU service cannot repeat a selected
pulse. An arming deadline of 10 µs and a 10 ms foreground watchdog fault safely;
ABORT/fault clears ENABLE, PWM output and timer. These are register-derived
limits, not oscilloscope validation of the controller/loaded rig.

Experiments record at 50 Hz into 4,096 fixed 48-byte records (192 KiB). The total
acquisition bound is 81.9 s; sinusoidal runs reserve up to 10 s for the final STOP
transfer to drain and accept 0.1–10 Hz within the peak-speed limit. USB1 PC disconnect
stops sinusoidal runs and retains partial data; quasi-static runs retain their
autonomous disconnect behavior. USB2 disconnect faults motion and retains data. Manual
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

Optional GPIO wiring for the retained pulse-driver implementation (unused by the
serial sine modes; leave disconnected for the current setup):

| Teensy 4.1 | XD-OEM | Signal |
| --- | --- | --- |
| 7 | 13 | PULSE |
| 8 | 14 | DIRECTION (high increments) |
| 9 | 15 | ENABLE (high enables) |
| 10 | 16 | INDEX (held low) |
| GND | 2 | Common ground (also shared through USB host) |

The [XD-OEM manual, sections 5.1 and 7.3](https://xeryon.com/download-files/Xeryon%20XD-OEM%20manual%20v3.0.pdf)
specifies 3.3 V logic. Verify the actual controller connector before hardware use;
Teensy pins are [not 5 V tolerant](https://www.pjrc.com/teensy/card11b_rev4_web.pdf).
Do not connect the nearby 5 V or 48 V supply/piezo pins. Fit external 10 kΩ
pulldowns on PULSE, ENABLE and INDEX to hold them low during reset/unpowered
Teensy. Use short signal wires with common ground. Verify direction unloaded
before attaching the load.

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

The XD-OEM manual defines TIME only as a timestamp with 0.1 ms resolution.
INFO=4 includes EPOS/STAT/DPOS/TIME, but neither that mode nor INFO=2 establishes
atomic sampling, acquisition time, clock origin, rollover modulus or transport
latency. INFO=2 remains selected to preserve requested safety/setup readbacks.
Records retain independent raw TIME and receipt age, plus unchanged EPOS receipt
age and generation. CSV exposes each receipt time and leaves controller-derived
EPOS time blank; metadata marks the association unvalidated. Plots and phase fits
use host receipt without unwrapping, offset estimation or artificial shifting.

Retained timing, final-STOP, and fault metadata support post-run
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
