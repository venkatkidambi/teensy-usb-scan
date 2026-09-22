# Autonomous experiments — firmware / desktop 0.2.0

The Teensy owns motion, timing, acquisition, faults and the retained dataset.
The desktop configures a run, sends one START, stays silent until completion
(except ABORT), and downloads and verifies the result before acknowledging it.
The desktop's 50 ms IPC receive loop only drains native memory; it does not poll
the Teensy. CSV is generated in desktop Rust only when Save CSV is selected.

## Existing hardware architecture reviewed

- Bare-metal `no_std` / `no_main`, no allocator, RTOS or async firmware. BSP startup,
  clocks, GPIO assignments and generated `t4link.x` remain unchanged.
- SysTick increments a wrapping 32-bit millisecond counter. Its existing plunger
  watchdog still runs there. PIT0 handles the LED. Experiment work runs in the
  foreground after pressure and USB2 service, before USB1 service.
- USB1 is a polled Rust CDC device. USB2 is TinyUSB's existing nonblocking CDC host
  on the known XD-C identity `04d8:ef4a`, 115200 8-N-1. No USB work was added to ISRs.
- Xeryon ASCII feedback populates the existing EPOS/DPOS/STAT/TIME cache with Teensy
  reception timestamps. Finite move completion still requires a matching DPOS
  received after command dispatch, then a later STAT with position-reached bit 10.
- Pressure remains the existing nonblocking 400 kHz I²C driver, pins 18/19,
  address 0x5C, a new acquisition attempt every 50 ms, a 20 ms transaction deadline,
  and the existing 16-sample average. Experiments store raw pressure.
- Plunger PWM, quadrature ISR, ADC, calibration and stop behavior remain unchanged.
  START rejects any active manual plunger or Xeryon motion.

## Controller and lifecycle

`ExperimentConfig` has `QuasiStatic` and `Sinusoidal` variants. One `Engine`
owns validation, state, acquisition, the fixed buffer, CRC, faults and transfer;
only target generation and completion conditions differ between modes.

```text
Dashboard -> Validate -> MoveToStart -> Armed -> Running
                        |               |         |
                        +---------------+---------+-> Complete / Aborted / Fault
                                                        -> DataReady
                                                        -> Transferring
                                                        -> Dashboard (verified ACK)
```

Validation and terminal states are synchronous transitions; Armed lasts until
the next foreground service. Any active state can abort/fault. The engine's
terminal outcome and reason remain in metadata after entering DataReady.

START checks the complete configuration, arithmetic overflow, rounded signed
26-bit positions, worst-case duration, sample count, available storage, hardware
identity, feedback freshness, healthy pressure, closed loop, valid encoder, and
stopped manual motion **before the first DPOS**. It does not home, find index,
change limits or change the encoder reference. Existing setup must be correct.

For quasi-static, start positioning completes first, then acquisition begins.
Step 1 is `start + increment`; there are exactly `steps` increments, each followed
by a hold. Holds begin only on confirmed arrival. Samples continue through every
move and hold. Each move has a 10 s deadline, including initial positioning.

For sinusoidal, initial positioning is to center. Phase is calculated from actual
elapsed Teensy milliseconds, not from loop counts. After the requested cycles,
an exact center target is sent and confirmed through the same finite-move feedback
mechanism. Acquisition continues through final settling, bounded by 10 s.
All terminal paths request STOP; metadata says whether it could be queued.

## Xeryon waveform strategy

The existing USB2 path sends bounded `DPOS=<signed encoder counts>\n` commands.
The manufacturer documents DPOS as an absolute closed-loop target, and STOP as
the stage stop command: [XD-C manual, sections 4.1–4.3](https://xeryon.com/download-files/Xeryon%20XD-C%20manual%20v3.1%20-%20JPL.pdf).

The Teensy produces a new sinusoidal target every 20 ms (50 Hz), with phase
`elapsed_ms * frequency_millihz / 1_000_000` modulo one cycle. A range-reduced
polynomial computes sine without a dependency or heap use; native numerical tests
bound its absolute sine error below 7e-10. Position quantization is 78 nm/count.
The trajectory command encoder uses bounded integer conversion rather than
general formatting in the waveform path.

No SCAN, open-loop MOVE, Windows waveform commands, SSPD updates, tuning, INFO,
or stored controller settings are used by experiments. The current controller
speed, acceleration and tuning determine actual tracking. This is sampled target
tracking through the existing point-to-point interface, not a claim of exact
continuous sinusoidal physical motion. Manual optional SSPD behavior is unchanged.

A new narrow C query checks whether the TX software FIFO is empty before each
experiment target. At most an in-flight packet and one waiting target can exist;
a nonempty queue at the next target dispatch faults instead of accumulating stale
targets. STOP clears unsent queued bytes using the existing transport operation.
An already submitted USB packet cannot be recalled.

## Timing, sample layout and memory

| Quantity | Value |
| --- | --- |
| Clock | Existing monotonic Teensy SysTick, 1 ms resolution, wrapping u32 |
| Acquisition schedule | 50 ms / 20 Hz, latest cached raw values |
| Pressure acquisition | Existing 50 ms attempt interval, completion latency unchanged |
| Sinusoidal targets | 20 ms / 50 Hz, 0.100–2.000 Hz wave frequency |
| Record | 40 bytes, ten explicit little-endian 32-bit words |
| Capacity | 4,096 records |
| Sample-buffer RAM | 163,840 bytes / 160 KiB, zero-initialized DTCM `.bss` |
| Maximum acquisition bound | 204,750 ms (204.75 s), including final settling |
| Maximum sinusoidal waveform duration | 194,750 ms, plus reserved 10 s settling |
| Initial positioning | Additional 10 s maximum, before acquisition |

Validation reserves `floor(duration_bound_ms / 50) + 1` records including t=0.
Quasi-static bound is `steps * (10_000 + hold_ms)`; sinusoidal bound is
`ceil(cycles * 1_000_000 / frequency_millihz) + 10_000`. Oversized runs are rejected,
never silently shortened. A 204.75 s bound reserves all 4,096 records.

| Offset | Field | Type / interpretation |
| --- | --- | --- |
| 0 | elapsed_ms | u32, actual logging time relative to acquisition start |
| 4 | epos | i32, raw encoder counts |
| 8 | pressure_raw | i32, existing driver value (currently unsigned 24-bit) |
| 12 | stat | u32, raw controller status |
| 16 | epos_age_ms | u32, logging time minus EPOS reception time |
| 20 | pressure_age_ms | u32, logging time minus pressure publication time |
| 24 | stat_age_ms | u32, logging time minus STAT reception time |
| 28 | target | i32, last commanded target at this logging instant |
| 32 | controller_ticks | u32, latest raw XD-C TIME, 0.1 ms units |
| 36 | controller_age_ms | u32, TIME reception age; UINT32_MAX means unavailable |

EPOS/STAT/TIME are asynchronous feedback, not a simultaneous measurement. The
pressure publication timestamp occurs after its temperature transaction, as in
the unchanged driver. Ages retain the available latency information; neither a
software timestamp nor TIME establishes sensor conversion time or clock alignment.
Pressure/position samples can repeat between sensor updates. The frontend plots
the paired logging times; CSV also exports all ages and raw encoder/target values.

The foreground performs at most one logging operation per poll. Late logging
skips expired slots rather than fabricating samples, recording missed-slot count
and maximum lateness. A sinusoidal dispatch delayed by a whole target interval
faults; smaller delays use current phase. Deadline checks and the prevalidated
sample budget bound storage, with a defensive invariant check before every write.
This is a bounded cooperative loop, not a measured hard-real-time guarantee.

The actual BSP memory split is 192 KiB ITCM, 320 KiB DTCM, and 512 KiB OCRAM.
The array is a static allocated in place, not a large temporary on the 16 KiB
stack. Existing USB DMA sections stay in DTCM, and no linker/cache/DMA policy was
changed. The BSP's 16 KiB OCRAM heap reservation remains unused by the firmware.
See the validation results below for the linked RAM footprint.

## Dataset protocol

Protocol 1 Dashboard commands remain compatible; HELLO adds `experiments: true`.
Maximum command length increases from 64 to 128 bytes before LF. IDs are nonzero
u32; the desktop reserves `0xf0000000` for its native transfer requests.

```text
START <id> Q <start_nm:i64> <step_nm:i64> <hold_ms:u32> <steps:u32>
START <id> S <center_nm:i64> <amplitude_nm:u32> <frequency_millihz:u32> <cycles:u32>
ABORT <id>
GET_DATA <id> <run>
GET_CHUNK <id> <run> <record_offset>
ACK_DATA_RECEIVED <id> <run> <count> <crc32_decimal>
```

START replies `experiment_ack` with run and action `start`. ABORT replies with
action `abort`; the separate retained dataset reports the outcome. All ordinary
commands, including PING, snapshots, stream changes and manual STOP, are busy
while the engine owns the hardware. ABORT is the exclusive running command.

On USB1 open, firmware sends one `experiment_state` announcement: `dashboard`,
`running` (also covers positioning/arming), or `data_ready`. The desktop waits
passively for that state, or a legacy Dashboard snapshot, before its handshake.
It never sends HELLO/PING into a recovered autonomous run. On completion, firmware
announces `data_ready` once per connection; this is not live telemetry.

The native receiver responds to data_ready with GET_DATA. `data_meta` JSON includes
version, run, requested integer-unit configuration, count, record size, payload
length, CRC, outcome, reason, STOP queue result, clock origin, elapsed time,
duration bound, reserved sample count, intervals, missed samples and lateness.

GET_CHUNK requests **16 records maximum** (640 binary bytes). Firmware sends one
JSON `data_chunk` header with id/run/record-offset/byte-length followed immediately
by exactly that many binary bytes. There is no newline or escaping after the
binary payload. The native decoder handles arbitrary fragmentation and embedded
LF/NUL/non-UTF8 bytes. Next request is sent only after the previous complete chunk.
There are no concurrent chunk streams or bulk operations in an ISR.

CRC-32/ISO-HDLC covers the concatenated encoded sample payload, initialized to
0xffffffff, reflected polynomial 0xedb88320, complemented on completion. JSON
metadata is validated separately; it is not included in the payload CRC.

Only after the full count, byte length, offsets, run and CRC match does native Rust
retain the decoded result and send ACK_DATA_RECEIVED. Firmware also requires all
records to have been transferred in sequence; a premature, wrong-run, wrong-count
or wrong-CRC acknowledgement cannot release ownership. Empty fault/abort datasets
still require metadata verification and ACK.

Disconnect, CRC mismatch or a 5 s transfer-response timeout does not discard the
Teensy copy. Reconnect and GET_DATA restart from offset zero. There is no automatic
retry of a START. A new run is refused until the old dataset is acknowledged.
After ACK, bytes remain intact until another START successfully passes validation;
invalid START does not clear them. The desktop keeps the verified result across
serial disconnect and page reload within the same process. Neither RAM copy is
persistent across power loss/process exit; select Save CSV for durable storage.
Save CSV opens a native Save As dialog for the destination folder and filename.
Desktop Rust writes the verified CSV after selection; cancelling or a file error
keeps the result available. The dialog releases the serial lock while open and
the write rechecks autonomous ownership before touching the file.

## Safety and PC disconnect policy

Autonomous operation explicitly ignores the PC heartbeat and USB1 DTR loss.
It continues on Teensy time and retains results until a PC returns. This requires
the Teensy to remain powered: disconnecting its only power source ends execution.
Manual Dashboard heartbeat/DTR stop behavior is unchanged, including the plunger.

ABORT immediately requests the existing nonblocking USB2 STOP and disables the
plunger. USB1 command reception remains serviced even when its IN reply is stalled,
so ABORT execution does not wait for a START acknowledgement to drain. A 500 ms
desktop ABORT response timeout reports uncertainty and advises reconnect/retry;
dropping DTR is deliberately **not** treated as an autonomous emergency stop.
The native START submission is ordered before any preempting ABORT; CSV and
configuration remain disabled after disconnect while ownership is unresolved.

During active states, Xeryon disconnect/epoch change, stale EPOS or STAT (3 s),
existing controller fault bits, end stops, invalid encoder/closed loop, index/scan
operations, move/trajectory/acquisition deadlines, transmit failure/backlog, or
pressure invalidity/transfer failure/250 ms staleness request STOP and retain a
faulted dataset. ABORT retains an aborted dataset. STOP delivery is best effort
when USB2 fails; queue acceptance is not physical stop confirmation. No software
path can guarantee actuator stopping after loss of its only communication link.

## Files and preservation review

New modules: `src/experiment.rs`, `src/experiment_data.rs`,
`desktop/src/experiment.rs`, `desktop/ui/experiment.js`, and
`desktop/tests/experiment_transfer.rs`.

Integration changes: firmware `main.rs`, `console.rs`, `protocol.rs`, `usb_host.rs`,
`xeryon_motion.rs`; desktop `main.rs`, `serial.rs`, `app.js`, `protocol.mjs`,
`index.html`, `style.css`; tests/smoke, protocol docs and package versions.
`c/usb_serial.c` / `.h` only add a read-only FIFO readiness query; existing send,
flush, clear, enumeration, RX and STOP logic are unchanged.

Untouched hardware modules: `pressure.rs`, `pressure_i2c.rs`, `motor.rs`,
`plunger.rs`, `xeryon.rs`, `line.rs`, `c/scanner.c`, USB configuration, all vendored
TinyUSB code, board pin/clock initialization, linker configuration, build script,
and upload scripts. Existing manual Xeryon commands, optional speed, position
feedback, pressure, plunger, USB identities and Dashboard controls remain intact.
The repository exposes index status but no PC INDX command; no existing command
was removed and no automatic index/homing operation was introduced.

The subsequent repository cleanup shares configuration construction, encoder
range constants, record decoding and chunk limits across firmware and desktop;
consolidates firmware error replies and desktop download completion/ownership
updates; and keeps one Tauri command registration list for normal and smoke builds.
Dashboard controls, legacy firmware compatibility, acquisition schedules, buffer
capacity, wire format and dataset acknowledgement semantics are preserved.
Save As adds a desktop-only dialog dependency; embedded dependencies are unchanged.

## Validation and physical acceptance

Native engine tests cover both trajectories, signed increments, feedback-driven
holds, clock rollover, full capacity reservation, invalid configuration, missed
samples, transport faults, pressure/USB2 faults, ABORT, manual heartbeat policy,
partial datasets, transfer restart and invalid/premature ACKs. Interoperability
tests send actual firmware metadata/records through the native binary decoder,
including corruption, arbitrary fragmentation, zero records and maximum payload.
Native/JavaScript tests cover command ownership, passive reconnect, Save As
selection/cancellation/failure, duplicate save clicks and ownership changes
while a save dialog is open.

Final validation on 2026-09-22 (Apple Silicon host):

| Check | Result |
| --- | --- |
| `bash tests/run.sh` | 29 firmware Rust tests and both C bridge suites passed |
| Desktop `cargo test --target aarch64-apple-darwin --locked` | 34 tests across native and firmware/receiver interoperability suites passed |
| `npm test` | 16 JavaScript protocol/session/UI tests passed |
| Firmware and desktop `cargo fmt --check` / `cargo check --locked` | Passed (desktop uses explicit native target) |
| Firmware `cargo build --release --locked` | Passed |
| `npm run build` | Vite assets and native macOS Tauri release executable built |
| Native Tauri `smoke-test` feature | Passed: Dashboard demo, native IPC, Save As command routing, tabs, both result canvases, outcome rendering, disconnect |
| `cargo objcopy --release --locked -- -O ihex target/firmware/firmware.hex` | HEX generated using the existing conversion pipeline |
| Full diff / whitespace review | Reviewed; `git diff --check` passed |

The existing vendored TinyUSB unused-function warning remains; there are no Rust
build errors. No Windows binary or physical hardware test was run. Save-path tests
inject a chooser; the native smoke test verifies the command rejects absent data.
Interactive OS dialog selection still needs a manual check on the target desktop.

Linked firmware measurements: sample array at `0x20005000`, length `0x28000`
(163,840 bytes); total `.bss` 186,368 bytes, `.data` 768 bytes, vector table
696 bytes, stack reservation 16,384 bytes. Including alignment gaps, DTCM ends
at `0x20032800`: **206,848 / 327,680 bytes**, leaving **120,832 bytes** unassigned.
`.text` is 74,716 bytes in ITCM; `.rodata` is 6,260 bytes in OCRAM. These are linker
sizes, not measured worst-case stack high-water marks.

Artifacts:

- HEX: `target/firmware/firmware.hex`
- Firmware ELF: `target/thumbv7em-none-eabihf/release/teensy-usb-scan`
- Desktop: `desktop/target/aarch64-apple-darwin/release/teensy-panel`
- HEX SHA-256: `76f40163b3be5909ed3d349e597b20d370e79135d6eda7ae3330479bdcff1165`

No firmware was flashed by these checks. Use the existing loader workflow only
after reviewing and deliberately approving a hardware trial.

Physical validation still required:

1. Recheck Dashboard movement, optional speed, plunger, pressure, both USB ports
   and existing index/reference setup with the new firmware.
2. Begin with small travel and amplitude. Measure 0.1–2 Hz EPOS tracking, phase
   lag, extrema, final center arrival, speed/acceleration limits, and mechanical
   limits under actual load. 50 Hz DPOS updates are an implementation choice to test.
3. Measure pressure/EPOS feedback cadence, publication/interface latency, loop
   lateness, missed samples and 160 KiB-buffer operation on the real board.
4. Exercise ABORT with USB1 backpressure, stale/disconnected USB2, sensor faults,
   initial/step/final-center timeout, and independently powered PC disconnect.
5. Interrupt downloads and verify retry, CRC and ACK on Windows COM6. Exercise
   Save CSV in the Windows WebView save/download UI; the macOS build is not a
   Windows executable or a physical Windows/Teensy communication test.
