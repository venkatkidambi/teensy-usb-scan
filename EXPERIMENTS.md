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
sine mode adds velocity command sequencing, controller setup and restoration.

```text
Dashboard -> Validate -> MoveToStart -> Armed -> Running
                        |               |         |
                        +---------------+---------+-> Complete / Aborted / Fault
                                                        -> Restoring (sine settings)
                                                        -> DataReady
                                                        -> Transferring
                                                        -> Dashboard (verified ACK)
```

Validation and terminal outcomes are synchronous transitions; Armed performs
scan setup for sine runs, and Restoring performs bounded cleanup. Any active state can abort/fault. The engine's
terminal outcome and reason remain in metadata after entering DataReady.

START checks the complete configuration, arithmetic overflow, rounded signed
26-bit positions, worst-case duration, sample count, available storage, hardware
identity, feedback freshness, healthy pressure, closed loop, valid encoder, and
stopped manual motion **before the first DPOS**. It does not home, find index,
or change the encoder reference. Sine setup later narrows controller limits as
described below. Existing index/reference and mechanical setup must be correct.

For quasi-static, start positioning completes first, then acquisition begins.
Step 1 is `start + increment`; there are exactly `steps` increments, each followed
by a hold. Holds begin only on confirmed arrival. Samples continue through every
move and hold. Each move has a 10 s deadline, including initial positioning.

For sinusoidal, initial positioning is to center. Phase is calculated from actual
elapsed Teensy milliseconds, not from loop counts. After the requested cycles,
an exact center target is sent and confirmed through the same finite-move feedback
mechanism. Acquisition continues through final settling, bounded by 10 s.
Terminal paths request STOP; scan settings restoration precedes DataReady.
Metadata says whether STOP could be queued and restoration writes were sent.

## Xeryon waveform strategy: 100 Hz speed control

Sinusoidal runs now use `SSPD` speed magnitude and `SCAN` direction with a
bounded proportional encoder correction. Initial positioning and final return
to center still use `DPOS`. The command meanings and units come from the
[XD-C manual, sections 4.2–4.3](https://xeryon.com/download-files/Xeryon%20XD-C%20manual%20v3.1%20-%20JPL.pdf).

Every 10 ms, using actual elapsed Teensy time:

```text
x_ref(t) = center + A sin(2πft)                   [µm]
v_ff(t)  = 2πfA cos(2πft)                         [µm/s]
e        = x_ref(EPOS reception time) - EPOS       [µm]
v_cmd    = v_ff + clamp(2e, -0.25 v_peak, +0.25 v_peak)
```

The initial gain is **2/s**, with a one-encoder-count (0.078 µm) correction
deadband and no integral term. The correction is capped at 25% of peak sine
speed. These are conservative starting parameters, not hardware-tuned gains.
The reference is evaluated at the cached encoder's reception time when computing
error, avoiding artificial error from comparing an old sample with a later sine
phase. Reception time is not the sensor's measurement time; remaining latency
and mechanical dynamics must be measured. A reused sample does not accumulate
integral correction. EPOS and STAT must both be less than 50 ms old at an update.

Speed is rounded to whole µm/s. An unchanged speed/direction sends nothing.
Zero speed explicitly uses `SCAN=0`, never `SSPD=0`. A reversal sequences
`SCAN=0`, changed `SSPD`, then the new `SCAN` direction, each in a separate USB
transfer. No next line is submitted until the previous transfer completes.
A pending sequence has a 10 ms deadline. A whole missed control interval faults;
smaller delays use current phase, never catch-up bursts. This is a cooperative
100 Hz schedule, not a guarantee of controller execution or physical bandwidth.
For small slow waves, 1 µm/s quantization can dominate or round the entire command
to zero; e.g. a 0.078 µm, 0.1 Hz wave is below the speed resolution.
The finite wave still starts at center with nonzero desired velocity and stops
after whole cycles; no extra ramp-in or ramp-out cycles are inserted.

### Setup and controller-side bounds

After confirmed initial positioning, before acquisition or scanning, the engine
sets INFO=0 before reading the original SSPD, POLI, LLIM and HLIM settings.
It does not query the previous INFO mode: modes 3, 4 and 7 have no requested
parameter slot and can leave that query unanswered. On exit it sets INFO=4
(EPOS, STAT, DPOS, TIME), including after partial setup failure; the previous
telemetry mode is not preserved. No settings are saved to controller flash.
Replies must be new publications after each query;
cached values cannot satisfy setup. Each operation has a 1.5 s deadline, and
setup has a 5 s total deadline (ordinary feedback health checks also apply).
Timeouts identify the setting and distinguish reads, verification readbacks,
and an unfinished USB transmission. Each command is sent as a separate USB
transfer. Setup waits 100 ms after observing completion of INFO=0, and 20 ms
after each other command, before advancing. Restoration writes also wait 20 ms
after transfer completion. These are conservative host delays, not measured
controller execution times; USB completion alone does not establish execution.
Unanswered setting reads/readbacks are retried at 250 ms intervals, at most
three transmissions total, without extending the original 1.5 s deadline or
accepting a cached value. Waveform commands retain their existing 100 Hz timing.

The controller travel envelope is center ± (amplitude + guard), where guard is
max(amplitude/4, 1.560 µm), rounded to encoder counts. It must fit the signed
encoder range and lie entirely inside the original LLIM/HLIM limits. The engine
never widens pre-existing limits during setup. It writes and reads back both
narrowed limits before permitting SCAN. The original SSPD setting must be at least
1.25 × requested peak velocity; an inadequate setting is rejected instead of
silently exceeding it. The engine does not home or change the encoder reference.

POLI is temporarily set to 2 ms and read back; INFO=4 enables EPOS, DPOS, STAT
and TIME feedback. Before INFO=4, setup also writes and reads back the rounded
phase-zero speed (2π × amplitude × frequency), preserving the original SSPD for
restoration. Missing or mismatched readback prevents scanning. The first control
tick uses that verified nominal speed; bounded position correction begins on the
next 10 ms tick. Feedback, tracking and travel checks still run on the first tick.
If the nominal speed rounds to zero, setup verifies SSPD=1 and the first tick
remains stopped; it never sends SSPD=0. Later waveform speed changes still use
USB completion without readback. This closes an unverified startup transition;
it does not establish the cause of the observed backward encoder response.
Before starting the wave, the engine requires fresh feedback
and two distinct EPOS reception timestamps. This requests frequent feedback but
does not assume the XD-C produces simultaneous 100 Hz measurements. Software
faults on excessive following error (greater than the guard), escape from the
controller envelope, stale feedback, index operations, limit/fault bits,
transport failure or missed deadlines. Scanning status is permitted only while
the engine owns sine scanning. Stopped SCAN/STOP transitions may temporarily
clear the closed-loop bit; an active scan still must report closed-loop control.

Tracking and travel faults have distinct reasons: `Scan tracking error exceeded
guard` and `Scan position reached travel bound`. Their result metadata includes
`scan_fault`, captured at the failing 10 ms control tick, even between acquisition
samples. It preserves elapsed time, EPOS counts/µm and reception age, STAT, the
reference at the encoder reception time, signed reference-minus-position error,
guard, and travel bounds in counts. Cleanup does not overwrite it. The desktop
shows these values in the result summary and retains them in CSV metadata.
This diagnostic change does not increase the allowed error or travel envelope.

The earlier 250 Hz DPOS build caused a reported freeze after actuator movement,
with the foreground LED stopping. Its root cause remains unresolved. The new
controller-side limits remain installed if Teensy execution stops, but they are
not an emergency stop and do not guarantee zero overshoot or protection after
encoder/controller failure. Independent stop access is still necessary for a
physical trial; the new control mode is not a demonstrated fix for the freeze.

### Stop, center and restore

At waveform completion, STOP cancels scanning. The original SSPD is restored
before issuing the final center DPOS, so a near-zero waveform speed cannot
strand the finite move. Center arrival still needs DPOS echo followed by a new
position-reached status with scanning off. ABORT and faults immediately request
STOP and cancel any pending speed/direction sequence.

All exits restore changed controller settings through a bounded foreground
Restoring state. After scanning has been requested, this first waits for USB TX
completion and a fresh, post-STOP status with scanning off and motor off or
position reached, before restoring the wider original limits. Restoration has
a 2 s deadline, keeps ABORT responsive, and does not add acquisition samples.
Repeated ABORT retries STOP without restarting the restoration deadline.
If cleanup fails, `settings_restore_error` preserves its reason separately from
the original run fault and appears in the desktop result summary. Restore
timeouts distinguish unfinished USB transmission, missing fresh post-STOP
status, scanning still reported, and motor-on without position-reached. These
diagnostics do not relax the conditions for restoring wider travel limits.
Disconnect, changed connection epoch, or failed restoration retains a dataset
with `settings_restore_sent=false`. Remaining narrow limits may stay installed;
check controller limits, speed and telemetry settings before further motion.
No SAVE command is issued. `settings_restore_sent=true` means the restore writes
completed USB transfer, not an independent readback of every restored setting.

The retained C transport checks both software FIFO emptiness and in-flight USB
completion. STOP clears unsent bytes but cannot recall an already submitted
packet. The host task processes at most four USB events per foreground pass;
this bounds event count, not individual callback or ISR duration.

## Timing, sample layout and memory

| Quantity | Value |
| --- | --- |
| Clock | Existing monotonic Teensy SysTick, 1 ms resolution, wrapping u32 |
| Acquisition schedule | 50 ms / 20 Hz, latest cached raw values |
| Pressure acquisition | Existing 50 ms attempt interval, completion latency unchanged |
| Sinusoidal velocity control | 10 ms / 100 Hz; 0.100–2.000 Hz wave frequency |
| Record | 40 bytes, ten explicit little-endian 32-bit words |
| Capacity | 4,096 records |
| Sample-buffer RAM | 163,840 bytes / 160 KiB, zero-initialized DTCM `.bss` |
| Maximum acquisition bound | 204,750 ms (204.75 s), including final settling |
| Maximum sinusoidal waveform duration | 194,750 ms, plus reserved 10 s settling |
| Initial positioning | Additional 10 s maximum, before acquisition |
| Scan setup / terminal restoration | Up to 5 s / 2 s, outside acquisition |

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
| 28 | target | i32, sine reference at logging time, or last quasi-static DPOS target |
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
and maximum lateness (these counters describe acquisition, not USB delivery).
A sinusoidal update delayed by a whole control interval (10 ms) faults, even if
the speed would be unchanged; smaller delays use current phase. Scheduling stays
anchored to acquisition start, without phase drift or catch-up bursts.
Deadline checks and the prevalidated
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
`running` (also covers positioning/arming/restoration), or `data_ready`. The desktop waits
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
existing controller fault bits, end stops, invalid encoder/closed loop, index
operations, unexpected scanning outside the sine mode, move/trajectory/acquisition deadlines, transmit failure/backlog, or
pressure invalidity/transfer failure/250 ms staleness request STOP and retain a
faulted dataset. ABORT retains an aborted dataset. STOP delivery is best effort
when USB2 fails; queue acceptance is not physical stop confirmation. No software
path can guarantee actuator stopping after loss of its only communication link.

## Files and preservation review

New modules: `src/experiment.rs`, `src/experiment_data.rs`, `src/sine_control.rs`,
`desktop/src/experiment.rs`, `desktop/ui/experiment.js`, and
`desktop/tests/experiment_transfer.rs`.

Integration changes: firmware `main.rs`, `console.rs`, `protocol.rs`, `usb_host.rs`,
`xeryon_motion.rs`; desktop `main.rs`, `serial.rs`, `app.js`, `protocol.mjs`,
`index.html`, `style.css`; tests/smoke, protocol docs and package versions.
`c/usb_serial.c` / `.h` now track pending TX completion as well as FIFO readiness;
STOP still clears unsent bytes. USB host event budgeting is configured in
`c/tusb_config.h`.

Untouched hardware modules: `pressure.rs`, `pressure_i2c.rs`, `motor.rs`,
`plunger.rs`, `xeryon.rs`, `line.rs`, `c/scanner.c`, all vendored
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

Current 100 Hz scan build: firmware engine and transport tests cover velocity
cadence, correction sign/cap, delayed feedback, stale samples, query readbacks,
controller limit preservation, STOP/reversal ordering, command stalls, Abort
mid-sequence, final-center arrival and settings restoration. A biased velocity
model checks that proportional feedback reduces position error; it is not a
measurement or model identification of the actual stage. Desktop interoperability
checks the unchanged binary format and the added control metadata. UI tests cover
capability gating and incomplete restoration messaging. No hardware trial has
been performed, and the earlier freeze remains unresolved.
Validation passed: 46 firmware Rust tests, both C bridge suites, 51 desktop/native
tests, and 18 JavaScript tests. Firmware release/HEX and Vite production assets
build successfully. The pre-existing TinyUSB unused-function warning remains.

Recovery build validation: 34 firmware Rust tests, both C bridge suites,
39 desktop/native interoperability tests and 17 JavaScript tests passed. Release
firmware/HEX and Vite assets rebuilt successfully. The C transport mock now models
FIFO drain separately from transfer completion, including queued manual commands,
missing completions, STOP replacement and disconnected callbacks. No physical
test or flash was performed; these tests do not establish the freeze's cause or
prove its resolution. The existing TinyUSB unused-function warning remains.

Earlier 250 Hz trajectory update validation (2026-09-23; subsequently withdrawn): 34 firmware Rust tests, both C
bridge suites, 39 desktop/native interoperability tests, and 17 JavaScript tests
passed. Firmware release build, HEX conversion and Vite production build passed.
New coverage checks 2 Hz command cadence, nearest-count accuracy across signed
travel, duplicate suppression, final-center confirmation, clock rollover, jitter,
deadline faults and failed sends. The existing TinyUSB unused-function warning
remains. No firmware was flashed and physical smoothness has not been measured.
The measurements and artifact hash below describe the earlier 2026-09-22 build.

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
   limits under actual load. Confirm LED/USB1 responsiveness, limit readbacks, speed
   update timing, EPOS age, bounded encoder correction, zero-speed transitions and
   settings restoration. The retained 20 Hz dataset cannot characterize 100 Hz
   command delivery or a CPU stall; use an independent trace for those checks.
3. Measure pressure/EPOS feedback cadence, publication/interface latency, loop
   lateness, missed samples and 160 KiB-buffer operation on the real board.
4. Exercise ABORT with USB1 backpressure, stale/disconnected USB2, sensor faults,
   initial/step/final-center timeout, and independently powered PC disconnect.
5. Interrupt downloads and verify retry, CRC and ACK on Windows COM6. Exercise
   Save CSV in the Windows WebView save/download UI; the macOS build is not a
   Windows executable or a physical Windows/Teensy communication test.
