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
- LPS22HHTR pressure uses nonblocking 400 kHz I²C on pins 18/19, address 0x5C,
  200 Hz sensor output, 1 ms data-ready checks and a five-byte repeated-start
  pressure/temperature burst. Transactions have a 20 ms deadline; missing fresh
  conversions for 100 ms invalidates the sensor. The 16-sample average remains
  for the dashboard; experiments store the latest raw pressure at 20 Hz.
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
identity, feedback freshness, healthy pressure, controller/encoder errors, and
stopped manual motion **before the first DPOS**. It does not home, find index,
change limits or change the encoder reference. Targets use the current EPOS
coordinate system; STAT bit 8 (index reference known) is not required. The initial
DPOS enters closed-loop positioning, which is required before acquisition begins.

For quasi-static, the required **Actuator speed (µm/s)** defaults to 10 in the UI;
the wire/configuration value is an integer from 1 to 16777215. Firmware advertises
`quasi_static_speed: true`; the UI requires this capability, and START Q without
a speed is rejected. The speed is retained as `speed_um_s` in dataset/CSV metadata.

Before any DPOS, firmware sends `SSPD=<speed>` and `SSPD=?` in separate ordered
USB transfers. It waits for the transfers to finish and a new matching readback,
then queries `TOU2=?`. Each readback has a 3 s deadline. Missing/mismatched speed,
invalid/missing TOU2, or transmit failure requests STOP and retains a faulted
dataset without moving. Cached replies from before setup do not count.
An enabled TOU2 must exceed the longest reserved movement time (start or step);
otherwise the run faults before moving with advice to increase speed or reduce
travel. TOU2=0 is accepted as already disabled; firmware never changes it.
These commands and units follow the [XD-C manual, sections 4.1–4.2](https://xeryon.com/download-files/Xeryon%20XD-C%20manual%20v3.1%20-%20JPL.pdf).

The confirmed speed applies to initial positioning and every step. It remains
the controller's current setting afterward, including for subsequent manual or
sinusoidal moves that do not select a speed; no SAVE command is sent. Start
positioning completes first, then acquisition begins.
Step 1 is `start + increment`; there are exactly `steps` increments, each followed
by a hold. Holds begin only on confirmed arrival. Samples continue through every
move and hold. Each movement deadline is `max(10000, 2 * ceil(distance_nm /
speed_um_s) + 1000)` milliseconds, capped at 204.75 s. Steps reserve an extra
78 nm for endpoint rounding; the start move uses the actual encoder distance.
The engine enforces these deadlines instead of the manual motion tracker's
60 s deadline. TOU3, ELIM, acceleration and travel protections remain unchanged;
the speed/readback checks cannot guarantee fault-free motion under every load.

For sinusoidal, initial positioning is to center. Phase is calculated from actual
elapsed Teensy milliseconds, not from loop counts. After the requested cycles,
an exact center target is sent and confirmed through the same finite-move feedback
mechanism. Acquisition continues through final settling, bounded by 10 s.
All terminal paths request STOP; metadata says whether it could be queued.

## Xeryon waveform strategy: 100 Hz position commands

After confirmed initial positioning, the Teensy sends one bounded
`DPOS=<signed encoder counts>\n` command every 10 ms. The target is
`center + amplitude * sin(2π * frequency * elapsed_seconds)`, rounded directly
to the nearest 0.078 µm encoder count. Phase comes from elapsed Teensy time,
including clock rollover, so foreground jitter cannot accumulate phase drift.
The 0.1–2 Hz configured wave frequency is separate from the 100 Hz command rate.

The foreground does not wait for each waveform target to be reached. It sends
one command per scheduled tick, including repeated quantized targets. Delays of
an entire 10 ms interval or an unfinished previous USB transfer fault the run;
there are no queued catch-up bursts. Final center arrival is confirmed before
completion. Acquisition continues at 20 Hz through final settling.

No SCAN velocity commands, encoder correction, controller-setting queries or
temporary setting changes are used. Existing controller speed and travel limits
apply. USB command timing does not guarantee mechanical tracking or feedback
cadence; those must be measured on hardware.

## Timing, sample layout and memory

| Quantity | Value |
| --- | --- |
| Clock | Existing monotonic Teensy SysTick, 1 ms resolution, wrapping u32 |
| Acquisition schedule | 50 ms / 20 Hz, latest cached raw values |
| Pressure acquisition | LPS22HHTR 200 Hz output; 1 ms readiness polling; burst read at 400 kHz |
| Sinusoidal targets | 10 ms / 100 Hz, 0.100–2.000 Hz wave frequency |
| Record | 40 bytes, ten explicit little-endian 32-bit words |
| Capacity | 4,096 records |
| Sample-buffer RAM | 163,840 bytes / 160 KiB, zero-initialized DTCM `.bss` |
| Maximum acquisition bound | 204,750 ms (204.75 s), including final settling |
| Maximum sinusoidal waveform duration | 194,750 ms, plus reserved 10 s settling |
| Initial positioning | Before acquisition: speed-based budget for quasi-static (at most 204.75 s); 10 s for sinusoidal |

Validation reserves `floor(duration_bound_ms / 50) + 1` records including t=0.
Quasi-static bound is `steps * (step_move_timeout_ms + hold_ms)`; sinusoidal bound is
`ceil(cycles * 1_000_000 / frequency_millihz) + 10_000`. Oversized runs are rejected,
never silently shortened. A 204.75 s bound reserves all 4,096 records.

| Offset | Field | Type / interpretation |
| --- | --- | --- |
| 0 | elapsed_ms | u32, actual logging time relative to acquisition start |
| 4 | epos | i32, raw encoder counts |
| 8 | pressure_raw | i32, sign-extended 24-bit LPS22HHTR counts (4096 counts/hPa) |
| 12 | stat | u32, raw controller status |
| 16 | epos_age_ms | u32, logging time minus EPOS reception time |
| 20 | pressure_age_ms | u32, logging time minus pressure publication time |
| 24 | stat_age_ms | u32, logging time minus STAT reception time |
| 28 | target | i32, last commanded target at this logging instant |
| 32 | controller_ticks | u32, latest raw XD-C TIME, 0.1 ms units |
| 36 | controller_age_ms | u32, TIME reception age; UINT32_MAX means unavailable |

EPOS/STAT/TIME are asynchronous feedback, not a simultaneous measurement. The
pressure publication timestamp occurs after the combined pressure/temperature burst
completes. Only a fresh conversion advances that timestamp. Ages retain the available latency information; neither a
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

Protocol 1 Dashboard commands remain compatible; HELLO adds `experiments: true` and `sine_position: true`.
Maximum command length increases from 64 to 128 bytes before LF. IDs are nonzero
u32; the desktop reserves `0xf0000000` for its native transfer requests.

```text
START <id> Q <start_nm:i64> <step_nm:i64> <hold_ms:u32> <steps:u32> <speed_um_s:u32>
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
existing controller fault bits (including encoder error), end stops, loss of closed
loop after initial positioning, index/scan
operations, move/trajectory/acquisition deadlines, transmit failure/backlog, or
pressure invalidity/transfer failure/250 ms staleness request STOP and retain a
faulted dataset. ABORT retains an aborted dataset. STOP delivery is best effort
when USB2 fails; queue acceptance is not physical stop confirmation. No software
path can guarantee actuator stopping after loss of its only communication link.

## Validation

The focused engine coverage checks 100 position commands per second, absolute
sine phase across clock rollover, final center completion, faults and ABORT.
Existing transport, configuration, quasi-static, dataset and desktop tests remain.
The scan setup/readback/restoration tests and diagnostic fixtures were removed
with the velocity implementation.

Run `bash tests/run.sh`, desktop Cargo tests with the native host target,
`npm test`, the firmware release build and `npm run build:ui` for software checks.
Physical 100 Hz USB dispatch and stage tracking still require a hardware trial.
