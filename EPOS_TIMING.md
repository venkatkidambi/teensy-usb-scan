# EPOS acquisition investigation

## Baseline (inspected before behavior changes)

The path is controller encoder → controller ASCII feedback generation → USB CDC
bulk IN → TinyUSB event queue/RX FIFO → `usb_serial_task` →
`scanner_serial_rx`/`xeryon::Decoder` → `Telemetry.samples[0]` →
`Engine::poll` → binary record → desktop plot/CSV.

No firmware command sets or queries INFO or POLI, and no EPOS queries are sent.
Feedback is unsolicited broadcast using the controller's inherited settings.
Actual INFO, POLI and EPOS rate cannot be recovered from this source tree.
If POLI is 97 ms and EPOS occurs once per interval, the rate is 10.31 Hz;
if tags are scheduled separately it can be slower. This is a hypothesis to
measure, not an observed hardware rate.

Every completed ASCII line in a 64-byte read receives the same SysTick millisecond
timestamp, taken at entry to the Rust callback. At 20 Hz the logger copies the
latest cache without requesting a fresh value or checking a sample identity.
Repeated values may be either new stationary measurements or the same old sample.
`epos_age_ms` is time since foreground host receipt, NOT encoder measurement age.
The desktop plots both signals at record elapsed time, discarding their independent
ages for plotting. Pressure publication occurs after a fresh I2C burst completes.

USB2 is serviced on each cooperative loop before the experiment and USB1. TinyUSB
handles at most four events per call; C then reads only 64 bytes from a 1024-byte
RX FIFO. Bursts can remain queued for later iterations. Sine control sends at
most two short lines per 10 ms tick on the same controller; TX is nonblocking and
an incomplete previous TX faults the experiment. I2C is nonblocking. USB1 suppresses
streaming during ownership. There are no measurements proving transport latency
or starvation bounds, nor overflow counters exposed by the existing bridge.

Protocol reference: [official XD-C manual v3.1, sections 4.1–4.3](https://xeryon.com/download-files/Xeryon%20XD-C%20manual%20v3.1%20-%20JPL.pdf).
INFO selects broadcast fields; mode 2 includes EPOS, DPOS, STAT and requested
parameters. POLI is a millisecond update interval (default 97, range 1–65535).
TIME is described as a timestamp with 0.1 ms resolution. The manual does not
establish a shared acquisition instant with EPOS or synchronization to Teensy.
Consequently TIME cannot presently correct EPOS timing. The exact scheduling of
individual tags within an interval also needs measurement on this controller.

## Implemented behavior

`xeryon::EXPERIMENT_POLI_MS = 20` is the single interval configuration, initially
targeting approximately 50 fresh EPOS publications/s, subject to physical validation.
INFO remains the full mode 2 to preserve Dashboard fields, DPOS completion tracking,
and the requested-parameter slot needed by SSPD/TOU2 verification. Avoiding a mode
without that slot is deliberate. Neither a 1 ms minimum nor an unverified per-tag
50 Hz guarantee is assumed. If physical tag scheduling gives less than 50 Hz,
measure first, then adjust this constant and repeat the acceptance tests.

Before initial motion, a bounded cooperative setup queries INFO and POLI in order,
sets INFO=2 then POLI=20, and separately queries/verifies both settings. Commands
are separately flushed and wait for TX idle. Each step has a 1.5 s deadline;
inherited modes that omit parameter replies yield null original values. Set/query
failures or mismatches fault before the first motion command. Existing safety,
pressure and disconnect checks and ABORT handling continue throughout setup.
No SAVE is sent. Runtime INFO/POLI remain after the run for Dashboard use; no
motion parameters or sensor settings are changed by this setup.

EPOS remains broadcast, not explicitly queried. Each complete parsed EPOS line
gets a SysTick host-receipt timestamp and a wrapping u32 generation, including
unchanged values. Times are monotonic modulo wrap with 1 ms resolution: multiple
publications received within one tick legitimately share a timestamp. Generation
orders them; time is never artificially advanced. Ages use wrapping subtraction.
Neither USB transit delay nor controller-side age is observable from this timestamp.
Pressure keeps its own fresh-conversion counter and I2C burst-completion timestamp.
These are software observation times, not proven physical sampling instants.

There are no EPOS query/response pairs, hence no EPOS RTT or midpoint estimate.
A midpoint would assume approximately symmetric transport delay even with reliable
pairing; broadcast traffic cannot supply that pairing. TIME is still retained raw
with its own receipt age for investigation, but is never associated with EPOS or
used to synchronize clocks. No phase, drift, or mechanical-delay correction exists.

The C bridge now drains at most four 64-byte chunks per service call, then returns
to the cooperative scheduler. TinyUSB's four-event budget is unchanged. Foreground
WFI is skipped while RX data remains queued. Pressure, USB2, experiment control,
and USB1/ABORT retain their existing order. TX still faults on an incomplete previous
waveform command; no queue, thread or blocking wait was added. The RX FIFO is
non-overwriting; TinyUSB defers IN requests when space is insufficient. That can
move backlog into the controller and does not establish loss-free delivery.

## Records and diagnostics

Dataset v2 is 48 bytes (the original 40 plus EPOS and pressure u32 generations).
At 4096 records the buffer is 196608 bytes / 192 KiB, up 32768 bytes. Logging stays
20 Hz, duration bound stays 204.75 s including settling (194.75 s maximum sine
waveform plus its 10 s reserve). CRC covers all 48 bytes; chunks carry at most
16 records / 768 bytes. The native decoder explicitly requires v2. Dashboard's
ASCII/JSON protocol remains v1. Firmware remains no_std/no_main and allocation-free.

Independent relative signal times are `i64(elapsed_ms) - i64(signal_age_ms)`.
Negative times retain the last publication before run start, including across
SysTick rollover. CSV appends `epos_host_receipt_s`, both generations, and
`epos_new`/`pressure_new`. All rows and raw values remain available. The position
plot uses unique identities as points without interpolation and overlays commanded
position at the record's experiment time. Pressure retains every 20 Hz logger
sample as a continuous line against experiment elapsed time (`time_s` in CSV);
no pressure I²C completion time column is exported.

Arrival instrumentation starts at RUNNING and freezes on the terminal decision.
It does not stream to the desktop during the experiment. EPOS statistics are
accumulated in the parser so multiple publications within one loop count separately.
Pressure statistics observe every fresh publication in the foreground loop.

| Metadata under `timing` | Meaning |
| --- | --- |
| `epos`, `pressure` | `updates`, rate in millihertz (`updates * 1000000 / elapsed_ms`), interval count/sum, min/max ms and mean µs; no interval exists unless count ≥2 |
| `epos_repeated_records`, `pressure_repeated_records` | Logged identities equal to the preceding record; the first row is a baseline |
| `epos_unlogged_updates` | Publications during RUNNING not selected by the 20 Hz logger, including overwritten cache updates; deliberate decimation, not proven transport loss |
| `max_epos_age_ms`, `max_pressure_age_ms` | Maximum observed receipt/publication age at every running engine poll, including a triggering stale-feedback poll |
| `malformed_lines` | Decoder errors during RUNNING; not proof of dropped measurements |
| `rx_peak_bytes` | Maximum RX FIFO occupancy observed before a bounded drain; not an instantaneous hardware high-water mark |
| `rx_backlog_polls` | Service calls with bytes still pending after the 256-byte budget |
| `max_usb_service_gap_ms` | Largest gap between C foreground USB service calls during RUNNING |
| `max_tx_busy_ms` | Longest observed continuous pending-TX span, including waveform command contention; not encoder RTT |
| `rx_overflow`, `epos_source_drops`, `epos_rtt` | null: no reliable count or pairing is available |
| `before` | Original INFO/POLI readbacks, null when not obtained |
| `info`, `poli_ms`, `feedback_verified` | Requested settings; verified must be true before interpreting them as successful configuration |

No full-rate sample history is implied: 20 Hz records select the newest available
values. Received but unlogged EPOS updates remain visible in counters, not as
invented measurements. Source-side missing samples cannot be counted without a
controller sequence identifier. Existing missed logging slots and maximum logging
lateness remain separate diagnostics.

## Required physical acceptance (not run; no flashing performed)

1. With the prior configuration, measure actual INFO/POLI and EPOS inter-arrival
   times. Do not identify 97 ms or 10.31 Hz as measured merely because they are defaults.
2. After separately authorized flashing, confirm fresh matching INFO=2/POLI=20
   readbacks, DPOS completion, Dashboard fields, SSPD/TOU2 checks and ABORT during
   every setup step. Verify settings remain runtime-only through a power cycle.
3. Run stationary and 0.1/1/2 Hz sine trials. Require approximately 50–100 Hz fresh
   EPOS arrivals, inspect min/mean/max intervals, repeated records and receipt ages.
   If tags alternate per POLI rather than per group, record the observed rate and
   revise the interval only after checking bandwidth and the following counters.
4. Compare USB service gap, RX peak/backlog, malformed lines and TX busy span with
   and without 100 Hz waveform traffic. Require no growing backlog, no malformed
   lines and no deadline/TX faults. Look for same-tick bursts indicating queued data.
   Host receipt age alone cannot rule out controller-side buffering.
5. Confirm pressure publication rate and intervals remain near the existing 200 Hz
   path and ages remain healthy. Verify independent CSV times and stationary EPOS
   identities, including negative pre-run timestamps and repeated-cache rows.
6. Exercise unplug, pressure failure, stale feedback, abort, terminal return,
   transfer retry/CRC and maximum-duration retention. Inspect timing diagnostics
   on faulted and aborted as well as completed runs.
7. Use an external shared physical reference/logic capture if absolute encoder
   acquisition timing or physical pressure lead must be established. TIME's exact
   relationship to encoder latching and clock synchronization still needs vendor
   clarification or direct measurement. No physical phase conclusion is warranted.

## Software validation on 2026-09-29

- `bash tests/run.sh`: both C suites and 52 firmware Rust tests passed.
- Native desktop Cargo tests: 62 tests across two suites passed, including
  firmware/desktop binary transfer, CRC and independent-time CSV interoperability.
- `npm test`: 17 UI/protocol tests passed.
- Firmware and desktop `cargo fmt --check`, and `git diff --check`: passed.
- Firmware release build and `npm run build` (Vite plus native Tauri release): passed.
- Native macOS smoke test: passed IPC, demo, disconnect, START click and validation;
  it opens no hardware serial port.
- `cargo objcopy --release --locked -- -O ihex target/firmware/firmware.hex`: passed.
  All 7719 Intel HEX records, their checksums and EOF were validated (347195 bytes).
  SHA-256: `5140fcb9b27e2c5ff669cc5fa80e4ed2be5160d6a38598b31000b41bab6367c5`.

Linked `.bss` is 223232 bytes; `.data` 960, `.text` 98152, `.rodata` 15416.
DTCM ends at `0x2003b800`, leaving 83968 bytes within its 320 KiB allocation,
after the existing 16 KiB stack reservation and linker alignment. The fixed buffer
occupies 196608 bytes of `.bss`. The unused 16 KiB OCRAM heap reservation is unchanged.
The release compiler still emits the pre-existing vendored TinyUSB unused-function
warning for `cdch_process_set_config`.

Coverage includes same-value/new vs cached identity, same-tick bursts, generation
and clock rollover, age reconstruction, setup serialization/readback failures,
speed verification after preparation, bounded RX draining/backlog/TX statistics,
metadata frame capacity, maximum dataset transfer, version rejection, and preserved
experiment safety/motion tests. RTT/midpoint tests are inapplicable to broadcast
EPOS; there is no such estimator in this implementation. No hardware was flashed.
