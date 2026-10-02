# PC ↔ Teensy protocol v1

Firmware 0.2.0 advertises `experiments: true`, START/ABORT and a retained binary dataset
extension. See [Dataset transfer](#dataset-transfer) for framing, raw record layout,
CRC, ownership, passive reconnect and ACK lifecycle.
All Dashboard commands below remain available when no experiment owns hardware.

Current firmware also advertises `quasi_static_speed: true`. Quasi-static START
requires `START <id> Q <start_nm> <step_nm> <hold_ms> <steps> <speed_um_s>` with
integer speed 1–16777215 µm/s; missing, fractional, zero or out-of-range speeds
are rejected. The desktop requires this capability before offering START Q.
Sinusoidal START keeps its existing arguments. Quasi-static metadata includes
`speed_um_s`. START acknowledges acceptance; controller speed/TOU2 readback
checks happen before motion and report any failure in the retained dataset.

Experiment preparation queries inherited INFO/POLI, sets INFO=2 and POLI=20 ms,
and requires fresh matching readbacks before motion. Each setup step has a
1.5 s deadline; unavailable original settings are reported as null. Runtime
settings remain after the run; no SAVE command writes controller nonvolatile memory.
Quasi-static SSPD/TOU2 readbacks have three-second deadlines. Firmware checks
TOU2 against the reserved movement budget and does not change that timeout.

Experiment preparation leaves PRO2 unchanged. The earlier gain-30 trial has been
removed; current firmware no longer emits `data_meta.scan_gain`. Older trial
datasets may contain `cap`, `before`, `applied` (null if unverified), and
`saved: false` in that object; the desktop still displays those recorded values.
The old trial's volatile gain survives a Teensy firmware update. After updating,
power-cycle/reset the Xeryon controller to reload saved settings, or explicitly
restore its pre-trial PRO2. The trial did not issue SAVE.

USB1: CDC ACM `5824:27dd`, 115200 8-N-1, no flow control, DTR asserted.
Baud rate is a CDC setting, not USB throughput. The desktop opens the port natively.
The actuator is on separate USB2 (`04d8:ef4a`).

Requests are ASCII lines terminated by LF, maximum 128 bytes before LF (64 on firmware before 0.2.0). CRLF also
works. Each request uses a nonzero u32 ID; only one request is in flight at a time.

| Request | Response | Effect |
| --- | --- | --- |
| `HELLO 1` | `hello`, id 1 | Protocol version, firmware version, board, capabilities |
| `PING 2` | `pong`, id 2 | Teensy uptime; desktop measures round-trip time |
| `SNAPSHOT 3` | `snapshot`, id 3 | Current cached measurements |
| `STREAM 4 0` | `ack`, id 4, stream false | Pause periodic USB1 snapshots |
| `STREAM 5 1` | `ack`, id 5, stream true | Resume snapshots |
| `PLUNGER_MOVE 6 U 64 10` | `plunger_ack`, action `move` | Relative up travel of 10 µm, PWM 64 |
| `PLUNGER_MOVE 7 D 64 10` | `plunger_ack`, action `move` | Relative down travel |
| `PLUNGER_STOP 8` | `plunger_ack`, action `stop` | Disable plunger motor |
| `PLUNGER_LIMITS 10 30 75` | `plunger_ack`, action `limits` | Set minimum/maximum potentiometer percentage while stopped; integers 0 ≤ min < max ≤ 100 |
| `PLUNGER_ZERO 9` | `plunger_ack`, action `zero` | Zero encoder while stopped |
| `XERYON_MOVE 10 U 10000 10000` | `xeryon_ack`, action `move`, target counts | Move Xeryon up 10 µm at 10000 µm/s (wire distance in nm) |
| `XERYON_MOVE 11 D 10000 10000` | `xeryon_ack`, action `move`, target counts | Move Xeryon down 10 µm at 10000 µm/s |
| `XERYON_STOP 12` | `xeryon_ack`, action `stop` | Queue Xeryon STOP |

Responses are newline-delimited JSON. Example:

```json
{"type":"hello","id":1,"protocol":1,"firmware":"0.1.0","board":"Teensy 4.1","motion":true,"pressure":true}
{"type":"snapshot","id":0,"ms":12500,"epoch":1,"vid":1240,"pid":61258,"mounted":true,"cdc":2,"records":30,"malformed":0,"skipped":0,"values":{"EPOS":{"value":-123,"ms":12450},"STAT":{"value":1344,"ms":12480}}}
```

- ID 0 snapshots are unsolicited, normally 10 Hz. `#` lines are best-effort logs.
- `epoch` changes when actuator cache resets; the UI clears history. `ms` is Teensy
  uptime in milliseconds. Unsigned subtraction handles 32-bit clock rollover.
- `vid`/`pid` are decimal USB2 IDs. `mounted` reports configuration, not freshness.
- `cdc`: 0 closed, 1 opening, 2 open, 3 failed, 4 timed out.
- `records` / `malformed` count decoded / rejected actuator lines. `skipped` counts
  snapshot opportunities coalesced by USB1 backpressure, not lost actuator samples.
- Missing samples are omitted. Retained tags: EPOS, DPOS, STAT, SRNO, SOFT, FREQ,
  TIME, SYNC. Each sample has its own `value` and Teensy reception `ms` timestamp.
- EPOS/DPOS are raw encoder counts (the desktop displays µm using 0.078 µm/count), FREQ is Hz, STAT is a 24-bit word, and TIME is controller
  time in 0.1 ms ticks. SOFT 20103 means firmware 2.1.3.

The UI marks samples older than three seconds stale. The desktop worker also
coalesces unsolicited snapshots while the UI is slow; its separate `coalesced`
counter is exposed by native IPC. All retained lines preserve reception order.
These are monitoring snapshots, not a lossless acquisition stream.

Unsupported input is rejected by both the native command allowlist and firmware.
The firmware sends `{"type":"error","id":0,"message":"invalid command"}` for
malformed commands after LF. Only the listed commands are accepted; arbitrary Xeryon commands are rejected. Pausing display updates never pauses actuator reception.

Xeryon-enabled firmware adds `xeryon: true` to HELLO; firmware 0.1.6+ also
adds `xeryon_speed: true` for speed-controlled moves. Firmware 0.1.7 adds
`xeryon_optional_speed: true` for moves that retain the controller's speed.
For older Xeryon firmware,
the UI sends the original `XERYON_MOVE <id> <U|D> <nm>` command and disables the
speed input. The controller keeps its existing speed setting in this mode.
With `xeryon_speed: true`, the UI includes integer speed 1–16777215 **µm/s**
after integer distance 78–2617245618 **nm** only when a speed is entered.
Leaving it blank sends the original command and preserves the controller's
current speed setting on firmware 0.1.7+. Firmware 0.1.6 requires an entered
speed; the UI reports this before a move. The desktop accepts both command shapes.
The UI converts µm to nm, allowing three decimal places. Firmware rounds travel
to counts as `(nm + 39) / 78`, then adds/subtracts from the latest EPOS. It checks
signed 26-bit EPOS/target bounds [-33554432, 33554431], known USB identity,
ready CDC, fresh EPOS/STAT (<3 seconds), motion state and controller faults.
With an entered speed (firmware 0.1.8+), it flushes `SSPD=<speed>\n` and then queues
`DPOS=<target>\n` as separate USB transfers. Without one, it queues only
`DPOS=<target>\n`. Limits, homing and tuning are unchanged. Settings
are not saved to controller flash. The existing 60-second move deadline also
applies to slow moves.

`xeryon_ack` means a whole command was queued into the nonblocking USB2 transmit
buffer; it is not an acknowledgement from the XD-C. `target` is encoder counts.
Snapshots add `xeryon_motion: {state, target}`; target is counts or null.
States: 0 idle, 1 awaiting target/reached feedback, 2 target reached,
3 Stop requested, 4 move interrupted. Reached requires fresh matching DPOS
received after the command, followed by STAT with bit 10 set. Overlapping moves
are rejected. Stop discards unsent queued bytes, but cannot recall an in-flight
USB packet. USB2 disconnection resets motion state and never replays commands.

STOP can retain scanning mode (STAT bit 13) with Motor on (bit 5) clear, as in
`STAT=8513`. Manual moves and experiment starts permit this stopped state.
Scanning with Motor on remains busy even if Position Reached is set; indexing,
pending tracked moves, stale feedback, and controller faults still block starts.

An active move requests best-effort STOP on USB1 disconnect, 3-second stale
EPOS/STAT, controller faults, 5-second heartbeat timeout or 60-second deadline.
Physical stop cannot be guaranteed if USB2 is unavailable. Desktop Stop preempts
pending requests and closes USB1 if unacknowledged after 500 ms. Pausing snapshots
does not pause firmware motion monitoring; periodic PING keeps its heartbeat alive.

Plunger-enabled firmware reports `motion: true`. Snapshots include an additive
`plunger` object: signed 32-bit `counts`, `counts_per_mm: 16688`, boolean `running`
and `up`, PWM `speed` (0–255), and 12-bit `pot` (or null when unavailable).
The desktop converts plunger counts to µm independently of Xeryon telemetry.
Moves require PWM 1–255 and integer travel 1–128684302 µm; firmware rejects moves
while running and rejects zeroing while running. A move acknowledgment means
accepted, not completed: `running` in subsequent snapshots reports progress.
Motor state does not reset when the USB2 Xeryon is unplugged.

Stop supersedes the desktop's pending request. If not acknowledged within 500 ms,
the UI closes the serial connection to drop DTR. Firmware stops on USB1 session
closure, five seconds without a valid command, or a 60-second move deadline.
Manual clients must send periodic PING commands during long moves.
The LPS22HHTR driver reports `pressure: true`. Its snapshot `pressure` object contains:

- `sensor: "lps22hh"`, `address: 92` (0x5C), `odr_hz: 200`, and `who`
  (WHO_AM_I byte, expected 179 / 0xB3, or null when unavailable).
- `raw` and `average`: signed 24-bit counts, -8388608 through 8388607,
  sign-extended into i32; pressure in hPa is `raw / 4096`.
  The average divides a zero-initialized 16-entry window by 16.
- `temperature`: signed 16-bit register value; °C is `temperature / 100`.
- `ms`: last fresh sample publication timestamp; `samples`: wrapping u32 count
  of fresh conversions read, nominally 200 per second.
- `valid`: false during initialization or after a sensor/transfer error. Numeric
  fields may retain old values and must not be displayed as current when invalid.
- `state`: 0 initializing, 1 waiting for first sample, 2 receiving,
  3 transaction timeout, 4 bus error / no acknowledgement, 5 unexpected device ID,
  6 no fresh conversion for 100 ms, 7 configuration readback mismatch.
- `errors`: wrapping u32 count of acquisition/setup failures.

The sensor uses SDA pin 18, SCL pin 19, 400 kHz I²C and repeated-start register
reads. Data-ready is checked every millisecond; one five-byte burst reads both
values only when fresh. Acquisition continues while streaming is paused.
Dashboard snapshots remain 10 Hz and experiment recording remains 50 Hz.
Update firmware and desktop together for this sensor-specific pressure schema.

Force calibration is a desktop presentation/export conversion; wire units remain
raw LPS counts. The dashboard and experiment plots display mN using the
[probe-derived calibration](../README.md#hardware-and-diagnostics). CSV appends
`force_n`, `force_mn` and `force_extrapolated` to the existing columns, preserving
`pressure_raw`. The CSV metadata adds `force_calibration` with the coefficients,
supported range, source hashes and source observations used for the export.

`plunger_io` remains unchanged: physical pad readbacks `standby`, `ain1`, `ain2`,
2-bit `encoder`, signed 16-bit `pwm_count`/`pwm_compare`, and boolean
`pwm_running`/`pwm_enabled`. Read snapshot records hardware diagnostics in the UI log.

Firmware with `plunger_limits: true` in HELLO enforces potentiometer bounds and reports `plunger.limits: {"min":30,"max":75}` in snapshots. Bounds reset to 30–75% at boot, survive serial reconnects, and reject outward moves while allowing inward recovery. Up increases the reading. Missing/stale potentiometer feedback prevents motion. Drive is disabled at a bound; sampling latency and mechanical coast can cause overshoot.

## Sinusoidal velocity control

HELLO adds `sine_velocity: true` and retains `sine_position: true` as the legacy UI
gate for desired sinusoidal position. START S accepts optional feed-forward parameters described below; default velocity
is `2π f A sin(2π f t)` using actual elapsed Teensy time. Changed integer SSPD
magnitudes use bounded adaptive 4–10 ms updates by default; theoretical sign
crossings are mandatory events. Ordinary busy updates coalesce and recompute
velocity at actual send time. Reversals wait for each USB completion: SSPD=0,
then SCAN=new sign, then current theoretical speed. Waveform TX and the full
reversal each have a 40 ms fault/STOP bound. No waveform DPOS or feedback drift
correction is used.
Initial DPOS targets `center - amplitude` and requires tracked Position Reached.
RUNNING starts with SSPD=0 then SCAN=1 at t=0 after initial positioning drains.
Position follows `center - A cos(2π f t)`. After the specified cycles, the waveform
ends with STOP at the original starting extremum (`center - amplitude`). No final
SSPD or DPOS positioning command is sent. Completion waits for STOP transmission
to drain, bounded by the existing 10 s reserve; it does not require Position
Reached or correct accumulated scan drift. ABORT/fault paths request STOP and
retain samples. SSPD is changed during the waveform and remains at the last setting.

Dataset version 2 uses 48-byte records: the original ten words followed by
`epos_generation` and `pressure_generation` (u32). The desktop rejects version 1
datasets instead of misdecoding them. Dashboard protocol remains version 1.
Position uses signed `elapsed_ms - epos_age_ms` for host receipt time. Repeated
position identities are suppressed in the plot, without interpolation. Pressure
plots every logger sample as a continuous line against experiment elapsed time
(`time_s` in CSV); no I²C completion time column is exported. Both identities and
per-signal new flags remain in CSV. During the sine waveform, `target` is the
theoretical position at actual logging time; the terminal sample at STOP is the
negative extremum, which remains the target while STOP drains.
Quasi-static retains its last commanded DPOS target. Metadata adds
`sine_control: "velocity_scan"` and
`sine_initialization: "negative_extremum_zero_velocity"` for sinusoidal runs and retains `trajectory_ms: 10`
and `sample_ms: 20`. Signed theoretical velocity can be calculated from saved
configuration and elapsed time during the waveform; no age field is replaced.
Controller faults include a specific `reason` and, when observed by the engine,
`fault_stat`: the triggering STAT word retained before STOP. The results display
this word because the live Dashboard may show a later, cleared status.
Positioning and acquisition report `running`; results remain owned until the
verified dataset acknowledgement. USB dispatch timing does not establish physical
tracking accuracy or feedback sampling rate.

Post-run `timing` metadata reports arrival statistics, cache reuse, unlogged EPOS
updates, receipt ages, parse errors and USB service/backlog/TX occupancy.
`epos_rtt`, `epos_source_drops` and `rx_overflow` are null (unavailable), not zero.
Receipt timestamps measure Teensy reception, not controller sampling time.
## Sinusoidal reversal diagnostics

For the velocity-reversal firmware, `data_meta` additionally declares
`reversal_count`, `reversal_total`, `reversal_window_ms` and an initially empty
`reversals` array. Each `data_chunk` JSON header supplies `reversal_offset` and
at most two reversal objects; its binary payload and CRC remain dataset v2.
The native receiver appends these objects in order to retained metadata and
requires the declared count before ACK. Restarting GET_DATA resets both payload
and diagnostic accumulation. Current firmware sends one reversal per two-sample
chunk; `reversal_count` is capped by the available chunks and 288 stored entries.
Validation limits runs to 144 cycles so accepted reversals fit the fixed storage.
The receiver also accepts older dataset-v2 chunks of up to sixteen samples and
two reversal objects. Nothing is streamed during RUNNING.

The 8192-byte JSON line limit accommodates these headers and final-return metadata. Older clients
can still decode the sample payload but need updating to retain the new
diagnostics. See the [reversal record implementation](../src/reversal.rs) for
fields and timestamp semantics.

## Adaptive scheduler diagnostics

`data_meta.scheduler` reports the selected `mode` (`adaptive` or `fixed100hz`),
interval/tolerance configuration, attempted/accepted waveform command counts,
SSPD rate and interval statistics, scheduling/reversal lateness, scheduled-to-queue
and USB completion latency statistics, held-command error, and TX queue statistics.
It also reports requested/coalesced updates, maximum consecutive coalescing,
busy-duration statistics, zero/direction/resume completion latency from the
theoretical crossing, and separate backpressure versus USB transfer fault counts.
See the [waveform transport implementation](../src/waveform_tx.rs) for diagnostic
definitions. Groups with no timing observations are null; missing
completions remain explicitly counted. These diagnostics are returned only after
RUNNING, retained by the desktop receiver, and exported in CSV metadata. Legacy
`trajectory_ms=10` is the fixed baseline/max evaluation interval; it no longer
describes every default-mode control interval. The sample format is unchanged.


### Final STOP evidence

`data_meta.final_return` (version 1, sinusoidal runs that reach the boundary) retains
host timestamps in wrapping absolute milliseconds. The legacy `final_return`
field name and version are retained for dataset compatibility. `target_counts`
and `theoretical_counts` both identify the starting extremum; `speed_um_s` is zero.
No final DPOS is sent: its token is zero, its timestamps are null, and `echo` is null.
Subtract `started_ms` modulo
2^32 for run-relative times. `stop` and `dpos` each contain an accepted command
`token` and `ms=[scheduled, queued, USB-completed]`; null means unobserved. USB
completion is not a controller acknowledgement. The DPOS `echo` is separate.
`boundary_feedback` and `final_feedback` contain EPOS, DPOS, STAT, TIME, in that
order; each entry is `[value, host_receipt_ms]` or null. `ptol_pto2_tout_dlay` uses
the same representation for read-only pre-motion controller queries.
`stat_transitions` retains up to eight distinct status words and receipt times;
bit 10 is Position Reached. Overflow is counted in `stat_transitions_dropped`.
The 50 Hz binary records retain the intermediate feedback and its ages, while
`final_feedback` captures completion/failure between logger ticks. No field
claims to be a synchronized physical measurement. `error_counts` and `error_um`
are signed EPOS minus the starting extremum, diagnostic only. `deadline_ms`,
`expired`, and `finished_ms` distinguish a timeout from successful completion.
`boundary_ms` is the first representable millisecond at or after the mathematical
cycle boundary; `observed_ms` is the actual foreground observation. The declared
zero theoretical velocity belongs to the exact cycle boundary, even when the
boundary is fractional or observed late. `waveform_tx_pending` reports USB busy;
`waveform_intent_pending` reports unsent/coalesced/reversal intent discarded at STOP.

The record format and DATA_VERSION remain unchanged. Deploy the updated desktop
receiver with this firmware because final metadata may exceed the old 4096-byte
line limit. Dataset CRC still covers binary records, as before.

## Dataset transfer

```text
START <id> Q <start_nm:i64> <step_nm:i64> <hold_ms:u32> <steps:u32> <speed_um_s:u32>
START <id> S <center_nm:i64> <amplitude_nm:u32> <frequency_millihz:u32> <cycles:u32> [<compensation_enabled:0|1> <compensation_ms:u32>]
ABORT <id>
GET_DATA <id> <run>
GET_CHUNK <id> <run> <record_offset>
ACK_DATA_RECEIVED <id> <run> <count> <crc32_decimal>
```

START replies with `experiment_ack`, run, and action `start`; ABORT uses action
`abort`. The retained dataset reports the terminal outcome. Hardware ownership
lasts through positioning, acquisition, and transfer until verified ACK. Ordinary
commands (including PING and manual STOP) return busy; use ABORT for an active run.
USB1 disconnect and PC heartbeat loss do not terminate autonomous acquisition.

On USB1 open, `experiment_state` announces `dashboard`, `running` (including
positioning/arming), or `data_ready`. The desktop waits passively before its
handshake so reconnect cannot inject HELLO/PING into a running experiment.
The native receiver downloads `data_ready` results using GET_DATA, then sequential
GET_CHUNK requests. `data_meta` includes configuration, run, version, record size,
count, bytes, CRC, outcome/reason, STOP queue result, timing, and retained diagnostics.

A `data_chunk` JSON header with id/run/record-offset/byte-length and reversal
metadata ends in LF, followed immediately by the declared binary bytes. There
is no delimiter after the payload. Current firmware sends at most two records
(96 bytes) per chunk; native reception accepts up to sixteen dataset-v2 records
for compatibility. Binary LF/NUL/non-UTF8 bytes are opaque to line framing.

Dataset v2 contains twelve explicit little-endian 32-bit words per 48-byte record:

| Byte offset | Field | Type |
| --- | --- | --- |
| 0 | elapsed_ms | u32, actual acquisition time |
| 4 | epos | i32, raw encoder counts |
| 8 | pressure_raw | i32, signed sensor counts |
| 12 | stat | u32 |
| 16 | epos_age_ms | u32 |
| 20 | pressure_age_ms | u32 |
| 24 | stat_age_ms | u32 |
| 28 | target | i32, theoretical sine / last quasi-static DPOS counts |
| 32 | controller_ticks | u32, independently cached TIME (0.1 ms units) |
| 36 | controller_age_ms | u32; UINT32_MAX means TIME unavailable |
| 40 | epos_generation | u32, publication identity including unchanged values |
| 44 | pressure_generation | u32, sensor publication identity |

Capacity is 4,096 records at 20 ms sampling. Position receipt time is signed
`elapsed_ms - epos_age_ms`; signals are asynchronous, not simultaneous measurements.
The terminal waveform sample and any subsequent samples while STOP drains record
the original starting extremum. Version 1 datasets are rejected rather than decoded
with the wrong layout. CSV preserves raw values, ages, identities, and diagnostics.

CRC-32/ISO-HDLC covers concatenated encoded records: initialize to 0xffffffff,
use reflected polynomial 0xedb88320, and complement the final value. JSON metadata
is validated separately and is not covered by this payload CRC. Native Rust
checks count, bytes, offsets, run, timestamps, diagnostic count, and CRC before
retaining the dataset and sending ACK_DATA_RECEIVED. Firmware also requires every
record to have transferred in sequence before accepting the matching count/CRC ACK.
Empty fault datasets still require verification and ACK.

Disconnect, CRC failure, or the five-second transfer-response timeout retains
the Teensy copy. Reconnect and GET_DATA restart from zero, including diagnostics.
START is never retried automatically. After ACK, bytes remain until a new START
passes validation. Desktop CSV/PNG export opens a native Save As dialog, releases
serial locks while it is open, and rechecks ownership before writing. Cancellation
or write failure leaves results available. RAM results are not durable across
power loss or desktop process exit.

Sinusoidal feed-forward compensation is experimental. New desktop START commands
always include both compensation fields; legacy omitted fields mean disabled,
17 ms configured. The firmware advertises `compensation_max_ms` in HELLO from
`MAX_COMPENSATION_MS`; whole values 0..25 ms are validated even when disabled.
Native configuration uses `compensation_enabled` (boolean) and `compensation_ms`
(unsigned integer), preserved in `data_meta.config` and CSV metadata.
`data_meta.feed_forward` labels the experiment and records `phase_deg =
360*f*compensation_ms/1000`, `applied_phase_deg` (zero when disabled), and the
250 ms `smoothstep_phase` activation policy. With `u=min(t/0.250,1)` and
`s=3u²−2u³`, only velocity uses `t+τs`; direction and adaptive error calculations
use that same monotone phase. Reference position, duration, all timestamp fields,
pressure acquisition, and final STOP timing remain unchanged. Compensation may
add a reversal just before the final STOP; the existing bounded diagnostics hold
all 288 reversals of a 144-cycle run. No mechanical-delay claim is implied.
