# PC ↔ Teensy protocol v1

Firmware 0.2.0 adds `experiments: true`, START/ABORT and a retained binary dataset
extension. See [EXPERIMENTS.md — Dataset protocol](../EXPERIMENTS.md#dataset-protocol)
for framing, raw record layout, CRC, ownership, passive reconnect and ACK lifecycle.
All Dashboard commands below remain available when no experiment owns hardware.

Current firmware also advertises `quasi_static_speed: true`. Quasi-static START
requires `START <id> Q <start_nm> <step_nm> <hold_ms> <steps> <speed_um_s>` with
integer speed 1–16777215 µm/s; missing, fractional, zero or out-of-range speeds
are rejected. The desktop requires this capability before offering START Q.
Sinusoidal START keeps its existing arguments. Quasi-static metadata includes
`speed_um_s`. START acknowledges acceptance; controller speed/TOU2 readback
checks happen before motion and report any failure in the retained dataset.

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
Dashboard snapshots remain 10 Hz and experiment recording remains 20 Hz.
Update firmware and desktop together for this sensor-specific pressure schema.

`plunger_io` remains unchanged: physical pad readbacks `standby`, `ain1`, `ain2`,
2-bit `encoder`, signed 16-bit `pwm_count`/`pwm_compare`, and boolean
`pwm_running`/`pwm_enabled`. Read snapshot records hardware diagnostics in the UI log.

Firmware with `plunger_limits: true` in HELLO enforces potentiometer bounds and reports `plunger.limits: {"min":30,"max":75}` in snapshots. Bounds reset to 30–75% at boot, survive serial reconnects, and reject outward moves while allowing inward recovery. Up increases the reading. Missing/stale potentiometer feedback prevents motion. Drive is disabled at a bound; sampling latency and mechanical coast can cause overshoot.

## 100 Hz sinusoidal positioning

HELLO includes `sine_position: true`. START S keeps the same parameters and
sends a sine-derived `DPOS` target every 10 ms, using elapsed Teensy time.
The wave frequency remains 0.1–2 Hz; 100 Hz is the command update rate.
Initial and final center positioning use confirmed finite moves. Controller
settings are left unchanged. ABORT and faults request STOP and retain samples.

The 40-byte record layout is unchanged. `target` is the last commanded position
at the logging instant. Metadata reports `trajectory_ms: 10` and `sample_ms: 50`.
Controller faults include a specific `reason` and, when observed by the engine,
`fault_stat`: the triggering STAT word retained before STOP. The results display
this word because the live Dashboard may show a later, cleared status.
Positioning and acquisition report `running`; results remain owned until the
verified dataset acknowledgement. USB dispatch timing does not establish physical
tracking accuracy or feedback sampling rate.
