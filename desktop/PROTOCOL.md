# PC ↔ Teensy protocol v1

USB1: CDC ACM `5824:27dd`, 115200 8-N-1, no flow control, DTR asserted.
Baud rate is a CDC setting, not USB throughput. The desktop opens the port natively.
The actuator is on separate USB2 (`04d8:ef4a`).

Requests are ASCII lines terminated by LF, maximum 64 bytes before LF. CRLF also
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
| `PLUNGER_ZERO 9` | `plunger_ack`, action `zero` | Zero encoder while stopped |
| `XERYON_MOVE 10 U 10000` | `xeryon_ack`, action `move`, target counts | Move Xeryon up 10 µm (wire distance in nm) |
| `XERYON_MOVE 11 D 10000` | `xeryon_ack`, action `move`, target counts | Move Xeryon down 10 µm |
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

Xeryon-enabled firmware (0.1.5+) adds `xeryon: true` to HELLO. The UI accepts
older HELLO messages without this capability but disables Xeryon controls.
`XERYON_MOVE` requires direction U/D and integer distance 78–2617245618 **nm**.
The UI converts µm to nm, allowing three decimal places. Firmware rounds travel
to counts as `(nm + 39) / 78`, then adds/subtracts from the latest EPOS. It checks
signed 26-bit EPOS/target bounds [-33554432, 33554431], known USB identity,
ready CDC, fresh EPOS/STAT (<3 seconds), motion state and controller faults.
It sends `DPOS=<target>\n`, without changing speed, limits, homing, or tuning.

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
Firmware 0.1.4 uses the NanoManipulator I²C pressure sensor and reports
`pressure: true`. Its snapshot `pressure` object contains:

- `sensor: "i2c"`, `address: 92` (0x5C), and `who` (WHO_AM_I byte, or null).
- `raw` and `average`: unsigned 24-bit raw counts, 0 through 16777215.
  The average uses 16 entries initialized to zero, matching the Arduino sketch.
- `temperature`: unsigned 16-bit raw register value, without unit conversion.
- `ms`: last successful sample timestamp; `samples`: wrapping u32 sample count.
- `valid`: false during initialization or after a transfer error. Numeric fields
  may retain old values and must not be displayed as current when invalid.
- `state`: 0 initializing, 1 waiting for first sample, 2 receiving,
  3 transaction timeout, 4 bus error / no acknowledgement.
- `errors`: wrapping u32 failed-transaction count.
- `saturated`: always false; retained for compatibility with the earlier schema.

The sensor uses SDA pin 18, SCL pin 19, 400 kHz. Acquisition continues while
streaming is paused. There are no new PC commands; WHO_AM_I and raw temperature
are included in snapshots. Update the desktop app with this firmware because
older parsers enforce the HX711 signed range.

The updated app also accepts legacy HX711 firmware: absent `sensor` identifies
its sign-inverted counts [-8388607, 8388608], legacy states (0 waiting for DOUT,
1 settling, 2 receiving, 3 conversion timeout, 4 invalid transfer), and optional
`pins` booleans `dout_high`, `sck_high`, `sck_requested_high`.

`plunger_io` remains unchanged: physical pad readbacks `standby`, `ain1`, `ain2`,
2-bit `encoder`, signed 16-bit `pwm_count`/`pwm_compare`, and boolean
`pwm_running`/`pwm_enabled`. Read snapshot records hardware diagnostics in the UI log.
