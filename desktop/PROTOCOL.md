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
malformed commands after LF. Only the listed commands are accepted; Xeryon remains read-only. Pausing display updates never pauses actuator reception.

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
HX711-enabled firmware reports `pressure: true`. Snapshots include an additive
`pressure` object with `raw`, `average`, `ms`, `samples`, `valid`, and `saturated`.
Raw/average are sign-inverted ADC counts in [-8388607, 8388608], not calibrated
pressure units. `average` covers up to the latest 16 valid readings. `ms` is the
last accepted conversion time; `samples` is a wrapping u32 accepted-sample count.
`valid: false` means startup, settling, timeout, or a failed transfer; numeric
fields may retain the last sample and must not be displayed as current data.
`saturated` flags either signed 24-bit ADC rail. HX711 uses A1/p15 DOUT, A0/p14
SCK, channel B/gain 32. Acquisition continues while streaming is paused.
No I²C scan, temperature, or WHO_AM_I operation applies to this device.
