# Sinusoidal velocity reversal investigation

## Finding and limits of evidence

The previous `ScanState::update` chose direction zero whenever the rounded SSPD
was zero, or whenever a nonzero direction changed sign. It sent `SSPD=0`,
`SCAN=0`, and on a later nonzero tick sent `SCAN=<sign>`, `SSPD=<magnitude>`.
Consequently **every extremum deliberately stopped and restarted scanning**.
Small amplitudes could enter this stopped mode before the true phase crossing,
for multiple ticks. The engine also waived the closed-loop status requirement
during the stopped phase and for 100 ms after restarting.

Both signs used exactly the same branch. No positive-only firmware path was
found. Unequal alignment of extrema with 10 ms ticks can give different event
timings at fractional frequencies. Mechanics, load, controller directional
settings and feedback timing could also produce the observed positive-peak
asymmetry; none has been measured here.

The [official XD-C v3.1 manual, section 4.2, pages 8–9](https://xeryon.com/download-files/Xeryon%20XD-C%20manual%20v3.1%20-%20JPL.pdf)
defines SCAN ±1 as closed-loop scanning in the specified direction, SCAN=0 as
stop, and SSPD as the scanning speed. It specifies no intervening stop command
for changing direction. It does **not** document internal landing-state behavior
at zero speed or guarantee a transient-free reversal. Removing the explicit stop
is therefore a supported command-level simplification, not proof of a mechanical
cure. Stop/re-entry is the leading software suspect; the positive-only physical
blip remains unverified until a hardware comparison.

The transport audit found: Rust sends at most two separate complete lines after
checking TX idle; the first is flushed immediately, the second waits for the
first USB completion. TinyUSB calls its success callback before flushing queued
output. FIFO space alone is not considered idle. A pending command at the next
trajectory update faults; the fix preserves this rule. No software evidence of
actual RX backlog on the user's hardware was available.

## New policy and illustrative trace

Direction is derived from `(elapsed_ms * frequency_mhz) % 1_000_000`, using u64
arithmetic. The outgoing sign is negative in [250000,750000), positive otherwise.
This avoids relying on a rounded magnitude or approximate cosine's sign at an
exact extremum. Velocity magnitude still follows `2πfA cos(2πft)` at actual
monotonic elapsed time, with the existing nearest-integer SSPD conversion.

1. A rounded zero before the phase crossing changes SSPD only.
2. At the first scheduler tick at/past the crossing, send explicit `SSPD=0`,
   followed by `SCAN=<new sign>`. No finite-speed direction flip is sent.
3. Retain zero for that tick. At the next existing scheduler tick, use the current
   waveform speed. If it still rounds to zero, send nothing until it changes.

The cached direction/speed form a small nonblocking two-stage state machine:
crossing tick selects direction at zero; following ticks resume speed updates.
There is no extra dwell timer, phase reset, replay, delay or blocking readback.
The actual zero-to-resume interval depends on scheduler timing and quantization;
it is nominally 10 ms when the next velocity rounds nonzero. Successful USB
completion, not physical zero velocity, gates the following update.

Illustrative **calculated**, not hardware-measured, 1000 µm amplitude / 1 Hz trace:

| Peak | Elapsed ms | SSPD before crossing | Old commands at crossing | New commands at crossing | Next tick |
| --- | --- | --- | --- | --- | --- |
| Positive | 220,230,240 → 250 | 1177,787,395 | SSPD=0; SCAN=0 | SSPD=0; SCAN=-1 | 260: SSPD=395 (old also sent SCAN=-1) |
| Negative | 720,730,740 → 750 | 1177,787,395 | SSPD=0; SCAN=0 | SSPD=0; SCAN=1 | 760: SSPD=395 (old also sent SCAN=1) |

At 2 Hz the first extremum is at 125 ms: the 130 ms tick selects zero/new sign,
and 140 ms resumes using v(140 ms). At 1.3 Hz the first extremum is approximately
192.308 ms: 200 ms performs the crossing and 210 ms resumes on an ideal schedule.
The phase always continues. Both polarities share this policy, including the
exact-quarter-cycle convention. One explicit zero is repeated at the crossing
if a preceding tick already rounded to zero; it is not retransmitted each tick.

STOP remains unchanged at waveform completion, final-center completion, ABORT
and fault. It still discards queued commands and cannot recall an in-flight USB
packet. A failed STOP enqueue remains a reported failure, not a claim that the
actuator stopped. Direction changes no longer renew the initial closed-loop
feedback grace: inactive closed loop at a reversal faults normally.

## Retained diagnostics

The existing 50 Hz records, raw EPOS plots, pressure acquisition at 200 Hz,
100 Hz control scheduler and waveform equation are unchanged. There is no drift
correction, filtering, interpolation or waveform DPOS. Quasi-static control is
unchanged. Diagnostic collection allocates no heap and sends nothing to USB1
during RUNNING. It observes existing feedback and command traffic only.

Fixed storage holds 288 reversal records, sufficient for all 286 extrema of the
longest valid 2 Hz / 143-cycle run. Reset happens only on an accepted new run.
Aborted/faulted datasets retain partial diagnostics. `reversal_total` counts
detected crossings; `reversal_count` is the retained count. They match with the
firmware's full buffer; a deliberately smaller test buffer reports truncation.

The initial `data_meta` declares these counts, `reversal_window_ms=40`, and an
empty `reversals` array. Each existing binary data chunk's JSON header carries
at most two reversals and their `reversal_offset`. The desktop validates ordering
and completeness before acknowledging and collects them into dataset metadata.
The normal CSV's first metadata row contains the full traces; the results panel
compares both peak polarities. The sample payload, record version, 50 Hz rate and
sample CRC are unchanged. Diagnostic JSON has the same integrity level as other
JSON metadata; it is not covered by the binary sample CRC.

Each reversal contains:

- `direction`: outgoing SCAN sign; -1 identifies a positive position peak, +1 a
  negative peak. `crossing_ms` is the actual scheduler detection time.
- `commands`: six ordered slots: the last three transmitted SSPD commands,
  explicit zero, direction change, first nonzero SSPD afterward. A command is
  `[kind,value,queued_ms,completed_ms]`, where kind 1 is SSPD and 2 is SCAN.
  Times are relative to Teensy run start with 1 ms resolution and clock-wrap
  handling. Queue timestamps come from C's accepted write; completion timestamps
  come from the TinyUSB success callback. Missing commands/completions are null,
  including interrupted or partially accepted reversals. Queue timestamp equality
  does not imply simultaneous USB delivery: the packets remain ordered.
- `feedback`: three `[elapsed_ms,EPOS,STAT,epos_age_ms,stat_age_ms]` snapshots at
  the preceding control tick, crossing tick, and first nonzero command. The last
  is null if resume was not captured. EPOS is raw encoder counts; STAT is raw bits.
  These are cached publications with their actual ages, not invented samples.
  Use the unchanged 50 Hz records for the entire before/after EPOS shape.
- `max_error_counts`: maximum absolute raw EPOS minus theoretical position at
  that EPOS's host receipt time, observed within ±40 ms of the theoretical peak.
  Each new EPOS generation is evaluated once per window. Multiply by 0.078 for
  µm. This is an observed feedback error, not a bound on inter-sample motion or a
  correction for USB transit/mechanical lag.
- `rx_backlog_polls`: increments of the existing post-drain backlog counter in
  that window. `tx_busy_polls`: foreground observations of pending TX, including
  normal transfer time; a nonzero count alone does not establish a command backlog.
  `stat_or` retains all observed status bits in the window. The existing timing
  metadata also supplies run-wide max TX busy time, service gap and RX peak.

Subtract queue or completion timestamps to compare zero→direction,
direction→first-nonzero, and zero→first-nonzero durations. Never treat a missing
completion as zero latency. These times do not expose the controller's command
execution instant, encoder latch time, source-side backlog, or internal landing
state. Those require hardware measurement/vendor information.

Only the USB1 output frame grows to 4096 bytes; ordinary frames remain 2048.
The desktop's existing 4096-byte JSON limit is preserved. Worst-case two-reversal
headers plus the 768-byte binary payload are tested against that bound.

## Hardware verification to run (no hardware flashed in this change)

1. After a separately arranged firmware update, keep the exact center, amplitude,
   frequency, load, orientation and controller settings of the blip-producing
   experiment. Run 10 cycles if they fit the existing duration limit; otherwise
   use the original cycle count. Save the raw CSV and compare with the previous
   raw dataset. Do not change directional settings or tune either peak separately.
2. With that same previously safe center/amplitude, run 5 cycles each at 1.0 Hz,
   1.3 Hz and 2.0 Hz, provided those speeds are within the already established
   hardware operating range. These exercise exact-tick, varying fractional-tick,
   and consistent half-tick extrema. Repeat each run three times under the same
   load. Do not infer a mechanical speed limit from the firmware's 24-bit range.
3. For each run require `reversal_count == reversal_total == 2 * cycles`. Inspect
   every six-slot command trace: explicit zero before the one direction command,
   no SCAN=0, no repeated direction, USB completions ordered, and the first new
   speed on the next scheduled tick unless it rounds zero. Compare the measured
   queue/completion durations at both polarities and check for TX/deadline faults.
4. Compare raw EPOS points in ±40 ms windows around every theoretical peak,
   using receipt times and identities in the existing CSV. Compare the positive
   and negative maximum errors, pre/cross/post STAT, and `stat_or`. Confirm the
   positive blip diminishes without a compensating negative blip or long flat
   interval. A residual asymmetric blip with symmetric command/completion timing
   points toward controller/mechanical behavior, rather than proving this policy
   sufficient. Preserve that evidence before any further tuning.
5. Check RX backlog, TX busy duration, service gap, malformed lines, fresh EPOS
   rate/ages, zero missed logger slots and approximately 200 Hz pressure updates.
   Same-tick receipt bursts or growing backlog weaken timing conclusions. Verify
   STAT scanning/closed-loop behavior during zero speed; unexpected closed-loop
   loss now safely faults rather than being hidden by a reversal grace period.
6. Repeat ABORT near each polarity's turnaround and final center return; confirm
   stopped motion and successful partial-data download. Exercise the established
   safe disconnect/fault checks and verify completion still stops and returns to
   center. Do not create a mechanical overtravel to test a fault.

The physical root cause and outcome remain pending these experiments.

## Software validation (2026-09-29)

- `bash tests/run.sh`: both C suites and all 59 firmware tests passed.
- Native desktop Cargo tests: 70 passed (12 application + 58 interoperability /
  shared firmware tests). This includes retained trace download, missing/incorrect
  diagnostic offsets, retry, and refusal to acknowledge incomplete diagnostics.
- `npm test`: all 17 UI/protocol tests passed, including both peak summaries.
- Firmware release build and `npm run build` (Vite + native Tauri release) passed.
- Native GUI smoke test passed IPC, demo, disconnect, START/validation, idle
  Dashboard and experiment layout checks; it opens no hardware serial port.
- Worst-case diagnostic frame bounds, timestamp rollover, partial send evidence,
  raw receipt-time error, bounded storage, and a complete 2 Hz / 143-cycle run
  with 286 retained reversals passed. The latter exercises crossings halfway
  between ticks across a monotonic clock wrap.
- `cargo objcopy --release --locked -- -O ihex target/firmware/firmware.hex`
  generated 364099 bytes. All 8095 Intel HEX records, checksums and EOF validated.
  SHA-256: `7f7d356577fedc488bad25dbf78d8da8d56c47bec64e32bed25411148ec41b30`.

Linked `.bss` is 280576 bytes, `.data` 960, `.text` 103792 and `.rodata` 15784.
DTCM ends at `0x20049800`, leaving 26624 bytes in its 320 KiB allocation after
the existing 16 KiB stack reservation and linker alignment. The existing unused
heap reservation remains unchanged; no allocation was added. Runtime stack and
scheduler latency still require physical validation. The build retains the
pre-existing vendored TinyUSB unused-function warning for `cdch_process_set_config`.
No hardware was flashed.
