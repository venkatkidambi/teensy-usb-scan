# Adaptive sinusoidal command scheduling

The default firmware uses adaptive feed-forward SSPD/SCAN updates. Position and
velocity still come from actual elapsed monotonic Teensy milliseconds. There is
no waveform DPOS, EPOS feedback correction, drift correction, latency advance,
or plot-time shift. Initial/final finite positioning and all STOP paths remain.

The request's phase interpretation was reversed: for
`v = 2πfA cos(2πft)`, `dv/dt = -(2πf)^2 A sin(2πft)` has largest magnitude at
**position extrema**, not position center crossings. Consequently the general
error criterion increases density near position extrema and reduces it near
position centers. Reducing velocity-update density at position extrema would
contradict the requested error objective.

## Algorithm and bounds

`src/scheduling.rs` defines:

| Configuration | Default |
| --- | --- |
| `MIN_CONTROL_INTERVAL` | 4 ms for ordinary evaluations |
| `MAX_CONTROL_INTERVAL` | 10 ms between planned evaluations |
| `MAX_VELOCITY_ERROR` | 0.07 of peak velocity |
| Effective per-run velocity tolerance | `max(1, 0.07 * 2πfA + 0.5)` µm/s |

The 0.5 µm/s allowance accounts for integer SSPD rounding; the 1 µm/s floor
avoids chasing sub-command-unit error in very small motions. Metadata reports
the effective absolute tolerance, rather than requiring the reader to infer it.

At each due event:

1. Calculate current velocity from **actual** elapsed time, without integrating
   phase or replaying missed updates. Derive direction from integer trajectory
   phase, independently of rounded speed.
2. Preserve the existing command cache. Send changed rounded SSPD only; at a
   sign crossing always send `SSPD=0`, then `SCAN=new sign` in ordered transfers.
   Never send SCAN=0 at a crossing. Resume current theoretical speed on the next
   scheduled evaluation, subject to normal integer deduplication.
3. Using the resulting held, signed integer SSPD, test candidate hold lengths
   4 through 10 ms and choose the longest whose maximum theoretical error stays
   within tolerance. Error includes both interval endpoints **and every possible
   intervening velocity extremum**; endpoints alone can miss a cosine maximum.
   Only two alternating extrema need examination even across a long missed poll.
   This is a bounded search, not a derivative division near zero acceleration.
4. Cap the next deadline at `ceil(next theoretical sign-crossing time)`. A
   crossing overrides the error threshold and may shorten an interval below
   4 ms, down to 1 ms. Following zero, compute the next deadline from held zero.
   Crossing times are absolute trajectory events, never accumulated periods.
5. Maximum-time evaluations still suppress identical SSPD. Thus a constant
   quantized speed can remain commanded longer than 10 ms without redundant
   heartbeat traffic. The maximum bound applies to evaluation, not mandatory
   retransmission of an unchanged command.

For the supported 0.1–2 Hz range, an ordinary 4 ms hold changes ideal velocity by
at most about 5.03% of peak. After a fractional-ms reversal, the maximum 1 ms
crossing quantization plus a 4 ms zero hold is below 6.29% of peak. The 7% budget
therefore accommodates both, including rounding. Tighter tuning must revisit
this relationship. The minimum interval takes precedence if a changed tolerance
cannot be met, and observed excess error is reported.

An on-time schedule bounds the **software held-command approximation**. It does
not bound actual motor speed, USB/controller execution delay, or foreground
jitter. A late poll uses the current waveform and records actual interval error;
it can exceed the target. Existing lateness of 10 ms or more faults and requests
STOP. An unfinished prior USB transfer at a due event also faults and requests
STOP, including at a priority crossing; reversal is never silently postponed.
The exact cycle-end deadline still overrides the adaptive schedule.

## Transport and service audit

The timebase is SysTick at 1 ms. Sub-ms scheduling or timing claims would be
unsupported. USB-host callbacks and command generation run cooperatively in the
foreground. The unchanged transport checks both in-flight transfer state and
FIFO availability; each update submits at most two complete, separately flushed
lines, with the second transfer following completion of the first. CDC line
coding is 115200, but this does not establish controller command throughput.
No measured device latency/throughput was available in this workspace.

The defaults retain the working 10 ms upper bound, allow ordinary evaluations
down to 4 ms, and avoid globally increasing traffic. On-time ordinary evaluations
are bounded to 100–250 Hz; priority crossings add at most 4 events/s at 2 Hz and
one SCAN line per crossing. Integer deduplication can make SSPD rates much lower
than 100 Hz. These are scheduling limits, not measured sustainable USB rates.
The crossing exception can produce adjacent commands 1 ms apart; the existing
TX-idle safety check remains authoritative if hardware cannot drain them.

The main loop is unchanged: pressure and USB host are serviced before the engine,
then USB1/ABORT, on every iteration. RX draining remains bounded at 256 bytes per
poll, with no WFI while RX is pending; pressure transactions also prevent WFI.
Pressure acquisition remains 200 Hz; logging remains 50 Hz; feedback INFO/POLI
setup, EPOS/STAT decoding, and safety freshness checks are unchanged. Lookahead
has at most seven candidate intervals. Held-error calculations run at due events
and terminal cleanup, not at every pressure FIFO poll. No heap, RTOS, sleeps,
extra readbacks, or RUNNING diagnostic streaming were introduced.

The added bounded calculation cost and actual service impact still require
hardware verification using the existing arrival/service metrics. In particular,
do not interpret unchanged source-code service order as a measured latency result.

## Post-run diagnostics

The `data_meta.scheduler` object is downloaded after the run and retained in
normal CSV metadata. Dataset v2 sample bytes, timestamps, CRC, logging cadence,
and plotting behavior are unchanged. Existing 4096-byte console/desktop framing
fits the additional metadata; no transfer frame enlargement was needed.

- `mode`, `min_ms`, `max_ms`, `max_velocity_error_um_s`: active policy and bounds
  (fixed mode reports 10 ms minimum). Legacy `trajectory_ms=10` remains for
  compatibility; use `scheduler` to interpret adaptive timing.
- `evaluations`, `commands_attempted`, `commands_sent`, `sspd_sent`, `sspd_hz`:
  waveform-only evaluation/line counts and effective SSPD rate, excluding final
  center positioning. Attempted lines are after deduplication; sent means accepted
  by the USB queue, not controller acknowledgement. An occupied transport blocks
  the attempt and increments `busy_due` instead. Rate uses waveform observation
  duration, excluding final-center settling.
- `sspd_interval_ms`: min/mean/max/count between actual accepted SSPD queue
  timestamps, including explicit reversal zeros and gaps from deduplication.
- `lateness_ms`: actual evaluation minus scheduled deadline, including a missed
  deadline that faults. `reversal_lateness_ms`: detected crossing minus the first
  representable theoretical crossing ms. Integer time quantization itself is
  less than 1 ms and is not included in this latter lateness statistic.
- `scheduled_to_queue_ms`, `queue_to_complete_ms`,
  `scheduled_to_complete_ms`: min/mean/max/count for waveform SSPD **and SCAN**
  lines. The engine retains scheduled times; C already timestamps accepted writes
  and TinyUSB success callbacks. Two fixed pending slots pair sequence IDs with
  callbacks, including clock wrap. Queue-to-complete includes second-command
  waiting time. USB submission time is not separately exposed. Completion means
  USB success, not the controller's physical execution instant.
- `completion_unobserved`: accepted waveform lines without an observed completion
  at diagnostic freeze, including interrupted/faulted runs. Missing observations
  are never counted as zero latency; an empty statistics group is `null`.
- `held_error_um_s`: continuous-interval maximum difference between the signed
  queued-command model and theoretical velocity, including zero-speed reversals,
  rounding, suppressed commands, and lateness. The model changes commands at the
  evaluation instant; queue/transfer delay is measured separately, not compensated.
  `over_budget_observations` counts examined nonempty intervals over the tolerance,
  not elapsed milliseconds and not physical tracking failures.
- `queue_peak`, `busy_due`: maximum accepted-minus-completed transport line count
  (including return traffic observed while RUNNING) and due evaluations blocked
  by pending TX. Existing `timing` retains RX peak/backlog polls, maximum service
  gap/TX busy time, EPOS/pressure arrival rates and ages, malformed lines and
  logger omissions. Existing reversal traces retain zero/SCAN/resume queue and
  completion times and raw EPOS/STAT observations.

The two recent completion timestamps may be overwritten if cleanup sends STOP
while a two-line command is still pending; such missing observations remain
explicitly unobserved. No compensation is enabled. The pure velocity function,
absolute scheduler deadline, and separate transport measurements leave a clear
place for a future, independently validated feed-forward time advance.

## Simulation and A/B build

Two-cycle, 1000 µm amplitude simulations with ideal foreground service and no
transport delay produced the following, including reversal zero holds:

| Frequency | Adaptive SSPD/s | Fixed SSPD/s | Adaptive max error µm/s | Fixed max error µm/s |
| --- | ---: | ---: | ---: | ---: |
| 1 Hz | 100.000 | 100.000 | 395.000 | 395.000 |
| 1.3 Hz | 109.162 | 100.065 | 569.072 | 1227.064 |
| 2 Hz | 144.000 | 100.000 | 872.196 | 2354.703 |

At 2 Hz the tolerance is 880.146 µm/s; the adaptive observed error is about 6.94%
of peak, versus 18.74% for fixed mode. These are mathematical approximation
errors, not EPOS tracking or latency measurements. Independent 0.1 ms cosine
sampling tests cover low-amplitude quantization and 0.1, 0.333, 1, 1.3, 1.999,
and 2 Hz waveforms. The 143-cycle engine test retains all 286 reversals across
clock wrap; default mode now reverses at 125 ms rather than the fixed 130 ms.

Engineering builds, with no normal UI setting:

```sh
rtk proxy cargo objcopy --release --locked --features fixed-100hz -- -O ihex target/firmware/firmware-fixed100hz.hex
rtk proxy cargo objcopy --release --locked -- -O ihex target/firmware/firmware.hex
```

Fixed100Hz preserves the previous absolute 10 ms control grid, including its
crossing quantization, for an honest baseline. Adaptive is the default. Both
collect identical diagnostics. Neither command flashes hardware.

## Hardware experiment to perform separately

1. Use the **same previously successful center, amplitude, controller settings,
   load, and orientation** as the reported 2 Hz phase-lag run. Do not substitute
   the illustrative 1000 µm simulation amplitude without an established safe run.
2. For each image, run five cycles at 1.0, 1.3, and 2.0 Hz, three repetitions per
   frequency: 18 runs total. Keep all settings except scheduler image/frequency
   identical, alternate fixed/adaptive ordering between repetitions, and save
   every raw CSV. Image installation is a separate hardware action.
3. At 2 Hz compare SSPD rate/intervals and held error against the fixed baseline;
   for a sufficiently resolved integer SSPD waveform expect roughly 144 versus
   100 SSPD/s. Require zero busy-at-due faults, zero missed logging slots, and
   inspect lateness and over-budget observations rather than assuming the ideal
   simulation bound held. Each completed five-cycle run must retain ten reversals.
4. Inspect every zero→SCAN→resume trace. At 2 Hz adaptive crossing targets are
   125, 375, 625, ... ms; fixed targets are 130, 380, 630, ... ms. At 1.3 Hz
   adaptive targets are ceil((250000 + k*500000)/1300) ms. Require ordered USB
   completions, no SCAN=0, and current-theory resume speed. Compare both polarities.
5. Compare scheduled→queue, queue→completion, and total delivery min/mean/max.
   Require complete observations for an uncomplicated completed run; investigate
   missing completions, growing backlog, or tail latency approaching the next
   deadline. These measurements do not establish controller execution latency;
   do not enable a phase advance from them alone.
6. Compare raw EPOS phase delay and shape with unchanged theoretical position at
   actual record time. Use recorded host receipt times/ages as before, with no
   post-processing alignment shifts. Check EPOS arrival rate/max age, STAT ages,
   RX backlog/service gaps and approximately 200 Hz pressure arrival rate against
   the fixed baseline. Any degradation blocks tightening the timing limits.
7. In both images repeat ABORT near each reversal polarity and during final
   return; verify STOP and partial download. Exercise the established safe
   disconnect/fault checks. Confirm cycle end and final-center completion remain
   unchanged. No hardware was flashed or driven during this implementation.

## Validation results

- Firmware native suite: 64 tests passed with adaptive defaults and 64 with
  `--cfg 'feature="fixed-100hz"'`; both C suites passed.
- Native desktop Rust: 75 tests passed (12 application, 63 interoperability/shared
  firmware), including adaptive metadata retention through download and CSV.
- JavaScript UI/protocol: all 17 tests passed.
- Firmware release builds passed in both modes; Vite and native Tauri release
  build passed. Native GUI smoke passed IPC, demo, disconnect, START validation,
  hidden-Dashboard idling, and stable experiment layout.
- Existing TinyUSB `cdch_process_set_config` unused-function warning remains.
- No hardware flash or motion was performed. Hardware command latency and service
  impact remain unmeasured; the error/rate table above is simulation only.

Generated artifacts:

- `target/firmware/firmware.hex`: 447435 bytes, 9947 valid Intel HEX records; SHA-256 `2b6ffa285fdbad87c0cb3881d2ab6863bb6c93c17bc5506143551fe0e659f980`.
- `target/firmware/firmware-fixed100hz.hex`: 447480 bytes, 9948 valid Intel HEX records; SHA-256 `56182785de85ec62594f9413af5386dbe15f9ea0ba1a0c25a8c51da1405bf1a8`.
