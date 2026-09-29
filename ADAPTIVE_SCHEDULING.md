# Transport-aware adaptive sinusoidal scheduling

The default remains adaptive feed-forward SSPD/SCAN control. Position and velocity
use actual elapsed Teensy monotonic milliseconds, including across clock wrap.
There is no waveform DPOS, EPOS feedback control, drift correction, phase advance,
or plot-time shift. Initial/final finite positioning remains unchanged.

## Root cause and correction

Previously, every due waveform evaluation checked `tx_idle()` and immediately
faulted with `Xeryon previous transmit incomplete` when false, even before speed
deduplication. FIFO space was correctly insufficient to prove USB completion:
TinyUSB removes bytes from its FIFO when submitting a packet. Adaptive evaluations
could arrive every 4 ms while that packet remained in flight. Healthy EPOS and
logging did not imply that the independent outgoing transfer had completed.

`src/scheduling.rs` still chooses useful evaluation deadlines; `src/waveform_tx.rs`
now controls transport admission. Ordinary busy evaluations update a single
pending deadline for diagnostics and continue running. No velocity values or
historical commands are buffered. On the first idle foreground poll, the engine
recalculates `v(now) = 2πfA cos(2πft)` and sends the newest changed rounded SSPD.
Repeated evaluations coalesce. If the latest speed equals the accepted speed,
the pending ordinary update can disappear without any transmission.

For example, evaluations at 100, 104 and 108 ms with TX becoming idle at 109 ms
produce one command calculated at 109 ms. The next sign crossing still uses the
original run clock. Coalescing cannot accumulate software phase drift, although
transport and mechanical delay can still affect physical motion.

## Bounds and reversals

| Setting | Bound |
| --- | --- |
| Ordinary adaptive evaluation interval | 4–10 ms |
| Velocity error target | max(1, 0.07 × peak velocity + 0.5) µm/s |
| Scheduler lateness fault, unchanged | >= 10 ms |
| Unfinished waveform TX deadline | >= 40 ms from acceptance |
| Complete reversal deadline | >= 40 ms from first representable theoretical crossing |

Lookahead tests the continuous held-command error, including intervening cosine
extrema, over candidate intervals 4–10 ms. The next theoretical sign crossing
caps the chosen interval and can shorten it below 4 ms. Ordinary deadlines advance
even while TX is busy; held-command error continues to be measured. The error
budget is a planning objective, not a guaranteed bound during backpressure.
The minimum interval was not lowered and fixed scheduling was not made default.

Normal backpressure has no reported transfer failure and remains below both
applicable 40 ms bounds. At a bound the run faults and requests priority STOP.
An observed successful completion at or beyond the TX deadline still faults.
Before a first engine-owned send, any inherited busy condition is timed from its
first observation. These are explicit engineering limits requiring validation
with the actual actuator/load; they do not establish a physical tracking bound.

Reversal phases are deterministic and nonblocking:

1. `Normal` detects the absolute sign crossing and enters `NeedZero`.
2. `NeedZero` waits for the preceding command, then submits mandatory `SSPD=0`,
   even when integer rounding already selected zero.
3. `AwaitZero` waits for USB completion before submitting `SCAN=new direction`.
4. `AwaitDirection` waits for that completion, then recomputes and submits SSPD
   from the current waveform time, including a rounded zero if applicable.
5. `AwaitResume` waits for resume completion and returns to `Normal`.

The total deadline includes waiting for prior ordinary traffic and all three
reversal packets. Intermediate progress does not restart it. Several transfers
each shorter than 40 ms can therefore still fault the reversal. Each reversal
line is submitted separately; it cannot be coalesced away or reordered. Ordinary
waveform traffic has at most one outstanding line. Initial SCAN entry retains its
existing ordered two-line SSPD/SCAN submission.

## Failures and STOP priority

TinyUSB's CDC host previously returned early on failed transfers without notifying
the application. A documented local `tuh_cdc_xfer_error_cb` extension now latches
reported transfer failures separately from backpressure, for either endpoint.
Partial application writes also latch failure. Rust checks the latch on every
active poll, faults immediately, and attempts STOP. Unreported/missing completions
remain busy and reach the 40 ms deadline. Disconnect, STAT fault, stale feedback,
pressure and existing acquisition safety checks remain in force.

STOP/ABORT/fault/completion bypass waveform admission. STOP still discards waiting
transport bytes and takes the next available transfer after any already submitted
packet. Once terminal, the engine never resumes waveform commands. Cycle end
requests STOP even with a waveform update pending; final-center SSPD and DPOS wait
for STOP completion. The existing final-return deadline and terminal STOP remain.

A submitted USB packet cannot be recalled. As before, `stop_queued` means accepted
for transmission, not confirmation of mechanical stopping. No software STOP can
guarantee delivery through a disconnected or permanently stalled endpoint; this
change retains priority STOP attempts and never reports those failures as healthy.

## Servicing and diagnostics

The main-loop order and RX burst budget are unchanged: pressure and TinyUSB host
service precede the engine, followed by USB1/ABORT. RX drains at most 256 bytes per
poll, and RX backlog prevents WFI. No extra flush loops, queries, blocking waits,
heap allocation, threads, RTOS, or async runtime were introduced. INFO/POLI setup,
EPOS/STAT parsing, 200 Hz pressure acquisition, 50 Hz logging, Dashboard,
quasi-static mode, sample layout, CRC and dataset transfer remain unchanged.

`data_meta.scheduler` is downloaded after the run and retained in CSV. No new
RUNNING diagnostic stream is sent. The existing 4096-byte metadata frame still
fits tested worst-case counters.

| Metric | Meaning |
| --- | --- |
| `evaluations` | Actual-time waveform calculations, including idle retries |
| `sspd_requested` | Due evaluations needing a speed update or reversal progress |
| `commands_attempted`, `commands_sent`, `sspd_sent` | Lines attempted/accepted; SSPD includes mandatory reversal zero/resume |
| `busy_due` | Needed due updates skipped/coalesced while TX is occupied |
| `max_consecutive_coalesced` | Largest consecutive sequence of those updates |
| `tx_busy_ms` | Count/min/mean/max busy episodes; terminal incomplete episode is censored at exit |
| `scheduled_to_queue_ms` | Latest retained due deadline to acceptance; reversal zero/direction use theoretical crossing |
| `queue_to_complete_ms`, `scheduled_to_complete_ms` | USB success latencies, not controller acknowledgements |
| `reversal_zero_complete_ms` | Crossing to zero-command USB completion |
| `reversal_direction_complete_ms` | Crossing to direction-command USB completion |
| `reversal_resume_complete_ms` | Crossing to first resumed SSPD USB completion, even if rounded zero |
| `backpressure_faults`, `usb_transfer_faults` | Deadline faults versus reported transfer/send failures |
| `sspd_hz`, `sspd_interval_ms` | Effective accepted SSPD rate and inter-command min/mean/max |
| `completion_unobserved` | Accepted waveform lines with no observed completion at diagnostic freeze |
| `held_error_um_s`, `over_budget_observations` | Signed queued-command approximation error and sampled over-budget intervals |
| `queue_peak`, `lateness_ms`, `reversal_lateness_ms` | Outstanding line peak and scheduling timing |

The existing `timing` object retains EPOS/pressure rates and receipt ages, RX
backlog/peak, USB service gap, malformed records, and logging omissions. Reversal
traces retain raw EPOS/STAT, explicit zero/direction/resume queue and completion
times. The trace's existing first-nonzero slot may follow a rounded-zero resume;
the new resume latency statistic always measures the first resume command.
Unknown completion groups remain null rather than inventing zero latency.

Effective SSPD rate depends on useful changes, rounding, USB completion time and
mandatory reversal packets. It may be lower than the evaluation rate. There is no
catch-up burst and no promised 144 Hz hardware rate. Mathematical scheduler tests
still use an ideal transport model; physical latency is neither corrected nor
inferred from that model. The new sustained-backpressure simulation services a
2 Hz run with 9 ms per transfer, one outstanding waveform line, ordered reversals,
50 Hz EPOS publications, maximum receipt age 19 ms, and no lost logging slots.
This is a host simulation, not a measurement of the device.

## Builds and hardware repeat

Both modes use the same transport policy. Fixed100Hz remains an engineering A/B
option on its absolute 10 ms grid:

```sh
rtk proxy cargo objcopy --release --locked --features fixed-100hz -- -O ihex target/firmware/firmware-fixed100hz.hex
rtk proxy cargo objcopy --release --locked -- -O ihex target/firmware/firmware.hex
```

Neither command flashes hardware. No hardware was flashed during this change.

After separately installing the adaptive image, repeat the failed **2.0 Hz** run
with exactly the same center, amplitude, cycle count, load, orientation and
controller settings. Save the complete CSV and metadata. Then repeat three
five-cycle runs at 2.0 Hz with the same established amplitude/center; each must
retain ten ordered reversals at 125, 375, 625, ... ms. Do not substitute the
simulation's 1000 µm amplitude unless it was already the established safe setting.

Require no immediate busy-at-due fault; nonzero `busy_due` is expected and allowed.
Require zero backpressure/USB faults, no missed logging samples, complete ordered
zero→SCAN→resume observations, and all reversal completion latencies below 40 ms.
Check resumed SSPD against its actual queue-time waveform, not the missed due
value. Compare measured rate and busy/queue/completion tails with the prior run.
Check EPOS remains approximately 50 Hz, maximum receipt age approximately 21 ms,
and USB service gap approximately 1 ms; investigate degradation before changing
bounds. Retain raw timestamps and phase, with no alignment or phase compensation.

Finally repeat ABORT during ordinary TX, near each reversal polarity and during
final return. Confirm priority STOP, no subsequent waveform command, partial
CSV availability, normal final-center completion, and one unchanged quasi-static
run. Fault/disconnect validation should use the existing established procedure.

## Validation results

- Firmware native Rust: 71 tests passed in adaptive mode and 71 in fixed-100Hz mode.
- Both native C suites passed, including STOP queue replacement, RX burst servicing,
  transfer failure latching and reconnect recovery.
- Desktop Rust: 82 tests passed (12 application and 70 interoperability/shared
  firmware), including new diagnostic metadata retention through CSV.
- UI/protocol: all 17 tests passed. Native GUI smoke passed demo, IPC, disconnect,
  START validation, stable layout and hidden-Dashboard idling.
- Firmware release builds and HEX generation passed for both modes. Vite and
  native Tauri release builds passed (`npm run build -- --no-bundle`).
- Firmware/desktop formatting and diff whitespace checks passed.
- The pre-existing TinyUSB unused `cdch_process_set_config` warning remains.
- No hardware flash or motion was performed. Device throughput, EPOS timing and
  physical stopping remain hardware acceptance measurements.

Generated artifacts (checksums and EOF records verified):

- `target/firmware/firmware.hex`: 442424 bytes; 9836 valid Intel HEX records; SHA-256 `860addac1e8fad5eda849db6c6f4c5bc0946ebc54e6882957bf84a48a7962056`.
- `target/firmware/firmware-fixed100hz.hex`: 442424 bytes; 9836 valid Intel HEX records; SHA-256 `60403c7a0a2b768cd480bdfd52d1dae9e185b977a89c17ee04ed650ff0864a07`.
