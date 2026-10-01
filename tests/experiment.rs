// Engine regressions live only in the firmware test binary.
include!("../src/experiment.rs");

#[cfg(test)]
mod tests {
    use super::*;
    #[derive(Default)]
    struct Io {
        targets: Vec<(i32, u32, bool)>,
        stops: usize,
        fail: bool,
        busy: bool,
        speeds: Vec<u32>,
        scans: Vec<xeryon_motion::ScanCommand>,
        events: Vec<String>,
        preparing: bool,
        usb_failed: bool,
        tracked: bool,
        clock: u32,
        progress: [u32; 6],
    }
    impl Io {
        fn queued(&mut self) {
            if self.tracked {
                self.progress[0] += 1;
                self.progress[2] = self.progress[3];
                self.progress[3] = self.clock;
                self.busy = true;
            }
        }
        fn complete_one(&mut self, now: u32) {
            assert!(self.progress[0] > self.progress[1]);
            self.progress[1] += 1;
            self.progress[4] = self.progress[5];
            self.progress[5] = now;
            self.busy = self.progress[0] != self.progress[1];
        }
    }
    impl Actuator for Io {
        fn tx_progress(&self) -> reversal::TxProgress {
            reversal::TxProgress(self.progress)
        }
        fn tx_failed(&self) -> bool {
            self.usb_failed
        }
        fn prepare_feedback(&mut self, _now: u32) -> Result<bool, &'static str> {
            Ok(!self.preparing)
        }
        fn scan(&mut self, commands: &[xeryon_motion::ScanCommand]) -> Result<(), &'static str> {
            if self.fail || self.busy {
                return Err("scan transmit failure");
            }
            self.scans.extend_from_slice(commands);
            for command in commands {
                if self.tracked {
                    self.progress[0] += 1;
                    self.progress[2] = self.progress[3];
                    self.progress[3] = self.clock;
                    self.busy = true;
                }
                let mut line = String::new();
                command.write(&mut line).unwrap();
                self.events.push(line);
            }
            Ok(())
        }
        fn query_motor_timeout(&mut self) -> Result<(), &'static str> {
            if self.fail {
                Err("timeout query transmit failure")
            } else {
                Ok(())
            }
        }
        fn speed(&mut self, speed: u32) -> Result<(), &'static str> {
            if self.fail {
                return Err("speed transmit failure");
            }
            self.speeds.push(speed);
            Ok(())
        }
        fn tx_idle(&self) -> bool {
            !self.busy
        }
        fn target(&mut self, target: i32, now: u32, track: bool) -> Result<(), &'static str> {
            if self.fail {
                return Err("transmit failure");
            }
            assert!(!self.busy, "DPOS must not overtake unfinished TX");
            self.queued();
            self.targets.push((target, now, track));
            self.events.push(format!("DPOS={target}\n"));
            Ok(())
        }
        fn stop(&mut self) -> Result<(), &'static str> {
            self.stops += 1;
            self.events.push("STOP\n".into());
            if self.fail {
                Err("stop failure")
            } else {
                self.queued();
                Ok(())
            }
        }
    }
    fn feedback(now: u32, target: i32, reached: bool) -> (Telemetry, Reading) {
        let mut t = Telemetry::new();
        t.mounted = true;
        t.cdc = 2;
        t.vid = 0x04d8;
        t.pid = 0xef4a;
        t.record(b"EPOS", i64::from(target), now);
        t.record(b"DPOS", i64::from(target), now);
        t.record(b"STAT", if reached { 1344 } else { 352 }, now);
        let mut motion = xeryon_motion::Motion::new();
        motion.start(target, now);
        motion.state = if reached { 2 } else { 1 };
        t.xeryon_motion = Some(motion);
        (
            t,
            Reading {
                valid: true,
                state: 2,
                raw: 123,
                ms: now,
                ..Reading::default()
            },
        )
    }
    fn sinusoidal() -> ExperimentConfig {
        ExperimentConfig::Sinusoidal {
            center_nm: 0,
            amplitude_nm: 7800,
            frequency_mhz: 1000,
            cycles: 1,
        }
    }
    #[test]
    fn records_distinguish_fresh_stationary_epos_from_reused_cache_and_keep_ages() {
        let mut records = [Record::EMPTY; 1024];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        start(&mut e, sinusoidal(), 0, &mut io);
        let (mut t, mut p) = feedback(2, 0, true);
        t.timing_active = true;
        for now in 3..=152 {
            if now == 3 || now == 53 {
                t.record(b"EPOS", 0, now);
            }
            if now == 4 {
                p.ms = now;
                p.samples += 1;
            }
            t.record(b"STAT", 1344, now);
            e.poll(now, &t, p, &mut io);
        }
        assert_eq!(e.len, 8);
        assert_eq!(e.samples[1].epos_generation, 2);
        assert_eq!(e.samples[2].epos_generation, 2);
        assert_eq!(e.samples[3].epos_generation, 3);
        assert_eq!(e.samples[1].epos_age_ms, 19);
        assert_eq!(e.samples[1].pressure_age_ms, 18);
        assert_eq!(
            i64::from(e.samples[1].elapsed_ms) - i64::from(e.samples[1].epos_age_ms),
            1
        );
        assert_eq!(e.repeated_epos, 5);
        assert_eq!(e.repeated_pressure, 6);
        assert_eq!(e.epos_arrivals.count, 2);
        assert_eq!(e.pressure_arrivals.count, 1);
        assert_eq!(e.max_epos_age, 99);
        e.abort(&mut io);
        let frozen = e.epos_arrivals.count;
        t.record(b"EPOS", 999, 153);
        e.poll(153, &t, p, &mut io);
        assert_eq!(e.epos_arrivals.count, frozen);
    }

    #[test]
    fn feedback_preparation_blocks_motion_and_old_speed_replies() {
        let mut records = [Record::EMPTY; 1024];
        let mut e = Engine::new(&mut records);
        let mut io = Io {
            preparing: true,
            ..Io::default()
        };
        let (mut t, p) = feedback(0, 0, true);
        let config = ExperimentConfig::QuasiStatic {
            start_nm: 0,
            step_nm: 78,
            hold_ms: 100,
            steps: 1,
            speed_um_s: 10,
        };
        e.accept(config, &t, p, false, 0).unwrap();
        e.begin(0, &mut io);
        t.record(b"SSPD", 99, 1); // old requested slot during feedback setup
        e.poll(1, &t, p, &mut io);
        assert!(io.speeds.is_empty() && io.targets.is_empty());
        io.preparing = false;
        e.poll(2, &t, p, &mut io);
        assert_eq!(io.speeds, [10]);
        e.poll(3, &t, p, &mut io);
        assert!(e.speed_pending && io.targets.is_empty());
        t.record(b"SSPD", 10, 4);
        e.poll(4, &t, p, &mut io);
        assert!(!e.speed_pending && e.timeout_pending);
        assert!(io.targets.is_empty());
        e.abort(&mut io);
        assert_eq!(e.state, State::DataReady);
        assert_eq!(io.stops, 1);
    }

    #[test]
    fn metadata_with_worst_case_timing_counters_fits_fixed_frame() {
        let mut records = [Record::EMPTY; 1024];
        let mut e = Engine::new(&mut records);
        start(&mut e, sinusoidal(), 0, &mut Io::default());
        e.epos_arrivals = crate::protocol::ArrivalStats {
            count: u32::MAX,
            first_ms: 0,
            last_ms: u32::MAX,
            sum_ms: u64::from(u32::MAX),
            min_ms: u32::MAX,
            max_ms: u32::MAX,
        };
        e.pressure_arrivals = e.epos_arrivals;
        e.transport = crate::protocol::TransportStats {
            rx_peak: u32::MAX,
            rx_backlog_polls: u32::MAX,
            max_service_gap_ms: u32::MAX,
            max_tx_busy_ms: u32::MAX,
        };
        e.repeated_epos = u32::MAX;
        e.repeated_pressure = u32::MAX;
        e.max_epos_age = u32::MAX;
        e.max_pressure_age = u32::MAX;
        e.feedback_before = [Some(i64::MAX), Some(i64::MAX)];
        e.run = u32::MAX;
        e.started_ms = u32::MAX;
        e.elapsed_ms = u32::MAX;
        e.missed_samples = u32::MAX;
        e.max_lateness_ms = u32::MAX;
        e.scheduling.sent(
            &[xeryon_motion::ScanCommand::Speed(MAX_SPEED_UM_S)],
            reversal::TxProgress([1, 0, 0, u32::MAX - 1, 0, 0]),
            0,
            1,
            0,
            1,
        );
        e.scheduling.refresh(reversal::TxProgress([
            1,
            1,
            0,
            u32::MAX - 1,
            0,
            u32::MAX - 2,
        ]));
        e.scheduling.sent(
            &[xeryon_motion::ScanCommand::Speed(0)],
            reversal::TxProgress([2, 1, 0, u32::MAX - 1, 0, 0]),
            0,
            u32::MAX,
            10,
            -1,
        );
        e.scheduling.evaluations = u32::MAX;
        e.scheduling.attempted = u32::MAX;
        e.scheduling.sent = u32::MAX;
        e.scheduling.sspd = u32::MAX;
        e.scheduling.over_budget = u32::MAX;
        e.scheduling.busy_due = u32::MAX;
        e.scheduling.queue_peak = u32::MAX;
        e.scheduling.sspd_requested = u32::MAX;
        e.scheduling.max_coalesced = u32::MAX;
        e.scheduling.backpressure_faults = u32::MAX;
        e.scheduling.usb_faults = u32::MAX;
        e.scheduling.busy_duration.add(u32::MAX);
        e.scheduling.reversal_zero.add(u32::MAX);
        e.scheduling.reversal_direction.add(u32::MAX);
        e.scheduling.reversal_resume.add(u32::MAX);
        e.scheduling.max_error = f64::from(MAX_SPEED_UM_S) * 2.0;
        e.scheduling.lateness.add(u32::MAX);
        e.scheduling.reversal_lateness.add(u32::MAX);
        let sample = crate::protocol::Sample {
            value: i32::MIN as i64,
            ms: u32::MAX,
            valid: true,
        };
        e.final_return = final_return::Diagnostics {
            boundary: Some(u32::MAX),
            observed: u32::MAX,
            theoretical_counts: i32::MIN,
            target: MAX_POSITION,
            speed: MAX_SPEED_UM_S,
            waveform_pending: true,
            intent_pending: true,
            boundary_feedback: [sample; 4],
            last: [sample; 4],
            settings: [sample; 4],
            echo: sample,
            status: [sample; 8],
            status_len: 8,
            status_dropped: u32::MAX,
            deadline: u32::MAX,
            expired: true,
            finished: Some(u32::MAX),
            ..Default::default()
        };
        for command in [&mut e.final_return.stop, &mut e.final_return.dpos] {
            command.queued(u32::MAX, reversal::TxProgress([u32::MAX; 6]));
            command.refresh(reversal::TxProgress([u32::MAX; 6]));
        }
        let mut frame = crate::protocol::FixedFrame::<8192>::new();
        e.metadata(&mut frame, u32::MAX).unwrap();
        assert!(frame.len < 8192);
    }
    #[test]
    fn ten_hz_runs_complete_with_bounded_downloadable_diagnostics() {
        for cycles in [1, 10, 144] {
            let mut records = [Record::EMPTY; CAPACITY];
            let mut turns = [reversal::Reversal::EMPTY; reversal::CAPACITY];
            let mut e = Engine::new(&mut records).with_reversals(&mut turns);
            let mut io = Io::default();
            let config = ExperimentConfig::Sinusoidal {
                center_nm: 0,
                amplitude_nm: 1_000_000,
                frequency_mhz: 10_000,
                cycles,
            };
            assert!(config.validate().is_ok());
            start(&mut e, config, u32::MAX - 500, &mut io);
            for elapsed in 1..=cycles * 100 + 3 {
                let now = e.started_ms.wrapping_add(elapsed);
                let (t, p) = feedback(now, e.target, true);
                e.poll(now, &t, p, &mut io);
            }
            assert_eq!(e.outcome, "completed");
            assert_eq!(e.reversals.total, cycles * 2 - 1); // final boundary stops for DPOS return
            let count = e.transferred_reversals();
            assert!(count <= reversal::CAPACITY);
            assert_eq!(count, (cycles * 2 - 1) as usize);
            let mut metadata = String::new();
            e.metadata(&mut metadata, 1).unwrap();
            assert!(metadata.contains(&format!("\"reversal_count\":{count},")));
        }
    }
    fn start(e: &mut Engine<'_>, config: ExperimentConfig, now: u32, io: &mut Io) {
        let (t, p) = feedback(now, 0, true);
        e.accept(config, &t, p, false, now).unwrap();
        e.begin(now, io);
        if let Some(speed) = config.speed() {
            let (mut t, p) = feedback(now, 0, true);
            t.record(b"SSPD", i64::from(speed), now);
            e.poll(now, &t, p, io);
            assert!(!e.speed_pending);
            t.record(b"TOU2", 60, now);
            e.poll(now, &t, p, io);
            assert!(!e.timeout_pending);
        }
        let (t, p) = feedback(now.wrapping_add(1), e.target, true);
        e.poll(now.wrapping_add(1), &t, p, io);
        assert_eq!(e.state, State::Armed);
        let (t, p) = feedback(now.wrapping_add(2), e.target, true);
        e.poll(now.wrapping_add(2), &t, p, io);
    }
    #[test]
    fn validation_rejects_before_motion_and_reserves_worst_case() {
        assert_eq!(sinusoidal().validate(), Ok(11000));
        assert_eq!(
            ExperimentConfig::QuasiStatic {
                speed_um_s: 10,
                start_nm: 0,
                step_nm: -78,
                hold_ms: 100,
                steps: 2
            }
            .validate(),
            Ok(20200)
        );
        for config in [
            ExperimentConfig::Sinusoidal {
                center_nm: 0,
                amplitude_nm: 10000,
                frequency_mhz: 10000,
                cycles: 145,
            },
            ExperimentConfig::QuasiStatic {
                speed_um_s: 10,
                start_nm: 0,
                step_nm: 0,
                hold_ms: 1,
                steps: 1,
            },
            ExperimentConfig::QuasiStatic {
                speed_um_s: 10,
                start_nm: 0,
                step_nm: -78,
                hold_ms: u32::MAX,
                steps: u32::MAX,
            },
            ExperimentConfig::QuasiStatic {
                speed_um_s: 10,
                start_nm: 0,
                step_nm: i64::MAX,
                hold_ms: 1,
                steps: 2,
            },
            ExperimentConfig::Sinusoidal {
                center_nm: i64::MAX,
                amplitude_nm: 78,
                frequency_mhz: 100,
                cycles: 1,
            },
            ExperimentConfig::Sinusoidal {
                center_nm: 0,
                amplitude_nm: 78,
                frequency_mhz: 10_001,
                cycles: 1,
            },
            ExperimentConfig::Sinusoidal {
                center_nm: 0,
                amplitude_nm: 78,
                frequency_mhz: 100,
                cycles: 21,
            },
        ] {
            assert!(config.validate().is_err(), "{config:?}");
        }
        let mut records = [Record::EMPTY; 20];
        let mut e = Engine::new(&mut records);
        let (t, p) = feedback(0, 0, true);
        assert!(e.accept(sinusoidal(), &t, p, false, 0).is_err());
        assert_eq!(e.state, State::Dashboard);
        assert_eq!(e.run, 0);
        assert_eq!(counts(-117), Ok(-2));

        let mut records = [Record::EMPTY; 1024];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        let (mut t, p) = feedback(0, 0, true);
        // Reported hardware status: closed loop + reached, index not known.
        t.record(b"STAT", 1089 | (1 << 12), 0);
        assert_eq!(
            e.accept(sinusoidal(), &t, p, false, 0),
            Err("Xeryon encoder error")
        );
        t.record(b"STAT", 1089, 0);
        e.accept(sinusoidal(), &t, p, false, 0).unwrap();
        e.begin(0, &mut io);
        t.xeryon_motion.as_mut().unwrap().target = Some(-100);
        e.poll(1, &t, p, &mut io);
        e.poll(2, &t, p, &mut io);
        assert_eq!(e.state, State::Running);
        e.abort(&mut io);

        // STOP can leave both closed-loop and reached clear. DPOS re-enters
        // positioning; pre-command feedback must not immediately fault it.
        let mut e = Engine::new(&mut records);
        t.record(b"STAT", 1 | (1 << 5), 0);
        assert_eq!(
            e.accept(sinusoidal(), &t, p, false, 0),
            Err("Stop manual motion before START")
        );
        t.record(b"STAT", 1, 0);
        e.accept(sinusoidal(), &t, p, false, 0).unwrap();
        e.begin(0, &mut io);
        t.xeryon_motion.as_mut().unwrap().state = 1;
        e.poll(1, &t, p, &mut io);
        assert_eq!(e.state, State::MoveToStart);
        t.record(b"STAT", 1089, 2);
        t.xeryon_motion.as_mut().unwrap().state = 2;
        e.poll(2, &t, p, &mut io);
        e.poll(3, &t, p, &mut io);
        assert_eq!(e.state, State::Running);
        t.record(b"STAT", 1, 103);
        e.poll(103, &t, p, &mut io);
        assert_eq!(e.reason, "Xeryon closed loop inactive");
    }
    #[test]
    fn quasi_static_setup_requires_fresh_speed_and_timeout_before_any_motion() {
        let config = ExperimentConfig::QuasiStatic {
            start_nm: 780,
            step_nm: 780,
            hold_ms: 100,
            steps: 1,
            speed_um_s: 10,
        };
        for failure in [
            "none",
            "timeout_disabled",
            "stale",
            "mismatch",
            "late_mismatch",
            "speed_tx",
            "timeout_tx",
            "timeout_missing",
            "timeout_invalid",
            "timeout_short",
            "abort",
        ] {
            let mut records = [Record::EMPTY; CAPACITY];
            let mut e = Engine::new(&mut records);
            let mut io = Io::default();
            let (mut t, p) = feedback(0, 0, true);
            t.record(b"SSPD", 10, 0); // Cached value must not satisfy the new run.
            t.record(b"TOU2", 60, 0);
            // A healthy controller can retain a failed manual tracking result.
            t.xeryon_motion.as_mut().unwrap().state = 4;
            e.accept(config, &t, p, false, 0).unwrap();
            io.fail = failure == "speed_tx";
            e.begin(0, &mut io);
            io.fail = false;
            e.poll(0, &t, p, &mut io);
            assert!(io.targets.is_empty());
            if failure == "abort" {
                e.abort(&mut io);
            }
            if failure != "stale" {
                // Same millisecond is valid when the readback generation advances.
                t.record(b"SSPD", if failure == "mismatch" { 11 } else { 10 }, 0);
            }
            io.busy = true;
            e.poll(0, &t, p, &mut io);
            assert!(io.targets.is_empty()); // Wait for the query transfer too.
            io.busy = false;
            io.fail = failure == "timeout_tx";
            e.poll(0, &t, p, &mut io);
            io.fail = false;
            assert!(io.targets.is_empty());
            e.poll(0, &t, p, &mut io);
            assert!(io.targets.is_empty()); // Cached TOU2 is insufficient.
            if failure == "late_mismatch" {
                t.record(b"SSPD", 11, 0);
            }
            if failure != "timeout_missing" {
                t.record(
                    b"TOU2",
                    match failure {
                        "timeout_invalid" => -1,
                        "timeout_short" => 10,
                        "timeout_disabled" => 0,
                        _ => 60,
                    },
                    0,
                );
            }
            e.poll(0, &t, p, &mut io);
            if matches!(failure, "none" | "timeout_disabled") {
                assert_eq!(io.speeds, [10]);
                assert_eq!(io.targets, [(10, 0, true)]);
                assert_eq!(e.state, State::MoveToStart);
                let mut metadata = String::new();
                e.metadata(&mut metadata, 1).unwrap();
                assert!(metadata.contains("\"speed_um_s\":10"));
            } else {
                let (mut t, p) = feedback(3000, 0, true);
                t.record(b"SSPD", 10, 3000);
                e.poll(3000, &t, p, &mut io);
                assert!(io.targets.is_empty(), "{failure}");
                assert!(e.data_ready(), "{failure}");
                assert_eq!(e.len, 0);
                assert_eq!(io.stops, 1);
                match failure {
                    "stale" => assert_eq!(e.reason, "Xeryon speed readback timeout"),
                    "mismatch" => assert_eq!(e.reason, "Xeryon speed readback mismatch"),
                    "timeout_missing" => assert_eq!(e.reason, "Xeryon TOU2 readback timeout"),
                    "abort" => assert_eq!(e.outcome, "aborted"),
                    _ => assert_eq!(e.outcome, "faulted"),
                }
            }
        }
    }
    #[test]
    fn slow_moves_use_speed_budget_for_start_steps_and_storage_across_clock_wrap() {
        let config = ExperimentConfig::QuasiStatic {
            start_nm: 39_000,
            step_nm: -39_000,
            hold_ms: 50,
            steps: 1,
            speed_um_s: 1,
        };
        assert_eq!(config.step_timeout(), Ok(79_156));
        assert_eq!(config.validate(), Ok(79_206));
        let mut records = [Record::EMPTY; CAPACITY];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        let begin = u32::MAX - 5;
        let (mut t, p) = feedback(begin, 0, true);
        e.accept(config, &t, p, false, begin).unwrap();
        e.begin(begin, &mut io);
        t.record(b"SSPD", 1, begin);
        e.poll(begin, &t, p, &mut io);
        t.record(b"TOU2", 120, begin);
        e.poll(begin, &t, p, &mut io);
        let later = begin.wrapping_add(60_001);
        let (t, p) = feedback(later, e.target, false);
        let mut motion = xeryon_motion::Motion::new();
        motion.start(e.target, begin);
        assert!(!motion.poll_policy(&t, later, true));
        e.poll(later, &t, p, &mut io);
        assert_eq!(e.state, State::MoveToStart);
        let now = begin.wrapping_add(70_000);
        let (t, p) = feedback(now, e.target, true);
        e.poll(now, &t, p, &mut io);
        e.poll(now, &t, p, &mut io);
        assert_eq!(e.state, State::Running);
        assert_eq!(io.targets.last().unwrap().0, 0);
        assert_eq!(e.required_samples, 3961);
        let later = now.wrapping_add(60_001);
        let (t, p) = feedback(later, e.target, false);
        e.poll(later, &t, p, &mut io);
        assert_eq!(e.state, State::Running);
        let deadline = now.wrapping_add(79_156);
        let (t, p) = feedback(deadline, e.target, false);
        e.poll(deadline, &t, p, &mut io);
        assert_eq!(e.reason, "Step move deadline");
        assert_eq!(io.stops, 1);
        assert!(config.move_timeout(200_000).is_err());
        assert!(
            ExperimentConfig::QuasiStatic {
                start_nm: 39_000,
                step_nm: -39_000,
                hold_ms: 50,
                steps: 3,
                speed_um_s: 1,
            }
            .validate()
            .is_err()
        );
        for speed_um_s in [0, MAX_SPEED_UM_S + 1] {
            assert!(
                ExperimentConfig::QuasiStatic {
                    start_nm: 0,
                    step_nm: 78,
                    hold_ms: 50,
                    steps: 1,
                    speed_um_s,
                }
                .validate()
                .is_err()
            );
        }
    }
    #[test]
    fn quasi_static_records_motion_and_holds_and_uses_confirmed_reach() {
        let mut records = [Record::EMPTY; 2048];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        start(
            &mut e,
            ExperimentConfig::QuasiStatic {
                speed_um_s: 10,
                start_nm: 0,
                step_nm: -780,
                hold_ms: 100,
                steps: 2,
            },
            0,
            &mut io,
        );
        assert_eq!(io.targets.last().unwrap().0, -10);
        for now in 3..=502 {
            let reached = now >= 102 && (e.step == 1 || now >= 302);
            let (t, p) = feedback(now, e.target, reached);
            e.poll(now, &t, p, &mut io);
            if e.data_ready() {
                break;
            }
        }
        assert_eq!(e.outcome, "completed");
        assert_eq!(io.stops, 1);
        assert_eq!(
            io.targets.iter().map(|v| v.0).collect::<Vec<_>>(),
            [0, -10, -20]
        );
        assert_eq!(e.elapsed_ms, 400);
        assert_eq!(e.len, 21);
        assert!(
            e.samples[..e.len]
                .windows(2)
                .all(|pair| pair[1].elapsed_ms - pair[0].elapsed_ms == 20)
        );
        assert!(e.samples[..e.len].iter().any(|r| r.stat & (1 << 10) == 0));
        assert!(io.scans.is_empty());
    }
    #[test]
    fn sine_uses_absolute_time_wraps_clock_and_finishes_without_pc() {
        let mut records = [Record::EMPTY; 1024];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        let begin = u32::MAX - 500;
        start(&mut e, sinusoidal(), begin, &mut io);
        for elapsed in 1..=1003 {
            let now = e.started_ms.wrapping_add(elapsed);
            let (t, p) = feedback(now, e.target, true);
            e.poll(now, &t, p, &mut io);
        }
        assert_eq!(e.outcome, "completed");
        assert_eq!(e.len, 51);
        assert_eq!(io.targets.len(), 2); // initial extremum and final center only
        for record in &e.samples[..e.len] {
            let elapsed = record.elapsed_ms;
            let expected = (-100.0 * (f64::from(elapsed) * core::f64::consts::TAU / 1000.0).cos())
                .round() as i32;
            assert_eq!(record.target, expected);
        }
        assert_eq!(io.targets.last().unwrap().0, 0);
        assert!(io.targets.last().unwrap().2);
        assert_eq!(e.samples[50].elapsed_ms, 1000);
        assert!(
            e.samples[..e.len]
                .windows(2)
                .all(|pair| pair[1].elapsed_ms - pair[0].elapsed_ms == 20)
        );
        assert_eq!(e.missed_samples, 0);
        assert_eq!(io.stops, 2); // waveform boundary, then terminal cleanup
        assert_eq!(io.events[..3], ["DPOS=-100\n", "SSPD=0\n", "SCAN=1\n"]);
        for i in 0..10000 {
            let phase = i as f64 / 10000.0;
            assert!((sine(phase) - (phase * core::f64::consts::TAU).sin()).abs() < 7e-10);
        }
    }
    #[test]
    fn extremum_initialization_matches_position_velocity_and_direction() {
        let center = 15_600;
        let amplitude = 7_800;
        let peak = sine_max_velocity(amplitude, 1000);
        for (ms, position, velocity, direction) in [
            (0, 100, 0.0, 1),
            (250, 200, peak, 1),
            (500, 300, 0.0, -1),
            (750, 200, -peak, -1),
            (1000, 100, 0.0, 1),
        ] {
            assert_eq!(sine_target(center, amplitude, 1000, ms), position);
            assert!((sine_velocity(amplitude, 1000, ms) - velocity).abs() < 1e-8);
            assert_eq!(sine_direction(1000, ms), direction);
        }
        assert!(sine_velocity(amplitude, 1000, 1) > 0.0);
        assert_eq!(sine_velocity(amplitude, 1000, 0), 0.0);
        for frequency in [100, 333, 1000, 1300, 10_000] {
            let end = 1_000_000u32.div_ceil(frequency);
            let mut previous = sine_direction(frequency, 0);
            let mut crossings = Vec::new();
            for ms in 1..=end {
                let direction = sine_direction(frequency, ms);
                if direction != previous {
                    crossings.push(ms);
                }
                previous = direction;
            }
            // Two mathematical reversals per complete cycle, excluding t=0.
            // At the final one the finite experiment stops for the DPOS return.
            assert_eq!(crossings, [500_000u32.div_ceil(frequency), end]);
        }
    }

    #[test]
    fn both_sine_extrema_are_validated_before_motion() {
        for center_nm in [
            i64::from(MIN_POSITION) * 78,
            i64::from(MAX_POSITION) * 78,
            i64::MIN,
        ] {
            let mut records = [Record::EMPTY; 1024];
            let mut e = Engine::new(&mut records);
            let config = ExperimentConfig::Sinusoidal {
                center_nm,
                amplitude_nm: 7800,
                frequency_mhz: 1000,
                cycles: 1,
            };
            let (t, p) = feedback(0, 0, true);
            assert!(e.accept(config, &t, p, false, 0).is_err());
            assert_eq!(e.state, State::Dashboard);
            assert!(!e.owns_hardware());
            assert_eq!(e.run, 0);
        }
    }

    #[test]
    fn move_to_negative_extremum_requires_reached_and_idle_before_zero_time() {
        use xeryon_motion::ScanCommand::*;
        for begin in [0, u32::MAX - 50] {
            let mut records = [Record::EMPTY; 1024];
            let mut e = Engine::new(&mut records);
            let mut io = Io::default();
            let (t, p) = feedback(begin, 0, true);
            e.accept(sinusoidal(), &t, p, false, begin).unwrap();
            e.begin(begin, &mut io);
            assert_eq!(io.targets, [(-100, begin, true)]);
            for elapsed in 1..100 {
                let now = begin.wrapping_add(elapsed);
                let (t, p) = feedback(now, -100, false);
                e.poll(now, &t, p, &mut io);
                assert_eq!(e.state, State::MoveToStart);
                assert!(io.scans.is_empty());
                assert_eq!(e.len, 0);
            }
            let now = begin.wrapping_add(100);
            let (t, p) = feedback(now, -100, true);
            io.busy = true;
            e.poll(now, &t, p, &mut io);
            assert_eq!(e.state, State::Armed);
            e.poll(now.wrapping_add(1), &t, p, &mut io);
            assert_eq!(e.state, State::Armed);
            assert!(io.scans.is_empty());
            io.busy = false;
            e.poll(now.wrapping_add(2), &t, p, &mut io);
            assert_eq!(e.started_ms, now.wrapping_add(2));
            assert_eq!(e.elapsed_ms, 0);
            assert_eq!(e.samples[0].target, -100);
            assert_eq!(io.scans, [Speed(0), Direction(1)]);
            assert_eq!(e.reversals.total, 0);
            assert_eq!(e.scheduling.reversal_lateness.count, 0);
        }
    }

    #[test]
    fn velocity_equation_cardinal_phases_and_peak_speed_validation() {
        let vmax = sine_max_velocity(1_000_000, 1000);
        assert!((vmax - core::f64::consts::TAU * 1000.0).abs() < 1e-9);
        for (ms, fraction) in [(0, 0.0), (250, 1.0), (500, 0.0), (750, -1.0), (1000, 0.0)] {
            assert!((sine_velocity(1_000_000, 1000, ms) - vmax * fraction).abs() < 1e-8);
        }
        let boundary =
            (f64::from(MAX_SPEED_UM_S) * 1_000_000.0 / (core::f64::consts::TAU * 2000.0)) as u32;
        let config = |amplitude_nm| ExperimentConfig::Sinusoidal {
            center_nm: 0,
            amplitude_nm,
            frequency_mhz: 2000,
            cycles: 1,
        };
        assert!(config(boundary).validate().is_ok());
        // Endpoints still fit: this fails specifically because of peak speed.
        assert!(counts(i64::from(boundary + 1)).is_ok());
        assert_eq!(
            config(boundary + 1).validate(),
            Err("Sinusoidal peak speed is outside representable SSPD range")
        );
        let mut records = [Record::EMPTY; CAPACITY];
        let mut e = Engine::new(&mut records);
        let (t, p) = feedback(0, 0, true);
        assert!(e.accept(config(boundary + 1), &t, p, false, 0).is_err());
        assert_eq!(e.state, State::Dashboard);
        assert!(
            ExperimentConfig::Sinusoidal {
                center_nm: 0,
                amplitude_nm: 78,
                frequency_mhz: 100,
                cycles: 1,
            }
            .validate()
            .is_err()
        ); // nonzero peak must round to at least 1 um/s
    }
    #[test]
    fn adaptive_jitter_uses_actual_time_and_crossings_override_small_speed_changes() {
        for amplitude_nm in [800, 1_000_000] {
            let mut records = [Record::EMPTY; CAPACITY];
            let mut e = Engine::new(&mut records);
            e.scheduling.mode = scheduling::Mode::Adaptive;
            let mut io = Io::default();
            start(
                &mut e,
                ExperimentConfig::Sinusoidal {
                    center_nm: 0,
                    amplitude_nm,
                    frequency_mhz: 1300,
                    cycles: 1,
                },
                u32::MAX - 100,
                &mut io,
            );
            let next = e.next_target;
            let actual = next + 3;
            let now = e.started_ms.wrapping_add(actual);
            let (t, p) = feedback(now, 999, true);
            e.poll(now, &t, p, &mut io);
            assert_eq!(
                e.scheduling.held,
                Some(f64::from(velocity_speed(sine_velocity(
                    amplitude_nm,
                    1300,
                    actual
                ))))
            );
            assert_eq!(e.scheduling.lateness.count, 2);
            for elapsed in actual + 1..=385 {
                let now = e.started_ms.wrapping_add(elapsed);
                let (t, p) = feedback(now, -123, true);
                e.poll(now, &t, p, &mut io);
                assert_eq!(e.state, State::Running);
            }
            // Zero is accepted at the crossing; direction waits for the next
            // idle observation, even with this instantly-completing test IO.
            assert_eq!(io.scans.last(), Some(&xeryon_motion::ScanCommand::Speed(0)));
            let now = e.started_ms.wrapping_add(386);
            let (t, p) = feedback(now, -123, true);
            e.poll(now, &t, p, &mut io);
            assert_eq!(
                &io.scans[io.scans.len() - 2..],
                &[
                    xeryon_motion::ScanCommand::Speed(0),
                    xeryon_motion::ScanCommand::Direction(-1)
                ]
            );
            assert_eq!(e.scheduling.reversal_lateness.count, 1);
            assert_eq!(
                e.next_target,
                scheduling::next(scheduling::Mode::Adaptive, amplitude_nm, 1300, 385, 0.0)
            );
            assert_eq!(io.targets.len(), 1); // no waveform DPOS despite arbitrary EPOS
            e.abort(&mut io);
            assert_eq!(io.events.last().unwrap(), "STOP\n");
        }
    }

    fn waveform_poll(e: &mut Engine<'_>, io: &mut Io, elapsed: u32) {
        let now = e.started_ms.wrapping_add(elapsed);
        io.clock = now;
        let (t, p) = feedback(now, 123, true);
        e.poll(now, &t, p, io);
    }

    #[test]
    fn completion_identity_rejects_old_future_and_duplicate_progress_at_each_stage() {
        for stage in 0..3 {
            for wrong in [0, 2] {
                let mut records = [Record::EMPTY; 1024];
                let mut e = Engine::new(&mut records);
                let mut io = Io::default();
                start(&mut e, sinusoidal(), 0, &mut io);
                for elapsed in 1..500 {
                    waveform_poll(&mut e, &mut io, elapsed);
                }
                io.tracked = true;
                waveform_poll(&mut e, &mut io, 500);
                for i in 0..stage {
                    io.complete_one(e.started_ms + 501 + i);
                    waveform_poll(&mut e, &mut io, 501 + i);
                }
                let sent = io.scans.len();
                let expected = io.progress[0];
                io.progress[1] = expected - 1 + wrong;
                io.busy = false; // An idle flag alone is not proof of this command.
                waveform_poll(&mut e, &mut io, 504 + stage);
                assert_eq!(e.reason, "Xeryon command completion identity mismatch");
                assert_eq!(io.scans.len(), sent);
                assert!(e.stop_queued && e.len > 0);
            }
        }
    }

    #[test]
    fn abort_disconnect_and_epoch_change_preempt_need_zero_and_every_wait() {
        for stage in 0..4 {
            for cause in 0..3 {
                let mut records = [Record::EMPTY; 1024];
                let mut e = Engine::new(&mut records);
                let mut io = Io::default();
                start(&mut e, sinusoidal(), 0, &mut io);
                for elapsed in 1..500 {
                    waveform_poll(&mut e, &mut io, elapsed);
                }
                io.busy = stage == 0;
                waveform_poll(&mut e, &mut io, 500);
                for i in 1..stage {
                    waveform_poll(&mut e, &mut io, 500 + i);
                }
                let sent = io.scans.len();
                let now = e.started_ms + 504;
                let (mut t, p) = feedback(now, 123, true);
                match cause {
                    0 => e.abort(&mut io),
                    1 => t.mounted = false,
                    _ => t.epoch += 1,
                }
                e.poll(now, &t, p, &mut io);
                assert!(e.data_ready() && e.stop_queued);
                io.busy = false;
                waveform_poll(&mut e, &mut io, 505);
                assert_eq!(io.scans.len(), sent);
                assert_eq!(io.stops, 1);
            }
        }
    }

    #[test]
    fn envelope_is_fault_only_fresh_new_generation_and_retains_trigger() {
        for stale in [false, true] {
            let mut records = [Record::EMPTY; 1024];
            let mut e = Engine::new(&mut records);
            e.envelope.margin_nm = Some(1000);
            e.envelope.max_receipt_age_ms = 25;
            let mut io = Io::default();
            start(&mut e, sinusoidal(), u32::MAX - 100, &mut io);
            let now = e.started_ms.wrapping_add(30);
            let (mut t, p) = feedback(now, 500, true);
            // Duplicate cache, even outside the envelope, never triggers.
            e.next_sample = 30;
            e.next_target = 30;
            e.poll(now, &t, p, &mut io);
            assert_eq!(e.state, State::Running);
            t.epos_generation += 1;
            if stale {
                t.samples[0].ms = now.wrapping_sub(26);
            }
            e.poll(now, &t, p, &mut io);
            if stale {
                assert_eq!(e.state, State::Running);
                assert!(e.envelope.violation.is_none());
            } else {
                assert_eq!(e.reason, "Sinusoidal trajectory envelope exceeded");
                assert_eq!(e.outcome, "faulted");
                assert!(e.stop_queued && e.len > 0);
                assert_eq!(e.envelope.violation, Some((500, now, t.epos_generation)));
                let sent = io.scans.len();
                waveform_poll(&mut e, &mut io, 31);
                assert_eq!(io.scans.len(), sent);
                assert_eq!(io.targets.len(), 1); // never a corrective DPOS
            }
        }
    }

    #[test]
    fn busy_updates_coalesce_and_send_current_time_without_phase_drift_or_fifo() {
        use xeryon_motion::ScanCommand::Speed;
        for begin in [0, u32::MAX - 230] {
            let mut records = [Record::EMPTY; 1024];
            let mut e = Engine::new(&mut records);
            e.scheduling.mode = scheduling::Mode::Adaptive;
            let mut io = Io::default();
            start(
                &mut e,
                ExperimentConfig::Sinusoidal {
                    center_nm: 0,
                    amplitude_nm: 1_000_000,
                    frequency_mhz: 2000,
                    cycles: 2,
                },
                begin,
                &mut io,
            );
            for elapsed in 1..225 {
                waveform_poll(&mut e, &mut io, elapsed);
            }
            e.next_target = 225;
            let before = io.scans.len();
            io.busy = true;
            for elapsed in 225..234 {
                waveform_poll(&mut e, &mut io, elapsed);
                assert_eq!(e.state, State::Running);
                assert_eq!(io.scans.len(), before); // no transport FIFO submissions
            }
            assert!(e.scheduling.busy_due >= 2);
            assert!(e.scheduling.max_coalesced >= 2);
            io.busy = false;
            waveform_poll(&mut e, &mut io, 234); // not a historical evaluation time
            assert_eq!(io.scans.len(), before + 1);
            assert_eq!(
                io.scans.last(),
                Some(&Speed(velocity_speed(sine_velocity(1_000_000, 2000, 234))))
            );
            assert_ne!(
                io.scans.last(),
                Some(&Speed(velocity_speed(sine_velocity(1_000_000, 2000, 233))))
            );
            waveform_poll(&mut e, &mut io, 235);
            assert_eq!(io.scans.len(), before + 1); // no replay of 225/229/233
            for elapsed in 236..=250 {
                waveform_poll(&mut e, &mut io, elapsed);
            }
            assert_eq!(io.scans.last(), Some(&Speed(0))); // original absolute crossing
            assert_eq!(e.scheduling.reversal_lateness.count, 1);
            assert_eq!(e.started_ms, begin.wrapping_add(2));
            assert_eq!(io.targets.len(), 1); // waveform never uses DPOS
            assert_eq!(e.missed_samples, 0);
        }
    }

    #[test]
    fn sustained_slow_usb_bounds_queue_and_preserves_feedback_and_completion_metrics() {
        let mut records = [Record::EMPTY; 1024];
        let mut turns = [reversal::Reversal::EMPTY; 4];
        let mut e = Engine::new(&mut records).with_reversals(&mut turns);
        let mut io = Io::default();
        start(
            &mut e,
            ExperimentConfig::Sinusoidal {
                center_nm: 0,
                amplitude_nm: 1_000_000,
                frequency_mhz: 2000,
                cycles: 2,
            },
            u32::MAX - 105,
            &mut io,
        );
        io.tracked = true;
        let (mut t, mut p) = feedback(e.started_ms, 0, true);
        t.timing_active = true;
        for elapsed in 1..1000 {
            let now = e.started_ms.wrapping_add(elapsed);
            io.clock = now;
            if io.busy && now.wrapping_sub(io.progress[3]) >= 9 {
                io.complete_one(now);
            }
            if elapsed % 20 == 0 {
                t.record(b"EPOS", i64::from(elapsed), now);
                t.record(b"STAT", (1 << 6) | (1 << 13), now);
            }
            p.ms = now;
            e.poll(now, &t, p, &mut io);
            assert_eq!(e.state, State::Running, "{} at {elapsed}", e.reason);
            assert!(io.progress[0] - io.progress[1] <= 1);
        }
        assert!(e.scheduling.busy_due > 20);
        assert_eq!(e.scheduling.backpressure_faults, 0);
        assert_eq!(e.scheduling.usb_faults, 0);
        assert_eq!(e.scheduling.queue_peak, 1);
        assert_eq!(e.scheduling.reversal_resume.count, 3);
        assert_eq!(e.missed_samples, 0);
        assert_eq!(e.epos_arrivals.count, 49);
        assert_eq!(e.max_epos_age, 19);
        let mut meta = String::new();
        e.metadata(&mut meta, 1).unwrap();
        assert!(
            meta.contains("\"min\":9,\"mean\":9.000,\"max\":9"),
            "{meta}"
        );
        // All three independent direction and resume packets complete in order.
        for turn in &e.reversals.records[..3] {
            let mut text = String::new();
            turn.write(&mut text).unwrap();
            assert!(!text.contains("null"), "{text}");
        }
    }

    #[test]
    fn reversal_waits_for_each_completion_and_resumes_actual_time() {
        use xeryon_motion::ScanCommand::*;
        let mut records = [Record::EMPTY; 1024];
        let mut turns = [reversal::Reversal::EMPTY; 4];
        let mut e = Engine::new(&mut records).with_reversals(&mut turns);
        let mut io = Io::default();
        start(&mut e, sinusoidal(), 0, &mut io);
        for elapsed in 1..500 {
            waveform_poll(&mut e, &mut io, elapsed);
        }
        let before = io.scans.len();
        io.busy = true; // ordinary command still completing at the crossing
        for elapsed in 500..505 {
            waveform_poll(&mut e, &mut io, elapsed);
        }
        assert_eq!(io.scans.len(), before);
        io.busy = false;
        waveform_poll(&mut e, &mut io, 505);
        assert_eq!(&io.scans[before..], &[Speed(0)]);
        io.busy = true;
        for elapsed in 506..513 {
            waveform_poll(&mut e, &mut io, elapsed);
        }
        assert_eq!(&io.scans[before..], &[Speed(0)]); // zero cannot disappear
        io.busy = false;
        waveform_poll(&mut e, &mut io, 513);
        assert_eq!(&io.scans[before..], &[Speed(0), Direction(-1)]);
        io.busy = true;
        for elapsed in 514..521 {
            waveform_poll(&mut e, &mut io, elapsed);
        }
        assert_eq!(&io.scans[before..], &[Speed(0), Direction(-1)]);
        io.busy = false;
        waveform_poll(&mut e, &mut io, 521);
        assert_eq!(
            &io.scans[before..],
            &[
                Speed(0),
                Direction(-1),
                Speed(velocity_speed(sine_velocity(7800, 1000, 521)))
            ]
        );
        waveform_poll(&mut e, &mut io, 522);
        assert_eq!(e.reversals.total, 1);
        assert_eq!(e.scheduling.reversal_zero.count, 1);
        assert_eq!(e.scheduling.reversal_direction.count, 1);
        assert_eq!(e.scheduling.reversal_resume.count, 1);
        let mut text = String::new();
        e.reversals.records[0].write(&mut text).unwrap();
        assert!(
            text.contains("[1,0,505,null],[2,-1,513,null],[1,6,521,null]"),
            "{text}"
        );
        assert_eq!(e.state, State::Running);
    }

    #[test]
    fn ordinary_and_reversal_backpressure_have_independent_safe_deadlines() {
        for reversal in [false, true] {
            let mut records = [Record::EMPTY; 1024];
            let mut e = Engine::new(&mut records);
            let mut io = Io::default();
            start(&mut e, sinusoidal(), u32::MAX - 520, &mut io);
            let start_busy = if reversal { 500 } else { 50 };
            for elapsed in 1..start_busy {
                waveform_poll(&mut e, &mut io, elapsed);
            }
            let mut fault_at = 0;
            for elapsed in start_busy..=start_busy + 40 {
                // Let zero complete partway through. Per-transfer busy durations
                // stay under 40 ms; the whole reversal must still hit its bound.
                io.busy = !(reversal && (elapsed == 500 || elapsed == 520));
                waveform_poll(&mut e, &mut io, elapsed);
                if e.data_ready() {
                    fault_at = elapsed;
                    break;
                }
            }
            assert_ne!(fault_at, 0);
            if reversal {
                assert_eq!(fault_at, 540);
                assert_eq!(e.reason, "Xeryon reversal backpressure deadline");
            } else {
                assert_eq!(e.reason, "Xeryon excessive transmit backpressure");
            }
            assert_eq!(e.scheduling.backpressure_faults, 1);
            assert_eq!(io.events.last().unwrap(), "STOP\n");
            assert!(e.stop_queued && e.len > 1);
        }
    }

    #[test]
    fn busy_deadline_uses_actual_completion_and_faults_at_exact_bound() {
        for delay in [39, 40] {
            let mut records = [Record::EMPTY; 1024];
            let mut e = Engine::new(&mut records);
            let mut io = Io::default();
            start(
                &mut e,
                ExperimentConfig::Sinusoidal {
                    center_nm: 0,
                    amplitude_nm: 1_000_000,
                    frequency_mhz: 1000,
                    cycles: 1,
                },
                u32::MAX - 70,
                &mut io,
            );
            for elapsed in 1..50 {
                waveform_poll(&mut e, &mut io, elapsed);
            }
            // Real command progress (not FIFO space) establishes completion.
            io.tracked = true;
            e.next_target = 50;
            waveform_poll(&mut e, &mut io, 50);
            assert!(io.busy);
            for elapsed in 51..50 + delay {
                waveform_poll(&mut e, &mut io, elapsed);
            }
            io.complete_one(e.started_ms.wrapping_add(50 + delay));
            waveform_poll(&mut e, &mut io, 50 + delay);
            if delay == 39 {
                assert_eq!(e.state, State::Running);
                assert_eq!(e.scheduling.backpressure_faults, 0);
            } else {
                assert_eq!(e.reason, "Xeryon excessive transmit backpressure");
                assert_eq!(io.stops, 1);
            }
        }
    }

    #[test]
    fn abort_and_real_usb_failure_override_every_reversal_stage_and_ordinary_tx() {
        for phase in [100, 500, 501, 502] {
            for abort in [false, true] {
                let mut records = [Record::EMPTY; 1024];
                let mut e = Engine::new(&mut records);
                let mut io = Io::default();
                start(&mut e, sinusoidal(), 0, &mut io);
                for elapsed in 1..=phase {
                    waveform_poll(&mut e, &mut io, elapsed);
                }
                io.busy = true;
                let sent = io.scans.len();
                if abort {
                    e.abort(&mut io);
                } else {
                    io.usb_failed = true;
                    waveform_poll(&mut e, &mut io, phase + 1);
                    assert_eq!(e.reason, "Xeryon USB transfer failed");
                    assert_eq!(e.scheduling.usb_faults, 1);
                }
                assert!(e.data_ready() && e.stop_queued);
                assert_eq!(io.events.last().unwrap(), "STOP\n");
                io.busy = false;
                waveform_poll(&mut e, &mut io, phase + 2);
                assert_eq!(io.scans.len(), sent);
                assert_eq!(io.stops, 1);
            }
        }
    }

    #[test]
    fn waveform_completion_preempts_pending_velocity_with_stop() {
        let mut records = [Record::EMPTY; 1024];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        start(&mut e, sinusoidal(), 0, &mut io);
        for elapsed in 1..990 {
            waveform_poll(&mut e, &mut io, elapsed);
        }
        io.busy = true;
        let sent = io.scans.len();
        for elapsed in 990..=1000 {
            waveform_poll(&mut e, &mut io, elapsed);
        }
        assert_eq!(e.center_move, CenterMove::Stopping);
        assert_eq!(io.scans.len(), sent);
        assert_eq!(io.events.last().unwrap(), "STOP\n");
        assert_eq!(io.stops, 1);
        for elapsed in 1001..1010 {
            waveform_poll(&mut e, &mut io, elapsed);
        }
        assert_eq!(io.targets.len(), 1); // return DPOS cannot overtake STOP
        assert_eq!(io.scans.len(), sent);
    }

    #[test]
    fn final_return_real_tracker_accepts_controller_tolerance_without_exact_epos() {
        // Include PTOL/PTO2 edges, both signs and tick rollover. Firmware never
        // reimplements the controller's tolerance decision from EPOS.
        for begin in [0, u32::MAX - 1050] {
            for error in [-10, -2, -1, 0, 1, 2, 10] {
                let mut records = [Record::EMPTY; 1024];
                let mut e = Engine::new(&mut records);
                let mut io = Io::default();
                start(&mut e, sinusoidal(), begin, &mut io);
                for elapsed in 1..=1002 {
                    waveform_poll(&mut e, &mut io, elapsed);
                }
                let sent = io.targets.last().unwrap().1;
                let (mut t, mut p) = feedback(sent, error, false);
                let mut tracker = xeryon_motion::Motion::new();
                tracker.start(0, sent);
                // Cached reached and matching echo belong to the previous move.
                t.record(b"DPOS", 0, sent.wrapping_sub(1));
                t.record(b"STAT", 1344, sent.wrapping_sub(1));
                for offset in 1..=5 {
                    let now = sent.wrapping_add(offset);
                    t.record(b"EPOS", i64::from(error), now);
                    p.ms = now;
                    match offset {
                        2 => {
                            // Fresh reached but wrong echo cannot complete.
                            t.record(b"DPOS", 1, now);
                            t.record(b"STAT", 1344, now);
                        }
                        3 => t.record(b"DPOS", 0, now), // correct echo, old STAT
                        4 => t.record(b"STAT", 1344, sent.wrapping_add(3)), // same tick
                        5 => t.record(b"STAT", 1344, now), // subsequent STAT
                        _ => {}
                    }
                    assert!(!tracker.poll_policy(&t, now, true));
                    t.xeryon_motion = Some(tracker);
                    e.poll(now, &t, p, &mut io);
                    assert_eq!(e.data_ready(), offset == 5);
                }
                assert_eq!(e.outcome, "completed");
                assert_eq!(e.final_return.echo.value, 0);
                assert_eq!(e.final_return.echo.ms, sent.wrapping_add(3));
                assert_eq!(e.final_return.last[0].value, i64::from(error));
                assert_eq!(e.final_return.finished, Some(sent.wrapping_add(5)));
                assert!(!e.final_return.expired);
                let mut json = String::new();
                e.metadata(&mut json, 1).unwrap();
                assert!(json.contains(&format!("\"error_counts\":{error},")));
            }
        }
    }

    #[test]
    fn final_return_captures_stop_drain_dpos_identity_and_between_tick_completion() {
        let mut records = [Record::EMPTY; 1024];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        start(&mut e, sinusoidal(), 0, &mut io);
        for elapsed in 1..1000 {
            waveform_poll(&mut e, &mut io, elapsed);
        }
        io.tracked = true;
        io.clock = e.started_ms + 1000;
        waveform_poll(&mut e, &mut io, 1000);
        assert_eq!(e.final_return.theoretical_counts, -100);
        assert!(sine_velocity(7800, 1000, 1000).abs() < 1e-9);
        assert_eq!(e.final_return.stop.queued, Some(io.clock));
        waveform_poll(&mut e, &mut io, 1001);
        assert_eq!(io.targets.len(), 1);
        io.complete_one(e.started_ms + 1002);
        io.clock = e.started_ms + 1002;
        waveform_poll(&mut e, &mut io, 1002); // SSPD return speed
        assert_eq!(e.final_return.stop.completed, Some(e.started_ms + 1002));
        assert_eq!(io.targets.len(), 1);
        waveform_poll(&mut e, &mut io, 1003);
        assert_eq!(io.targets.len(), 1);
        io.complete_one(e.started_ms + 1004);
        io.clock = e.started_ms + 1004;
        waveform_poll(&mut e, &mut io, 1004); // DPOS accepted exactly once
        let events = io.events.len();
        waveform_poll(&mut e, &mut io, 1005);
        assert_eq!(io.events.len(), events);
        assert_eq!(io.targets.len(), 2);
        assert_eq!(e.final_return.dpos.queued, Some(e.started_ms + 1004));
        assert!(!e.data_ready()); // even a reached test snapshot cannot beat TX drain
        io.complete_one(e.started_ms + 1006);
        let now = e.started_ms + 1006;
        let (t, p) = feedback(now, 0, true);
        e.poll(now, &t, p, &mut io);
        assert_eq!(e.outcome, "completed");
        assert_eq!(e.final_return.dpos.completed, Some(e.started_ms + 1006));
        assert_eq!(e.final_return.finished, Some(e.started_ms + 1006));
        assert_eq!(e.final_return.last[2].ms, e.started_ms + 1006);
        assert_eq!(io.targets.len(), 2);
        assert!(io.events[events..].iter().all(|event| event == "STOP\n"));
    }

    #[test]
    fn final_return_speed_is_reasserted_even_if_stop_discarded_cached_speed() {
        let mut records = [Record::EMPTY; 1024];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        start(&mut e, sinusoidal(), 0, &mut io);
        for elapsed in 1..=1000 {
            waveform_poll(&mut e, &mut io, elapsed);
        }
        let speed = xeryon_motion::ScanCommand::Speed(49);
        e.scan.commit(speed); // accepted before STOP; execution is not established
        let sent = io.scans.len();
        waveform_poll(&mut e, &mut io, 1001);
        assert_eq!(io.scans.len(), sent + 1);
        assert_eq!(io.scans.last(), Some(&speed));
        assert_eq!(io.targets.len(), 1);
        waveform_poll(&mut e, &mut io, 1002);
        assert_eq!(io.targets.len(), 2);
    }

    #[test]
    fn fixed_jitter_uses_actual_velocity_and_logs_current_theoretical_position() {
        use xeryon_motion::ScanCommand::*;
        let mut records = [Record::EMPTY; 1024];
        let mut e = Engine::new(&mut records);
        e.scheduling.mode = scheduling::Mode::Fixed100Hz;
        let mut io = Io::default();
        let config = ExperimentConfig::Sinusoidal {
            center_nm: -780,
            amplitude_nm: 1_000_000,
            frequency_mhz: 1000,
            cycles: 1,
        };
        start(&mut e, config, 0, &mut io);
        assert_eq!(io.scans, [Speed(0), Direction(1)]);
        for elapsed in [13, 29, 31, 43, 53] {
            let now = e.started_ms + elapsed;
            let (mut t, p) = feedback(now, 12345, false); // no EPOS correction
            t.record(b"STAT", (1 << 6) | (1 << 13), now);
            e.poll(now, &t, p, &mut io);
            let expected = (core::f64::consts::TAU
                * 1000.0
                * (core::f64::consts::TAU * f64::from(elapsed) / 1000.0).sin())
            .round() as u32;
            assert_eq!(io.scans.last(), Some(&Speed(expected)));
            assert_eq!(e.state, State::Running);
        }
        assert_eq!(io.targets.len(), 1);
        assert_eq!(e.len, 3);
        assert_eq!(e.samples[1].elapsed_ms, 29);
        assert_eq!(e.samples[2].elapsed_ms, 43);
        for record in &e.samples[1..e.len] {
            assert_eq!(
                record.target,
                sine_target(-780, 1_000_000, 1000, record.elapsed_ms)
            );
            assert_eq!(record.epos, 12345);
        }
        assert_eq!(e.max_lateness_ms, 9);
    }
    #[test]
    fn exact_cycles_stop_then_drain_then_return_and_confirm_center() {
        let mut records = [Record::EMPTY; CAPACITY];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        start(
            &mut e,
            ExperimentConfig::Sinusoidal {
                center_nm: 780,
                amplitude_nm: 1_000_000,
                frequency_mhz: 1300,
                cycles: 2,
            },
            0,
            &mut io,
        );
        let boundary = 1539; // ceil(2 / 1.3 seconds), first representable ms
        for elapsed in 1..boundary {
            let now = e.started_ms + elapsed;
            let (t, p) = feedback(now, 10, true);
            e.poll(now, &t, p, &mut io);
            assert_eq!(io.stops, 0);
            assert_eq!(io.targets.len(), 1);
        }
        let now = e.started_ms + boundary;
        let (mut t, mut p) = feedback(now, 10, true);
        e.poll(now, &t, p, &mut io);
        assert_eq!(io.stops, 1);
        assert_eq!(io.events.last().unwrap(), "STOP\n");
        assert_eq!(e.state, State::Running);
        io.busy = true;
        t.record(b"STAT", 0, now); // STOP may leave closed loop inactive
        e.poll(now + 1, &t, p, &mut io);
        assert_eq!(io.targets.len(), 1);
        io.busy = false;
        e.poll(now + 2, &t, p, &mut io);
        assert_eq!(
            io.scans.last(),
            Some(&xeryon_motion::ScanCommand::Speed(8168))
        );
        io.busy = true;
        e.poll(now + 3, &t, p, &mut io);
        assert_eq!(io.targets.len(), 1);
        io.busy = false;
        e.poll(now + 4, &t, p, &mut io);
        assert_eq!(io.targets.last(), Some(&(10, now + 4, true)));
        t.xeryon_motion.as_mut().unwrap().state = 1;
        t.record(b"STAT", 1 << 6, now + 5);
        e.poll(now + 5, &t, p, &mut io);
        assert!(!e.data_ready());
        t.xeryon_motion.as_mut().unwrap().state = 2;
        t.xeryon_motion.as_mut().unwrap().target = Some(11);
        e.poll(now + 6, &t, p, &mut io);
        assert!(!e.data_ready());
        t.xeryon_motion.as_mut().unwrap().target = Some(10);
        t.record(b"STAT", (1 << 6) | (1 << 10), now + 7);
        p.ms = now + 7;
        e.poll(now + 7, &t, p, &mut io);
        assert_eq!(e.outcome, "completed");
        assert_eq!(io.stops, 2);
        assert_eq!(io.targets.len(), 2);
    }

    #[test]
    fn scan_status_is_scoped_and_mode_feedback_grace_is_bounded() {
        let (mut t, p) = feedback(0, 0, true);
        t.record(b"STAT", (1 << 6) | (1 << 13), 0);
        assert!(Engine::healthy(&t, p, 0, true, false).is_err());
        assert!(Engine::healthy(&t, p, 0, true, true).is_ok());
        for bit in [1, 2, 3, 9, 12, 14, 15, 16, 18, 21] {
            t.record(b"STAT", (1 << 6) | (1 << 13) | (1 << bit), 0);
            assert!(Engine::healthy(&t, p, 0, false, true).is_err());
        }
        let mut records = [Record::EMPTY; 1024];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        start(&mut e, sinusoidal(), 0, &mut io);
        for elapsed in 1..=501 {
            let now = e.started_ms + elapsed;
            let (mut t, p) = feedback(now, e.target, true);
            // A zero-speed reversal stays in SCAN: it must not renew the
            // initial mode-entry grace or suppress closed-loop safety.
            let stat = if elapsed >= 501 {
                0
            } else {
                (1 << 6) | (1 << 13)
            };
            t.record(b"STAT", stat, now);
            e.poll(now, &t, p, &mut io);
            if elapsed < 501 {
                assert_eq!(e.state, State::Running);
            }
        }
        assert_eq!(e.reason, "Xeryon closed loop inactive");
        assert_eq!(io.stops, 1);
    }
    #[test]
    fn waveform_send_failure_and_return_timeout_attempt_stop() {
        for failure in ["send", "stop", "return"] {
            let mut records = [Record::EMPTY; 1024];
            let mut e = Engine::new(&mut records);
            let mut io = Io::default();
            start(&mut e, sinusoidal(), 0, &mut io);
            for elapsed in 1..=11000 {
                let now = e.started_ms + elapsed;
                let (t, p) = feedback(now, e.target, false);
                if (failure == "send" && elapsed == 30) || (failure == "stop" && elapsed == 1000) {
                    io.fail = true;
                }
                e.poll(now, &t, p, &mut io);
                if e.data_ready() {
                    break;
                }
            }
            assert_eq!(e.outcome, "faulted");
            assert_eq!(io.events.last().unwrap(), "STOP\n");
            assert_eq!(e.stop_queued, failure == "return");
            assert_eq!(
                e.reason,
                match failure {
                    "send" => "scan transmit failure",
                    "stop" => "stop failure",
                    _ => "Final center move deadline",
                }
            );
        }
    }
    #[test]
    fn faults_abort_and_deadline_preserve_partial_data() {
        for condition in 0..8 {
            let mut records = [Record::EMPTY; 1024];
            let mut e = Engine::new(&mut records);
            let mut io = Io::default();
            start(&mut e, sinusoidal(), 0, &mut io);
            let (mut t, mut p) = feedback(12, e.target, true);
            let now = match condition {
                0 => {
                    t.mounted = false;
                    12
                }
                1 => {
                    t.record(b"STAT", 1 << 12, 12);
                    12
                }
                2 => {
                    p.valid = false;
                    12
                }
                3 => {
                    p.ms = u32::MAX - 300;
                    12
                }
                4 => {
                    e.abort(&mut io);
                    12
                }
                5 => {
                    io.usb_failed = true;
                    12
                }
                6 => 42,
                _ => {
                    t.epoch = 1;
                    12
                }
            };
            e.poll(now, &t, p, &mut io);
            assert!(e.data_ready(), "{condition}");
            // At 40 ms the due record is retained before the trajectory deadline faults.
            assert_eq!(e.len, if condition == 6 { 2 } else { 1 });
            assert_eq!(io.stops, 1);
            if condition == 1 {
                assert_eq!(e.reason, "Xeryon encoder error");
                // A later healthy status must not erase the fault evidence.
                t.record(b"STAT", 1089, now + 1);
                e.poll(now + 1, &t, p, &mut io);
                let mut meta = String::new();
                e.metadata(&mut meta, 1).unwrap();
                assert!(meta.contains("\"fault_stat\":4096"));
            }
            assert_eq!(
                e.outcome,
                if condition == 4 { "aborted" } else { "faulted" }
            );
            assert!(e.accept(sinusoidal(), &t, p, false, now).is_err());
        }
    }
    #[test]
    fn transfer_never_releases_on_wrong_or_premature_ack_and_can_restart() {
        let mut records = [Record::EMPTY; 1024];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        start(&mut e, sinusoidal(), 0, &mut io);
        e.abort(&mut io);
        let run = e.run;
        let crc = e.checksum();
        assert!(e.acknowledge(run, e.len, crc).is_err());
        e.transfer(run).unwrap();
        assert!(e.acknowledge(run, e.len, crc).is_err());
        assert!(e.chunk(run, 1).is_err());
        assert_eq!(e.chunk(run, 0).unwrap().len(), 1);
        e.transfer(run).unwrap(); // reconnect/download from zero, same payload
        assert_eq!(e.chunk(run, 0).unwrap()[0].pressure_raw, 123);
        assert!(e.acknowledge(run + 1, 1, crc).is_err());
        assert!(e.acknowledge(run, 1, crc ^ 1).is_err());
        e.acknowledge(run, 1, crc).unwrap();
        assert_eq!(e.state, State::Dashboard);
        assert_eq!(e.len, 1);
        let (t, p) = feedback(5, 0, true);
        assert!(
            e.accept(
                ExperimentConfig::Sinusoidal {
                    center_nm: 0,
                    amplitude_nm: 0,
                    frequency_mhz: 1000,
                    cycles: 1
                },
                &t,
                p,
                false,
                5
            )
            .is_err()
        );
        assert_eq!(e.len, 1);
        assert_eq!(e.checksum(), crc);
        assert_eq!(!crc32(!0, b"123456789"), 0xcbf4_3926);
        let mut meta = String::new();
        e.metadata(&mut meta, 1).unwrap();
        assert!(meta.contains("\"outcome\":\"aborted\""));
    }
    #[test]
    fn acquisition_bound_is_enforced_and_late_samples_are_not_fabricated() {
        let mut records = [Record::EMPTY; CAPACITY];
        let mut e = Engine::new(&mut records);
        let mut io = Io::default();
        let config = ExperimentConfig::QuasiStatic {
            speed_um_s: 10,
            start_nm: 0,
            step_nm: 78,
            hold_ms: MAX_DURATION_MS - MOVE_TIMEOUT_MS,
            steps: 1,
        };
        start(&mut e, config, 0, &mut io);
        assert_eq!(e.required_samples, CAPACITY);
        assert_eq!(config.validate(), Ok(81_900));
        assert!(
            ExperimentConfig::QuasiStatic {
                speed_um_s: 10,
                start_nm: 0,
                step_nm: 78,
                hold_ms: MAX_DURATION_MS - MOVE_TIMEOUT_MS + 1,
                steps: 1,
            }
            .validate()
            .is_err()
        );
        let (t, p) = feedback(202, 1, true);
        e.poll(202, &t, p, &mut io);
        assert_eq!(e.len, 2);
        assert_eq!(e.missed_samples, 9);
        for elapsed in (220..=MAX_DURATION_MS).step_by(20) {
            let now = e.started_ms + elapsed;
            let (t, p) = feedback(now, e.target, true);
            e.poll(now, &t, p, &mut io);
        }
        assert!(e.len <= CAPACITY);
        assert_eq!(e.outcome, "completed");
    }
    #[test]
    fn manual_heartbeat_is_preserved_but_autonomous_policy_ignores_it() {
        let (t, _) = feedback(6000, 0, false);
        let mut motion = xeryon_motion::Motion::new();
        motion.start(0, 0);
        assert!(!motion.poll_policy(&t, 6000, true));
        assert!(motion.poll(&t, 6000));
        assert_eq!(
            motion.interruption_reason,
            Some("desktop heartbeat missing")
        );
    }
}
