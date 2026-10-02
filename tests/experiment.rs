// Engine regressions live only in the firmware test binary.
include!("../src/experiment.rs");
#[cfg(test)]
mod tests {
    use super::*;
    use crate::trajectory::Status;
    #[derive(Default)]
    struct Io {
        targets: Vec<(i32, u32, bool)>,
        stops: usize,
        fail: bool,
        busy: bool,
        speeds: Vec<u32>,
        preparing: bool,
        usb_failed: bool,
        gpio: Option<Status>,
        gpio_preparing: bool,
    }
    impl Actuator for Io {
        fn tx_failed(&self) -> bool {
            self.usb_failed
        }
        fn prepare_feedback(&mut self, _: u32) -> Result<bool, &'static str> {
            Ok(!self.preparing)
        }
        fn prepare_gpio(&mut self, _: u32) -> Result<bool, &'static str> {
            Ok(!self.gpio_preparing && !self.busy)
        }
        fn start_gpio(&mut self, _: ExperimentConfig) -> Result<(), &'static str> {
            self.gpio = Some(Status::Running);
            Ok(())
        }
        fn gpio_status(&self) -> Status {
            self.gpio.unwrap_or(Status::Stopped)
        }
        fn stop_gpio(&mut self) {
            if self.gpio.is_some() {
                self.gpio = Some(Status::Stopped);
            }
        }
        fn query_motor_timeout(&mut self) -> Result<(), &'static str> {
            if self.fail {
                Err("timeout query transmit failure")
            } else {
                Ok(())
            }
        }
        fn speed(&mut self, s: u32) -> Result<(), &'static str> {
            if self.fail {
                return Err("speed transmit failure");
            }
            self.speeds.push(s);
            Ok(())
        }
        fn tx_idle(&self) -> bool {
            !self.busy
        }
        fn target(&mut self, t: i32, now: u32, track: bool) -> Result<(), &'static str> {
            if self.fail {
                return Err("transmit failure");
            }
            assert!(!self.busy);
            self.targets.push((t, now, track));
            Ok(())
        }
        fn stop(&mut self) -> Result<(), &'static str> {
            self.stops += 1;
            if self.fail {
                Err("stop failure")
            } else {
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
                cycles: 1000,
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
        assert!(io.gpio.is_none());
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
    fn stopped_scan_status_allows_next_experiment_start() {
        for config in [
            sinusoidal(),
            ExperimentConfig::QuasiStatic {
                start_nm: 0,
                step_nm: 780,
                hold_ms: 100,
                steps: 1,
                speed_um_s: 1000,
            },
        ] {
            let mut records = [Record::EMPTY; 1024];
            let mut e = Engine::new(&mut records);
            let mut io = Io::default();
            let (mut t, p) = feedback(0, -100, false);
            t.xeryon_motion.as_mut().unwrap().state = 3; // previous STOP
            t.record(b"STAT", 8513, 0);
            e.accept(config, &t, p, false, 0).unwrap();
            e.begin(0, &mut io);
            // Cached stopped-scan feedback remains valid during setup/DPOS.
            e.poll(1, &t, p, &mut io);
            assert_eq!(e.state, State::MoveToStart);
            if config == sinusoidal() {
                assert_eq!(io.targets, [(-100, 0, true)]);
                t.record(b"STAT", 1344, 2);
                t.xeryon_motion.as_mut().unwrap().state = 2;
                e.poll(2, &t, p, &mut io);
                e.poll(3, &t, p, &mut io);
                assert_eq!(e.state, State::Running);
            }
        }
        for extra in [1 << 5, (1 << 5) | (1 << 10), 1 << 9, 1 << 12, 1 << 18] {
            let mut records = [Record::EMPTY; 1024];
            let mut e = Engine::new(&mut records);
            let (mut t, p) = feedback(0, -100, true);
            t.record(b"STAT", 8513 | extra, 0);
            assert!(e.accept(sinusoidal(), &t, p, false, 0).is_err());
        }
    }
    #[test]
    fn faults_abort_and_deadline_preserve_partial_data() {
        for condition in 0..9 {
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
                6 => {
                    io.gpio = Some(Status::Fault);
                    42
                }
                8 => {
                    e.disconnected(&mut io);
                    12
                }
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
            assert_eq!(io.gpio, Some(Status::Stopped));
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
                    cycles: 1,
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
    #[test]
    fn gpio_start_waits_for_arrival_setup_then_stop_drains_before_confirmed_center_return() {
        for begin in [0, u32::MAX - 900] {
            let mut records = [Record::EMPTY; 1024];
            let mut e = Engine::new(&mut records);
            let mut io = Io {
                gpio_preparing: true,
                ..Io::default()
            };
            let (t, p) = feedback(begin, 0, true);
            e.accept(sinusoidal(), &t, p, false, begin).unwrap();
            e.begin(begin, &mut io);
            let (t, p) = feedback(begin.wrapping_add(1), -100, false);
            e.poll(begin.wrapping_add(1), &t, p, &mut io);
            assert!(io.gpio.is_none());
            let (t, p) = feedback(begin.wrapping_add(2), -100, true);
            e.poll(begin.wrapping_add(2), &t, p, &mut io);
            e.poll(begin.wrapping_add(3), &t, p, &mut io);
            assert_eq!(e.state, State::Armed);
            assert!(io.gpio.is_none());
            io.gpio_preparing = false;
            e.poll(begin.wrapping_add(4), &t, p, &mut io);
            assert_eq!(io.gpio, Some(Status::Running));
            assert!(io.speeds.is_empty());
            let boundary = e.started_ms.wrapping_add(1000);
            let (t, p) = feedback(boundary, -100, true);
            io.gpio = Some(Status::Complete);
            io.busy = true;
            e.poll(boundary, &t, p, &mut io);
            assert_eq!(io.gpio, Some(Status::Stopped));
            assert_eq!(io.stops, 1);
            e.poll(boundary.wrapping_add(1), &t, p, &mut io);
            assert_eq!(io.targets.len(), 1);
            assert!(!e.data_ready());
            io.busy = false;
            e.poll(boundary.wrapping_add(2), &t, p, &mut io);
            assert_eq!(io.targets.last().unwrap().0, 0);
            assert!(!e.data_ready());
            let (mut t, p) = feedback(boundary.wrapping_add(3), 0, false);
            t.record(b"STAT", 0, boundary.wrapping_add(3)); // stale STOP status grace
            e.poll(boundary.wrapping_add(3), &t, p, &mut io);
            assert!(!e.data_ready());
            let (t, p) = feedback(boundary.wrapping_add(25), 0, true);
            e.poll(boundary.wrapping_add(25), &t, p, &mut io);
            assert_eq!(e.outcome, "completed");
            assert!(e.data_ready());
            assert_eq!(io.targets.len(), 2);
            assert_eq!(io.stops, 1);
            let mut metadata = crate::protocol::Frame::new();
            e.metadata(&mut metadata, 1).unwrap();
        }
    }
    #[test]
    fn return_timeout_or_abort_stops_gpio_and_retains_data() {
        for abort in [false, true] {
            let mut records = [Record::EMPTY; 1024];
            let mut e = Engine::new(&mut records);
            let mut io = Io::default();
            start(&mut e, sinusoidal(), 0, &mut io);
            io.gpio = Some(Status::Complete);
            let (t, p) = feedback(1002, -100, true);
            e.poll(1002, &t, p, &mut io);
            e.poll(1003, &t, p, &mut io);
            if abort {
                e.abort(&mut io);
            } else {
                let (t, p) = feedback(11002, 0, false);
                e.poll(11002, &t, p, &mut io);
            }
            assert_eq!(io.gpio, Some(Status::Stopped));
            assert!(e.data_ready());
            assert_eq!(e.outcome, if abort { "aborted" } else { "faulted" });
        }
    }
}
