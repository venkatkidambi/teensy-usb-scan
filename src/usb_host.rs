//! Rust owns board initialization and logging. TinyUSB owns USB2 after init.
use core::{
    cell::RefCell,
    fmt,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use cortex_m::peripheral::{NVIC, SYST, syst::SystClkSource};
use teensy4_bsp::{board, hal, ral};

static AUTONOMOUS: AtomicBool = AtomicBool::new(false);

static MILLIS: AtomicU32 = AtomicU32::new(0);
static ATTACHES: AtomicU32 = AtomicU32::new(0);
static REMOVES: AtomicU32 = AtomicU32::new(0);
static INTERRUPTS: AtomicU32 = AtomicU32::new(0);
static XD_C: AtomicBool = AtomicBool::new(false);
static MOTION: cortex_m::interrupt::Mutex<RefCell<crate::xeryon_motion::Motion>> =
    cortex_m::interrupt::Mutex::new(RefCell::new(crate::xeryon_motion::Motion::new()));
static TELEMETRY: cortex_m::interrupt::Mutex<RefCell<crate::xeryon::Decoder>> =
    cortex_m::interrupt::Mutex::new(RefCell::new(crate::xeryon::Decoder::new()));
static CACHE: cortex_m::interrupt::Mutex<RefCell<crate::protocol::Telemetry>> =
    cortex_m::interrupt::Mutex::new(RefCell::new(crate::protocol::Telemetry::new()));
static GPIO_SETUP: cortex_m::interrupt::Mutex<RefCell<crate::xeryon::GpioSetup>> =
    cortex_m::interrupt::Mutex::new(RefCell::new(crate::xeryon::GpioSetup::new()));
static FEEDBACK_SETUP: cortex_m::interrupt::Mutex<RefCell<crate::xeryon::FeedbackSetup>> =
    cortex_m::interrupt::Mutex::new(RefCell::new(crate::xeryon::FeedbackSetup::new()));

// The test owns motion from the first STOP through verified GPIO=0 restoration.
// A failed restoration keeps the interlock latched until XERYON_STOP retries it.
#[derive(Clone, Copy, PartialEq, Eq)]
enum GpioPhase {
    Idle,
    Stop,
    Configure,
    Baseline,
    Synchronize,
    Pulsing,
    Settling,
    Restore,
    Failed,
}
struct GpioTest {
    phase: GpioPhase,
    setup: crate::xeryon::GpioSetup,
    positive: bool,
    steps: u32,
    expected: Option<i32>,
    since: u32,
    query: usize,
    query_since: u32,
    origin: i32,
    start: i32,
    observed: Option<[i64; 2]>,
    matched_since: Option<u32>,
    verified: bool,
    reference_confirmed: Option<u32>,
    error: Option<&'static str>,
}
static GPIO_TEST: cortex_m::interrupt::Mutex<RefCell<GpioTest>> =
    cortex_m::interrupt::Mutex::new(RefCell::new(GpioTest {
        phase: GpioPhase::Idle,
        setup: crate::xeryon::GpioSetup::new(),
        positive: true,
        steps: 0,
        expected: None,
        since: 0,
        query: 0,
        query_since: 0,
        origin: 0,
        start: 0,
        observed: None,
        matched_since: None,
        verified: false,
        reference_confirmed: None,
        error: None,
    }));

pub fn gpio_busy() -> bool {
    cortex_m::interrupt::free(|cs| GPIO_TEST.borrow(cs).borrow().phase != GpioPhase::Idle)
}

pub fn gpio_test_status(out: &mut impl fmt::Write, id: u32) -> fmt::Result {
    cortex_m::interrupt::free(|cs| {
        let test = GPIO_TEST.borrow(cs).borrow();
        let state = match test.phase {
            GpioPhase::Idle => "idle",
            GpioPhase::Stop => "stopping",
            GpioPhase::Configure => "configuring",
            GpioPhase::Baseline | GpioPhase::Synchronize => "baseline",
            GpioPhase::Pulsing => "pulsing",
            GpioPhase::Settling => "settling",
            GpioPhase::Restore => "restoring",
            GpioPhase::Failed => "failed",
        };
        write!(
            out,
            "{{\"type\":\"gpio_status\",\"id\":{id},\"state\":\"{state}\",\"error\":\"{}\",\"test\":",
            test.error.unwrap_or("")
        )?;
        if let Some(expected) = test.expected {
            write!(
                out,
                "{{\"start\":{},\"expected\":{expected},\"verified\":{},\"epos\":",
                test.start, test.verified
            )?;
            if let Some([epos, dpos]) = test.observed {
                write!(out, "{epos},\"dpos\":{dpos}")?;
            } else {
                write!(out, "null,\"dpos\":null")?;
            }
            write!(out, "}}")?;
        } else {
            write!(out, "null")?;
        }
        writeln!(out, "}}")
    })
}

pub fn start_gpio_test(positive: bool, steps: u32, now: u32) -> Result<(), &'static str> {
    if gpio_busy() || AUTONOMOUS.load(Ordering::Relaxed) {
        return Err("GPIO or experiment busy");
    }
    crate::gpio_control::Burst::new(steps)?;
    let t = telemetry();
    if t.xeryon_motion.is_some_and(|m| m.state == 1) {
        return Err("USB motion busy");
    }
    crate::xeryon_motion::target(&t, positive, steps * 78, now)?;
    if !unsafe { usb_serial_tx_idle() } {
        return Err("USB transmit busy");
    }
    crate::gpio_trajectory::stop();
    // An accepted test starts with STOP, never with a pulse or an index command.
    send_gpio_line(b"STOP\n")?;
    cortex_m::interrupt::free(|cs| {
        *GPIO_TEST.borrow(cs).borrow_mut() = GpioTest {
            phase: GpioPhase::Stop,
            setup: crate::xeryon::GpioSetup::new(),
            positive,
            steps,
            expected: None,
            since: now,
            query: 0,
            query_since: now,
            origin: 0,
            start: 0,
            observed: None,
            matched_since: None,
            verified: false,
            reference_confirmed: None,
            error: None,
        };
    });
    Ok(())
}

fn send_gpio_line(bytes: &[u8]) -> Result<(), &'static str> {
    // Foreground only; C copies a complete bounded line. STOP discards queued TX.
    if unsafe { usb_serial_send(bytes.as_ptr(), bytes.len() as u32, bytes == b"STOP\n") } {
        Ok(())
    } else {
        Err("GPIO USB transmit unavailable")
    }
}

impl GpioTest {
    fn query_again(&mut self, now: u32) {
        self.query = 0;
        self.query_since = now;
    }

    fn restore(&mut self, now: u32) {
        crate::gpio_trajectory::stop();
        self.phase = GpioPhase::Restore;
        self.setup = crate::xeryon::GpioSetup::new();
        self.since = now;
    }

    fn poll(&mut self, t: &crate::protocol::Telemetry, now: u32) -> Result<(), &'static str> {
        use crate::{trajectory::Status, xeryon_motion as motion};
        if matches!(self.phase, GpioPhase::Idle | GpioPhase::Failed) {
            return Ok(());
        }
        let idle = unsafe { usb_serial_tx_idle() };
        if !motion::ready(t) || unsafe { usb_serial_tx_failed() } {
            return Err("GPIO USB connection lost or transmit failed");
        }
        if self.phase == GpioPhase::Restore {
            if now.wrapping_sub(self.since) >= 5000 {
                return Err("GPIO restore timeout; retry XERYON_STOP");
            }
            // A priority STOP may still be draining from the preceding poll.
            if !idle {
                return Ok(());
            }
            if self.setup.poll_serial(now, t, idle, send_gpio_line)? {
                self.phase = GpioPhase::Idle;
            }
            return Ok(());
        }
        if let Some(reason) = motion::controller_fault(t.samples[2].value) {
            return Err(reason);
        }
        if !motion::fresh(t, 0, now) || !motion::fresh(t, 2, now) {
            return Err("GPIO position/status feedback stale");
        }
        // Pulse completion must not disable the controller before it lands.
        // Refresh the still-running ENABLE watchdog throughout settling.
        if self.phase == GpioPhase::Settling
            && crate::gpio_trajectory::steps_status() != Status::Complete
        {
            return Err("GPIO settling stopped or watchdog expired");
        }
        if now.wrapping_sub(self.since) >= 5000 {
            return Err(if self.phase == GpioPhase::Synchronize {
                "GPIO reference did not acknowledge or settle before timeout"
            } else if self.phase == GpioPhase::Settling {
                if self
                    .observed
                    .is_some_and(|v| Some(v[1]) != self.expected.map(i64::from))
                {
                    "GPIO target readback mismatch after 5 s; inspect last test counts"
                } else {
                    "GPIO position did not settle with fresh feedback within 5 s"
                }
            } else {
                "GPIO test phase timeout"
            });
        }
        match self.phase {
            GpioPhase::Stop if idle => {
                self.phase = GpioPhase::Configure;
                self.since = now;
            }
            GpioPhase::Configure => {
                if !idle {
                    return Ok(());
                }
                if self.setup.poll(now, t, idle, send_gpio_line)? {
                    self.phase = GpioPhase::Baseline;
                    self.since = now;
                    self.query_again(now);
                }
            }
            GpioPhase::Baseline | GpioPhase::Synchronize | GpioPhase::Settling => {
                // Explicit queries work even when a parameter is not streamed.
                let queries: &[&[u8]] = &[b"EPOS=?\n", b"DPOS=?\n", b"STAT=?\n"];
                if idle && self.query < queries.len() {
                    send_gpio_line(queries[self.query])?;
                    self.query += 1;
                    return Ok(());
                }
                let fresh = t.samples[..3].iter().all(|s| {
                    s.valid && {
                        let age = s.ms.wrapping_sub(self.query_since);
                        age > 0 && age < 0x8000_0000 && now.wrapping_sub(s.ms) < 250
                    }
                });
                if !idle || self.query < queries.len() || !fresh {
                    return Ok(());
                }
                if self.phase == GpioPhase::Baseline {
                    self.origin = crate::gpio_control::position(t, now, self.since)?;
                    // ENABLE/PULSE remain low throughout this serialized USB
                    // preparation. Hold the measured position, not the retained
                    // old target; never change tuning or reset the encoder.
                    let mut frame = crate::protocol::Frame::new();
                    crate::xeryon_motion::write_move(&mut frame, self.origin, None)?;
                    send_gpio_line(frame.pending())?;
                    self.phase = GpioPhase::Synchronize;
                    self.since = now;
                    self.query_again(now);
                } else if self.phase == GpioPhase::Synchronize {
                    if t.samples[1].value == i64::from(self.origin)
                        && self.reference_confirmed.is_none()
                    {
                        self.reference_confirmed = Some(now);
                        self.query_again(now);
                        return Ok(());
                    }
                    let after_reference = self.reference_confirmed.is_some_and(|confirmed| {
                        t.samples[..3].iter().all(|s| {
                            let age = s.ms.wrapping_sub(confirmed);
                            age > 0 && age < 0x8000_0000
                        })
                    });
                    if t.samples[1].value != i64::from(self.origin)
                        || motion::moving(t.samples[2].value)
                        || t.samples[2].value & (1 << 10) == 0
                        || !after_reference
                    {
                        // Await the acknowledged reference and stationary state.
                        // The phase deadline also covers a controller that never
                        // settles. Stop/disconnect always cancels preparation.
                        self.query_again(now);
                        return Ok(());
                    }
                    let expected = crate::gpio_control::target(
                        t,
                        self.positive,
                        self.steps,
                        now,
                        self.reference_confirmed.unwrap(),
                        self.origin,
                    )?;
                    crate::gpio_trajectory::start_steps(self.positive, self.steps)?;
                    self.expected = Some(expected);
                    self.start = t.samples[0].value as i32;
                    self.phase = GpioPhase::Pulsing;
                    self.since = now;
                } else {
                    // Freeze these readings before STOP / GPIO=0 restoration,
                    // which may itself change the controller's reported target.
                    self.observed = Some([t.samples[0].value, t.samples[1].value]);
                    let complete = self.expected.is_some_and(|expected| {
                        crate::gpio_control::completed(t, expected, now, self.query_since)
                    });
                    if complete {
                        if self
                            .matched_since
                            .is_some_and(|since| now.wrapping_sub(since) >= 100)
                        {
                            self.verified = true;
                            self.restore(now);
                            return Ok(());
                        }
                        self.matched_since.get_or_insert(now);
                    } else {
                        self.matched_since = None;
                    }
                    // A delayed DPOS is pending feedback, not an immediate
                    // wiring fault. Require new EPOS/DPOS/STAT on every round;
                    // the five-second phase deadline is never extended.
                    self.query_again(now);
                }
            }
            GpioPhase::Pulsing => match crate::gpio_trajectory::steps_status() {
                Status::Running => {}
                Status::Complete => {
                    self.phase = GpioPhase::Settling;
                    self.since = now;
                    self.query_again(now);
                }
                _ => return Err("GPIO pulse train stopped or watchdog expired"),
            },
            _ => {}
        }
        Ok(())
    }
}

fn poll_gpio_test(t: &crate::protocol::Telemetry, now: u32) {
    cortex_m::interrupt::free(|cs| {
        let mut test = GPIO_TEST.borrow(cs).borrow_mut();
        if let Err(reason) = test.poll(t, now) {
            crate::gpio_trajectory::stop();
            let _ = send_gpio_line(b"STOP\n");
            test.error = Some(reason);
            if test.phase == GpioPhase::Restore {
                test.phase = GpioPhase::Failed;
            } else {
                test.restore(now);
            }
            log::warn!("GPIO test: {}", reason);
        }
    });
}

pub fn telemetry() -> crate::protocol::Telemetry {
    cortex_m::interrupt::free(|cs| {
        let mut t = *CACHE.borrow(cs).borrow();
        t.xeryon_motion = Some(*MOTION.borrow(cs).borrow());
        t.feedback_before = FEEDBACK_SETUP.borrow(cs).borrow().before;
        // Foreground-only bounded diagnostic snapshot through the C boundary.
        unsafe {
            usb_serial_stats(
                &mut t.transport.rx_peak,
                &mut t.transport.rx_backlog_polls,
                &mut t.transport.max_service_gap_ms,
                &mut t.transport.max_tx_busy_ms,
            );
        }
        t
    })
}

pub fn heartbeat(now: u32) {
    cortex_m::interrupt::free(|cs| MOTION.borrow(cs).borrow_mut().heartbeat(now));
}

pub fn move_xeryon(up: bool, nm: u32, speed: Option<u32>, now: u32) -> Result<i32, &'static str> {
    if gpio_busy() {
        return Err("GPIO owns motion; wait for idle or XERYON_STOP");
    }
    let t = telemetry();
    if t.xeryon_motion.is_some_and(|m| m.state == 1) {
        return Err("Wait for the Xeryon move to finish or press Stop");
    }
    let target = crate::xeryon_motion::target(&t, up, nm, now)?;
    let mut frame = crate::protocol::Frame::new();
    crate::xeryon_motion::write_move(&mut frame, target, speed)?;
    let bytes = frame.pending();
    if speed.is_some() {
        // Flush SSPD and DPOS as separate USB transfers. The XD-C processes
        // each line on newline, and the working DPOS-only path uses one packet.
        let split = bytes
            .iter()
            .position(|&byte| byte == b'\n')
            .ok_or("Invalid Xeryon frame")?
            + 1;
        // SAFETY: main context; C copies each bounded line before returning.
        if !unsafe { usb_serial_send(bytes.as_ptr(), split as u32, false) } {
            return Err("Xeryon speed USB transmit unavailable");
        }
        if !unsafe { usb_serial_send(bytes[split..].as_ptr(), (bytes.len() - split) as u32, false) }
        {
            return Err("Xeryon target USB transmit unavailable");
        }
    } else if !unsafe { usb_serial_send(bytes.as_ptr(), bytes.len() as u32, false) } {
        return Err("Xeryon target USB transmit unavailable");
    }
    cortex_m::interrupt::free(|cs| MOTION.borrow(cs).borrow_mut().start(target, now));
    Ok(target)
}

pub fn stop_xeryon() -> Result<(), &'static str> {
    crate::gpio_trajectory::stop();
    cortex_m::interrupt::free(|cs| {
        let mut test = GPIO_TEST.borrow(cs).borrow_mut();
        if test.phase != GpioPhase::Idle {
            test.restore(millis());
        }
    });
    if !crate::xeryon_motion::ready(&telemetry()) {
        return Err("Xeryon is not connected");
    }
    // SAFETY: main context, static command, bounded nonblocking copy.
    let sent = unsafe { usb_serial_send(b"STOP\n".as_ptr(), 5, true) };
    cortex_m::interrupt::free(|cs| MOTION.borrow(cs).borrow_mut().state = if sent { 3 } else { 4 });
    if sent {
        Ok(())
    } else {
        Err("Xeryon Stop could not be queued")
    }
}

pub fn disconnect_xeryon() {
    let active = cortex_m::interrupt::free(|cs| MOTION.borrow(cs).borrow().state == 1);
    if (active || gpio_busy()) && !AUTONOMOUS.load(Ordering::Relaxed) {
        let _ = stop_xeryon();
    }
}

fn reset_telemetry() {
    cortex_m::interrupt::free(|cs| {
        *TELEMETRY.borrow(cs).borrow_mut() = crate::xeryon::Decoder::new();
    });
}

unsafe extern "C" {
    fn usb_serial_progress(values: *mut u32);
    fn usb_serial_rx_pending() -> bool;
    fn usb_serial_timing(on: bool);
    fn usb_serial_stats(peak: *mut u32, backlog: *mut u32, gap: *mut u32, tx_ms: *mut u32);
    fn usb_serial_tx_idle() -> bool;
    fn usb_serial_tx_failed() -> bool;
    fn usb_serial_send(data: *const u8, len: u32, stop: bool) -> bool;
    fn scanner_init() -> bool;
    fn scanner_task();
    fn scanner_interrupt();
    fn scanner_report() -> bool;
    fn scanner_registers(command: *mut u32, status: *mut u32, port: *mut u32, mode: *mut u32);
}

pub fn millis() -> u32 {
    MILLIS.load(Ordering::Relaxed)
}

pub fn start_tick(syst: &mut SYST) {
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(board::ARM_FREQUENCY / 1000 - 1);
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();
}

#[unsafe(no_mangle)]
extern "C" fn SysTick() {
    let now = MILLIS.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
    crate::motor::tick(now);
    crate::fault::tick(now);
}

#[unsafe(no_mangle)]
extern "C" fn tusb_time_millis_api() -> u32 {
    millis()
}

pub fn init(
    ccm: &mut ral::ccm::CCM,
    analog: &mut ral::ccm_analog::CCM_ANALOG,
    gpio3: &mut hal::gpio::Port,
    gpr: &mut ral::iomuxc_gpr::IOMUXC_GPR,
) {
    hal::ccm::analog::pll7::restart(analog);
    hal::ccm::clock_gate::usb().set(ccm, hal::ccm::clock_gate::ON);

    // board::t41 does not return or use USBPHY2. Acquire it here once, then
    // relinquish all USB2/PHY2 register access to TinyUSB after scanner_init.
    let phy = unsafe { ral::usbphy::USBPHY2::instance() };
    ral::write_reg!(ral::usbphy, phy, CTRL_CLR, SFTRST: 1, CLKGATE: 1);
    ral::write_reg!(ral::usbphy, phy, CTRL_SET, ENUTMILEVEL2: 1, ENUTMILEVEL3: 1);
    ral::write_reg!(ral::usbphy, phy, PWD, 0);
    // Same PHY calibration as TinyUSB's upstream i.MX RT board initialization.
    ral::modify_reg!(ral::usbphy, phy, TX, D_CAL: 12, TXCAL45DP: 6, TXCAL45DN: 6);

    // Teensy 4.1 host VBUS load-switch enable: GPIO_EMC_40 / GPIO3_IO26.
    // This internal pad is not among the user pins returned by board::t41.
    // Select standard GPIO3, rather than the alternate fast GPIO8 alias.
    ral::modify_reg!(ral::iomuxc_gpr, gpr, GPR28, |v| v & !(1 << 26));
    let mut enable = unsafe { hal::iomuxc::pads::gpio_emc::GPIO_EMC_40::new() };
    hal::iomuxc::configure(
        &mut enable,
        hal::iomuxc::Config::zero().set_drive_strength(hal::iomuxc::DriveStrength::R0),
    );
    let power = gpio3.output(enable).unwrap();
    power.set();
    let start = millis();
    while millis().wrapping_sub(start) < 10 {
        cortex_m::asm::wfi();
    }
    // SAFETY: clocks, PHY, VBUS, vector and timebase are ready; called once.
    assert!(
        unsafe { scanner_init() },
        "TinyUSB host initialization failed"
    );
}

pub fn poll() {
    let attaches = ATTACHES.swap(0, Ordering::Relaxed);
    let removes = REMOVES.swap(0, Ordering::Relaxed);
    if attaches != 0 {
        log::info!(
            "USB2 connection detected ({} event(s)); enumerating",
            attaches
        );
    }
    if removes != 0 {
        log::info!("USB2 disconnection detected ({} event(s))", removes);
    }
    // SAFETY: host task is called only from the single main-loop context.
    unsafe { scanner_task() };
    let t = telemetry();
    poll_gpio_test(&t, millis());
    let stop = cortex_m::interrupt::free(|cs| {
        let mut motion = MOTION.borrow(cs).borrow_mut();
        if AUTONOMOUS.load(Ordering::Relaxed) {
            motion.poll_policy(&t, millis(), true)
        } else {
            motion.poll(&t, millis())
        }
    });
    if stop {
        let reason = cortex_m::interrupt::free(|cs| MOTION.borrow(cs).borrow().interruption_reason);
        let result = stop_xeryon();
        cortex_m::interrupt::free(|cs| MOTION.borrow(cs).borrow_mut().state = 4);
        log::warn!(
            "Xeryon motion interrupted: {}; Stop queued={}",
            reason.unwrap_or("unknown reason"),
            result.is_ok()
        );
    }
}

pub fn autonomous(on: bool) {
    AUTONOMOUS.store(on, Ordering::Relaxed);
    if on {
        cortex_m::interrupt::free(|cs| {
            *GPIO_SETUP.borrow(cs).borrow_mut() = crate::xeryon::GpioSetup::new()
        });
        cortex_m::interrupt::free(|cs| {
            *FEEDBACK_SETUP.borrow(cs).borrow_mut() = crate::xeryon::FeedbackSetup::new()
        });
    }
}

pub fn rx_pending() -> bool {
    // SAFETY: foreground-only RX occupancy query.
    unsafe { usb_serial_rx_pending() }
}

pub struct ExperimentActuator;
impl crate::experiment::Actuator for ExperimentActuator {
    fn prepare_serial(&mut self, now: u32) -> Result<bool, &'static str> {
        if gpio_busy() {
            return Err("GPIO test owns motion");
        }
        crate::gpio_trajectory::stop();
        let t = telemetry();
        cortex_m::interrupt::free(|cs| {
            GPIO_SETUP
                .borrow(cs)
                .borrow_mut()
                .poll_serial(now, &t, self.tx_idle(), |bytes| {
                    // SAFETY: foreground-only, complete bounded line copied by C.
                    if unsafe { usb_serial_send(bytes.as_ptr(), bytes.len() as u32, false) } {
                        Ok(())
                    } else {
                        Err("Serial mode setup transmit unavailable")
                    }
                })
        })
    }
    fn serial_commands(
        &mut self,
        commands: &[crate::experiment::serial_sine::Command],
    ) -> Result<(), &'static str> {
        if gpio_busy() {
            return Err("GPIO test owns motion");
        }
        if !self.tx_idle() {
            return Err("Serial waveform previous transmit incomplete");
        }
        // Each line is separately flushed; at most two lines per setpoint.
        for command in commands {
            let mut frame = crate::protocol::Frame::new();
            command.write(&mut frame)?;
            let bytes = frame.pending();
            if !unsafe { usb_serial_send(bytes.as_ptr(), bytes.len() as u32, false) } {
                return Err("Serial waveform transmit unavailable");
            }
        }
        Ok(())
    }
    fn prepare_feedback(&mut self, now: u32) -> Result<bool, &'static str> {
        let t = telemetry();
        let idle = self.tx_idle();
        cortex_m::interrupt::free(|cs| {
            FEEDBACK_SETUP
                .borrow(cs)
                .borrow_mut()
                .poll(now, &t, idle, |bytes| {
                    // SAFETY: main context, bounded complete line copied by C.
                    if unsafe { usb_serial_send(bytes.as_ptr(), bytes.len() as u32, false) } {
                        Ok(())
                    } else {
                        Err("Xeryon feedback setup transmit unavailable")
                    }
                })
        })
    }
    fn start_timing(&mut self) {
        cortex_m::interrupt::free(|cs| {
            let mut t = CACHE.borrow(cs).borrow_mut();
            t.epos_arrivals = Default::default();
            t.timing_active = true;
        });
        // SAFETY: reset only foreground diagnostic counters, no USB state change.
        unsafe { usb_serial_timing(true) };
    }
    fn end_timing(&mut self) {
        cortex_m::interrupt::free(|cs| CACHE.borrow(cs).borrow_mut().timing_active = false);
        unsafe { usb_serial_timing(false) };
    }
    fn tx_progress(&self) -> crate::experiment::final_return::TxProgress {
        let mut values = [0; 6];
        // SAFETY: foreground-only snapshot into six u32 values.
        unsafe { usb_serial_progress(values.as_mut_ptr()) };
        crate::experiment::final_return::TxProgress(values)
    }
    fn prepare_gpio(&mut self, now: u32) -> Result<bool, &'static str> {
        if gpio_busy() {
            return Err("GPIO test owns motion");
        }
        let t = telemetry();
        cortex_m::interrupt::free(|cs| {
            GPIO_SETUP
                .borrow(cs)
                .borrow_mut()
                .poll(now, &t, self.tx_idle(), |bytes| {
                    if unsafe { usb_serial_send(bytes.as_ptr(), bytes.len() as u32, false) } {
                        Ok(())
                    } else {
                        Err("GPIO setup transmit unavailable")
                    }
                })
        })
    }
    fn start_gpio(
        &mut self,
        config: crate::experiment_data::ExperimentConfig,
    ) -> Result<(), &'static str> {
        if gpio_busy() {
            return Err("GPIO test owns motion");
        }
        crate::gpio_trajectory::start(config)
    }
    fn gpio_status(&self) -> crate::trajectory::Status {
        crate::gpio_trajectory::status()
    }
    fn stop_gpio(&mut self) {
        crate::gpio_trajectory::stop();
    }
    fn speed(&mut self, speed_um_s: u32) -> Result<(), &'static str> {
        if !self.tx_idle() {
            return Err("Xeryon previous transmit incomplete");
        }
        let mut frame = crate::protocol::Frame::new();
        crate::xeryon_motion::write_speed(&mut frame, speed_um_s)?;
        let bytes = frame.pending();
        // Each line is flushed independently, as in manual speed-controlled moves.
        // Query explicitly: setting a parameter need not echo on every controller.
        if !unsafe { usb_serial_send(bytes.as_ptr(), bytes.len() as u32, false) }
            || !unsafe { usb_serial_send(b"SSPD=?\n".as_ptr(), 7, false) }
        {
            return Err("Xeryon speed transmit unavailable");
        }
        Ok(())
    }
    fn tx_idle(&self) -> bool {
        // SAFETY: foreground-only transport query.
        unsafe { usb_serial_tx_idle() }
    }
    fn tx_failed(&self) -> bool {
        // SAFETY: foreground-only latched CDC transfer result.
        unsafe { usb_serial_tx_failed() }
    }
    fn query_motor_timeout(&mut self) -> Result<(), &'static str> {
        // SAFETY: foreground only, bounded copy after speed transfers drain.
        if !self.tx_idle() || !unsafe { usb_serial_send(b"TOU2=?\n".as_ptr(), 7, false) } {
            return Err("Xeryon TOU2 query transmit unavailable");
        }
        Ok(())
    }
    fn target(&mut self, target: i32, now: u32, track: bool) -> Result<(), &'static str> {
        if gpio_busy() {
            return Err("GPIO test owns motion");
        }
        // SAFETY: foreground-only query into the existing C transport. Fault
        // instead of queuing a target behind an unfinished USB transfer.
        if !unsafe { usb_serial_tx_idle() } {
            return Err("Xeryon previous transmit incomplete");
        }
        // Bounded integer encoding for the existing finite-position path.
        let mut bytes = [0u8; 20];
        bytes[..5].copy_from_slice(b"DPOS=");
        let mut end = 5;
        if target < 0 {
            bytes[end] = b'-';
            end += 1;
        }
        let mut digits = [0u8; 10];
        let mut n = target.unsigned_abs();
        let mut pos = digits.len();
        loop {
            pos -= 1;
            digits[pos] = b'0' + (n % 10) as u8;
            n /= 10;
            if n == 0 {
                break;
            }
        }
        let count = digits.len() - pos;
        bytes[end..end + count].copy_from_slice(&digits[pos..]);
        end += count;
        bytes[end] = b'\n';
        end += 1;
        // SAFETY: foreground only; C copies this complete short line.
        if !unsafe { usb_serial_send(bytes.as_ptr(), end as u32, false) } {
            return Err("Xeryon target transmit unavailable");
        }
        if track {
            cortex_m::interrupt::free(|cs| MOTION.borrow(cs).borrow_mut().start(target, now));
        }
        Ok(())
    }
    fn stop(&mut self) -> Result<(), &'static str> {
        stop_xeryon()
    }
}

pub fn report() {
    let (mut command, mut status, mut port, mut mode) = (0, 0, 0, 0);
    // SAFETY: initialized USB2 registers are read only, into live local values.
    unsafe { scanner_registers(&mut command, &mut status, &mut port, &mut mode) };
    log::info!(
        "USB2 connected={} irqs={} USBCMD={:08x} USBSTS={:08x} PORTSC1={:08x} USBMODE={:08x}",
        port & 1 != 0,
        INTERRUPTS.load(Ordering::Relaxed),
        command,
        status,
        port,
        mode
    );
    // SAFETY: shared C scanner state is only touched from main context.
    if !unsafe { scanner_report() } {
        log::info!("No device descriptor available yet");
    }
}

#[unsafe(no_mangle)]
extern "C" fn USB_OTG2() {
    INTERRUPTS.fetch_add(1, Ordering::Relaxed);
    // SAFETY: TinyUSB enables this interrupt only after initializing its state.
    unsafe { scanner_interrupt() };
}

#[unsafe(no_mangle)]
extern "C" fn scanner_irq_enable(irq: i32) {
    if irq == ral::Interrupt::USB_OTG2 as i32 {
        // SAFETY: only TinyUSB calls this; USB2's handler is installed above.
        unsafe { NVIC::unmask(ral::Interrupt::USB_OTG2) };
    }
}

#[unsafe(no_mangle)]
extern "C" fn scanner_irq_disable(irq: i32) {
    if irq == ral::Interrupt::USB_OTG2 as i32 {
        NVIC::mask(ral::Interrupt::USB_OTG2);
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}

#[unsafe(no_mangle)]
extern "C" fn scanner_bus_event(event: u32) {
    match event {
        0 => {
            ATTACHES.fetch_add(1, Ordering::Relaxed);
        }
        1 => {
            crate::gpio_trajectory::stop();
            REMOVES.fetch_add(1, Ordering::Relaxed);
        }
        _ => {}
    }
}

#[unsafe(no_mangle)]
extern "C" fn scanner_device(
    address: u8,
    vid: u16,
    pid: u16,
    class: u8,
    sub: u8,
    protocol: u8,
    mounted: bool,
) {
    // Confirmed descriptor identity of the user's XD-C. Other CDC devices
    // retain escaped raw logging instead of receiving Xeryon interpretations.
    XD_C.store(vid == 0x04d8 && pid == 0xef4a, Ordering::Relaxed);
    cortex_m::interrupt::free(|cs| {
        let mut cache = CACHE.borrow(cs).borrow_mut();
        if !mounted {
            cache.reset();
        }
        cache.vid = vid;
        cache.pid = pid;
        cache.mounted = mounted;
    });
    log::info!(
        "addr={} VID={:04x} PID={:04x} class={:02x} subclass={:02x} protocol={:02x} configured={}",
        address,
        vid,
        pid,
        class,
        sub,
        protocol,
        mounted
    );
}

#[unsafe(no_mangle)]
extern "C" fn scanner_removed(address: u8) {
    crate::gpio_trajectory::stop();
    XD_C.store(false, Ordering::Relaxed);
    reset_telemetry();
    cortex_m::interrupt::free(|cs| {
        *MOTION.borrow(cs).borrow_mut() = crate::xeryon_motion::Motion::new()
    });
    cortex_m::interrupt::free(|cs| CACHE.borrow(cs).borrow_mut().reset());
    log::info!("Device removed: address={}", address);
}

#[unsafe(no_mangle)]
extern "C" fn scanner_serial_event(event: u8, address: u8, interface: u8) {
    if event >= 2 {
        crate::gpio_trajectory::stop();
    }
    reset_telemetry();
    cortex_m::interrupt::free(|cs| {
        let mut cache = CACHE.borrow(cs).borrow_mut();
        cache.cdc = match event {
            0 => 1,
            1 => 2,
            2 => 0,
            3 => 3,
            _ => 4,
        };
        if event == 2 {
            cache.samples.fill(crate::protocol::Sample::default());
        }
    });
    let message = match event {
        0 => "CDC ACM detected",
        1 => "CDC ACM open: 115200 8-N-1, DTR asserted",
        2 => "CDC ACM disconnected",
        3 => "CDC ACM setup failed; unplug/reconnect to retry",
        4 => "CDC ACM setup timed out; unplug/reconnect to retry",
        _ => return,
    };
    log::info!("USB2 addr={} interface={}: {}", address, interface, message);
}

struct SerialBytes<'a>(&'a [u8]);

impl fmt::Display for SerialBytes<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use fmt::Write;
        for &byte in self.0 {
            for escaped in core::ascii::escape_default(byte) {
                f.write_char(char::from(escaped))?;
            }
        }
        Ok(())
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn scanner_serial_rx(data: *const u8, len: u32) {
    if data.is_null() || len > 64 {
        return;
    }
    // SAFETY: C supplies a live stack buffer for this main-context callback.
    let bytes = unsafe { core::slice::from_raw_parts(data, len as usize) };
    if XD_C.load(Ordering::Relaxed) {
        // One bounded critical section per received chunk, no string formatting
        // or USB IO inside it. Foreground USB1 serializes a copy at 10 Hz.
        cortex_m::interrupt::free(|cs| {
            let mut decoder = TELEMETRY.borrow(cs).borrow_mut();
            let mut cache = CACHE.borrow(cs).borrow_mut();
            for &byte in bytes {
                match decoder.push(byte) {
                    Some(Ok(record)) => cache.record(record.tag(), record.value(), millis()),
                    Some(Err(_)) => cache.malformed = cache.malformed.wrapping_add(1),
                    None => {}
                }
            }
        });
    } else {
        log::info!("USB2 CDC RX: {}", SerialBytes(bytes));
    }
}

struct UsbString<'a>(&'a [u8]);

impl fmt::Display for UsbString<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use fmt::Write;
        let units = self
            .0
            .chunks_exact(2)
            .map(|b| u16::from_le_bytes([b[0], b[1]]));
        for ch in char::decode_utf16(units) {
            let ch = ch.unwrap_or(char::REPLACEMENT_CHARACTER);
            f.write_char(if ch.is_control() { ' ' } else { ch })?;
        }
        Ok(())
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn scanner_string(kind: u8, data: *const u8, bytes: u16) {
    if data.is_null() || bytes > 254 || bytes % 2 != 0 {
        return;
    }
    // SAFETY: the C bridge provides a live static buffer of at least `bytes`
    // bytes for this call. No reference is kept after the callback returns.
    let data = unsafe { core::slice::from_raw_parts(data, usize::from(bytes)) };
    let label = match kind {
        0 => "manufacturer",
        1 => "product",
        2 => "serial",
        _ => return,
    };
    log::info!("{}: {}", label, UsbString(data));
}
