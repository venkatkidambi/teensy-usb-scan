//! XD-OEM manual sections 5.1 / 7.3: GPIO=2, STPS=1, 3.3 V logic.
//! Teensy 7/8/9/10 -> XD-OEM 13 PULSE / 14 DIRECTION / 15 ENABLE / 16 INDEX.
//! Ground is shared through USB host. INDEX is always driven low, never pulsed.
//! External pulldowns on PULSE, ENABLE and INDEX are needed to hold them low
//! before firmware initialization / during reset (internal pulls start at init).
//! Direction high is used for positive steps; verify sign against EPOS unloaded.
//! GPT1 free-runs at 1 MHz. Compare IRQs evaluate absolute elapsed time, with
//! at most one edge each; no catch-up bursts, USB calls, allocation, or waits.
use crate::{
    experiment_data::ExperimentConfig,
    trajectory::{Pulses, Status, TICK_US, Trajectory},
};
use core::cell::RefCell;
use cortex_m::{
    interrupt::{self, Mutex},
    peripheral::NVIC,
};
use teensy4_bsp::{
    board,
    hal::{
        self,
        gpt::{ClockSource, Gpt, Mode, OutputCompareRegister as Ocr},
    },
    pins::common,
    ral,
};
static DRIVER: Mutex<RefCell<Option<Driver>>> = Mutex::new(RefCell::new(None));
struct Driver {
    timer: Gpt,
    pulse: hal::gpio::Output,
    direction: hal::gpio::Output,
    enable: hal::gpio::Output,
    run: Option<Pulses>,
    reference: Option<i32>,
    positive: bool,
    index: hal::gpio::Output,
    epoch: u32,
    heartbeat: u32,
    last_rise: u32,
}
impl Driver {
    fn low(&mut self) {
        self.enable.clear();
        self.pulse.clear();
        self.index.clear();
        self.timer.set_output_interrupt_on_compare(Ocr::OCR1, false);
    }
}
pub fn init(
    mut timer: Gpt,
    gpio: &mut hal::gpio::Port,
    mut pulse: common::P7,
    mut direction: common::P8,
    mut enable: common::P9,
    mut index: common::P10,
) {
    let cfg = hal::iomuxc::Config::zero()
        .set_drive_strength(hal::iomuxc::DriveStrength::R0_7)
        .set_pull_keeper(Some(hal::iomuxc::PullKeeper::Pulldown100k));
    hal::iomuxc::configure(&mut pulse, cfg);
    hal::iomuxc::configure(&mut direction, cfg);
    hal::iomuxc::configure(&mut enable, cfg);
    hal::iomuxc::configure(&mut index, cfg);
    // Clear output latches before switching pad direction, including warm starts.
    emergency_low();
    // Direction latch is also cleared before muxing it to an output.
    use hal::iomuxc::gpio::Pin;
    let bank = unsafe { &*ral::gpio::GPIO2 };
    ral::write_reg!(
        ral::gpio,
        bank,
        DR_CLEAR,
        1 << <common::P8 as Pin<2>>::OFFSET
    );
    let pulse = gpio.output(pulse).unwrap();
    pulse.clear();
    let direction = gpio.output(direction).unwrap();
    direction.clear();
    let enable = gpio.output(enable).unwrap();
    enable.clear();
    let index = gpio.output(index).unwrap();
    index.clear();
    timer.disable();
    timer.set_clock_source(ClockSource::PeripheralClock);
    timer.set_divider(board::PERCLK_FREQUENCY / 1_000_000);
    timer.set_mode(Mode::FreeRunning);
    timer.set_wait_mode_enable(true);
    timer.set_reset_on_enable(false);
    timer.enable();
    interrupt::free(|cs| {
        *DRIVER.borrow(cs).borrow_mut() = Some(Driver {
            timer,
            pulse,
            direction,
            enable,
            run: None,
            reference: None,
            positive: false,
            index,
            epoch: 0,
            heartbeat: 0,
            last_rise: 0,
        })
    });
    // SAFETY: this module exclusively owns GPT1 and its initialized IRQ state.
    unsafe {
        NVIC::unmask(ral::Interrupt::GPT1);
    }
}
pub fn start(config: ExperimentConfig) -> Result<(), &'static str> {
    let trajectory = Trajectory::new(config)?;
    if position() != Some(trajectory.start) {
        return Err("GPIO start reference mismatch");
    }
    activate(trajectory)
}
pub fn approach(from: i32, to: i32) -> Result<(), &'static str> {
    activate(Trajectory::approach(from, to)?)
}
pub fn position() -> Option<i32> {
    interrupt::free(|cs| {
        DRIVER
            .borrow(cs)
            .borrow()
            .as_ref()
            .and_then(|d| d.reference)
    })
}
pub fn invalidate_reference() {
    interrupt::free(|cs| {
        if let Some(d) = DRIVER.borrow(cs).borrow_mut().as_mut() {
            d.reference = None;
        }
    });
}
fn activate(trajectory: Trajectory) -> Result<(), &'static str> {
    interrupt::free(|cs| {
        let mut slot = DRIVER.borrow(cs).borrow_mut();
        let d = slot.as_mut().ok_or("GPIO unavailable")?;
        if d.run.as_ref().is_some_and(|p| p.status == Status::Running) {
            return Err("GPIO busy");
        }
        // Preserve ENABLE and direction from the completed approach. No serial
        // handoff, pulse, or direction transition occurs between trajectories.
        let enabled = d.run.as_ref().is_some_and(|p| p.status == Status::Complete);
        let mut pulses = Pulses::new(trajectory);
        pulses.positive = d.positive;
        d.run = Some(pulses);
        d.reference = Some(trajectory.start);
        d.pulse.clear();
        d.enable.set();
        cortex_m::asm::dsb();
        let now = d.timer.count();
        // On first enable, allow 500 us setup before starting trajectory time.
        d.epoch = now.wrapping_add(if enabled { 0 } else { 500 });
        d.heartbeat = now;
        d.last_rise = now;
        d.timer.clear_elapsed(Ocr::OCR1);
        d.timer
            .set_output_compare_count(Ocr::OCR1, d.epoch.wrapping_add(TICK_US));
        d.timer.set_output_interrupt_on_compare(Ocr::OCR1, true);
        Ok(())
    })
}
pub fn status() -> Status {
    interrupt::free(|cs| {
        let mut slot = DRIVER.borrow(cs).borrow_mut();
        let Some(d) = slot.as_mut() else {
            return Status::Stopped;
        };
        d.heartbeat = d.timer.count();
        d.run.as_ref().map_or(Status::Stopped, |p| p.status)
    })
}
pub fn stop() {
    interrupt::free(|cs| {
        if let Some(d) = DRIVER.borrow(cs).borrow_mut().as_mut() {
            d.low();
            if let Some(p) = d.run.as_mut() {
                if p.status != Status::Complete {
                    d.reference = None;
                }
                p.stop();
            }
        }
    });
}
/// Terminal fault path: interrupts are already disabled; never borrow state.
pub fn emergency_low() {
    use hal::iomuxc::gpio::Pin;
    let gpio = unsafe { &*ral::gpio::GPIO2 };
    ral::write_reg!(
        ral::gpio,
        gpio,
        DR_CLEAR,
        (1 << <common::P7 as Pin<2>>::OFFSET)
            | (1 << <common::P9 as Pin<2>>::OFFSET)
            | (1 << <common::P10 as Pin<2>>::OFFSET)
    );
}
#[unsafe(no_mangle)]
extern "C" fn GPT1() {
    interrupt::free(|cs| {
        if let Some(d) = DRIVER.borrow(cs).borrow_mut().as_mut() {
            if !d.timer.is_elapsed(Ocr::OCR1) {
                return;
            }
            d.timer.clear_elapsed(Ocr::OCR1);
            let now = d.timer.count();
            if let Some(p) = d.run.as_mut() {
                if p.status == Status::Running {
                    p.tick(now.wrapping_sub(d.epoch));
                }
                if now.wrapping_sub(d.heartbeat) > 10_000 {
                    p.status = Status::Fault;
                    p.high = false;
                }
                if matches!(p.status, Status::Fault | Status::Stopped) {
                    d.reference = None;
                    d.low();
                } else {
                    if !p.high {
                        d.pulse.clear();
                    }
                    d.positive = p.positive;
                    if p.positive {
                        d.direction.set();
                    } else {
                        d.direction.clear();
                    }
                    if p.high {
                        // Check the physical edge interval too, independently of
                        // computation/IRQ jitter. Never exceed the manual's 20 kHz.
                        if d.timer.count().wrapping_sub(d.last_rise) < 50 {
                            p.status = Status::Fault;
                            p.high = false;
                            d.reference = None;
                            d.low();
                            return;
                        }
                        d.pulse.set();
                        d.last_rise = d.timer.count();
                        d.reference = Some(p.position);
                    }
                    // Complete holds ENABLE until fresh endpoint confirmation;
                    // the watchdog remains active throughout settling.
                    // Schedule from the hardware clock after bounded work. The
                    // trajectory phase always uses actual time, never IRQ count.
                    d.timer
                        .set_output_compare_count(Ocr::OCR1, d.timer.count().wrapping_add(TICK_US));
                }
            } else {
                d.low();
            }
        }
    });
    cortex_m::asm::dsb();
}
