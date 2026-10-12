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
    burst: Option<crate::gpio_control::Burst>,
    index: hal::gpio::Output,
    last_edge: u32,
    epoch: u32,
    heartbeat: u32,
    last_rise: u32,
}
impl Driver {
    fn low(&mut self) {
        self.enable.clear();
        self.pulse.clear();
        self.index.clear();
        self.last_edge = self.timer.count();
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
            burst: None,
            index,
            last_edge: 0,
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
    interrupt::free(|cs| {
        let mut slot = DRIVER.borrow(cs).borrow_mut();
        let d = slot.as_mut().ok_or("GPIO unavailable")?;
        if d.burst
            .as_ref()
            .is_some_and(|b| b.status == Status::Running)
        {
            return Err("GPIO pulse test busy");
        }
        d.burst = None;
        d.low();
        d.direction.set();
        d.run = Some(Pulses::new(trajectory));
        d.epoch = d.timer.count();
        d.heartbeat = d.epoch;
        d.last_rise = d.epoch;
        d.enable.set();
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
            if let Some(b) = d.burst.as_mut() {
                b.stop(Status::Stopped);
            }
            if let Some(p) = d.run.as_mut() {
                p.stop();
            }
        }
    });
}
/// Called only after the USB coordinator verifies configuration and takes ownership.
pub fn start_steps(positive: bool, steps: u32) -> Result<(), &'static str> {
    let burst = crate::gpio_control::Burst::new(steps)?;
    interrupt::free(|cs| {
        let mut slot = DRIVER.borrow(cs).borrow_mut();
        let d = slot.as_mut().ok_or("GPIO unavailable")?;
        if d.run.as_ref().is_some_and(|p| p.status == Status::Running)
            || d.burst
                .as_ref()
                .is_some_and(|b| b.status == Status::Running)
            || d.timer.count().wrapping_sub(d.last_edge) < crate::gpio_control::HALF_US
        {
            return Err("GPIO busy or direction hold incomplete");
        }
        d.low();
        d.run = None;
        if positive {
            d.direction.set();
        } else {
            d.direction.clear();
        }
        d.burst = Some(burst);
        d.enable.set();
        cortex_m::asm::dsb();
        // Both DIR and ENABLE settle for >=500 us before the first rising edge.
        d.last_edge = d.timer.count();
        d.heartbeat = d.last_edge;
        d.timer.clear_elapsed(Ocr::OCR1);
        d.timer.set_output_compare_count(
            Ocr::OCR1,
            d.last_edge.wrapping_add(crate::gpio_control::HALF_US),
        );
        d.timer.set_output_interrupt_on_compare(Ocr::OCR1, true);
        Ok(())
    })
}

pub fn steps_status() -> Status {
    interrupt::free(|cs| {
        let mut slot = DRIVER.borrow(cs).borrow_mut();
        let Some(d) = slot.as_mut() else {
            return Status::Stopped;
        };
        d.heartbeat = d.timer.count();
        d.burst.as_ref().map_or(Status::Stopped, |b| b.status)
    })
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
            if let Some(b) = d.burst.as_mut() {
                if now.wrapping_sub(d.heartbeat) > 10_000 {
                    b.stop(Status::Fault);
                } else if now.wrapping_sub(d.last_edge) >= crate::gpio_control::HALF_US {
                    b.tick();
                }
                if b.status != Status::Running {
                    d.low();
                } else {
                    if b.high {
                        d.pulse.set();
                    } else {
                        d.pulse.clear();
                    }
                    cortex_m::asm::dsb();
                    d.last_edge = d.timer.count();
                    d.timer.set_output_compare_count(
                        Ocr::OCR1,
                        d.last_edge.wrapping_add(crate::gpio_control::HALF_US),
                    );
                }
                return;
            }
            if let Some(p) = d.run.as_mut() {
                p.tick(now.wrapping_sub(d.epoch));
                if now.wrapping_sub(d.heartbeat) > 10_000 {
                    p.status = Status::Fault;
                    p.high = false;
                }
                if p.status != Status::Running {
                    d.low();
                } else {
                    if !p.high {
                        d.pulse.clear();
                    }
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
                            d.low();
                            return;
                        }
                        d.pulse.set();
                        d.last_rise = d.timer.count();
                    }
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
