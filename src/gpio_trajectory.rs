//! XD-C v3.0 manual sections 5.1 / 7.2 (2025-07-28): GPIO=2, STPS=1.
//! Teensy 7 -> bank 1 pin 23 DI0/PULSE, 8 -> pin 25 DI1/DIRECTION,
//! 9 -> pin 27 DI2/ENABLE; Teensy GND -> pin 19 GND. These inputs are
//! documented 3.3 V, matching Teensy outputs: no level shifter for this XD-C.
//! Never connect 5 V/48 V pins to Teensy. Verify bank/revision before wiring;
//! a different controller with higher-voltage inputs requires a suitable buffer.
//! External 10k pulldowns on PULSE and ENABLE keep them inactive during reset.
//! Hold INDEX (pin 30) low to prevent unintended index searches.
//! Direction high selects increment; verify physical positive direction unloaded.
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
    epoch: u32,
    heartbeat: u32,
    last_rise: u32,
}
impl Driver {
    fn low(&mut self) {
        self.enable.clear();
        self.pulse.clear();
        self.timer.set_output_interrupt_on_compare(Ocr::OCR1, false);
    }
}
pub fn init(
    mut timer: Gpt,
    gpio: &mut hal::gpio::Port,
    mut pulse: common::P7,
    mut direction: common::P8,
    mut enable: common::P9,
) {
    let cfg = hal::iomuxc::Config::zero()
        .set_drive_strength(hal::iomuxc::DriveStrength::R0_7)
        .set_pull_keeper(Some(hal::iomuxc::PullKeeper::Pulldown100k));
    hal::iomuxc::configure(&mut pulse, cfg);
    hal::iomuxc::configure(&mut direction, cfg);
    hal::iomuxc::configure(&mut enable, cfg);
    // Clear output latches before switching pad direction, including warm starts.
    emergency_low();
    let pulse = gpio.output(pulse).unwrap();
    pulse.clear();
    let direction = gpio.output(direction).unwrap();
    direction.clear();
    let enable = gpio.output(enable).unwrap();
    enable.clear();
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
            if let Some(p) = d.run.as_mut() {
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
        (1 << <common::P7 as Pin<2>>::OFFSET) | (1 << <common::P9 as Pin<2>>::OFFSET)
    );
}
#[unsafe(no_mangle)]
extern "C" fn GPT1() {
    interrupt::free(|cs| {
        if let Some(d) = DRIVER.borrow(cs).borrow_mut().as_mut() {
            d.timer.clear_elapsed(Ocr::OCR1);
            let now = d.timer.count();
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
