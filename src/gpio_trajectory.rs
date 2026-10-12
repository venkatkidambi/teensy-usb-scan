//! XD-OEM manual sections 5.1 / 7.3: GPIO=2, STPS=1, 3.3 V logic.
//! Teensy 7/8/9/10 -> XD-OEM 13 PULSE / 14 DIRECTION / 15 ENABLE / 16 INDEX.
//! Ground is shared through USB host. INDEX is always driven low, never pulsed.
//! External pulldowns on PULSE, ENABLE and INDEX are needed to hold them low
//! before firmware initialization / during reset (internal pulls start at init).
//! Direction high is used for positive steps; verify sign against EPOS unloaded.
//! FlexPWM1 SM3/B (pin 7): 50 us slots, rise at 20 us, fall at 45 us.
//! DMA0 clears OUTEN at EVERY reload before its completion IRQ selects a pulse.
//! Missed IRQs therefore leave output disabled, never repeat a stale pulse.
//! The ISR must finish arming by 10 us: >=10 us direction setup, >=25 us hold.
//! GPT1 is the independent 1 MHz service/watchdog clock. No USB or waits here.
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
        dma::channel::{Channel as DmaChannel, Configuration},
        flexpwm::{self, Channel, SM},
        gpt::{ClockSource, Gpt, Mode},
    },
    pins::common,
    ral,
};
const SM: SM = SM::SM3;
// 150 MHz IPG / 2 = 75 MHz. Reject clock-policy changes at build time.
const _: () = assert!(board::IPG_FREQUENCY == 150_000_000 && TICK_US == 50);
const PERIOD: i16 = 3750;
const ARM_DEADLINE: i16 = 750; // 10 us
// DMA reads immutable, uncached DTCM, not a cached flash/stack buffer.
#[unsafe(link_section = ".bss.gpio_dma")]
static DMA_OFF: u16 = 0;
static DRIVER: Mutex<RefCell<Option<Driver>>> = Mutex::new(RefCell::new(None));
struct Driver {
    timer: Gpt,
    pwm: flexpwm::Pwm,
    dma: DmaChannel,
    direction: hal::gpio::Output,
    enable: hal::gpio::Output,
    run: Option<Pulses>,
    reference: Option<i32>,
    positive: bool,
    index: hal::gpio::Output,
    elapsed: u32,
    setup_slots: u32,
    serviced: u32,
    heartbeat: u32,
}
impl Driver {
    fn fail(&mut self) {
        self.reference = None;
        if let Some(p) = self.run.as_mut() {
            p.status = Status::Fault;
            p.high = false;
        }
        self.low();
    }
    fn low(&mut self) {
        self.enable.clear();
        self.pwm
            .set_output_enable(flexpwm::Mask::empty(), Channel::B);
        self.pwm.set_run(flexpwm::Mask::empty());
        self.dma.disable();
        self.dma.clear_interrupt();
        self.index.clear();
    }
}
pub fn init(
    mut timer: Gpt,
    mut pwm: flexpwm::Pwm,
    mut dma: DmaChannel,
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
    // PWM1 is exclusively owned here; disable it before muxing the pad.
    pwm.set_run(flexpwm::Mask::empty());
    pwm.set_output_enable(flexpwm::Mask::empty(), Channel::B);
    let regs = unsafe { &*ral::pwm::PWM1 };
    // Explicit local clock/sync, normal polarity and no deadtime/fractional delay.
    // Do not inherit loader state. No external PWM fault inputs are wired.
    ral::write_reg!(ral::pwm::sm, &regs.SM[3], SMCTRL2, 0);
    ral::write_reg!(ral::pwm::sm, &regs.SM[3], SMCTRL, 0);
    ral::write_reg!(ral::pwm::sm, &regs.SM[3], SMOCTRL, 0);
    ral::write_reg!(ral::pwm::sm, &regs.SM[3], SMFRCTRL, 0);
    ral::write_reg!(ral::pwm::sm, &regs.SM[3], SMDTCNT0, 0);
    ral::write_reg!(ral::pwm::sm, &regs.SM[3], SMDTCNT1, 0);
    ral::write_reg!(ral::pwm::sm, &regs.SM[3], SMDISMAP0, 0);
    ral::write_reg!(ral::pwm::sm, &regs.SM[3], SMINTEN, 0);
    ral::write_reg!(ral::pwm, regs, DTSRCSEL, 0);
    hal::iomuxc::flexpwm::prepare(&mut pulse);
    pwm.clear_load_ok(SM.mask());
    pwm.set_output_masks(flexpwm::OutputMasks::empty());
    pwm.set_clock_select(SM, flexpwm::ClockSelect::Ipg);
    pwm.set_prescaler(SM, flexpwm::Prescaler::Prescaler2);
    pwm.set_pair_operation(SM, flexpwm::PairOperation::Independent);
    pwm.set_wait_enable(SM, true);
    pwm.set_debug_enable(SM, false);
    pwm.set_load_mode(SM, flexpwm::LoadMode::reload_full());
    pwm.set_load_frequency(SM, 1);
    pwm.set_initial_count(SM, 0);
    pwm.set_value(SM, flexpwm::FULL_RELOAD_VALUE_REGISTER, PERIOD - 1);
    pwm.set_turn_on(SM, Channel::B, 1500); // 20 us
    pwm.set_turn_off(SM, Channel::B, 3375); // 45 us: 25 us high
    pwm.set_load_ok(SM.mask());
    // RT1060 RM 55.8.21: VALDE requests DMA at reload; DMA done clears RF.
    // DMAMUX source 39 = PWM1 SM3 value request (RM table 4-3).
    ral::write_reg!(ral::pwm::sm, &regs.SM[3], SMDMAEN, VALDE: 1);
    dma.disable();
    dma.reset();
    dma.set_channel_configuration(Configuration::enable(39));
    // SAFETY: DMA0 has exclusive ownership; fixed 16-bit addresses, no increments,
    // static DTCM source, PWM1 OUTEN destination. DMA can only disable outputs.
    unsafe {
        dma.set_source_address(core::ptr::addr_of!(DMA_OFF));
        dma.set_source_attributes::<u16>(0);
        dma.set_destination_address(core::ptr::addr_of!(regs.OUTEN).cast::<u16>().cast_mut());
        dma.set_destination_attributes::<u16>(0);
        dma.set_minor_loop_bytes(2);
        dma.set_transfer_iterations(1);
    }
    dma.set_disable_on_completion(false);
    dma.set_interrupt_on_completion(true);
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
            pwm,
            dma,
            direction,
            enable,
            run: None,
            reference: None,
            positive: false,
            index,
            elapsed: 0,
            setup_slots: 0,
            serviced: 0,
            heartbeat: 0,
        })
    });
    // SAFETY: DMA0 is owned here; channel 16 is unused. State is initialized.
    unsafe {
        NVIC::unmask(ral::Interrupt::DMA0_DMA16);
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
        d.pwm.set_output_enable(flexpwm::Mask::empty(), Channel::B);
        // Restart the slot clock even when handing off from the approach.
        // Leave the first period disabled so a previously high PWM latch clears.
        d.pwm.set_run(flexpwm::Mask::empty());
        d.dma.disable();
        d.enable.set();
        d.elapsed = 0;
        // 500 us enable setup; preserve enable across the approach handoff.
        d.setup_slots = if enabled { 0 } else { 500 / TICK_US };
        let now = d.timer.count();
        d.heartbeat = now;
        d.serviced = now;
        // In DMA mode RF is cleared by DMA done, not a CPU W1C write.
        let regs = unsafe { &*ral::pwm::PWM1 };
        ral::write_reg!(ral::pwm::sm, &regs.SM[3], SMDMAEN, 0);
        d.pwm.clear_status(SM, flexpwm::Status::all());
        ral::write_reg!(ral::pwm::sm, &regs.SM[3], SMDMAEN, VALDE: 1);
        d.dma.clear_complete();
        d.dma.clear_interrupt();
        d.dma.clear_error();
        // SAFETY: descriptor remains the fixed disable-only transfer from init.
        unsafe {
            d.dma.enable();
        }
        d.pwm.set_run(SM.mask());
        Ok(())
    })
}
pub fn status() -> Status {
    interrupt::free(|cs| {
        let mut slot = DRIVER.borrow(cs).borrow_mut();
        let Some(d) = slot.as_mut() else {
            return Status::Stopped;
        };
        let now = d.timer.count();
        if d.run
            .as_ref()
            .is_some_and(|p| matches!(p.status, Status::Running | Status::Complete))
            && (d.dma.is_error() || now.wrapping_sub(d.serviced) > 100)
        {
            d.fail();
        }
        d.heartbeat = now;
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
    // Safe before init too (BSP enables PWM clocks before this call).
    let pwm = unsafe { &*ral::pwm::PWM1 };
    ral::write_reg!(ral::pwm, pwm, OUTEN, 0);
    ral::write_reg!(ral::pwm, pwm, MCTRL, 0);
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
extern "C" fn DMA0_DMA16() {
    interrupt::free(|cs| {
        let mut slot = DRIVER.borrow(cs).borrow_mut();
        let Some(d) = slot.as_mut() else {
            return;
        };
        if !d.dma.is_interrupt() {
            return;
        }
        d.dma.clear_interrupt();
        let now = d.timer.count();
        // DMA has already disabled this slot. Never arm late or catch up.
        if d.dma.is_error()
            || now.wrapping_sub(d.serviced) > 75
            || now.wrapping_sub(d.heartbeat) > 10_000
            || d.pwm.sm_count(SM) >= ARM_DEADLINE
        {
            d.fail();
            return;
        }
        d.serviced = now;
        if d.setup_slots != 0 {
            d.setup_slots -= 1;
            return;
        }
        // Time advances by hardware reloads, not by ISR completion time.
        d.elapsed += TICK_US;
        let Some(p) = d.run.as_mut() else {
            d.low();
            return;
        };
        p.tick(d.elapsed);
        if matches!(p.status, Status::Fault | Status::Stopped) {
            d.fail();
            return;
        }
        d.positive = p.positive;
        if p.positive {
            d.direction.set();
        } else {
            d.direction.clear();
        }
        // Recheck AFTER trajectory evaluation, before enabling the compare output.
        // Direction setup >=10 us; preceding rise-to-direction hold >=25 us.
        if d.pwm.sm_count(SM) >= ARM_DEADLINE || d.timer.count().wrapping_sub(now) >= 10 {
            d.fail();
            return;
        }
        if p.high {
            d.pwm.set_output_enable(SM.mask(), Channel::B);
        }
        d.reference = Some(p.position);
        // Complete holds ENABLE and continues the watchdog until fresh endpoint
        // confirmation; DMA leaves PULSE disabled on all subsequent slots.
    });
    cortex_m::asm::dsb();
}
