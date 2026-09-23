//! GPIO plunger driver, ported from the supplied Motor.cpp pin assignments.
use crate::plunger::{self, Motion};
use core::cell::RefCell;
use cortex_m::{
    interrupt::{self, Mutex},
    peripheral::NVIC,
};
use teensy4_bsp::{
    hal::{
        self,
        flexpwm::{self, Channel, SM},
    },
    pins::common,
    ral,
};

const SM: SM = SM::SM2;
// Hardware PWM at ~4.58 kHz with the BSP's 150 MHz IPG clock.
const PERIOD: i16 = 32767;
static MOTOR: Mutex<RefCell<Option<Motor>>> = Mutex::new(RefCell::new(None));
struct Motor {
    pwm: flexpwm::Pwm,
    ain1: hal::gpio::Output,
    ain2: hal::gpio::Output,
    standby: hal::gpio::Output,
    c1: hal::gpio::Input,
    c2: hal::gpio::Input,
    previous: u8,
    motion: Motion,
    last_command: u32,
    started: u32,
    pot: Option<u16>,
    pot_ms: u32,
}
impl Motor {
    fn state(&self) -> u8 {
        (u8::from(self.c1.is_set()) << 1) | u8::from(self.c2.is_set())
    }
    fn stop(&mut self) {
        self.standby.clear();
        self.ain1.clear();
        self.ain2.clear();
        self.pwm.clear_load_ok(SM.mask());
        self.pwm.set_turn_off(SM, Channel::A, 0);
        self.pwm.set_load_ok(SM.mask());
        self.motion.stop();
    }
}
pub struct Pins {
    pub pwm: common::P2,
    pub ain1: common::P3,
    pub ain2: common::P4,
    pub standby: common::P5,
    pub c1: common::P11,
    pub c2: common::P12,
}
pub fn init(
    mut pins: Pins,
    gpio2: &mut hal::gpio::Port,
    gpio4: &mut hal::gpio::Port,
    mut pwm: flexpwm::Pwm,
) {
    let output = hal::iomuxc::Config::zero().set_drive_strength(hal::iomuxc::DriveStrength::R0_7);
    hal::iomuxc::configure(&mut pins.pwm, output);
    hal::iomuxc::configure(&mut pins.ain1, output);
    hal::iomuxc::configure(&mut pins.ain2, output);
    hal::iomuxc::configure(&mut pins.standby, output);
    // Teensy Arduino pinMode enables the input path (SION), including outputs.
    // Keep pad readback enabled for diagnostics and force encoder input routing.
    hal::iomuxc::set_sion(&mut pins.ain1);
    hal::iomuxc::set_sion(&mut pins.ain2);
    hal::iomuxc::set_sion(&mut pins.standby);
    hal::iomuxc::set_sion(&mut pins.c1);
    hal::iomuxc::set_sion(&mut pins.c2);
    let standby = gpio4.output(pins.standby).unwrap();
    standby.clear();
    let ain1 = gpio4.output(pins.ain1).unwrap();
    ain1.clear();
    let ain2 = gpio4.output(pins.ain2).unwrap();
    ain2.clear();
    let pullup = hal::iomuxc::Config::zero()
        .set_pull_keeper(Some(hal::iomuxc::PullKeeper::Pullup22k))
        .set_hysteresis(hal::iomuxc::Hysteresis::Enabled);
    hal::iomuxc::configure(&mut pins.c1, pullup);
    hal::iomuxc::configure(&mut pins.c2, pullup);
    let c1 = gpio2.input(pins.c1).unwrap();
    let c2 = gpio2.input(pins.c2).unwrap();
    gpio2
        .set_interrupt(&c1, Some(hal::gpio::Trigger::EitherEdge))
        .unwrap();
    gpio2
        .set_interrupt(&c2, Some(hal::gpio::Trigger::EitherEdge))
        .unwrap();
    c1.clear_triggered();
    c2.clear_triggered();
    hal::iomuxc::flexpwm::prepare(&mut pins.pwm);
    pwm.set_run(flexpwm::Mask::empty());
    pwm.clear_load_ok(SM.mask());
    pwm.set_output_masks(flexpwm::OutputMasks::empty());
    pwm.set_clock_select(SM, flexpwm::ClockSelect::Ipg);
    pwm.set_prescaler(SM, flexpwm::Prescaler::Prescaler1);
    pwm.set_pair_operation(SM, flexpwm::PairOperation::Independent);
    pwm.set_wait_enable(SM, true);
    pwm.set_debug_enable(SM, false);
    pwm.set_load_mode(SM, flexpwm::LoadMode::reload_full());
    pwm.set_load_frequency(SM, 1);
    pwm.set_initial_count(SM, 0);
    pwm.set_value(SM, flexpwm::FULL_RELOAD_VALUE_REGISTER, PERIOD);
    pwm.set_turn_on(SM, Channel::A, 0);
    pwm.set_turn_off(SM, Channel::A, 0);
    pwm.set_output_enable(SM.mask(), Channel::A);
    pwm.set_load_ok(SM.mask());
    pwm.set_run(SM.mask());
    let mut motor = Motor {
        pwm,
        ain1,
        ain2,
        standby,
        c1,
        c2,
        previous: 0,
        motion: Motion::default(),
        last_command: 0,
        started: 0,
        pot: None,
        pot_ms: 0,
    };
    motor.previous = motor.state();
    interrupt::free(|cs| *MOTOR.borrow(cs).borrow_mut() = Some(motor));
    // SAFETY: pins and shared state are initialized; this module owns this IRQ.
    unsafe {
        NVIC::unmask(ral::Interrupt::GPIO2_COMBINED_0_15);
    }
}

pub fn command(up: bool, speed: u8, um: u32, now: u32) -> Result<(), &'static str> {
    interrupt::free(|cs| {
        let mut slot = MOTOR.borrow(cs).borrow_mut();
        let m = slot.as_mut().ok_or("Plunger unavailable")?;
        if now.wrapping_sub(m.pot_ms) >= 100 || !m.motion.limits.allows(up, m.pot) {
            return Err("Potentiometer limit reached or feedback unavailable");
        }
        m.motion.start(up, speed, um)?;
        m.last_command = now;
        m.started = now;
        m.standby.clear();
        if up {
            m.ain1.set();
            m.ain2.clear();
        } else {
            m.ain1.clear();
            m.ain2.set();
        }
        m.pwm.clear_load_ok(SM.mask());
        m.pwm.set_turn_off(
            SM,
            Channel::A,
            ((u32::from(speed) * PERIOD as u32) / 255) as i16,
        );
        m.pwm.set_load_ok(SM.mask());
        m.standby.set();
        Ok(())
    })
}
pub fn stop() {
    interrupt::free(|cs| {
        if let Some(m) = MOTOR.borrow(cs).borrow_mut().as_mut() {
            m.stop();
        }
    });
}
pub fn set_limits(min: u8, max: u8) -> Result<(), &'static str> {
    let limits = plunger::Limits::new(min, max)?;
    interrupt::free(|cs| {
        let mut slot = MOTOR.borrow(cs).borrow_mut();
        let m = slot.as_mut().ok_or("Plunger unavailable")?;
        if m.motion.running {
            return Err("Stop the plunger before changing limits");
        }
        m.motion.limits = limits;
        Ok(())
    })
}
fn pot_sample(value: Option<u16>, now: u32) {
    interrupt::free(|cs| {
        if let Some(m) = MOTOR.borrow(cs).borrow_mut().as_mut() {
            m.pot = value;
            m.pot_ms = now;
            if m.motion.running && !m.motion.limits.allows(m.motion.up, value) {
                m.stop();
            }
        }
    });
}
pub fn zero() -> Result<(), &'static str> {
    interrupt::free(|cs| {
        let mut slot = MOTOR.borrow(cs).borrow_mut();
        let m = slot.as_mut().ok_or("Plunger unavailable")?;
        m.motion.zero()?;
        m.previous = m.state();
        Ok(())
    })
}
pub fn heartbeat(now: u32) {
    interrupt::free(|cs| {
        if let Some(m) = MOTOR.borrow(cs).borrow_mut().as_mut() {
            m.last_command = now;
        }
    });
}
pub fn snapshot() -> Motion {
    interrupt::free(|cs| {
        MOTOR
            .borrow(cs)
            .borrow()
            .as_ref()
            .map(|m| m.motion)
            .unwrap_or_default()
    })
}
pub fn io_snapshot() -> crate::plunger::Io {
    interrupt::free(|cs| {
        let slot = MOTOR.borrow(cs).borrow();
        let Some(m) = slot.as_ref() else {
            return crate::plunger::Io::default();
        };
        crate::plunger::Io {
            standby: m.standby.is_pad_high(),
            ain1: m.ain1.is_pad_high(),
            ain2: m.ain2.is_pad_high(),
            encoder: m.state(),
            pwm_count: m.pwm.sm_count(SM),
            pwm_compare: m.pwm.turn_off(SM, Channel::A),
            pwm_running: m.pwm.run().contains(SM.mask()),
            pwm_enabled: m.pwm.output_enable(Channel::A).contains(SM.mask()),
        }
    })
}
// Called from SysTick: motor stops even if foreground USB or a peripheral stalls.
pub fn tick(now: u32) {
    interrupt::free(|cs| {
        if let Some(m) = MOTOR.borrow(cs).borrow_mut().as_mut() {
            if m.motion.running
                && (now.wrapping_sub(m.last_command) >= 5000
                    || now.wrapping_sub(m.pot_ms) >= 100
                    || now.wrapping_sub(m.started) >= 60000)
            {
                m.stop();
            }
        }
    });
}
#[unsafe(no_mangle)]
extern "C" fn GPIO2_COMBINED_0_15() {
    interrupt::free(|cs| {
        if let Some(m) = MOTOR.borrow(cs).borrow_mut().as_mut() {
            m.c1.clear_triggered();
            m.c2.clear_triggered();
            let current = m.state();
            let delta = plunger::encoder_delta(m.previous, current);
            m.previous = current;
            if m.motion.step(delta) {
                m.stop();
            }
        }
    });
    cortex_m::asm::dsb();
}

/// ADC1 is owned exclusively here. Conversion is started and polled separately.
pub struct Pot {
    _adc: hal::adc::Adc,
    input: hal::adc::AnalogInput,
    pending: bool,
    last: u32,
    pub value: Option<u16>,
}
impl Pot {
    pub fn new(mut adc: hal::adc::Adc, pin: common::P20) -> Self {
        adc.set_resolution(hal::adc::ResolutionBits::Res12);
        adc.set_averaging(hal::adc::AveragingCount::Avg16);
        let input = adc.input::<_, 1>(pin).unwrap();
        Self {
            _adc: adc,
            input,
            pending: false,
            last: 0,
            value: None,
        }
    }
    pub fn poll(&mut self, now: u32) {
        // SAFETY: self owns ADC1 via _adc; only this foreground method accesses
        // its registers after setup. No ADC interrupts or DMA are enabled.
        let adc = unsafe { &*ral::adc::ADC1 };
        if self.pending {
            if ral::read_reg!(ral::adc, adc, HS, COCO0) != 0 {
                self.value = Some(ral::read_reg!(ral::adc, adc, R0) as u16);
                pot_sample(self.value, now);
                self.pending = false;
            } else if now.wrapping_sub(self.last) >= 10 {
                self.value = None;
                pot_sample(None, now);
                self.pending = false;
            }
        } else if now.wrapping_sub(self.last) >= 5 {
            self.last = now;
            ral::write_reg!(ral::adc, adc, HC0, self.input.channel());
            self.pending = true;
        }
    }
}
