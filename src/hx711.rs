//! HX711 GPIO transport: Teensy A1/p15 DOUT, A0/p14 SCK, channel B / gain 32.
use crate::pressure::Bus;
use cortex_m::interrupt;
use hal::iomuxc::gpio::Pin;
use teensy4_bsp::{board, hal, pins::common, ral};

pub struct Hx711 {
    data: hal::gpio::Input,
    clock: hal::gpio::Output,
}
impl Hx711 {
    pub fn new(
        gpr: &mut ral::iomuxc_gpr::IOMUXC_GPR,
        mut data: common::P15,
        mut clock: common::P14,
    ) -> Self {
        // Match Teensy Arduino INPUT_PULLUP/OUTPUT, including hysteresis and SION.
        // GPIO6 is the fast alias of GPIO1; Arduino uses it for A0 and A1.
        const DATA_BIT: u32 = <common::P15 as Pin<1>>::OFFSET;
        const CLOCK_BIT: u32 = <common::P14 as Pin<1>>::OFFSET;
        const MASK: u32 = (1 << DATA_BIT) | (1 << CLOCK_BIT);
        hal::iomuxc::configure(
            &mut data,
            hal::iomuxc::Config::zero()
                .set_drive_strength(hal::iomuxc::DriveStrength::R0_7)
                .set_pull_keeper(Some(hal::iomuxc::PullKeeper::Pullup22k))
                .set_hysteresis(hal::iomuxc::Hysteresis::Enabled),
        );
        hal::iomuxc::configure(
            &mut clock,
            hal::iomuxc::Config::zero().set_drive_strength(hal::iomuxc::DriveStrength::R0_7),
        );
        hal::iomuxc::set_sion(&mut data);
        hal::iomuxc::set_sion(&mut clock);

        // SAFETY: board::t41 neither uses nor returns GPIO6. This module owns its
        // A0/A1 bits; all other GPIO users keep their pads on the standard banks.
        let gpio6 = unsafe { ral::gpio::GPIO6::instance() };
        // Preload low BEFORE enabling/routing the output; do not power down the
        // sensor during startup. This matches the working HX711 Arduino begin().
        ral::write_reg!(ral::gpio, gpio6, DR_CLEAR, 1 << CLOCK_BIT);
        let mut port = hal::gpio::Port::new(gpio6);
        let output = hal::gpio::Output::without_pin(&mut port, CLOCK_BIT);
        let input = hal::gpio::Input::without_pin(&mut port, DATA_BIT);
        // The pin types describe GPIO1. Prepare their common ALT5 mux, then
        // explicitly select GPIO6 for just these two pads in GPR26.
        hal::iomuxc::gpio::prepare(&mut clock);
        hal::iomuxc::gpio::prepare(&mut data);
        ral::modify_reg!(ral::iomuxc_gpr, gpr, GPR26, |v| v | MASK);
        cortex_m::asm::dsb();
        Self {
            data: input,
            clock: output,
        }
    }
    pub fn with_pin_levels(
        &self,
        mut reading: crate::pressure::Reading,
    ) -> crate::pressure::Reading {
        reading.pins = Some(crate::pressure::PinLevels {
            dout_high: self.data.is_set(),
            sck_high: self.clock.is_pad_high(),
            sck_requested_high: self.clock.is_set(),
        });
        reading
    }
    fn pulse(&self) -> bool {
        // Only the high pulse masks interrupts. An ISR stretching SCK high
        // beyond 60 us would power down the HX711 and lose channel selection.
        let bit = interrupt::free(|_| {
            self.clock.set();
            cortex_m::asm::delay(board::ARM_FREQUENCY / 1_000_000);
            let bit = self.data.is_set();
            self.clock.clear();
            cortex_m::asm::dsb(); // Complete the low write before unmasking IRQs.
            bit
        });
        // Low may safely stretch while encoder/USB interrupts run.
        cortex_m::asm::delay(board::ARM_FREQUENCY / 1_000_000);
        bit
    }
}
impl Bus for Hx711 {
    fn ready(&self) -> bool {
        !self.data.is_set()
    }
    fn read(&mut self) -> Option<u32> {
        let word = crate::pressure::read_channel_b(|| self.pulse());
        // DOUT must return high after clock 25; catches a stuck-low data line.
        self.data.is_set().then_some(word)
    }
}
