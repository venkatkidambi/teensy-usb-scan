//! HX711 GPIO transport: Teensy A1/p15 DOUT, A0/p14 SCK, channel B / gain 32.
use crate::pressure::Bus;
use cortex_m::interrupt;
use teensy4_bsp::{board, hal, pins::common};

pub struct Hx711 {
    data: hal::gpio::Input,
    clock: hal::gpio::Output,
}
impl Hx711 {
    pub fn new(port: &mut hal::gpio::Port, mut data: common::P15, mut clock: common::P14) -> Self {
        hal::iomuxc::configure(
            &mut data,
            hal::iomuxc::Config::zero().set_pull_keeper(Some(hal::iomuxc::PullKeeper::Pullup100k)),
        );
        hal::iomuxc::configure(
            &mut clock,
            hal::iomuxc::Config::zero().set_drive_strength(hal::iomuxc::DriveStrength::R0),
        );
        let clock = port.output(clock).unwrap();
        clock.clear();
        // Reset serial framing after a Teensy reboot, even if the previous
        // firmware stopped mid-word. Power-down requires SCK high >60 us.
        clock.set();
        cortex_m::asm::delay(board::ARM_FREQUENCY / 1_000_000 * 80);
        clock.clear();
        Self {
            data: port.input(data).unwrap(),
            clock,
        }
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
