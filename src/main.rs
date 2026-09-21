#![no_std]
#![no_main]

use teensy4_bsp::{self as bsp, board, hal::pit::Channel};
mod console;
mod line;
mod protocol;
mod usb_host;
mod xeryon;

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pins,
        mut gpio2,
        mut pit,
        usb,
        mut ccm,
        mut ccm_analog,
        mut gpio3,
        mut iomuxc_gpr,
        ..
    } = board::t41(board::instances());

    let led = board::led(&mut gpio2, pins.p13);
    led.set();

    let mut core = cortex_m::Peripherals::take().unwrap();
    usb_host::start_tick(&mut core.SYST);
    let mut console = console::Console::new(usb);
    usb_host::init(&mut ccm, &mut ccm_analog, &mut gpio3, &mut iomuxc_gpr);
    log::info!("Teensy USB2 host scanner ready; connect one device to the host header");
    let mut last_report = usb_host::millis();

    // The BSP configures the PIT clock. Toggle every 500 ms (one full blink/s).
    let channel = Channel::Chan0;
    pit.set_load_timer_value(channel, board::PERCLK_FREQUENCY / 2);
    pit.enable(channel);
    loop {
        console.poll(usb_host::millis());
        usb_host::poll();
        if pit.is_elapsed(channel) {
            pit.clear_elapsed(channel);
            led.toggle();
        }
        let now = usb_host::millis();
        if now.wrapping_sub(last_report) >= 5_000 {
            last_report = now;
            log::info!("Scanner alive ({} ms); current device:", now);
            usb_host::report();
        }
        // SysTick wakes us every millisecond; host events can wake us sooner.
        // USB1 is polled, so an unopened serial terminal never blocks scanning.
        cortex_m::asm::wfi();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    // A stopped heartbeat indicates a fault; no debugger or USB host is required.
    loop {
        core::hint::spin_loop();
    }
}
