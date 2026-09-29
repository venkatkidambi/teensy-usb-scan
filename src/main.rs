#![no_std]
#![no_main]

use teensy4_bsp::{self as bsp, board, hal::pit::Channel};
mod command;
mod console;
mod experiment;
mod experiment_data;
mod fault;
mod line;
mod motor;
mod plunger;
mod plunger_config;
mod pressure;
mod pressure_i2c;
mod protocol;
mod usb_host;
mod xeryon;
mod xeryon_motion;

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        pins,
        mut gpio2,
        mut gpio4,
        flexpwm4,
        adc1,
        lpi2c1,
        mut pit,
        usb,
        mut ccm,
        mut ccm_analog,
        mut gpio3,
        mut iomuxc_gpr,
        ..
    } = board::t41(board::instances());

    // The motor HAL uses GPIO2/4, not the fast GPIO7/9 aliases.
    // Select the owned pads explicitly rather than relying on reset/loader state.
    use teensy4_bsp::{hal::iomuxc::gpio::Pin, pins::common, ral};
    const MOTOR_GPIO2_PINS: u32 = (1 << <common::P6 as Pin<2>>::OFFSET)
        | (1 << <common::P11 as Pin<2>>::OFFSET)
        | (1 << <common::P12 as Pin<2>>::OFFSET);
    const MOTOR_GPIO4_PINS: u32 =
        (1 << <common::P4 as Pin<4>>::OFFSET) | (1 << <common::P5 as Pin<4>>::OFFSET);
    ral::modify_reg!(ral::iomuxc_gpr, iomuxc_gpr, GPR27, |v| v
        & !MOTOR_GPIO2_PINS);
    ral::modify_reg!(ral::iomuxc_gpr, iomuxc_gpr, GPR29, |v| v
        & !MOTOR_GPIO4_PINS);
    cortex_m::asm::dsb();

    motor::init(
        motor::Pins {
            pwm: pins.p2,
            ain1: pins.p4,
            ain2: pins.p5,
            standby: pins.p6,
            c1: pins.p11,
            c2: pins.p12,
        },
        &mut gpio2,
        &mut gpio4,
        flexpwm4,
    );
    let mut pressure_bus = pressure_i2c::PressureI2c::new(lpi2c1, pins.p18, pins.p19);
    let mut pressure = pressure::Pressure::new();
    let mut pot = motor::Pot::new(adc1, pins.p20);
    let led = board::led(&mut gpio2, pins.p13);
    led.set();

    let mut core = cortex_m::Peripherals::take().unwrap();
    usb_host::start_tick(&mut core.SYST);
    // Zero-initialized DTCM (.bss), never materialized as a large stack value.
    // SAFETY: entry runs once; the foreground engine exclusively owns this array.
    static mut EXPERIMENT_SAMPLES: [experiment_data::Record; experiment_data::CAPACITY] =
        [experiment_data::Record::EMPTY; experiment_data::CAPACITY];
    let samples = unsafe { &mut *core::ptr::addr_of_mut!(EXPERIMENT_SAMPLES) };
    let mut experiment = experiment::Engine::new(samples);
    let mut console = console::Console::new(usb);
    usb_host::init(&mut ccm, &mut ccm_analog, &mut gpio3, &mut iomuxc_gpr);
    log::info!("Teensy USB2 host scanner ready; connect one device to the host header");
    let mut last_report = usb_host::millis();

    // The BSP configures the PIT clock. Toggle every 500 ms (one full blink/s).
    let channel = Channel::Chan0;
    pit.set_load_timer_value(channel, board::PERCLK_FREQUENCY / 2);
    pit.enable(channel);
    fault::heartbeat(usb_host::millis());
    loop {
        fault::mark(match experiment.state {
            experiment::State::MoveToStart => 2,
            experiment::State::Armed => 3,
            experiment::State::Running => 4,
            _ => 1,
        });
        let now = usb_host::millis();
        pressure.poll(now, &mut pressure_bus);
        pot.poll(now);
        usb_host::poll();
        experiment.poll(
            usb_host::millis(),
            &usb_host::telemetry(),
            pressure.reading,
            &mut usb_host::ExperimentActuator,
        );
        console.poll(
            usb_host::millis(),
            pot.value,
            pressure.reading,
            &mut experiment,
        );
        if pit.is_elapsed(channel) {
            pit.clear_elapsed(channel);
            led.toggle();
        }
        let now = usb_host::millis();
        if !experiment.owns_hardware() && now.wrapping_sub(last_report) >= 5_000 {
            last_report = now;
            log::info!("Scanner alive ({} ms); current device:", now);
            usb_host::report();
        }
        // SysTick wakes us every millisecond; host events can wake us sooner.
        // USB1 is polled, so an unopened serial terminal never blocks scanning.
        fault::heartbeat(usb_host::millis());
        // Drain/refill I2C FIFOs at bus speed, with USB and motion still serviced
        // on every iteration. Sleep only between sensor transactions.
        if !pressure.busy() && !usb_host::rx_pending() {
            cortex_m::asm::wfi();
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    fault::panic(info)
}
