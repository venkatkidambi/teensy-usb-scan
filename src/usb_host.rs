//! Rust owns board initialization and logging. TinyUSB owns USB2 after init.
use core::{
    fmt,
    sync::atomic::{AtomicU32, Ordering},
};
use cortex_m::peripheral::{NVIC, SYST, syst::SystClkSource};
use teensy4_bsp::{board, hal, ral};

static MILLIS: AtomicU32 = AtomicU32::new(0);
static ATTACHES: AtomicU32 = AtomicU32::new(0);
static REMOVES: AtomicU32 = AtomicU32::new(0);

unsafe extern "C" {
    fn scanner_init() -> bool;
    fn scanner_task();
    fn scanner_interrupt();
    fn scanner_report() -> bool;
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
    MILLIS.fetch_add(1, Ordering::Relaxed);
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
}

pub fn report() {
    // SAFETY: shared C scanner state is only touched from main context.
    if !unsafe { scanner_report() } {
        log::info!("No device descriptor available yet");
    }
}

#[unsafe(no_mangle)]
extern "C" fn USB_OTG2() {
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
    log::info!("Device removed: address={}", address);
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
