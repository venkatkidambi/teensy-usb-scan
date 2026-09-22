//! Rust owns board initialization and logging. TinyUSB owns USB2 after init.
use core::{
    cell::RefCell,
    fmt,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use cortex_m::peripheral::{NVIC, SYST, syst::SystClkSource};
use teensy4_bsp::{board, hal, ral};

static MILLIS: AtomicU32 = AtomicU32::new(0);
static ATTACHES: AtomicU32 = AtomicU32::new(0);
static REMOVES: AtomicU32 = AtomicU32::new(0);
static INTERRUPTS: AtomicU32 = AtomicU32::new(0);
static XD_C: AtomicBool = AtomicBool::new(false);
static MOTION: cortex_m::interrupt::Mutex<RefCell<crate::xeryon_motion::Motion>> =
    cortex_m::interrupt::Mutex::new(RefCell::new(crate::xeryon_motion::Motion::new()));
static TELEMETRY: cortex_m::interrupt::Mutex<RefCell<crate::xeryon::Decoder>> =
    cortex_m::interrupt::Mutex::new(RefCell::new(crate::xeryon::Decoder::new()));
static CACHE: cortex_m::interrupt::Mutex<RefCell<crate::protocol::Telemetry>> =
    cortex_m::interrupt::Mutex::new(RefCell::new(crate::protocol::Telemetry::new()));

pub fn telemetry() -> crate::protocol::Telemetry {
    cortex_m::interrupt::free(|cs| {
        let mut t = *CACHE.borrow(cs).borrow();
        t.xeryon_motion = Some(*MOTION.borrow(cs).borrow());
        t
    })
}

pub fn heartbeat(now: u32) {
    cortex_m::interrupt::free(|cs| MOTION.borrow(cs).borrow_mut().heartbeat(now));
}

pub fn move_xeryon(up: bool, nm: u32, speed: u32, now: u32) -> Result<i32, &'static str> {
    let t = telemetry();
    if t.xeryon_motion.is_some_and(|m| m.state == 1) {
        return Err("Wait for the Xeryon move to finish or press Stop");
    }
    let target = crate::xeryon_motion::target(&t, up, nm, now)?;
    let mut frame = crate::protocol::Frame::new();
    crate::xeryon_motion::write_move(&mut frame, target, speed)?;
    // SAFETY: called only in main context; C copies the complete command.
    if !unsafe {
        usb_serial_send(
            frame.pending().as_ptr(),
            frame.pending().len() as u32,
            false,
        )
    } {
        return Err("Xeryon USB transmit unavailable");
    }
    cortex_m::interrupt::free(|cs| MOTION.borrow(cs).borrow_mut().start(target, now));
    Ok(target)
}

pub fn stop_xeryon() -> Result<(), &'static str> {
    if !crate::xeryon_motion::ready(&telemetry()) {
        return Err("Xeryon is not connected");
    }
    // SAFETY: main context, static command, bounded nonblocking copy.
    let sent = unsafe { usb_serial_send(b"STOP\n".as_ptr(), 5, true) };
    cortex_m::interrupt::free(|cs| MOTION.borrow(cs).borrow_mut().state = if sent { 3 } else { 4 });
    if sent {
        Ok(())
    } else {
        Err("Xeryon Stop could not be queued")
    }
}

pub fn disconnect_xeryon() {
    let active = cortex_m::interrupt::free(|cs| MOTION.borrow(cs).borrow().state == 1);
    if active {
        let _ = stop_xeryon();
    }
}

fn reset_telemetry() {
    cortex_m::interrupt::free(|cs| {
        *TELEMETRY.borrow(cs).borrow_mut() = crate::xeryon::Decoder::new();
    });
}

unsafe extern "C" {
    fn usb_serial_send(data: *const u8, len: u32, stop: bool) -> bool;
    fn scanner_init() -> bool;
    fn scanner_task();
    fn scanner_interrupt();
    fn scanner_report() -> bool;
    fn scanner_registers(command: *mut u32, status: *mut u32, port: *mut u32, mode: *mut u32);
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
    let now = MILLIS.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
    crate::motor::tick(now);
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
    let t = telemetry();
    let stop = cortex_m::interrupt::free(|cs| MOTION.borrow(cs).borrow_mut().poll(&t, millis()));
    if stop {
        let result = stop_xeryon();
        cortex_m::interrupt::free(|cs| MOTION.borrow(cs).borrow_mut().state = 4);
        log::warn!(
            "Xeryon motion interrupted (feedback, connection, fault or timeout); Stop queued={}",
            result.is_ok()
        );
    }
}

pub fn report() {
    let (mut command, mut status, mut port, mut mode) = (0, 0, 0, 0);
    // SAFETY: initialized USB2 registers are read only, into live local values.
    unsafe { scanner_registers(&mut command, &mut status, &mut port, &mut mode) };
    log::info!(
        "USB2 connected={} irqs={} USBCMD={:08x} USBSTS={:08x} PORTSC1={:08x} USBMODE={:08x}",
        port & 1 != 0,
        INTERRUPTS.load(Ordering::Relaxed),
        command,
        status,
        port,
        mode
    );
    // SAFETY: shared C scanner state is only touched from main context.
    if !unsafe { scanner_report() } {
        log::info!("No device descriptor available yet");
    }
}

#[unsafe(no_mangle)]
extern "C" fn USB_OTG2() {
    INTERRUPTS.fetch_add(1, Ordering::Relaxed);
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
    // Confirmed descriptor identity of the user's XD-C. Other CDC devices
    // retain escaped raw logging instead of receiving Xeryon interpretations.
    XD_C.store(vid == 0x04d8 && pid == 0xef4a, Ordering::Relaxed);
    cortex_m::interrupt::free(|cs| {
        let mut cache = CACHE.borrow(cs).borrow_mut();
        if !mounted {
            cache.reset();
        }
        cache.vid = vid;
        cache.pid = pid;
        cache.mounted = mounted;
    });
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
    XD_C.store(false, Ordering::Relaxed);
    reset_telemetry();
    cortex_m::interrupt::free(|cs| {
        *MOTION.borrow(cs).borrow_mut() = crate::xeryon_motion::Motion::new()
    });
    cortex_m::interrupt::free(|cs| CACHE.borrow(cs).borrow_mut().reset());
    log::info!("Device removed: address={}", address);
}

#[unsafe(no_mangle)]
extern "C" fn scanner_serial_event(event: u8, address: u8, interface: u8) {
    reset_telemetry();
    cortex_m::interrupt::free(|cs| {
        let mut cache = CACHE.borrow(cs).borrow_mut();
        cache.cdc = match event {
            0 => 1,
            1 => 2,
            2 => 0,
            3 => 3,
            _ => 4,
        };
        if event == 2 {
            cache.samples.fill(crate::protocol::Sample::default());
        }
    });
    let message = match event {
        0 => "CDC ACM detected",
        1 => "CDC ACM open: 115200 8-N-1, DTR asserted",
        2 => "CDC ACM disconnected",
        3 => "CDC ACM setup failed; unplug/reconnect to retry",
        4 => "CDC ACM setup timed out; unplug/reconnect to retry",
        _ => return,
    };
    log::info!("USB2 addr={} interface={}: {}", address, interface, message);
}

struct SerialBytes<'a>(&'a [u8]);

impl fmt::Display for SerialBytes<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use fmt::Write;
        for &byte in self.0 {
            for escaped in core::ascii::escape_default(byte) {
                f.write_char(char::from(escaped))?;
            }
        }
        Ok(())
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn scanner_serial_rx(data: *const u8, len: u32) {
    if data.is_null() || len > 64 {
        return;
    }
    // SAFETY: C supplies a live stack buffer for this main-context callback.
    let bytes = unsafe { core::slice::from_raw_parts(data, len as usize) };
    if XD_C.load(Ordering::Relaxed) {
        // One bounded critical section per received chunk, no string formatting
        // or USB IO inside it. Foreground USB1 serializes a copy at 10 Hz.
        let now = millis();
        cortex_m::interrupt::free(|cs| {
            let mut decoder = TELEMETRY.borrow(cs).borrow_mut();
            let mut cache = CACHE.borrow(cs).borrow_mut();
            for &byte in bytes {
                match decoder.push(byte) {
                    Some(Ok(record)) => cache.record(record.tag(), record.value(), now),
                    Some(Err(_)) => cache.malformed = cache.malformed.wrapping_add(1),
                    None => {}
                }
            }
        });
    } else {
        log::info!("USB2 CDC RX: {}", SerialBytes(bytes));
    }
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
