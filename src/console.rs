//! USB1 is owned solely by the foreground loop. No heap, threads, or blocking IO.
use crate::protocol::{Command, Commands, Frame};
use core::{cell::RefCell, fmt::Write};
use cortex_m::interrupt::{self, Mutex};
use teensy4_bsp::usbd::{self, BusAdapter};
use usb_device::{
    bus::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
};
use usbd_serial::CdcAcmClass;

static MEMORY: usbd::EndpointMemory<1280> = usbd::EndpointMemory::new();
static ENDPOINTS: usbd::EndpointState<6> = usbd::EndpointState::new();
static DIAGNOSTIC: Mutex<RefCell<Frame>> = Mutex::new(RefCell::new(Frame::new()));
static LOGGER: Logger = Logger;
struct Logger;
impl log::Log for Logger {
    fn enabled(&self, _: &log::Metadata<'_>) -> bool {
        true
    }
    fn log(&self, record: &log::Record<'_>) {
        // Best effort diagnostics cannot delay telemetry or command replies.
        if !interrupt::free(|cs| DIAGNOSTIC.borrow(cs).borrow().pending().is_empty()) {
            return;
        }
        let mut line = Frame::new();
        if writeln!(line, "# {} {}", record.level(), record.args()).is_err() {
            return;
        }
        interrupt::free(|cs| *DIAGNOSTIC.borrow(cs).borrow_mut() = line);
    }
    fn flush(&self) {}
}

pub struct Console {
    class: CdcAcmClass<'static, BusAdapter>,
    device: UsbDevice<'static, BusAdapter>,
    configured: bool,
    open: bool,
    streaming: bool,
    last_snapshot: u32,
    skipped: u32,
    commands: Commands,
    rx: [u8; 512],
    rx_len: usize,
    rx_pos: usize,
    tx: Frame,
}

impl Console {
    pub fn new(instances: usbd::Instances<1>) -> Self {
        static BUS: static_cell::StaticCell<UsbBusAllocator<BusAdapter>> =
            static_cell::StaticCell::new();
        // SAFETY: only this Console accesses USB1, always from the foreground.
        // USB1 interrupts remain disabled. USB2 has a separate driver and ISR.
        let adapter = unsafe {
            BusAdapter::without_critical_sections(instances, &MEMORY, &ENDPOINTS, usbd::Speed::High)
        };
        adapter.set_interrupts(false);
        let bus = BUS.init(UsbBusAllocator::new(adapter));
        let class = CdcAcmClass::new(bus, 512);
        // Preserve the existing logger's VID/PID; this is a local prototype ID.
        let device = UsbDeviceBuilder::new(bus, UsbVidPid(0x5824, 0x27dd))
            .strings(&[StringDescriptors::default()
                .product("Teensy telemetry")
                .manufacturer("Teensy USB Lab")])
            .unwrap()
            .device_class(usbd_serial::USB_CLASS_CDC)
            .max_packet_size_0(64)
            .unwrap()
            .build();
        for i in 1..8 {
            for dir in [usb_device::UsbDirection::In, usb_device::UsbDirection::Out] {
                device
                    .bus()
                    .enable_zlt(usb_device::endpoint::EndpointAddress::from_parts(i, dir));
            }
        }
        log::set_logger(&LOGGER).unwrap();
        log::set_max_level(log::LevelFilter::Info);
        Self {
            class,
            device,
            configured: false,
            open: false,
            streaming: true,
            last_snapshot: 0,
            skipped: 0,
            commands: Commands::new(),
            rx: [0; 512],
            rx_len: 0,
            rx_pos: 0,
            tx: Frame::new(),
        }
    }

    fn snapshot(
        &mut self,
        id: u32,
        now: u32,
        pot: Option<u16>,
        pressure: crate::pressure::Reading,
    ) -> core::fmt::Result {
        let mut telemetry = crate::usb_host::telemetry();
        telemetry.pressure = Some(pressure);
        telemetry.plunger_io = Some(crate::motor::io_snapshot());
        telemetry.plunger = Some((crate::motor::snapshot(), pot));
        telemetry.snapshot(&mut self.tx, id, now, self.skipped)
    }
    fn motor_reply(
        &mut self,
        id: u32,
        action: &str,
        result: Result<(), &'static str>,
    ) -> core::fmt::Result {
        match result {
            Ok(()) => writeln!(
                self.tx,
                "{{\"type\":\"plunger_ack\",\"id\":{id},\"action\":\"{action}\"}}"
            ),
            Err(message) => writeln!(
                self.tx,
                "{{\"type\":\"error\",\"id\":{id},\"message\":\"{message}\"}}"
            ),
        }
    }

    pub fn poll(&mut self, now: u32, pot: Option<u16>, pressure: crate::pressure::Reading) {
        self.device.poll(&mut [&mut self.class]);
        let configured = self.device.state() == UsbDeviceState::Configured;
        if configured && !self.configured {
            self.device.bus().configure();
        }
        self.configured = configured;
        let open = configured && self.class.dtr();
        if self.open && !open {
            crate::motor::stop();
            crate::usb_host::disconnect_xeryon();
        }
        if !open || !self.open {
            self.commands = Commands::new();
            self.rx_len = 0;
            self.rx_pos = 0;
            self.tx.clear();
            self.streaming = true;
            self.last_snapshot = now;
        }
        self.open = open;
        if !open {
            return;
        }

        let due = self.streaming && now.wrapping_sub(self.last_snapshot) >= 100;
        if !self.tx.pending().is_empty() {
            let packet = &self.tx.pending()[..self.tx.pending().len().min(512)];
            match self.class.write_packet(packet) {
                Ok(n) => self.tx.consume(n),
                Err(usb_device::UsbError::WouldBlock) => {}
                Err(_) => {
                    self.tx.clear();
                    self.open = false;
                    crate::motor::stop();
                    crate::usb_host::disconnect_xeryon();
                    return;
                }
            }
            if !self.tx.pending().is_empty() {
                if due {
                    self.skipped = self.skipped.wrapping_add(1);
                    self.last_snapshot = now;
                }
                return;
            }
        }

        // At most one USB OUT packet and one command per poll. Retain surplus
        // commands until their predecessor's response has left the TX buffer.
        if self.rx_pos == self.rx_len {
            self.rx_pos = 0;
            self.rx_len = self.class.read_packet(&mut self.rx).unwrap_or(0);
        }
        while self.rx_pos < self.rx_len {
            let byte = self.rx[self.rx_pos];
            self.rx_pos += 1;
            if let Some(command) = self.commands.push(byte) {
                if command.is_ok() {
                    crate::motor::heartbeat(now);
                    crate::usb_host::heartbeat(now);
                }
                let result = match command {
                    Ok(Command::Hello(id)) => writeln!(
                        self.tx,
                        "{{\"type\":\"hello\",\"id\":{id},\"protocol\":1,\"firmware\":\"{}\",\"board\":\"Teensy 4.1\",\"motion\":true,\"xeryon\":true,\"xeryon_speed\":true,\"pressure\":true}}",
                        env!("CARGO_PKG_VERSION")
                    ),
                    Ok(Command::Ping(id)) => {
                        writeln!(self.tx, "{{\"type\":\"pong\",\"id\":{id},\"ms\":{now}}}")
                    }
                    Ok(Command::XeryonMove(id, up, nm, speed)) => {
                        match crate::usb_host::move_xeryon(up, nm, speed, now) {
                            Ok(target) => writeln!(
                                self.tx,
                                "{{\"type\":\"xeryon_ack\",\"id\":{id},\"action\":\"move\",\"target\":{target}}}"
                            ),
                            Err(message) => writeln!(
                                self.tx,
                                "{{\"type\":\"error\",\"id\":{id},\"message\":\"{message}\"}}"
                            ),
                        }
                    }
                    Ok(Command::XeryonStop(id)) => match crate::usb_host::stop_xeryon() {
                        Ok(()) => writeln!(
                            self.tx,
                            "{{\"type\":\"xeryon_ack\",\"id\":{id},\"action\":\"stop\"}}"
                        ),
                        Err(message) => writeln!(
                            self.tx,
                            "{{\"type\":\"error\",\"id\":{id},\"message\":\"{message}\"}}"
                        ),
                    },
                    Ok(Command::Snapshot(id)) => self.snapshot(id, now, pot, pressure),
                    Ok(Command::Move(id, up, speed, um)) => {
                        let result = crate::motor::command(up, speed, um, now);
                        self.motor_reply(id, "move", result)
                    }
                    Ok(Command::Stop(id)) => {
                        crate::motor::stop();
                        self.motor_reply(id, "stop", Ok(()))
                    }
                    Ok(Command::Zero(id)) => {
                        let result = crate::motor::zero();
                        self.motor_reply(id, "zero", result)
                    }
                    Ok(Command::Stream(id, on)) => {
                        self.streaming = on;
                        writeln!(self.tx, "{{\"type\":\"ack\",\"id\":{id},\"stream\":{on}}}")
                    }
                    Err(()) => self.tx.write_str(
                        "{\"type\":\"error\",\"id\":0,\"message\":\"invalid command\"}\n",
                    ),
                };
                if result.is_err() {
                    self.tx.clear();
                }
                return;
            }
        }
        if due {
            self.last_snapshot = now;
            if self.snapshot(0, now, pot, pressure).is_err() {
                self.tx.clear();
            }
        } else {
            interrupt::free(|cs| {
                core::mem::swap(&mut self.tx, &mut *DIAGNOSTIC.borrow(cs).borrow_mut())
            });
        }
    }
}
