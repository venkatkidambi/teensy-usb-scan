//! Nonblocking 400 kHz LPI2C1 register transactions, with repeated START reads.
#[cfg(not(test))]
use crate::pressure::Bus;
use crate::pressure::{Error, Operation};
use core::task::Poll;

// RT1062 has four entries in each FIFO. A register read needs five commands,
// and a pressure/temperature burst receives five bytes: service both FIFOs.
const FIFO_DEPTH: u16 = 4;
const TRANSACTION_TIMEOUT_MS: u32 = 20;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Command {
    StartWrite,
    StartRead,
    Transmit(u8),
    Receive(u8),
    Stop,
}
#[derive(Default)]
struct Status {
    busy: bool,
    stopped: bool,
    error: bool,
}
trait Controller {
    fn status(&self) -> Status;
    fn begin(&mut self);
    fn reset(&mut self);
    fn tx_count(&self) -> u16;
    fn send(&mut self, command: Command);
    fn receive(&mut self) -> Option<u8>;
}
#[derive(Default)]
struct Transfer {
    operation: Option<Operation>,
    started: u32,
    next: usize,
    received: usize,
    bytes: [u8; 5],
}
impl Transfer {
    fn abort(&mut self, bus: &mut impl Controller, error: Error) -> Poll<Result<[u8; 5], Error>> {
        bus.reset();
        self.operation = None;
        Poll::Ready(Err(error))
    }
    fn poll(
        &mut self,
        bus: &mut impl Controller,
        op: Operation,
        now: u32,
    ) -> Poll<Result<[u8; 5], Error>> {
        if self.operation.is_none() {
            if bus.status().busy {
                return self.abort(bus, Error::Bus);
            }
            bus.begin();
            self.operation = Some(op);
            self.started = now;
            self.next = 0;
            self.received = 0;
            self.bytes = [0; 5];
        }
        if self.operation != Some(op) || bus.status().error {
            return self.abort(bus, Error::Bus);
        }
        let (register, value, expected) = op.transaction();
        let commands = if value.is_some() { 4 } else { 5 };
        while self.next < commands && bus.tx_count() < FIFO_DEPTH {
            let command = match (self.next, value) {
                (0, _) => Command::StartWrite,
                (1, _) => Command::Transmit(register),
                (2, Some(value)) => Command::Transmit(value),
                (2, None) => Command::StartRead,
                (3, None) => Command::Receive(expected as u8),
                _ => Command::Stop,
            };
            bus.send(command);
            self.next += 1;
        }
        // Capture STOP before draining RX: a final byte can arrive immediately
        // after an empty read, so a later STOP check could report a false short read.
        let status = bus.status();
        if status.error {
            return self.abort(bus, Error::Bus);
        }
        while self.received < expected {
            let Some(byte) = bus.receive() else { break };
            self.bytes[self.received] = byte;
            self.received += 1;
        }
        if status.stopped && !status.busy {
            if self.next != commands || self.received != expected {
                return self.abort(bus, Error::Bus);
            }
            self.operation = None;
            return Poll::Ready(Ok(self.bytes));
        }
        if now.wrapping_sub(self.started) >= TRANSACTION_TIMEOUT_MS {
            return self.abort(bus, Error::Timeout);
        }
        Poll::Pending
    }
}

#[cfg(not(test))]
use teensy4_bsp::{board, hal, pins::common, ral};
#[cfg(not(test))]
pub struct PressureI2c {
    bus: hal::lpi2c::Lpi2c,
    transfer: Transfer,
}
#[cfg(not(test))]
impl PressureI2c {
    pub fn new(instance: ral::lpi2c::LPI2C1, mut sda: common::P18, mut scl: common::P19) -> Self {
        // External pull-ups must meet Fast-mode rise time. HAL enables open drain.
        let config = hal::iomuxc::Config::zero()
            .set_drive_strength(hal::iomuxc::DriveStrength::R0_4)
            .set_pull_keeper(Some(hal::iomuxc::PullKeeper::Pullup22k))
            .set_hysteresis(hal::iomuxc::Hysteresis::Enabled);
        hal::iomuxc::configure(&mut sda, config);
        hal::iomuxc::configure(&mut scl, config);
        Self {
            // LPS22HH datasheet Table 7: 400 kHz maximum for I2C (not I3C).
            bus: board::lpi2c(instance, scl, sda, board::Lpi2cClockSpeed::KHz400),
            transfer: Transfer::default(),
        }
    }
}
#[cfg(not(test))]
impl Bus for PressureI2c {
    fn poll(&mut self, op: Operation, now: u32) -> Poll<Result<[u8; 5], Error>> {
        self.transfer.poll(&mut self.bus, op, now)
    }
}
#[cfg(not(test))]
impl Controller for hal::lpi2c::Lpi2c {
    fn status(&self) -> Status {
        use hal::lpi2c::ControllerStatus as S;
        let status = self.controller_status();
        Status {
            busy: status.intersects(S::BUS_BUSY | S::CONTROLLER_BUSY),
            stopped: status.contains(S::STOP_DETECTED),
            error: status.intersects(
                S::NACK_DETECTED | S::ARBITRATION_LOST | S::FIFO_ERROR | S::PIN_LOW_TIMEOUT,
            ),
        }
    }
    fn begin(&mut self) {
        self.clear_fifo();
        self.clear_controller_status(hal::lpi2c::ControllerStatus::all());
    }
    fn reset(&mut self) {
        self.reset_controller();
        self.disabled(|bus| bus.set_timings(&board::lpi2c_baud(board::Lpi2cClockSpeed::KHz400)));
        self.set_controller_enable(true);
    }
    fn tx_count(&self) -> u16 {
        self.controller_fifo_status().txcount
    }
    fn send(&mut self, command: Command) {
        use crate::pressure::ADDRESS;
        use hal::lpi2c::ControllerCommand as C;
        self.enqueue_controller_command(match command {
            Command::StartWrite => C::write(ADDRESS),
            Command::StartRead => C::read(ADDRESS),
            Command::Transmit(byte) => C::Transmit { byte },
            Command::Receive(count) => C::Receive { count },
            Command::Stop => C::Stop,
        });
    }
    fn receive(&mut self) -> Option<u8> {
        self.read_data_register()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    #[derive(Default)]
    struct Fake {
        tx: VecDeque<Command>,
        rx: VecDeque<u8>,
        sent: Vec<Command>,
        busy: bool,
        stopped: bool,
        error: bool,
        receiving: u8,
        byte: u8,
        resets: usize,
        late_stop: bool,
    }
    impl Controller for Fake {
        fn status(&self) -> Status {
            Status {
                busy: self.busy,
                stopped: self.stopped,
                error: self.error,
            }
        }
        fn begin(&mut self) {
            self.tx.clear();
            self.rx.clear();
            self.stopped = false;
        }
        fn reset(&mut self) {
            let resets = self.resets + 1;
            *self = Self {
                resets,
                ..Self::default()
            };
        }
        fn tx_count(&self) -> u16 {
            self.tx.len() as u16
        }
        fn send(&mut self, command: Command) {
            assert!(self.tx.len() < FIFO_DEPTH as usize, "TX overflow");
            self.tx.push_back(command);
            self.sent.push(command);
        }
        fn receive(&mut self) -> Option<u8> {
            let byte = self.rx.pop_front();
            if byte.is_none() && self.late_stop {
                // Final byte and STOP arrive just after an empty FIFO read.
                self.late_stop = false;
                self.clock();
                self.clock();
            }
            byte
        }
    }
    impl Fake {
        fn clock(&mut self) {
            if self.receiving != 0 {
                // The real controller stretches SCL until RX space is available.
                if self.rx.len() < FIFO_DEPTH as usize {
                    self.byte += 1;
                    self.rx.push_back(self.byte);
                    self.receiving -= 1;
                }
            } else {
                match self.tx.pop_front() {
                    Some(Command::StartWrite | Command::StartRead) => self.busy = true,
                    Some(Command::Receive(count)) => self.receiving = count,
                    Some(Command::Stop) => {
                        self.stopped = true;
                        self.busy = false;
                    }
                    _ => {}
                }
            }
        }
    }
    #[test]
    fn register_reads_use_repeated_start_and_service_both_four_entry_fifos() {
        for (op, register, count) in [
            (Operation::Identify, 0x0f, 1),
            (Operation::Verify, 0x10, 2),
            (Operation::Status, 0x27, 1),
            (Operation::Sample, 0x28, 5),
        ] {
            let mut transfer = Transfer::default();
            let mut bus = Fake::default();
            assert_eq!(transfer.poll(&mut bus, op, 0), Poll::Pending);
            assert_eq!(bus.tx.len(), 4); // STOP must wait for space.
            let mut result = Poll::Pending;
            for _ in 0..20 {
                bus.clock();
                result = transfer.poll(&mut bus, op, 0);
                if result.is_ready() {
                    break;
                }
            }
            let mut expected = [0; 5];
            for i in 0..count {
                expected[i] = i as u8 + 1;
            }
            assert_eq!(result, Poll::Ready(Ok(expected)));
            assert_eq!(
                bus.sent,
                [
                    Command::StartWrite,
                    Command::Transmit(register),
                    Command::StartRead,
                    Command::Receive(count as u8),
                    Command::Stop
                ]
            );
        }
    }
    #[test]
    fn initialization_writes_match_lps22hh_registers() {
        for (op, register, value) in [
            (Operation::Interface, 0x0e, 0x02),
            (Operation::Control, 0x11, 0x10),
            (Operation::Configure, 0x10, 0x72),
        ] {
            let mut transfer = Transfer::default();
            let mut bus = Fake::default();
            assert_eq!(transfer.poll(&mut bus, op, 0), Poll::Pending);
            assert_eq!(
                bus.sent,
                [
                    Command::StartWrite,
                    Command::Transmit(register),
                    Command::Transmit(value),
                    Command::Stop
                ]
            );
            for _ in 0..4 {
                bus.clock();
            }
            assert_eq!(transfer.poll(&mut bus, op, 0), Poll::Ready(Ok([0; 5])));
        }
    }
    #[test]
    fn errors_short_reads_and_wrapping_deadlines_reset_the_controller() {
        for fault in 0..3 {
            let mut transfer = Transfer::default();
            let mut bus = Fake::default();
            let start = u32::MAX - 10;
            assert_eq!(
                transfer.poll(&mut bus, Operation::Sample, start),
                Poll::Pending
            );
            match fault {
                0 => bus.error = true,
                1 => {
                    bus.stopped = true;
                    bus.busy = false;
                }
                _ => {}
            }
            let expected = if fault == 2 {
                Error::Timeout
            } else {
                Error::Bus
            };
            assert_eq!(
                transfer.poll(&mut bus, Operation::Sample, 9),
                Poll::Ready(Err(expected))
            );
            assert_eq!(bus.resets, 1);
            assert!(transfer.operation.is_none());
            assert_eq!(
                transfer.poll(&mut bus, Operation::Identify, 10),
                Poll::Pending
            );
        }
        let mut bus = Fake {
            busy: true,
            ..Fake::default()
        };
        assert_eq!(
            Transfer::default().poll(&mut bus, Operation::Identify, 0),
            Poll::Ready(Err(Error::Bus))
        );
    }
    #[test]
    fn final_byte_arriving_with_stop_is_not_a_short_read() {
        let mut transfer = Transfer::default();
        let mut bus = Fake::default();
        assert_eq!(transfer.poll(&mut bus, Operation::Sample, 0), Poll::Pending);
        for _ in 0..4 {
            bus.clock();
        } // START, register, repeated START, receive
        assert_eq!(transfer.poll(&mut bus, Operation::Sample, 0), Poll::Pending); // queue STOP
        for _ in 0..4 {
            bus.clock();
        } // RX FIFO full, fifth byte pending
        assert_eq!(bus.rx.len(), 4);
        bus.late_stop = true;
        assert_eq!(transfer.poll(&mut bus, Operation::Sample, 0), Poll::Pending);
        assert!(bus.stopped);
        assert_eq!(
            transfer.poll(&mut bus, Operation::Sample, 0),
            Poll::Ready(Ok([1, 2, 3, 4, 5]))
        );
        assert_eq!(bus.resets, 0);
    }
}
