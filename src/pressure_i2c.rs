//! Nonblocking LPI2C1 transport on Teensy Wire pins 18 (SDA), 19 (SCL).
use crate::pressure::{ADDRESS, Bus, Error, Operation};
use core::task::Poll;
use teensy4_bsp::{
    board,
    hal::{
        self,
        lpi2c::{ControllerCommand as Command, ControllerStatus as Status},
    },
    pins::common,
    ral,
};

pub struct PressureI2c {
    bus: hal::lpi2c::Lpi2c,
    active: bool,
    started: u32,
    expected: usize,
    received: usize,
    bytes: [u8; 3],
}
impl PressureI2c {
    pub fn new(instance: ral::lpi2c::LPI2C1, mut sda: common::P18, mut scl: common::P19) -> Self {
        // Match Wire pull-ups/hysteresis; HAL prepare enables open drain and SION.
        let config = hal::iomuxc::Config::zero()
            .set_drive_strength(hal::iomuxc::DriveStrength::R0_4)
            .set_pull_keeper(Some(hal::iomuxc::PullKeeper::Pullup22k))
            .set_hysteresis(hal::iomuxc::Hysteresis::Enabled);
        hal::iomuxc::configure(&mut sda, config);
        hal::iomuxc::configure(&mut scl, config);
        Self {
            bus: board::lpi2c(instance, scl, sda, board::Lpi2cClockSpeed::KHz400),
            active: false,
            started: 0,
            expected: 0,
            received: 0,
            bytes: [0; 3],
        }
    }
    fn abort(&mut self, error: Error) -> Poll<Result<[u8; 3], Error>> {
        // Reset cancels queued commands and releases the controller's bus drive.
        self.bus.reset_controller();
        self.bus
            .disabled(|bus| bus.set_timings(&board::lpi2c_baud(board::Lpi2cClockSpeed::KHz400)));
        self.bus.set_controller_enable(true);
        self.active = false;
        Poll::Ready(Err(error))
    }
}
impl Bus for PressureI2c {
    fn poll(&mut self, op: Operation, now: u32) -> Poll<Result<[u8; 3], Error>> {
        if !self.active {
            if self
                .bus
                .controller_status()
                .intersects(Status::BUS_BUSY | Status::CONTROLLER_BUSY)
            {
                return self.abort(Error::Bus);
            }
            self.bus.clear_fifo();
            self.bus.clear_controller_status(Status::all());
            self.bytes = [0; 3];
            self.received = 0;
            self.started = now;
            use Operation::*;
            let (write, count): (&[u8], usize) = match op {
                Configure => (&[0x10, 0x72], 0),
                WhoPointer => (&[0x0f], 0),
                WhoRead => (&[], 1),
                PressurePointer => (&[0xa8], 0),
                PressureRead => (&[], 3),
                TemperaturePointer => (&[0xab], 0),
                TemperatureRead => (&[], 2),
            };
            self.expected = count;
            self.bus.enqueue_controller_command(if count == 0 {
                Command::write(ADDRESS)
            } else {
                Command::read(ADDRESS)
            });
            for &byte in write {
                self.bus
                    .enqueue_controller_command(Command::Transmit { byte });
            }
            if count != 0 {
                self.bus
                    .enqueue_controller_command(Command::Receive { count: count as u8 });
            }
            // At most four commands: fits the hardware TX FIFO. Separate STOPs
            // match Wire.endTransmission() followed by Wire.requestFrom().
            self.bus.enqueue_controller_command(Command::Stop);
            self.active = true;
            return Poll::Pending;
        }
        let status = self.bus.controller_status();
        if status.intersects(
            Status::NACK_DETECTED
                | Status::ARBITRATION_LOST
                | Status::FIFO_ERROR
                | Status::PIN_LOW_TIMEOUT,
        ) {
            return self.abort(Error::Bus);
        }
        while self.received < self.expected {
            let Some(byte) = self.bus.read_data_register() else {
                break;
            };
            self.bytes[self.received] = byte;
            self.received += 1;
        }
        if status.contains(Status::STOP_DETECTED) && !status.contains(Status::CONTROLLER_BUSY) {
            if self.received != self.expected {
                return self.abort(Error::Bus);
            }
            self.active = false;
            return Poll::Ready(Ok(self.bytes));
        }
        if now.wrapping_sub(self.started) >= 20 {
            return self.abort(Error::Timeout);
        }
        Poll::Pending
    }
}
