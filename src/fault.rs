//! Fatal diagnostics that work without USB, SysTick, or the foreground stack.
//! These handlers halt; they never restart an experiment or reset the controller.
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use teensy4_bsp::{board, hal::iomuxc::gpio::Pin, pins::common, ral};

static PHASE: AtomicU32 = AtomicU32::new(1);
static PANIC_LINE: AtomicU32 = AtomicU32::new(0);
static LAST_LOOP_MS: AtomicU32 = AtomicU32::new(0);
static MONITOR_ENABLED: AtomicBool = AtomicBool::new(false);

pub fn mark(phase: u32) {
    PHASE.store(phase, Ordering::Relaxed);
}

pub fn heartbeat(now: u32) {
    LAST_LOOP_MS.store(now, Ordering::Relaxed);
    MONITOR_ENABLED.store(true, Ordering::Relaxed);
}

// SysTick can identify a foreground stall even if USB can no longer respond.
// Enable only after board/USB initialization, where waiting is intentional.
pub fn tick(now: u32) {
    if MONITOR_ENABLED.load(Ordering::Relaxed)
        && now.wrapping_sub(LAST_LOOP_MS.load(Ordering::Relaxed)) >= 1_000
    {
        // SAFETY: terminal path; the interrupted foreground is never resumed.
        unsafe { enter_fault(5) }
    }
}

pub fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    PANIC_LINE.store(info.location().map_or(0, |p| p.line()), Ordering::Relaxed);
    // SAFETY: terminal path; the abandoned stack is never used again.
    unsafe { enter_fault(2) }
}

// A separate, aligned stack lets a stack-overflow HardFault report itself.
static mut FAULT_STACK: [u64; 128] = [0; 128];

#[repr(C)]
pub struct Report {
    kind: u32,
    phase: u32,
    sp: u32,
    exception: u32,
    cfsr: u32,
    hfsr: u32,
    mmfar: u32,
    bfar: u32,
    panic_line: u32,
}

// Inspect with a debugger if available. Not retained across a power cycle.
#[unsafe(no_mangle)]
static mut FATAL_REPORT: Report = Report {
    kind: 0,
    phase: 0,
    sp: 0,
    exception: 0,
    cfsr: 0,
    hfsr: 0,
    mmfar: 0,
    bfar: 0,
    panic_line: 0,
};

#[unsafe(naked)]
unsafe extern "C" fn enter_fault(_kind: u32) -> ! {
    core::arch::naked_asm!(
        "cpsid i",
        "mrs r1, msp",
        "mrs r2, ipsr",
        "ldr r3, ={stack}",
        "add.w r3, r3, #1024",
        "msr msp, r3",
        "b {halt}",
        stack = sym FAULT_STACK,
        halt = sym halt,
    );
}

#[unsafe(naked)]
#[unsafe(no_mangle)]
unsafe extern "C" fn HardFault() -> ! {
    core::arch::naked_asm!("movs r0, #3", "b {entry}", entry = sym enter_fault);
}

#[unsafe(naked)]
#[unsafe(no_mangle)]
unsafe extern "C" fn DefaultHandler() -> ! {
    core::arch::naked_asm!("movs r0, #4", "b {entry}", entry = sym enter_fault);
}

extern "C" fn halt(kind: u32, sp: u32, exception: u32) -> ! {
    let phase = PHASE.load(Ordering::Relaxed);
    // SAFETY: interrupts are disabled and this path never returns. Use raw
    // GPIO writes, avoiding locks/RefCells that may have caused the panic.
    let gpio2 = unsafe { &*ral::gpio::GPIO2 };
    let gpio4 = unsafe { &*ral::gpio::GPIO4 };
    ral::write_reg!(
        ral::gpio,
        gpio2,
        DR_CLEAR,
        1 << <common::P6 as Pin<2>>::OFFSET
    );
    ral::write_reg!(
        ral::gpio,
        gpio4,
        DR_CLEAR,
        (1 << <common::P4 as Pin<4>>::OFFSET) | (1 << <common::P5 as Pin<4>>::OFFSET)
    );
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(FATAL_REPORT),
            Report {
                kind,
                phase,
                sp,
                exception,
                cfsr: core::ptr::read_volatile(0xe000_ed28 as *const u32),
                hfsr: core::ptr::read_volatile(0xe000_ed2c as *const u32),
                mmfar: core::ptr::read_volatile(0xe000_ed34 as *const u32),
                bfar: core::ptr::read_volatile(0xe000_ed38 as *const u32),
                panic_line: PANIC_LINE.load(Ordering::Relaxed),
            },
        );
    }
    // The LED is configured before experiments become available. Delay uses
    // CPU cycles, so neither SysTick nor USB needs to be running.
    const LED: u32 = 1 << <common::P13 as Pin<2>>::OFFSET;
    loop {
        // One long pulse, fault-count short pulses, then phase-count pulses.
        ral::write_reg!(ral::gpio, gpio2, DR_SET, LED);
        cortex_m::asm::delay(board::ARM_FREQUENCY);
        ral::write_reg!(ral::gpio, gpio2, DR_CLEAR, LED);
        cortex_m::asm::delay(board::ARM_FREQUENCY / 2);
        for count in [kind, phase] {
            for _ in 0..count {
                ral::write_reg!(ral::gpio, gpio2, DR_SET, LED);
                cortex_m::asm::delay(board::ARM_FREQUENCY / 5);
                ral::write_reg!(ral::gpio, gpio2, DR_CLEAR, LED);
                cortex_m::asm::delay(board::ARM_FREQUENCY / 5);
            }
            cortex_m::asm::delay(board::ARM_FREQUENCY);
        }
    }
}
