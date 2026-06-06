//! `embassy-time-driver` implementation backed by the K210 CLINT.
//!
//! `mtime` is the monotonic counter; `mtimecmp[0]` arms an alarm that
//! fires the machine timer interrupt on hart 0. The driver is
//! single-hart at this stage; the hart 1 alarm is added when the second
//! hart comes online.
//!
//! `mtime` ticks at the CPU clock divided by 50, matching both NuttX
//! (`arch/risc-v/src/k210/k210_timerisr.c`) and the Kendryte standalone
//! SDK. At the default 400 MHz core clock that is 8 MHz. The firmware
//! must select a matching `embassy-time` tick rate (for example the
//! `tick-hz-8_000_000` feature) so that `Duration` arithmetic agrees
//! with wall-clock time.
//!
//! Before any `Timer::after` will fire, the firmware must:
//!
//! 1. Call [`init`] once.
//! 2. Enable `mstatus.MIE` (typically via `riscv::interrupt::enable`).
//!
//! `init` programs `mtimecmp[0]` to `u64::MAX` so the spurious
//! mtime >= mtimecmp condition that holds at reset does not trigger an
//! immediate interrupt, then unmasks `mie.MTIE`.

use core::cell::RefCell;
use core::task::Waker;

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

const CLINT_BASE: usize = 0x0200_0000;
const MTIME: *const u64 = (CLINT_BASE + 0xBFF8) as *const u64;
const MTIMECMP0: *mut u64 = (CLINT_BASE + 0x4000) as *mut u64;

/// `mie.MTIE` mask. Machine timer interrupt enable bit.
const MIE_MTIE: usize = 1 << 7;

struct K210TimeDriver {
    queue: Mutex<RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(
    static DRIVER: K210TimeDriver = K210TimeDriver {
        queue: Mutex::new(RefCell::new(Queue::new())),
    }
);

/// Initialise the time driver hardware.
///
/// Disarms `mtimecmp[0]` and unmasks the machine timer interrupt in
/// `mie`. The caller is responsible for enabling `mstatus.MIE` after
/// this returns. Calling more than once is harmless.
pub fn init() {
    unsafe {
        core::ptr::write_volatile(MTIMECMP0, u64::MAX);
        core::arch::asm!(
            "csrrs zero, mie, {bits}",
            bits = in(reg) MIE_MTIE,
            options(nostack),
        );
    }
}

impl K210TimeDriver {
    /// Arm `mtimecmp[0]` for the given tick.
    ///
    /// Returns `false` if `at` has already passed by the time the write
    /// completes, in which case the caller should poll the queue with
    /// the new `now()` and try again.
    fn set_alarm(&self, _cs: CriticalSection, at: u64) -> bool {
        unsafe { core::ptr::write_volatile(MTIMECMP0, at) };
        at > self.now()
    }
}

impl Driver for K210TimeDriver {
    fn now(&self) -> u64 {
        // `mtime` is naturally 64-bit on RV64; a single load is atomic.
        unsafe { core::ptr::read_volatile(MTIME) }
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        });
    }
}

/// Machine timer trap entry. Overrides the `DefaultHandler` weak symbol
/// provided by `riscv-rt`.
#[allow(non_snake_case)]
#[no_mangle]
extern "C" fn MachineTimer() {
    critical_section::with(|cs| {
        let mut queue = DRIVER.queue.borrow(cs).borrow_mut();
        let mut next = queue.next_expiration(DRIVER.now());
        while !DRIVER.set_alarm(cs, next) {
            next = queue.next_expiration(DRIVER.now());
        }
    });
}
