//! `critical-section` implementation for the dual-hart K210.
//!
//! The standard `critical-section-single-hart` impl from the `riscv`
//! crate only clears `mstatus.MIE` on the current hart. On the K210 that
//! is unsound: a critical section taken on hart 0 does not block hart 1
//! from racing on the same shared state. This module pairs the per-hart
//! interrupt mask with a single spinlock so that at most one hart at a
//! time holds a critical section.
//!
//! Enable with the `critical-section-impl` Cargo feature. The
//! implementation is registered via `critical_section::set_impl!`, so
//! pulling the feature in is all a consumer needs to satisfy users of
//! the `critical-section` crate (e.g. `embassy-sync`).

use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::{Impl, RawRestoreState};

struct K210CriticalSection;

critical_section::set_impl!(K210CriticalSection);

/// Cross-hart spinlock. `true` while a critical section is held.
static LOCK: AtomicBool = AtomicBool::new(false);

unsafe impl Impl for K210CriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        // Atomically read and clear `mstatus.MIE` (bit 3). The previous
        // value is needed so `release` can restore the prior interrupt
        // state without enabling interrupts that the caller had off.
        let mstatus: usize;
        core::arch::asm!(
            "csrrci {0}, mstatus, 0b1000",
            out(reg) mstatus,
            options(nostack),
        );
        let was_enabled = (mstatus & 0b1000) != 0;

        // Acquire the cross-hart spinlock. With interrupts already off
        // on this hart, the only contender is the other hart. Both
        // harts run with interrupts off while holding the lock, so this
        // wait is bounded by the other hart's critical-section length.
        while LOCK
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            core::hint::spin_loop();
        }

        was_enabled
    }

    unsafe fn release(restore: RawRestoreState) {
        // Release the spinlock before re-enabling interrupts so an ISR
        // taken between the store and the `csrsi` cannot observe a held
        // lock and deadlock waiting for itself.
        LOCK.store(false, Ordering::Release);
        if restore {
            core::arch::asm!("csrsi mstatus, 0b1000", options(nostack));
        }
    }
}
