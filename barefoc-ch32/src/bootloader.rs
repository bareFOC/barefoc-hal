//! BareBoot handsfree re-entry for WCH CH32 (RISC-V) targets.
//!
//! Thin architecture adapter over the portable `barefoc-boot` client core.
//! Cookie derivation and bus trigger decoders live there; this module only
//! supplies the RISC-V-specific pieces described in
//! `bareBoot/docs/handsfree-app-updates.md` section 4.1:
//!
//! 1. Mask interrupts with `csrci mstatus, 8` (clear the MIE bit).
//! 2. Write `REQUEST_BOOTLOADER` to the mailbox at `sram_top - 8`.
//! 3. Emit a `compiler_fence(SeqCst)` to order the store before reset.
//! 4. Request a system reset through the PFIC: write
//!    `0xE000_E048 = 0x5AA5_0000 | (1 << 7)`.
//! 5. Spin forever.
//!
//! # Mailbox location
//!
//! Unlike Cortex-M targets, CH32 parts have no useful bootloader vector table
//! at a known flash base, so the mailbox is the fixed fallback slot
//! `sram_top - 8` ([`barefoc_boot::BootTarget::fallback_mailbox_addr`]). Build a
//! target with [`target_from_sram_top`] (or `BootTarget::new(0, 0, sram_top)`)
//! and pass it to [`enter_bareboot`].
//!
//! # Presence policy
//!
//! [`enter_bareboot`] always resets using the fallback mailbox. The checked
//! variant [`request_bootloader_checked`] skips the vector-table presence test
//! when `flash_base == 0` (the normal CH32 case), since there is nothing to
//! inspect; it only returns [`NotPresent`] when a non-zero `flash_base` is
//! supplied and its vector table does not look like BareBoot.

use barefoc::boot::{self, BootTarget, REQUEST_BOOTLOADER};

/// PFIC configuration register used for the WCH system reset.
const PFIC_CFGR: *mut u32 = 0xE000_E048 as *mut u32;
/// PFIC key + system-reset bit (doc section 3.7 / 4.1).
const PFIC_SYS_RESET: u32 = 0x5AA5_0000 | (1 << 7);

/// Build a CH32 [`BootTarget`] from the top of SRAM.
///
/// `flash_base` and `app_base` are zero because there is no discoverable
/// vector table; the mailbox resolves to `sram_top - 8` via
/// `BootTarget::fallback_mailbox_addr`.
pub const fn target_from_sram_top(sram_top: u32) -> BootTarget {
    BootTarget::new(0, 0, sram_top)
}

/// Returned by [`request_bootloader_checked`] when a supplied vector table does
/// not look like a Katapult/BareBoot-compatible bootloader.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NotPresent;

/// Optional driver-disarm callback invoked before the cookie is written.
static mut BEFORE_RESET_HOOK: Option<fn()> = None;

/// Install (or clear with `None`) the driver-disarm callback run by
/// [`enter_bareboot`] just before it writes the boot cookie.
///
/// The hook runs with interrupts already disabled and must not block.
pub fn before_reset_hook(hook: fn()) {
    // SAFETY: single-core RISC-V with interrupts masked around the only
    // writer/reader; the hook is a plain function pointer with no state.
    unsafe {
        BEFORE_RESET_HOOK = Some(hook);
    }
}

/// Clear a previously installed driver-disarm callback.
pub fn clear_before_reset_hook() {
    // SAFETY: see `before_reset_hook`.
    unsafe {
        BEFORE_RESET_HOOK = None;
    }
}

/// Mask interrupts (clear `mstatus.MIE`) and run the driver-disarm hook.
#[inline]
fn disable_irq_and_disarm() {
    // SAFETY: `csrci mstatus, 8` is a privileged RISC-V instruction that only
    // clears the machine interrupt-enable bit; it has no memory side effects.
    unsafe {
        core::arch::asm!("csrci mstatus, 8", options(nostack, preserves_flags));
        if let Some(hook) = BEFORE_RESET_HOOK {
            hook();
        }
    }
}

/// Finish the reset sequence after the cookie store. Never returns.
fn reset_after_cookie() -> ! {
    // Doc section 4.1 step 3: order the volatile store before the reset.
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    // SAFETY: PFIC_CFGR is a defined WCH memory-mapped system register;
    // writing the unlock key with the system-reset bit requests a chip reset.
    unsafe {
        core::ptr::write_volatile(PFIC_CFGR, PFIC_SYS_RESET);
    }
    loop {
        core::hint::spin_loop();
    }
}

/// Perform handsfree BareBoot entry on CH32. Never returns.
///
/// Writes `REQUEST_BOOTLOADER` to `sram_top - 8`, then requests a PFIC system
/// reset. Interrupts are masked first and the optional driver-disarm hook runs
/// before the cookie write.
pub fn enter_bareboot(target: BootTarget) -> ! {
    disable_irq_and_disarm();

    let mailbox = target.fallback_mailbox_addr();
    // SAFETY: `sram_top - 8` is an aligned, preserved SRAM slot reserved for
    // the cross-reset token (doc sections 3.7 and 4.1).
    unsafe {
        core::ptr::write_volatile(mailbox as *mut u64, REQUEST_BOOTLOADER);
    }
    reset_after_cookie()
}

/// Checked variant of [`enter_bareboot`].
///
/// For the normal CH32 target (`flash_base == 0`) there is no vector table to
/// inspect, so this proceeds directly. When a non-zero `flash_base` is given,
/// returns [`NotPresent`] instead of resetting if its vector table does not
/// look like BareBoot.
pub fn request_bootloader_checked(target: BootTarget) -> Result<(), NotPresent> {
    if target.flash_base != 0 && !target.is_present() {
        return Err(NotPresent);
    }
    enter_bareboot(target)
}

/// Clear the fallback mailbox cookie for `target` (cancel a pending request).
pub fn clear_boot_cookie(target: BootTarget) {
    // SAFETY: see `enter_bareboot` (aligned, preserved SRAM slot).
    unsafe {
        core::ptr::write_volatile(target.fallback_mailbox_addr() as *mut u64, 0u64);
    }
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
}
