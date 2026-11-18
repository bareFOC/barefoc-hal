//! BareBoot handsfree re-entry for Espressif ESP32 targets.
//!
//! Thin architecture adapter over the portable `barefoc-boot` client core.
//! This module is only compiled when the `bareboot` feature is enabled together
//! with one of the target-chip features (`esp32`, `esp32s3`, `esp32c3`,
//! `esp32c6`), because the reset is performed by
//! `esp_hal::system::software_reset()` from the pinned local esp-hal (~1.2.0).
//!
//! Source contract: `bareBoot/docs/handsfree-app-updates.md` section 4.1 (the
//! generic "mask interrupts, write cookie, fence, reset" sequence) adapted to
//! the ESP32 memory map:
//!
//! 1. Mask interrupts (RISC-V targets only, via `csrci mstatus, 8`).
//! 2. Write `REQUEST_BOOTLOADER` to the mailbox at `sram_top - 8`.
//! 3. Emit a `compiler_fence(SeqCst)` to order the store before the reset.
//! 4. Call `esp_hal::system::software_reset()` (never returns).
//!
//! # Mailbox location
//!
//! ESP32 parts have no STM32-style bootloader vector table at a known flash
//! base, so the mailbox is the fixed fallback slot `sram_top - 8`
//! ([`barefoc_boot::BootTarget::fallback_mailbox_addr`]). Build a target with
//! [`target_from_sram_top`] (or `BootTarget::new(0, 0, sram_top)`).
//!
//! # Interrupt masking gap (Xtensa)
//!
//! The pinned esp-hal exposes no portable "mask all interrupts" helper, and
//! the Xtensa `RSIL` instruction requires either a third-party crate or target
//! inline asm that cannot be validated here. On Xtensa targets the caller MUST
//! quiesce the motor driver (via [`before_reset_hook`]) and ensure no interrupt
//! can write the mailbox slot between the cookie store and the reset. RISC-V
//! targets (ESP32-C3/C6) clear `mstatus.MIE` in this module. This is a
//! documented gap rather than a silent omission.
//!
//! # Presence policy
//!
//! [`enter_bareboot`] always resets using the fallback mailbox. The checked
//! variant [`request_bootloader_checked`] skips the vector-table presence test
//! when `flash_base == 0` (the normal ESP32 case); it only returns
//! [`NotPresent`] when a non-zero `flash_base` is supplied and its vector table
//! does not look like BareBoot.

use barefoc::boot::{self, BootTarget, REQUEST_BOOTLOADER};

/// Build an ESP32 [`BootTarget`] from the top of internal SRAM.
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
/// The hook must quiesce the motor driver; on Xtensa it is also the caller's
/// opportunity to guarantee no interrupt races the mailbox write (see the
/// module-level "Interrupt masking gap" note).
pub fn before_reset_hook(hook: fn()) {
    // SAFETY: the hook is a plain function pointer with no state; the reset
    // sequence on the single core runs to completion without re-entry.
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

/// Mask interrupts on RISC-V ESP32 targets (no-op on Xtensa).
#[cfg(target_arch = "riscv32")]
#[inline]
fn mask_interrupts() {
    // SAFETY: `csrci mstatus, 8` only clears the machine interrupt-enable bit.
    unsafe {
        core::arch::asm!("csrci mstatus, 8", options(nostack, preserves_flags));
    }
}

/// Xtensa targets have no portable mask helper in the pinned esp-hal; the
/// caller must quiesce interrupts via other means (see module docs).
#[cfg(not(target_arch = "riscv32"))]
#[inline]
fn mask_interrupts() {}

/// Perform handsfree BareBoot entry on ESP32. Never returns.
///
/// Writes `REQUEST_BOOTLOADER` to `sram_top - 8`, then resets through
/// `esp_hal::system::software_reset()`. Interrupts are masked on RISC-V targets
/// and the optional driver-disarm hook always runs before the cookie write.
pub fn enter_bareboot(target: BootTarget) -> ! {
    mask_interrupts();
    // SAFETY: see `before_reset_hook`.
    unsafe {
        if let Some(hook) = BEFORE_RESET_HOOK {
            hook();
        }
    }

    let mailbox = target.fallback_mailbox_addr();
    // SAFETY: `sram_top - 8` is an aligned, preserved SRAM slot reserved for
    // the cross-reset token (doc sections 3.7 and 4.1). A warm software reset
    // does not power-cycle internal SRAM, so the cookie survives until the
    // bootloader reads it.
    unsafe {
        core::ptr::write_volatile(mailbox as *mut u64, REQUEST_BOOTLOADER);
    }
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    esp_hal::system::software_reset()
}

/// Checked variant of [`enter_bareboot`].
///
/// For the normal ESP32 target (`flash_base == 0`) there is no vector table to
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
