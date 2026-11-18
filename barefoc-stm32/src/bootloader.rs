//! BareBoot handsfree re-entry for STM32 (Cortex-M) targets.
//!
//! This module is a thin, architecture-specific adapter over the portable
//! `barefoc-boot` client core. All bus trigger decoding and mailbox cookie
//! derivation live in `barefoc-boot`; this module only supplies the
//! Cortex-M-specific pieces: interrupt masking, DCache maintenance (M7), and
//! the `SCB_AIRCR` system reset.
//!
//! Source contract: `bareBoot/docs/handsfree-app-updates.md`, sections 2, 4.1
//! and 6 (authoritative). The reset sequence here mirrors the document's
//! `enter_bareboot_cortex_m` exactly:
//!
//! 1. Disable interrupts (`cpsid i`) **before** touching the cookie.
//! 2. Write `REQUEST_BOOTLOADER` to the mailbox derived from the bootloader's
//!    initial SP (`BootTarget::mailbox_addr` via `barefoc_boot::write_cookie`).
//! 3. Insert a `compiler_fence(SeqCst)` so the store is ordered before reset.
//! 4. Clean DCache on Cortex-M7 targets (cfg-gated).
//! 5. Write `SCB_AIRCR = (0x05FA << 16) | (1 << 2)` (`SYSRESETREQ`).
//! 6. Spin forever; the reset must take effect.
//!
//! # Presence policy (doc section 4.1)
//!
//! [`enter_bareboot`] calls [`barefoc_boot::BootTarget::is_present`] first:
//!
//! - **present** — write the cookie to the mailbox derived from the bootloader
//!   vector table (`mailbox_addr()`), preserving the Katapult contract;
//! - **not present** — **use the fallback mailbox** (`sram_top - 8`) instead of
//!   no-op'ing. This is a best-effort entry for targets whose vector table is
//!   not (yet) patched; it keeps the historical `request_bootloader()`
//!   behaviour of always requesting a reset. Callers that need to avoid a
//!   reset when no compatible bootloader is installed should use
//!   [`request_bootloader_checked`], which returns [`NotPresent`] instead.

use barefoc_boot::{BootTarget, REQUEST_BOOTLOADER};

/// Default bootloader flash base used by the legacy Katapult compatibility API.
///
/// Matches `crate::FLASH_BOOT_ADDRESS`. New callers should build their own
/// [`BootTarget`] when the board uses a different layout.
pub const DEFAULT_FLASH_BASE: u32 = 0x0800_0000;
/// Default application base (STM32H503 application is linked at 0x08002000).
pub const DEFAULT_APP_BASE: u32 = 0x0800_2000;
/// Default top of SRAM (STM32H503: 32 KiB at 0x20000000).
pub const DEFAULT_SRAM_TOP: u32 = 0x2000_8000;

/// The [`BootTarget`] used by the legacy `katapult` compatibility helpers.
pub fn default_target() -> BootTarget {
    BootTarget::new(DEFAULT_FLASH_BASE, DEFAULT_APP_BASE, DEFAULT_SRAM_TOP)
}

/// Returned by [`request_bootloader_checked`] when the bootloader vector table
/// does not look like a Katapult/BareBoot-compatible bootloader.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NotPresent;

/// Optional driver-disarm callback invoked immediately before the cookie is
/// written and the reset is issued.
///
/// Installed via [`before_reset_hook`]. It runs with interrupts already
/// disabled and must not block, allocate, or rely on being rescheduled.
static mut BEFORE_RESET_HOOK: Option<fn()> = None;

/// Install (or clear with `None`) the driver-disarm callback run by
/// [`enter_bareboot`] just before it writes the boot cookie.
///
/// The hook is a plain `fn()` with no context. It is called with interrupts
/// disabled, in the exact context that invoked the bootloader request.
pub fn before_reset_hook(hook: fn()) {
    // SAFETY: single-core Cortex-M targets with interrupts disabled around the
    // only writer/reader; the hook is a plain function pointer with no state.
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

/// Mask interrupts and run the driver-disarm hook, in that order.
fn disable_irq_and_disarm() {
    // Section 4.1: mask interrupts BEFORE the cookie write.
    cortex_m::interrupt::disable();
    // SAFETY: see `before_reset_hook` (single-core, interrupts masked).
    unsafe {
        if let Some(hook) = BEFORE_RESET_HOOK {
            hook();
        }
    }
}

/// Clean the DCache line(s) covering the mailbox on Cortex-M7 targets.
///
/// Cortex-M7 has a write-back DCache, so a volatile store to SRAM may sit in
/// cache and be lost before the bootloader reads it after reset. The M7
/// reference implementation uses `SCB_CleanDCache_by_Addr`; here the equivalent
/// CP15 `DCCISW` sequence (`mcr p15, 0, r0, c7, c10, 1`) cleans a single 32-byte
/// line containing the 8-byte cookie. This is compiled only when the
/// `dcache-m7` feature is enabled (default off), because there is no stable
/// built-in cfg for Cortex-M7 and enabling it on a non-M7 part is illegal.
#[cfg(feature = "dcache-m7")]
#[inline]
fn clean_dcache_line(addr: usize) {
    // Align down to a 32-byte (D-cache line size) boundary.
    let line = addr & !31usize;
    // SAFETY: CP15 DCCISW is a privileged M7 cache-maintenance instruction.
    // It only reads the cache tag/RAM for this line and writes back clean data;
    // it has no side effects on addresses outside the line. Caller guarantees
    // the feature is only enabled on a Cortex-M7 part and `addr` is mapped.
    unsafe {
        core::arch::asm!(
            "mcr p15, 0, {addr}, c7, c10, 1",
            "dsb sy",
            addr = in(reg) line,
            options(nostack, preserves_flags),
        );
    }
}

/// No-op fallback for non-M7 targets (the default).
#[cfg(not(feature = "dcache-m7"))]
#[inline]
fn clean_dcache_line(_addr: usize) {}

/// Finish the reset sequence after the cookie store. Never returns.
///
/// Emits the explicit post-write `compiler_fence(SeqCst)` (doc section 4.1
/// step 3), cleans the mailbox DCache line on M7, then writes the exact
/// `SCB_AIRCR` reset key + `SYSRESETREQ` value from the document.
fn reset_after_cookie(mailbox: usize) -> ! {
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    clean_dcache_line(mailbox);

    // Section 4.1: exact SYSRESETREQ write, `(0x05FA << 16) | (1 << 2)`.
    const SCB_AIRCR: *mut u32 = 0xE000_ED0C as *mut u32;
    // SAFETY: SCB_AIRCR is a defined Cortex-M system register; writing the
    // unlock key with SYSRESETREQ requests a system reset (doc section 4.1).
    unsafe {
        core::ptr::write_volatile(SCB_AIRCR, (0x05FA << 16) | (1 << 2));
    }

    // The reset must take effect; spin if it is somehow deferred.
    loop {
        core::hint::spin_loop();
    }
}

/// Perform handsfree BareBoot entry on Cortex-M. Never returns.
///
/// Presence policy: if the bootloader appears present the cookie is written
/// through `barefoc-boot` to the derived mailbox; otherwise the fallback
/// mailbox (`sram_top - 8`) is used (see module-level docs). Interrupts are
/// masked and the driver-disarm hook runs before either write.
pub fn enter_bareboot(target: BootTarget) -> ! {
    disable_irq_and_disarm();

    if target.is_present() {
        let mailbox = target.mailbox_addr();
        barefoc_boot::write_cookie(&target);
        reset_after_cookie(mailbox)
    } else {
        // No discoverable vector table: best-effort fallback mailbox.
        let mailbox = target.fallback_mailbox_addr();
        // SAFETY: `sram_top - 8` is an aligned, preserved SRAM slot reserved
        // for the cross-reset token (doc sections 3.7 and 4.1).
        unsafe {
            core::ptr::write_volatile(mailbox as *mut u64, REQUEST_BOOTLOADER);
        }
        reset_after_cookie(mailbox)
    }
}

/// Checked variant of [`enter_bareboot`].
///
/// Returns [`NotPresent`] without writing anything or resetting when
/// `target.is_present()` is false, so an application without a patched
/// bootloader can keep running. See the module-level "Presence policy".
pub fn request_bootloader_checked(target: BootTarget) -> Result<(), NotPresent> {
    if !target.is_present() {
        return Err(NotPresent);
    }
    disable_irq_and_disarm();
    let mailbox = target.mailbox_addr();
    barefoc_boot::write_cookie(&target);
    reset_after_cookie(mailbox)
}

/// Compatibility wrapper: clears the cookie derived from `target`.
pub fn clear_boot_cookie(target: BootTarget) {
    barefoc_boot::clear_cookie(&target);
}
