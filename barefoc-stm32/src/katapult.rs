//! Legacy Katapult compatibility shim over the BareBoot client.
//!
//! This module keeps the historical public API (`request_bootloader`,
//! `clear_boot_cookie`, `CANBOOT_MAGIC_ADDR`, `REQUEST_CANBOOT`,
//! `FLASH_BOOT_ADDRESS`) so existing applications such as FishyServo keep
//! compiling. The implementation now lives in [`crate::bootloader`] and in the
//! portable `barefoc-boot` crate. New code should prefer the `bareboot` module
//! API directly:
//!
//! - [`crate::bootloader::enter_bareboot`] — never returns, presence-checked
//!   with fallback mailbox;
//! - [`crate::bootloader::request_bootloader_checked`] — returns
//!   [`crate::bootloader::NotPresent`] instead of resetting when no compatible
//!   bootloader is installed;
//! - [`crate::bootloader::before_reset_hook`] — driver disarm callback.
//!
//! All items below are deprecated aliases; they forward to the current API
//! rather than duplicating the mailbox/reset logic.

#[deprecated(note = "use barefoc::boot::REQUEST_BOOTLOADER (or ::REQUEST_CANBOOT) instead")]
pub const REQUEST_CANBOOT: u64 = barefoc::boot::REQUEST_BOOTLOADER;

#[deprecated(note = "use barefoc::boot::BootTarget / bootloader::default_target() instead")]
pub const FLASH_BOOT_ADDRESS: usize = crate::bootloader::DEFAULT_FLASH_BASE as usize;

#[deprecated(
    note = "derive the mailbox from the bootloader vector table via BootTarget::mailbox_addr()"
)]
pub const CANBOOT_MAGIC_ADDR: usize = 0x2000_7FF8;

/// Legacy re-entry helper: requests BareBoot entry for the default STM32 layout.
///
/// Delegates to [`crate::bootloader::enter_bareboot`], which masks interrupts,
/// runs the optional driver-disarm hook, checks bootloader presence, writes the
/// cookie to the derived (or fallback) mailbox, and resets. Never returns.
#[deprecated(
    note = "use crate::bootloader::enter_bareboot or request_bootloader_checked with an explicit BootTarget"
)]
pub fn request_bootloader() -> ! {
    crate::bootloader::enter_bareboot(crate::bootloader::default_target())
}

/// Legacy cold-boot cookie clear for the default STM32 layout.
#[deprecated(note = "use crate::bootloader::clear_boot_cookie with an explicit BootTarget")]
pub fn clear_boot_cookie() {
    crate::bootloader::clear_boot_cookie(crate::bootloader::default_target())
}

// New API re-exports, so `katapult`-only consumers can migrate without changing
// the module path they import from.
pub use crate::bootloader::{
    before_reset_hook, clear_before_reset_hook, clear_boot_cookie as clear_boot_cookie_checked,
    default_target, enter_bareboot, request_bootloader_checked, NotPresent, DEFAULT_APP_BASE,
    DEFAULT_FLASH_BASE, DEFAULT_SRAM_TOP,
};
