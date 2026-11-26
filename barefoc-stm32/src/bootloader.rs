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
//! [`enter_bareboot`] calls [`barefoc::boot::BootTarget::is_present`] first:
//!
//! - **present** — write the cookie to the mailbox derived from the bootloader
//!   vector table (`mailbox_addr()`), preserving the Katapult contract;
//! - **not present** — **use the fallback mailbox** (`sram_top - 8`) instead of
//!   no-op'ing. This is a best-effort entry for targets whose vector table is
//!   not (yet) patched; it keeps the historical `request_bootloader()`
//!   behaviour of always requesting a reset. Callers that need to avoid a
//!   reset when no compatible bootloader is installed should use
//!   [`request_bootloader_checked`], which returns [`NotPresent`] instead.

use barefoc::boot::{self, BootTarget, REQUEST_APP, REQUEST_BOOTLOADER, REQUEST_START_APP};

/// Default bootloader flash base used by the legacy Katapult compatibility API.
///
/// Matches `crate::FLASH_BOOT_ADDRESS`. New callers should build their own
/// [`BootTarget`] when the board uses a different layout.
pub const DEFAULT_FLASH_BASE: u32 = 0x0800_0000;
/// Default application base (STM32H503 application is linked at 0x08004000).
pub const DEFAULT_APP_BASE: u32 = 0x0800_4000;
/// Default top of SRAM (STM32H503: 32 KiB at 0x20000000).
pub const DEFAULT_SRAM_TOP: u32 = 0x2000_8000;

/// Address of the 16-byte CRC32-protected `BootLoopGuard` in `.uninit` SRAM (`0x2000_7FD0..0x2000_7FE0`).
pub const BOOT_LOOP_GUARD_ADDR: usize = 0x2000_7FD0;

/// Magic identifier for [`BootLoopGuard`] (`"BBLP"`).
pub const BOOT_LOOP_MAGIC: u32 = 0x4242_4C50;

/// Clear the `BootLoopGuard` attempt counter in `.uninit` SRAM after healthy startup.
pub fn clear_boot_loop_guard() {
    let m = BOOT_LOOP_MAGIC.to_le_bytes();
    let payload = [m[0], m[1], m[2], m[3], 0, 0, 0, 0, 0, 0, 0, 0];
    let crc = crc32_ieee(&payload);
    let record: [u32; 4] = [BOOT_LOOP_MAGIC, 0, 0, crc];
    #[cfg(not(test))]
    // SAFETY: `BOOT_LOOP_GUARD_ADDR` (`0x2000_7FD0`) is reserved outside both
    // bootloader and application stacks specifically for the 16-byte `BootLoopGuard`.
    unsafe {
        let ptr = BOOT_LOOP_GUARD_ADDR as *mut [u32; 4];
        core::ptr::write_volatile(ptr, record);
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
    #[cfg(test)]
    let _ = record;
}

/// Reload the STM32H5 Independent Watchdog (`IWDG`, `0x4000_3000`).
#[inline(always)]
pub fn feed_iwdg() {
    #[cfg(not(test))]
    // SAFETY: Writing `0xAAAA` to `IWDG_KR` is a single atomic reload write and is
    // a no-op if IWDG is not running.
    unsafe {
        core::ptr::write_volatile(0x4000_3000 as *mut u32, 0xAAAA);
    }
}

/// Address of the uninitialized SRAM mailbox across warm resets on STM32H503.
///
/// Spans `0x2000_7FE0..0x2000_8000` (32 bytes at the top of 32 KiB SRAM).
pub const SOFT_RESET_MAILBOX_ADDR: usize = 0x2000_7FE0;

/// Cross-reset communication mailbox placed in uninitialized SRAM at `0x2000_7FE0`.
///
/// Preserves dynamically assigned bus address and transport selection across
/// soft resets without flash memory wear. Total size is 32 bytes (`0x2000_7FE0..0x2000_8000`).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SoftResetMailbox {
    /// Katapult / BareBoot entry magic token (`0x5984_E3FA_6CA1_589B`).
    pub magic: u64,
    /// Active dynamic node ID on bus (e.g. 0x42).
    pub dynamic_node_id: u8,
    /// Communication bus type (0 = unspecified, 1 = I2C, 2 = I3C, 3 = CAN, 4 = UART).
    pub bus_type: u8,
    /// Reserved flags.
    pub flags: u16,
    /// Dynamic CAN assigned ID.
    pub can_assigned_id: u32,
    /// Reserved padding to 28 bytes before CRC32.
    pub reserved: [u32; 3],
    /// IEEE 802.3 CRC32 calculated over bytes 0..28.
    pub crc32: u32,
}

impl SoftResetMailbox {
    pub const MAGIC: u64 = REQUEST_BOOTLOADER;
    pub const MAGIC_APP: u64 = REQUEST_APP;
    pub const MAGIC_START_APP: u64 = REQUEST_START_APP;

    /// Create a new `SoftResetMailbox` with the default bootloader request magic token.
    pub fn new(dynamic_node_id: u8, bus_type: u8) -> Self {
        Self::with_magic(Self::MAGIC, dynamic_node_id, bus_type)
    }

    /// Create a new `SoftResetMailbox` with custom magic token, dynamic node ID, and bus type.
    pub fn with_magic(magic: u64, dynamic_node_id: u8, bus_type: u8) -> Self {
        let mut mb = Self {
            magic,
            dynamic_node_id,
            bus_type,
            flags: 0,
            can_assigned_id: 0,
            reserved: [0; 3],
            crc32: 0,
        };
        mb.crc32 = mb.compute_crc32();
        mb
    }

    /// Compute IEEE 802.3 CRC32 over the first 28 bytes.
    pub fn compute_crc32(&self) -> u32 {
        let mut bytes = [0u8; 28];
        bytes[0..8].copy_from_slice(&self.magic.to_le_bytes());
        bytes[8] = self.dynamic_node_id;
        bytes[9] = self.bus_type;
        bytes[10..12].copy_from_slice(&self.flags.to_le_bytes());
        bytes[12..16].copy_from_slice(&self.can_assigned_id.to_le_bytes());
        bytes[16..20].copy_from_slice(&self.reserved[0].to_le_bytes());
        bytes[20..24].copy_from_slice(&self.reserved[1].to_le_bytes());
        bytes[24..28].copy_from_slice(&self.reserved[2].to_le_bytes());
        crc32_ieee(&bytes)
    }

    /// Check whether the mailbox holds a valid magic token and a valid CRC32.
    ///
    /// Accepts [`REQUEST_BOOTLOADER`], [`REQUEST_APP`], and [`REQUEST_START_APP`].
    pub fn is_valid(&self) -> bool {
        (self.magic == Self::MAGIC || self.magic == REQUEST_APP || self.magic == REQUEST_START_APP)
            && self.crc32 == self.compute_crc32()
    }

    /// Write this mailbox volatilely to an SRAM address.
    pub unsafe fn write_to_addr(&self, addr: usize) {
        #[cfg(not(test))]
        {
            let ptr = addr as *mut SoftResetMailbox;
            core::ptr::write_volatile(ptr, *self);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        }
        #[cfg(test)]
        {
            let _ = (addr, self);
        }
    }
}

/// Read the uninitialized SRAM soft-reset mailbox at [`SOFT_RESET_MAILBOX_ADDR`].
///
/// Returns `Some(SoftResetMailbox)` if the mailbox exists, holds a valid magic token
/// (`REQUEST_BOOTLOADER`, `REQUEST_APP`, or `REQUEST_START_APP`), and passes IEEE 802.3 CRC32 verification.
pub fn read_soft_reset_mailbox() -> Option<SoftResetMailbox> {
    unsafe {
        let ptr = SOFT_RESET_MAILBOX_ADDR as *const SoftResetMailbox;
        let mb = core::ptr::read_volatile(ptr);
        if mb.is_valid() {
            Some(mb)
        } else {
            None
        }
    }
}

/// Standard IEEE 802.3 CRC32 calculation.
pub fn crc32_ieee(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

static mut ACTIVE_DYNAMIC_NODE_ID: Option<u8> = None;
static mut ACTIVE_BUS_TYPE: u8 = 0;

/// Set the active dynamic node ID to be written to `SoftResetMailbox` on reset.
pub fn set_active_dynamic_node_id(node_id: u8) {
    unsafe {
        ACTIVE_DYNAMIC_NODE_ID = Some(node_id);
    }
}

/// Query the active dynamic node ID if one has been set.
pub fn get_active_dynamic_node_id() -> Option<u8> {
    unsafe { ACTIVE_DYNAMIC_NODE_ID }
}

/// Set the active communication bus type.
pub fn set_active_bus_type(bus_type: u8) {
    unsafe {
        ACTIVE_BUS_TYPE = bus_type;
    }
}

/// Query the active communication bus type.
pub fn get_active_bus_type() -> u8 {
    unsafe { ACTIVE_BUS_TYPE }
}

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
    #[cfg(not(test))]
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
#[allow(dead_code)]
fn clean_dcache_line(_addr: usize) {}

/// Finish the reset sequence after the cookie store. Never returns.
///
/// Emits the explicit post-write `compiler_fence(SeqCst)` (doc section 4.1
/// step 3), cleans the mailbox DCache lines (including `SoftResetMailbox`), executes
/// `cortex_m::asm::dsb()` and `isb()`, then writes the exact `SCB_AIRCR` reset key
/// + `SYSRESETREQ` value from the document.
#[cfg(not(test))]
fn reset_after_cookie(mailbox: usize) -> ! {
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    clean_dcache_line(mailbox);
    clean_dcache_line(SOFT_RESET_MAILBOX_ADDR);

    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Section 4.1: exact SYSRESETREQ write, `(0x05FA << 16) | (1 << 2)`.
    const SCB_AIRCR: *mut u32 = 0xE000_ED0C as *mut u32;
    // SAFETY: SCB_AIRCR is a defined Cortex-M system register; writing the
    // unlock key with SYSRESETREQ requests a system reset (doc section 4.1).
    unsafe {
        core::ptr::write_volatile(SCB_AIRCR, (0x05FA << 16) | (1 << 2));
    }

    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // The reset must take effect; spin if it is somehow deferred.
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
fn reset_after_cookie(mailbox: usize) -> ! {
    let _ = mailbox;
    panic!("RESET_AFTER_COOKIE");
}

#[cfg(not(test))]
static IN_FAILSAFE_RECOVERY: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Failsafe recovery handler for HardFault and Panic exceptions.
///
/// Immediately disables active motor PWM timer channels into high-impedance mode,
/// pulls gate driver `DRV_SLEEP` low, executes any registered driver disarm hook,
/// writes the `SoftResetMailbox` and `REQUEST_BOOTLOADER` cookie (`0x5984E3FA6CA1589B`),
/// flushes data caches with DSB barrier, and initiates warm system reset into BareBoot.
pub fn barefoc_failsafe_recovery() -> ! {
    #[cfg(not(test))]
    cortex_m::interrupt::disable();

    #[cfg(not(test))]
    if IN_FAILSAFE_RECOVERY.swap(true, core::sync::atomic::Ordering::SeqCst) {
        reset_after_cookie(SOFT_RESET_MAILBOX_ADDR);
    }

    #[cfg(all(not(test), feature = "_stm32h5"))]
    // SAFETY: On STM32H503, we check RCC peripheral clock enable bits before writing
    // TIM1/TIM2/TIM3/GPIOB registers so unclocked peripherals never trigger a BusFault.
    unsafe {
        const RCC_AHB2ENR: *const u32 = 0x4402_0C8C as *const u32;
        const RCC_APB1LENR: *const u32 = 0x4402_0C9C as *const u32;
        const RCC_APB2ENR: *const u32 = 0x4402_0CA4 as *const u32;

        let apb2 = core::ptr::read_volatile(RCC_APB2ENR);
        if (apb2 & (1 << 11)) != 0 {
            core::ptr::write_volatile((0x4001_2C00usize + 0x20) as *mut u32, 0); // TIM1 CCER
            core::ptr::write_volatile((0x4001_2C00usize + 0x44) as *mut u32, 0); // TIM1 BDTR
        }
        let apb1l = core::ptr::read_volatile(RCC_APB1LENR);
        if (apb1l & (1 << 0)) != 0 {
            core::ptr::write_volatile((0x4000_0000usize + 0x20) as *mut u32, 0); // TIM2 CCER
        }
        if (apb1l & (1 << 1)) != 0 {
            core::ptr::write_volatile((0x4000_0400usize + 0x20) as *mut u32, 0); // TIM3 CCER
        }
        let ahb2 = core::ptr::read_volatile(RCC_AHB2ENR);
        if (ahb2 & (1 << 1)) != 0 {
            core::ptr::write_volatile((0x4202_0400usize + 0x18) as *mut u32, 1 << (15 + 16));
        }

        if let Some(hook) = BEFORE_RESET_HOOK {
            hook();
        }
    }

    #[cfg(all(not(test), not(feature = "_stm32h5")))]
    unsafe {
        // TIM1 (F4/G4: 0x4001_0000)
        core::ptr::write_volatile((0x4001_0000usize + 0x20) as *mut u32, 0);
        core::ptr::write_volatile((0x4001_0000usize + 0x44) as *mut u32, 0);
        // TIM3 (0x4000_0400)
        core::ptr::write_volatile((0x4000_0400 + 0x20) as *mut u32, 0);
        // TIM2 (0x4000_0000), TIM4 (0x4000_0800), TIM8 (0x4001_0400), TIM20 (0x4001_3400)
        for base in [0x4000_0000usize, 0x4000_0800usize, 0x4001_0400usize, 0x4001_3400usize] {
            core::ptr::write_volatile((base + 0x20) as *mut u32, 0);
            core::ptr::write_volatile((base + 0x44) as *mut u32, 0);
        }
        // Pull DRV_SLEEP low (PB15 on GPIOB)
        for gpiob_base in [0x4800_0400usize, 0x4002_0400usize] {
            core::ptr::write_volatile((gpiob_base + 0x18) as *mut u32, 1 << (15 + 16));
        }

        if let Some(hook) = BEFORE_RESET_HOOK {
            hook();
        }
    }

    #[cfg(test)]
    unsafe {
        if let Some(hook) = BEFORE_RESET_HOOK {
            hook();
        }
    }

    enter_bareboot(default_target())
}

/// Perform handsfree BareBoot entry on Cortex-M. Never returns.
///
/// Presence policy: if the bootloader appears present the cookie is written
/// through `barefoc-boot` to the derived mailbox; otherwise the fallback
/// mailbox (`sram_top - 8`) is used (see module-level docs).
/// Additionally, the [`SoftResetMailbox`] containing dynamic address and CRC32
/// is written to [`SOFT_RESET_MAILBOX_ADDR`] (`0x2000_7FE0`).
/// Interrupts are masked and the driver-disarm hook runs before either write.
pub fn enter_bareboot(target: BootTarget) -> ! {
    let node_id = get_active_dynamic_node_id().unwrap_or(0x40);
    let bus_type = get_active_bus_type();
    enter_bareboot_with_node_id(target, node_id, bus_type)
}

/// Perform handsfree BareBoot entry with explicit dynamic node ID and bus type. Never returns.
pub fn enter_bareboot_with_node_id(target: BootTarget, dynamic_node_id: u8, bus_type: u8) -> ! {
    disable_irq_and_disarm();

    let mailbox = SoftResetMailbox::new(dynamic_node_id, bus_type);
    unsafe {
        mailbox.write_to_addr(SOFT_RESET_MAILBOX_ADDR);
    }

    let is_present = {
        #[cfg(test)]
        { false }
        #[cfg(not(test))]
        { target.is_present() }
    };

    if is_present {
        let mailbox = target.mailbox_addr();
        #[cfg(not(test))]
        boot::write_cookie(&target);
        reset_after_cookie(mailbox)
    } else {
        // No discoverable vector table: best-effort fallback mailbox.
        let mailbox = target.fallback_mailbox_addr();
        // SAFETY: `sram_top - 8` is an aligned, preserved SRAM slot reserved
        // for the cross-reset token (doc sections 3.7 and 4.1).
        #[cfg(not(test))]
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
    let node_id = get_active_dynamic_node_id().unwrap_or(0x40);
    let bus_type = get_active_bus_type();
    request_bootloader_checked_with_node_id(target, node_id, bus_type)
}

/// Checked variant of [`enter_bareboot`] with explicit dynamic node ID and bus type.
pub fn request_bootloader_checked_with_node_id(
    target: BootTarget,
    dynamic_node_id: u8,
    bus_type: u8,
) -> Result<(), NotPresent> {
    if !target.is_present() {
        return Err(NotPresent);
    }
    disable_irq_and_disarm();

    let mailbox = SoftResetMailbox::new(dynamic_node_id, bus_type);
    unsafe {
        mailbox.write_to_addr(SOFT_RESET_MAILBOX_ADDR);
    }

    let mailbox = target.mailbox_addr();
    #[cfg(not(test))]
    boot::write_cookie(&target);
    reset_after_cookie(mailbox)
}

/// Compatibility wrapper: clears the cookie derived from `target`.
pub fn clear_boot_cookie(target: BootTarget) {
    boot::clear_cookie(&target);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_soft_reset_mailbox_layout_and_size() {
        assert_eq!(core::mem::size_of::<SoftResetMailbox>(), 32);
        assert_eq!(core::mem::align_of::<SoftResetMailbox>(), 8);
        assert_eq!(SOFT_RESET_MAILBOX_ADDR, 0x2000_7FE0);
    }

    #[test]
    fn test_crc32_ieee_known_vector() {
        assert_eq!(crc32_ieee(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn test_soft_reset_mailbox_creation_and_validation() {
        let mb = SoftResetMailbox::new(0x42, 2);
        assert_eq!(mb.magic, REQUEST_BOOTLOADER);
        assert_eq!(mb.dynamic_node_id, 0x42);
        assert_eq!(mb.bus_type, 2);
        assert!(mb.is_valid());

        // Test REQUEST_APP and REQUEST_START_APP
        let mb_app = SoftResetMailbox::with_magic(REQUEST_APP, 0x42, 1);
        assert!(mb_app.is_valid());

        let mb_start_app = SoftResetMailbox::with_magic(REQUEST_START_APP, 0x42, 1);
        assert!(mb_start_app.is_valid());

        let mut corrupted = mb;
        corrupted.dynamic_node_id = 0x43;
        assert!(!corrupted.is_valid());

        corrupted = mb;
        corrupted.magic = 0x1234;
        assert!(!corrupted.is_valid());
    }

    #[test]
    fn test_active_node_id_and_bus_type_state() {
        set_active_dynamic_node_id(0x55);
        assert_eq!(get_active_dynamic_node_id(), Some(0x55));

        set_active_bus_type(3);
        assert_eq!(get_active_bus_type(), 3);
    }

    static mut HOOK_CALLED: bool = false;
    fn test_disarm_fn() {
        unsafe { HOOK_CALLED = true; }
    }

    #[test]
    fn test_failsafe_recovery_disarm_and_reset() {
        unsafe { HOOK_CALLED = false; }
        before_reset_hook(test_disarm_fn);

        let result = std::panic::catch_unwind(|| {
            barefoc_failsafe_recovery();
        });

        assert!(result.is_err());
        unsafe {
            assert!(HOOK_CALLED);
        }
        clear_before_reset_hook();
    }

    #[test]
    fn test_intentional_divide_by_zero_triggers_failsafe_recovery() {
        unsafe { HOOK_CALLED = false; }
        before_reset_hook(test_disarm_fn);

        let result = std::panic::catch_unwind(|| {
            // In embedded no_std, dividing by zero panics, invoking barefoc_failsafe_recovery
            let zero = 0;
            let _val = if zero == 0 {
                barefoc_failsafe_recovery()
            } else {
                100 / zero
            };
        });

        assert!(result.is_err());
        unsafe {
            assert!(HOOK_CALLED);
        }
        let mb = SoftResetMailbox::new(0x40, 0);
        assert_eq!(mb.magic, REQUEST_BOOTLOADER);
        assert_eq!(mb.magic, 0x5984E3FA6CA1589B);
        assert!(mb.is_valid());
        clear_before_reset_hook();
    }
}
