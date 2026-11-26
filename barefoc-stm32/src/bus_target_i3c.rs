//! STM32 I3C **target** adapter for the `barefoc-multibus` register-pointer server.
//!
//! Drives the STM32H5 `I3C2` peripheral (`PB3` = SCL, `PB4` = SDA, AF3) in
//! MIPI I3C target mode for `barefoc-multibus` [`RegServer`] transactions,
//! including `ENTDAA` / `RSTDAA` dynamic address handling, `TGTTDR` TX FIFO
//! preloading, and handsfree bootloader entry via `0xF0` or CCC `RSTACT`.

#[cfg(feature = "bareboot")]
use barefoc::boot::BootTarget;
use barefoc_multibus::transport::{RegServer, TargetTransport};
use core::ptr::{read_volatile, write_volatile};

use crate::bus_target_i2c::validate_address;
pub use crate::bus_target_i2c::{
    Error, ServiceOutcome, WriteFrameHook, WriteFrameHookCtx, READ_WINDOW,
};

const WRITE_BUF_LEN: usize = 64;

const RCC_BASE: u32 = 0x4402_0C00;
const RCC_APB3RSTR: *mut u32 = (RCC_BASE + 0x080) as *mut u32;
const RCC_AHB2ENR: *mut u32 = (RCC_BASE + 0x08C) as *mut u32;
const RCC_APB3ENR: *mut u32 = (RCC_BASE + 0x0A8) as *mut u32;
const APB3RSTR_I3C2RST: u32 = 1 << 9;

const GPIOB_BASE: u32 = 0x4202_0400;
const GPIOB_MODER: *mut u32 = GPIOB_BASE as *mut u32;
const GPIOB_OTYPER: *mut u32 = (GPIOB_BASE + 0x04) as *mut u32;
const GPIOB_OSPEEDR: *mut u32 = (GPIOB_BASE + 0x08) as *mut u32;
const GPIOB_PUPDR: *mut u32 = (GPIOB_BASE + 0x0C) as *mut u32;
const GPIOB_IDR: *const u32 = (GPIOB_BASE + 0x10) as *const u32;
const GPIOB_AFRL: *mut u32 = (GPIOB_BASE + 0x20) as *mut u32;
const PB4_SDA_MASK: u32 = 1 << 4;

const UID_BASE: u32 = 0x08FF_F800;

const I3C2_BASE: u32 = 0x4400_3000;
const I3C2_CFGR: *mut u32 = (I3C2_BASE + 0x04) as *mut u32;
const I3C2_RDR: *const u32 = (I3C2_BASE + 0x10) as *const u32;
const I3C2_TDR: *mut u32 = (I3C2_BASE + 0x18) as *mut u32;
const I3C2_TGTTDR: *mut u32 = (I3C2_BASE + 0x24) as *mut u32;
const I3C2_SR: *const u32 = (I3C2_BASE + 0x30) as *const u32;
const I3C2_SER: *const u32 = (I3C2_BASE + 0x34) as *const u32;
const I3C2_RMR: *const u32 = (I3C2_BASE + 0x40) as *const u32;
const I3C2_EVR: *const u32 = (I3C2_BASE + 0x50) as *const u32;
const I3C2_IER: *mut u32 = (I3C2_BASE + 0x54) as *mut u32;
const I3C2_CEVR: *mut u32 = (I3C2_BASE + 0x58) as *mut u32;
const I3C2_DEVR0: *mut u32 = (I3C2_BASE + 0x60) as *mut u32;
const I3C2_MAXRLR: *mut u32 = (I3C2_BASE + 0x90) as *mut u32;
const I3C2_MAXWLR: *mut u32 = (I3C2_BASE + 0x94) as *mut u32;
const I3C2_TIMINGR0: *mut u32 = (I3C2_BASE + 0xA0) as *mut u32;
const I3C2_TIMINGR1: *mut u32 = (I3C2_BASE + 0xA4) as *mut u32;
const I3C2_TIMINGR2: *mut u32 = (I3C2_BASE + 0xA8) as *mut u32;
const I3C2_BCR: *mut u32 = (I3C2_BASE + 0xC0) as *mut u32;
const I3C2_DCR: *mut u32 = (I3C2_BASE + 0xC4) as *mut u32;
const I3C2_EPIDR: *mut u32 = (I3C2_BASE + 0xD4) as *mut u32;

const CFGR_RXFLUSH: u32 = 1 << 9;
const CFGR_TXFLUSH: u32 = 1 << 13;

// I3C_EVR bits (verified against STM32H5 i3c_v1.rs)
const EVR_TXFEF: u32 = 1 << 1;
const EVR_TXFNFF: u32 = 1 << 4;
const EVR_RXFNEF: u32 = 1 << 5;
const EVR_FCF: u32 = 1 << 9;
const EVR_RXTGTENDF: u32 = 1 << 10;
const EVR_ERRF: u32 = 1 << 11;
const EVR_DAUPDF: u32 = 1 << 24;
const EVR_RSTF: u32 = 1 << 27;

/// Interrupt bits enabled on `I3C2_IER` to wake an async I3C target task without polling delay.
pub const I3C2_EVENT_IRQ_MASK: u32 =
    EVR_RXFNEF | EVR_FCF | EVR_RXTGTENDF | EVR_ERRF | EVR_DAUPDF | EVR_RSTF;

/// Compute the `I3C2_IER` interrupt mask for the current TX FIFO state.
///
/// Enables `EVR_TXFNFF` only while `read_pos < read_len` so multi-byte reads
/// exceeding the hardware TX FIFO depth (`> 8` bytes) are refilled in ISR
/// without causing an interrupt storm once the TX buffer is exhausted.
#[inline]
pub(crate) fn desired_ier_for_tx(read_pos: usize, read_len: usize) -> u32 {
    if read_pos < read_len {
        I3C2_EVENT_IRQ_MASK | EVR_TXFNFF
    } else {
        I3C2_EVENT_IRQ_MASK
    }
}

const DEVR0_RSTACT_SHIFT: u32 = 22;
const DEVR0_RSTACT_MASK: u32 = 0x03;
const DEVR0_RSTVAL: u32 = 1 << 24;

const CCC_BCAST_RSTACT: u8 = 0x2A;
const CCC_DIRECT_RSTACT: u8 = 0x9A;
const CCC_DIRECT_RSTACT_ALT: u8 = 0xAA;
const RSTACT_DEF_TGT_RESET: u8 = 0x02;

/// Pure predicate checking whether an I3C `EVR_RSTF` event should trigger
/// bootloader failsafe recovery.
///
/// If `DEVR0.RSTVAL` is set, recovery triggers unless `DEVR0.RSTACT == 0`
/// (`NO_RESET`). If `RSTVAL` is clear (unprepared Target Reset Pattern),
/// recovery triggers immediately so wedged targets remain recoverable.
#[inline]
pub(crate) fn should_trigger_rstf_recovery(devr0: u32) -> bool {
    let rstval = (devr0 & DEVR0_RSTVAL) != 0;
    let rstact = ((devr0 >> DEVR0_RSTACT_SHIFT) & DEVR0_RSTACT_MASK) as u8;
    !(rstval && rstact == 0)
}

/// Pure predicate checking whether a received CCC (`rcode`) and optional
/// `DEVR0` / RX defining byte represent `CCC RSTACT` (`0x2A`/`0x9A`/`0xAA`)
/// requesting target reset (`0x02`).
#[inline]
pub(crate) fn is_rstact_target_reset(rcode: u8, devr0: u32, rx_byte: Option<u8>) -> bool {
    if rcode != CCC_BCAST_RSTACT && rcode != CCC_DIRECT_RSTACT && rcode != CCC_DIRECT_RSTACT_ALT {
        return false;
    }
    let hw_latched = (devr0 & DEVR0_RSTVAL) != 0
        && (((devr0 >> DEVR0_RSTACT_SHIFT) & DEVR0_RSTACT_MASK) as u8) == RSTACT_DEF_TGT_RESET;
    let rx_matched = rx_byte == Some(RSTACT_DEF_TGT_RESET);
    hw_latched || rx_matched
}

const SR_XDCNT_MASK: u32 = 0xFFFF;
const SR_DIR_READ: u32 = 1 << 18;

/// Clamp the read-prefill window to the remaining map extent from `ptr`.
#[inline]
fn clamped_read_window(ptr: usize, extent: usize) -> usize {
    extent.saturating_sub(ptr).min(READ_WINDOW)
}

/// Update consecutive `SDA == 0` tick count and trigger recovery once `threshold_ticks`
/// is reached without bus progress.
#[inline]
pub(crate) fn update_stuck_sda_counter(
    sda_low: bool,
    bus_progress: bool,
    consecutive_ticks: &mut u32,
    threshold_ticks: u32,
) -> bool {
    if !sda_low || bus_progress {
        *consecutive_ticks = 0;
        return false;
    }
    *consecutive_ticks = consecutive_ticks.saturating_add(1);
    if *consecutive_ticks >= threshold_ticks {
        *consecutive_ticks = 0;
        true
    } else {
        false
    }
}

/// STM32H5 I3C2 target adapter around [`RegServer`].
pub struct Stm32I3cTarget<'d, C, const CAP: usize, const MAX_ENTRIES: usize> {
    _marker: core::marker::PhantomData<&'d ()>,
    server: RegServer<C, CAP, MAX_ENTRIES>,
    write_buf: [u8; WRITE_BUF_LEN],
    write_len: usize,
    read_buf: [u8; READ_WINDOW],
    read_start_ptr: u16,
    read_len: usize,
    read_pos: usize,
    addr7: u8,
    last_ccc: u8,
    write_hook: Option<WriteFrameHook>,
    write_hook_ctx: Option<WriteFrameHookCtx<C>>,
    #[cfg(feature = "bareboot")]
    bootloader_target: Option<BootTarget>,
}

impl<'d, C, const CAP: usize, const MAX_ENTRIES: usize>
    Stm32I3cTarget<'d, C, CAP, MAX_ENTRIES>
{
    /// Bind `I3C2` (`PB3` = SCL, `PB4` = SDA, AF3) as an I3C target with
    /// initial dynamic/static address `addr7` and register server `server`.
    pub fn new(addr7: u8, server: RegServer<C, CAP, MAX_ENTRIES>) -> Result<Self, Error> {
        validate_address(addr7)?;
        Self::init_i3c2(addr7);
        let mut target = Self {
            _marker: core::marker::PhantomData,
            server,
            write_buf: [0u8; WRITE_BUF_LEN],
            write_len: 0,
            read_buf: [0u8; READ_WINDOW],
            read_start_ptr: 0,
            read_len: 0,
            read_pos: 0,
            addr7,
            last_ccc: 0,
            write_hook: None,
            write_hook_ctx: None,
            #[cfg(feature = "bareboot")]
            bootloader_target: None,
        };
        target.preload_read();
        Ok(target)
    }

    fn init_i3c2(addr7: u8) {
        // SAFETY: Direct PAC-level initialization of STM32H503 GPIOB (PB3/PB4 AF3)
        // and I3C2 target registers at fixed Non-Secure peripheral addresses.
        unsafe {
            write_volatile(RCC_AHB2ENR, read_volatile(RCC_AHB2ENR) | 0x02);
            write_volatile(RCC_APB3ENR, read_volatile(RCC_APB3ENR) | (1 << 9));
            let _ = read_volatile(RCC_APB3ENR);

            let moder = read_volatile(GPIOB_MODER);
            write_volatile(GPIOB_MODER, (moder & !0x03C0) | 0x0280);
            write_volatile(GPIOB_OTYPER, read_volatile(GPIOB_OTYPER) & !0x18);
            write_volatile(GPIOB_OSPEEDR, read_volatile(GPIOB_OSPEEDR) | 0x03C0);
            write_volatile(GPIOB_PUPDR, (read_volatile(GPIOB_PUPDR) & !0x03C0) | 0x0140);
            write_volatile(
                GPIOB_AFRL,
                (read_volatile(GPIOB_AFRL) & !0x000F_F000) | 0x0003_3000,
            );

            let uid0 = read_volatile(UID_BASE as *const u32);
            let uid1 = read_volatile((UID_BASE + 4) as *const u32);
            let uid2 = read_volatile((UID_BASE + 8) as *const u32);
            let instance_id = (uid0 ^ uid1 ^ uid2) & 0x0F;

            write_volatile(I3C2_CFGR, 0);
            write_volatile(I3C2_CFGR, (1 << 3) | (1 << 4)); // RSTPTRN | EXITPTRN
            write_volatile(I3C2_BCR, 0x06);
            write_volatile(I3C2_DCR, 0x63);
            write_volatile(
                I3C2_EPIDR,
                (0x0114 << 17) | (instance_id << 12) | 0x0001,
            );
            write_volatile(I3C2_DEVR0, ((addr7 as u32 & 0x7F) << 1) | 1);
            write_volatile(I3C2_MAXRLR, 128);
            write_volatile(I3C2_MAXWLR, 128);
            write_volatile(I3C2_TIMINGR0, 0x1818_0303);
            write_volatile(I3C2_TIMINGR1, (64 << 16) | 64);
            write_volatile(I3C2_TIMINGR2, 0);
            write_volatile(I3C2_CEVR, 0xFFFF_FFFF);
            write_volatile(I3C2_CFGR, read_volatile(I3C2_CFGR) | 1);
        }
    }

    /// Borrow the underlying [`RegServer`].
    pub fn server(&self) -> &RegServer<C, CAP, MAX_ENTRIES> {
        &self.server
    }

    /// Mutably borrow the underlying [`RegServer`] to publish telemetry.
    pub fn server_mut(&mut self) -> &mut RegServer<C, CAP, MAX_ENTRIES> {
        &mut self.server
    }

    /// Return the configured 7-bit address.
    pub fn addr7(&self) -> u8 {
        self.addr7
    }

    /// Register a raw write-frame observer invoked before the register server.
    pub fn set_write_frame_hook(&mut self, hook: Option<WriteFrameHook>) {
        self.write_hook = hook;
    }

    /// Register a context-capable write-frame observer.
    pub fn set_write_frame_hook_ctx(&mut self, hook: Option<WriteFrameHookCtx<C>>) {
        self.write_hook_ctx = hook;
    }

    /// Arm handsfree BareBoot bootloader entry on `0xF0` or CCC `RSTACT`.
    #[cfg(feature = "bareboot")]
    pub fn enable_bootloader(&mut self, target: BootTarget) {
        self.bootloader_target = Some(target);
    }

    #[inline]
    fn desired_ier(&self) -> u32 {
        desired_ier_for_tx(self.read_pos, self.read_len)
    }

    /// Return `true` when `PB4` (`I3C2_SDA`) is currently reading low (`0`).
    pub fn is_sda_low(&self) -> bool {
        // SAFETY: Reading read-only `GPIOB_IDR` register.
        unsafe { (read_volatile(GPIOB_IDR) & PB4_SDA_MASK) == 0 }
    }

    /// Perform a full `RCC_APB3RSTR` peripheral reset of `I3C2` and re-arm the target.
    pub fn reset_i3c2_peripheral(&mut self) {
        // SAFETY: Resets only the `I3C2` peripheral via `RCC_APB3RSTR.I3C2RST` (bit 9)
        // to release a wedged target FSM holding `PB4` (`SDA`) low, then re-initializes
        // `I3C2` at the active address and preloads the TX FIFO.
        unsafe {
            write_volatile(I3C2_CFGR, 0);
            let rstr = read_volatile(RCC_APB3RSTR);
            write_volatile(RCC_APB3RSTR, rstr | APB3RSTR_I3C2RST);
            for _ in 0..8 {
                core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
            }
            write_volatile(RCC_APB3RSTR, rstr & !APB3RSTR_I3C2RST);
        }
        self.write_len = 0;
        self.last_ccc = 0;
        Self::init_i3c2(self.addr7);
        self.preload_read();
    }

    /// Poll `PB4` (`SDA`) and reset `I3C2` if `SDA == 0` persists across `threshold_ticks`
    /// without bus event progress. Returns `(recovery_triggered, still_wedged)`.
    pub fn check_and_recover_stuck_sda(
        &mut self,
        low_ticks: &mut u32,
        threshold_ticks: u32,
    ) -> (bool, bool) {
        let sda_low = self.is_sda_low();
        // SAFETY: Reading read-only `I3C2_EVR`.
        let bus_progress = unsafe {
            (read_volatile(I3C2_EVR)
                & (EVR_FCF | EVR_RXFNEF | EVR_DAUPDF | EVR_RSTF | EVR_ERRF))
                != 0
        };
        if update_stuck_sda_counter(sda_low, bus_progress, low_ticks, threshold_ticks) {
            self.reset_i3c2_peripheral();
            return (true, self.is_sda_low());
        }
        (false, false)
    }

    /// Return `true` if any enabled I3C2 event/error flag is pending in hardware.
    pub fn irq_events_pending(&self) -> bool {
        // SAFETY: Reading read-only status bits in `I3C2_EVR`.
        unsafe { (read_volatile(I3C2_EVR) & self.desired_ier()) != 0 }
    }

    /// Mask all `I3C2` interrupt sources (typically called from ISR before waking task).
    pub fn mask_irqs(&self) {
        // SAFETY: Clearing `I3C2_IER` prevents level-triggered ISR re-entry until serviced.
        unsafe {
            write_volatile(I3C2_IER, 0);
        }
    }

    /// Unmask `I3C2` interrupt sources relevant to the current RX/TX state.
    pub fn unmask_irqs(&self) {
        // SAFETY: Writing `I3C2_IER` enables hardware wakeup for the next I3C2 event.
        unsafe {
            write_volatile(I3C2_IER, self.desired_ier());
        }
    }

    /// Return `true` when no partial RX frame is currently buffered.
    pub fn is_frame_idle(&self) -> bool {
        self.write_len == 0
    }

    /// Refresh the preloaded TX FIFO from the current register map when the bus is idle.
    ///
    /// Should be called after telemetry updates so subsequent host reads observe fresh
    /// register bytes without colliding with an in-flight I3C frame.
    pub fn refresh_read_buffer(&mut self) {
        if !self.is_frame_idle() {
            return;
        }
        // SAFETY: `I3C2` is clocked and enabled for the lifetime of `Stm32I3cTarget`.
        let evr = unsafe { read_volatile(I3C2_EVR) };
        if evr & (EVR_RXFNEF | EVR_FCF) != 0 {
            return;
        }
        self.preload_read();
    }

    /// Reposition the register pointer after a short read or non-read frame.
    fn reposition_read_ptr(&mut self, delivered: usize) {
        let extent = self.server.map().extent();
        let next = usize::from(self.read_start_ptr).saturating_add(delivered);
        if next < extent {
            let _ = self.server.on_write(&(next as u16).to_be_bytes());
        }
    }

    /// Drain all currently available bytes from the I3C2 RX FIFO into `write_buf`.
    ///
    /// # Safety
    /// Caller must ensure `I3C2` is clocked and enabled.
    unsafe fn drain_rx_fifo(&mut self) {
        while read_volatile(I3C2_EVR) & EVR_RXFNEF != 0 {
            let b = (read_volatile(I3C2_RDR) & 0xFF) as u8;
            if self.write_len < self.write_buf.len() {
                self.write_buf[self.write_len] = b;
                self.write_len += 1;
            }
        }
    }

    /// Preload the I3C2 TX FIFO from the current register pointer.
    fn preload_read(&mut self) {
        let ptr = usize::from(self.server.map().pointer());
        let extent = self.server.map().extent();
        let len = clamped_read_window(ptr, extent);
        let filled = if len > 0 {
            self.server
                .on_read(len, &mut self.read_buf[..len])
                .unwrap_or(0)
        } else {
            0
        };
        self.read_start_ptr = ptr as u16;
        self.read_len = filled;
        self.read_pos = 0;
        // SAFETY: Clear TGTTDR and flush any unconsumed TX FIFO bytes via CFGR.TXFLUSH
        // without toggling CFGR.EN (which would reset the I3C2 target state machine on
        // the bus), then arm TGTTDR and push available FIFO bytes.
        unsafe {
            write_volatile(I3C2_TGTTDR, 0);
            if read_volatile(I3C2_EVR) & EVR_TXFEF == 0 {
                write_volatile(I3C2_CFGR, read_volatile(I3C2_CFGR) | CFGR_TXFLUSH);
                for _ in 0..64 {
                    if read_volatile(I3C2_EVR) & EVR_TXFEF != 0 {
                        break;
                    }
                }
            }
            if filled > 0 {
                write_volatile(I3C2_TGTTDR, (1 << 16) | (filled as u32));
                while self.read_pos < self.read_len
                    && (read_volatile(I3C2_EVR) & EVR_TXFNFF != 0)
                {
                    write_volatile(I3C2_TDR, self.read_buf[self.read_pos] as u32);
                    self.read_pos += 1;
                }
            }
        }
    }

    /// Non-blocking poll/service of `I3C2` target events.
    pub fn service_once(&mut self) -> Result<ServiceOutcome, Error> {
        unsafe {
            let evr = read_volatile(I3C2_EVR);

            if evr & EVR_RSTF != 0 {
                write_volatile(I3C2_CEVR, EVR_RSTF);
                let devr0 = read_volatile(I3C2_DEVR0);
                #[cfg(feature = "bareboot")]
                if self.bootloader_target.is_some() && should_trigger_rstf_recovery(devr0) {
                    crate::bootloader::clear_boot_loop_guard();
                    crate::bootloader::barefoc_failsafe_recovery();
                }
                let _ = devr0;
            }

            if evr & EVR_DAUPDF != 0 {
                write_volatile(I3C2_CEVR, EVR_DAUPDF);
                let devr0 = read_volatile(I3C2_DEVR0);
                if devr0 & 1 != 0 {
                    self.addr7 = ((devr0 >> 1) & 0x7F) as u8;
                    #[cfg(feature = "bareboot")]
                    {
                        crate::bootloader::set_active_dynamic_node_id(self.addr7);
                        crate::bootloader::set_active_bus_type(1);
                    }
                }
            }

            let rmr = read_volatile(I3C2_RMR);
            let rcode = ((rmr >> 8) & 0xFF) as u8;
            if rcode != self.last_ccc {
                self.last_ccc = rcode;
                if rcode == 0x06 {
                    // RSTDAA: clear DAVAL so ENTDAA can re-assign
                    write_volatile(I3C2_DEVR0, read_volatile(I3C2_DEVR0) & !1);
                }
                #[cfg(feature = "bareboot")]
                if self.bootloader_target.is_some()
                    && is_rstact_target_reset(rcode, read_volatile(I3C2_DEVR0), None)
                {
                    crate::bootloader::clear_boot_loop_guard();
                    crate::bootloader::barefoc_failsafe_recovery();
                }
            }

            if evr & EVR_ERRF != 0 {
                let _ = read_volatile(I3C2_SER);
                write_volatile(I3C2_CEVR, EVR_ERRF | EVR_FCF | EVR_RXTGTENDF);
                write_volatile(
                    I3C2_CFGR,
                    read_volatile(I3C2_CFGR) | CFGR_RXFLUSH | CFGR_TXFLUSH,
                );
                self.write_len = 0;
                self.preload_read();
                return Ok(ServiceOutcome::Idle);
            }

            self.drain_rx_fifo();

            while self.read_pos < self.read_len && (read_volatile(I3C2_EVR) & EVR_TXFNFF != 0) {
                write_volatile(I3C2_TDR, self.read_buf[self.read_pos] as u32);
                self.read_pos += 1;
            }

            let evr_end = read_volatile(I3C2_EVR);
            if evr_end & (EVR_FCF | EVR_RXTGTENDF) != 0 {
                self.drain_rx_fifo();
                write_volatile(I3C2_CEVR, EVR_FCF | EVR_RXTGTENDF);

                #[cfg(feature = "bareboot")]
                if self.bootloader_target.is_some() {
                    let rx_first = if self.write_len > 0 {
                        Some(self.write_buf[0])
                    } else {
                        None
                    };
                    if is_rstact_target_reset(self.last_ccc, read_volatile(I3C2_DEVR0), rx_first) {
                        crate::bootloader::clear_boot_loop_guard();
                        crate::bootloader::barefoc_failsafe_recovery();
                    }
                }

                if self.write_len > 0 {
                    let n = self.write_len;
                    self.write_len = 0;

                    // Check CCC RSTACT defining byte 0x02 or raw 0xF0 strobe
                    #[cfg(feature = "bareboot")]
                    if let Some(_target) = self.bootloader_target {
                        if n == 1 && barefoc::boot::trigger_i3c_byte(self.write_buf[0]) {
                            crate::bootloader::clear_boot_loop_guard();
                            crate::bootloader::barefoc_failsafe_recovery();
                        }
                    }

                    if let Some(hook) = self.write_hook {
                        hook(&self.write_buf[..n]);
                    }
                    if let Some(hook_ctx) = self.write_hook_ctx {
                        hook_ctx(&self.write_buf[..n], self.server.map_mut().handler());
                    }

                    return match self.server.on_write(&self.write_buf[..n]) {
                        Ok(()) => {
                            self.preload_read();
                            Ok(ServiceOutcome::WritePhase { bytes: n })
                        }
                        Err(e) => {
                            self.preload_read();
                            Ok(ServiceOutcome::WriteRejected { bytes: n, error: e })
                        }
                    };
                } else {
                    let sr = read_volatile(I3C2_SR);
                    if (sr & SR_DIR_READ) == 0 {
                        // Non-read frame with no RX payload (e.g. ENTDAA/RSTDAA CCC):
                        // restore pointer so preloaded window stays aligned without skipping bytes.
                        self.reposition_read_ptr(0);
                        self.preload_read();
                        return Ok(ServiceOutcome::Idle);
                    }
                    // Read frame completed; reposition pointer on short reads, then re-arm TX FIFO.
                    let requested = self.read_len;
                    let delivered = ((sr & SR_XDCNT_MASK) as usize).min(requested);
                    if delivered < requested {
                        self.reposition_read_ptr(delivered);
                    }
                    self.preload_read();
                    return Ok(ServiceOutcome::ReadPhase {
                        requested,
                        delivered,
                    });
                }
            }
        }

        Ok(ServiceOutcome::Idle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamped_read_window_bounds_to_remaining_extent() {
        assert_eq!(clamped_read_window(0, 121), READ_WINDOW);
        assert_eq!(clamped_read_window(0x64, 0x79), 0x15);
        assert_eq!(clamped_read_window(0x70, 0x79), 0x09);
        assert_eq!(clamped_read_window(0x78, 0x79), 0x01);
        assert_eq!(clamped_read_window(0x79, 0x79), 0x00);
        assert_eq!(clamped_read_window(0x80, 0x79), 0x00);
    }

    #[test]
    fn rstf_and_rstact_predicates_cover_all_escape_forms() {
        // RSTVAL=0 (unprepared target reset pattern) -> triggers recovery
        assert!(should_trigger_rstf_recovery(0));
        // RSTVAL=1, RSTACT=0 (explicit NO_RESET) -> suppressed
        assert!(!should_trigger_rstf_recovery(DEVR0_RSTVAL));
        // RSTVAL=1, RSTACT=2 (target system reset) -> triggers recovery
        assert!(should_trigger_rstf_recovery(DEVR0_RSTVAL | (2 << DEVR0_RSTACT_SHIFT)));

        // CCC RSTACT forms: 0x2A, 0x9A, 0xAA with defining byte 0x02 (HW or RX)
        let devr0_rst2 = DEVR0_RSTVAL | (2 << DEVR0_RSTACT_SHIFT);
        assert!(is_rstact_target_reset(0x2A, devr0_rst2, None));
        assert!(is_rstact_target_reset(0x9A, 0, Some(0x02)));
        assert!(is_rstact_target_reset(0xAA, devr0_rst2, None));
        assert!(!is_rstact_target_reset(0x2A, DEVR0_RSTVAL, Some(0x00)));
        assert!(!is_rstact_target_reset(0x06, devr0_rst2, Some(0x02)));
    }

    #[test]
    fn stuck_sda_counter_triggers_at_threshold_and_resets_on_high_or_progress() {
        let mut ticks = 0;
        for _ in 0..49 {
            assert!(!update_stuck_sda_counter(true, false, &mut ticks, 50));
        }
        assert_eq!(ticks, 49);
        // Bus progress clears counter
        assert!(!update_stuck_sda_counter(true, true, &mut ticks, 50));
        assert_eq!(ticks, 0);
        // Idle high clears counter
        ticks = 10;
        assert!(!update_stuck_sda_counter(false, false, &mut ticks, 50));
        assert_eq!(ticks, 0);
        // Reaches threshold and resets for next cycle
        for _ in 0..49 {
            assert!(!update_stuck_sda_counter(true, false, &mut ticks, 50));
        }
        assert!(update_stuck_sda_counter(true, false, &mut ticks, 50));
        assert_eq!(ticks, 0);
    }

    #[test]
    fn desired_ier_for_tx_enables_txfnff_only_when_bytes_remain() {
        assert_eq!(desired_ier_for_tx(0, 0), I3C2_EVENT_IRQ_MASK);
        assert_eq!(desired_ier_for_tx(8, 8), I3C2_EVENT_IRQ_MASK);
        assert_eq!(desired_ier_for_tx(32, 32), I3C2_EVENT_IRQ_MASK);
        assert_eq!(
            desired_ier_for_tx(8, 32),
            I3C2_EVENT_IRQ_MASK | EVR_TXFNFF
        );
        assert_eq!(
            desired_ier_for_tx(0, 16),
            I3C2_EVENT_IRQ_MASK | EVR_TXFNFF
        );
    }
}
