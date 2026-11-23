//! STM32 I2C **target** (slave) adapter for the `barefoc-multibus`
//! register-pointer server core.
//!
//! Binds the upstream embassy `i2c::v2` slave `listen()` API
//! (`ref/embassy/embassy-stm32/src/i2c/v2.rs`: `I2c::into_slave_multimaster`,
//! `blocking_listen()`, `blocking_respond_to_write()`, `blocking_respond_to_read()`)
//! to the bus-agnostic [`barefoc_multibus::transport::TargetTransport`] event
//! surface of the [`barefoc_multibus::transport::RegServer`], so an STM32H5
//! acts as a register-map server on the wire. The duty loop stays in the
//! **application's single task** — the adapter is "service bytes into the
//! map", not an extra executor task (design D4).
//!
//! # CRITICAL: handsfree bootloader entry requires interrupt context
//!
//! When the `bareboot` feature is enabled, [`Stm32I2cTarget::enable_bootloader`]
//! arms a one-byte `0xF0` strobe decoder. **You MUST drive
//! [`Stm32I2cTarget::service_once`] from interrupt context** — i.e. from an
//! Embassy `InterruptExecutor`-backed task or a real I2C event ISR — for the
//! strobe to be reliable under a starved motor task. A cooperative
//! (non-interrupt) Embassy task can be indefinitely starved by a non-yielding
//! duty-cycle loop; the host's `0xF0` write is then silently missed and
//! handsfree entry fails exactly when it is most needed. An interrupt handler
//! preempts a busy task (provided interrupts remain enabled) and makes entry
//! deterministic.
//!
//! This requirement comes from `bareBoot/docs/handsfree-app-updates.md`
//! section 6 ("Application-Side Trigger Robustness"). Decode exactly a
//! one-byte write `[0xF0]` — multi-byte frames are normal register traffic and
//! must never be consumed as a trigger. The stale pattern of reserving an
//! application register at offset `0xF0` for this purpose is explicitly
//! rejected by that section; the raw-frame hook path here keeps the trigger
//! ahead of the register server regardless of register-map health.
//!
//! # Katapult pin / address compatibility (Protocol Sync Rule)
//!
//! The production profile on STM32H503 is **I2C1, PB6 = SCL, PB7 = SDA, target
//! address `0x30`** — the identical pins and address the Katapult-I2C
//! bootloader targets (`ref/katapult/src/i2c_target.c`). The two modes
//! coexist because the bootloader only owns the pins during its boot window;
//! once the application initializes the target after the boot window has
//! expired it answers at `0x30` with the application's register-pointer
//! protocol, while the bootloader's Klipper-flashed flashing protocol remains
//! reachable via the boot signature path
//! (`crate::katapult::request_bootloader`).
//!
//! # Concurrency contract (design D4, as implemented)
//!
//! With the embassy **blocking** listen API there is no ISR buffering layer at
//! all: the event loop runs in **application task context**, and the I2C
//! hardware clock-stretches the bus (SBC mode, `NOSTRETCH = 0` configured by
//! `into_slave_multimaster`) to cover the server's response latency while the
//! task drains bytes into the register server. The register map is therefore
//! never touched from ISR context — the D4 invariant holds by construction.
//! (The katapult-style IRQ ring buffer + event-flag design is *not* required
//! for the blocking-first adapters; it would only reappear if the event loop
//! were moved into IRQ context, e.g. around the async `listen()`.)
//!
//! # Read prefill policy
//!
//! On a read command the adapter prefills the response with
//! `min(READ_WINDOW, extent − latched pointer)` bytes (default window `32`)
//! before releasing the clock stretch and letting the hardware clock the
//! reply; the master then controls how many bytes it actually takes via its
//! final NACK. Upstream `SLAVE_READ_FILLER` semantics cover both directions:
//!
//! - a **short master** (reads fewer bytes than were prefetched) is detected
//!   via `SendStatus::LeftoverBytes(n)`; the surplus was already streamed off
//!   the server's read window, so the adapter re-latches the register pointer
//!   to `start + bytes_actually_delivered` (pointer-only phase, no register
//!   state moves) and the *next* read transaction continues the stream at the
//!   exact byte where this one ended;
//! - a **long master** (reads past the prefetched window) under-runs the
//!   slave; the wire then reads the `0xFF` filler (upstream constant
//!   `i2c::v2::SLAVE_READ_FILLER`), so the host must issue a fresh pointer
//!   latch — the same convention as an out-of-extent pointer read.
//!
//! # Rejected writes (NAK-cover behavior)
//!
//! Embassy's blocking slave `respond()` primitives have no server-level NACK
//! support, so a write phase the server rejects (`Unmapped`, `ReadOnly`) is
//! **consumed and ignored on the wire** — no register byte moves (`RegMap`
//! whole-phase validation guarantees this). The rejection stays visible to
//! the application through [`ServiceOutcome::WriteRejected`] and is logged via
//! `defmt` when that feature is enabled. Hosts that need a hard NAK must poll
//! the server state instead (the map also stays untouched on rejection).
//!
//! # Doc-level porting reference (katapult `i2c_target.c`)
//!
//! ```ignore
//! // Production profile: I2C1, PB6=SCL, PB7=SDA, target address 0x30.
//! let i2c = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, i2c::Config::default());
//! let server = RegServer::new(miniata_map()); // RegMap descriptor set
//! let mut target = Stm32I2cTarget::new::<I2C1>(i2c, 0x30, server)?;
//! loop {
//!     // Application duty cycle first: publish telemetry, step the motor loop.
//!     target.server_mut().map_mut().set(REG_PRESENT_OUT_POS, &pos).ok();
//!     // One bus event; immediate when the bus is idle, clock-stretch-bounded
//!     // while a transaction is on the wire.
//!     match target.service_once::<I2C1>() {
//!         Ok(ServiceOutcome::WriteRejected { .. }) => {} // logged above the fold
//!         Ok(_) => {}
//!         Err(_) => break, // unrecoverable bus-level fault; re-init required
//!     }
//! }
//! ```

#[cfg(feature = "bareboot")]
use barefoc::boot::BootTarget;
use barefoc_multibus::transport::{RegServer, TargetTransport};
use embassy_stm32::i2c::{self, I2c, Master, MultiMaster, SlaveAddrConfig, SlaveCommandKind};
use embassy_stm32::mode::Blocking;
use embassy_stm32::pac;

/// Maximum write-phase frame staged by the adapter (pointer byte + payload).
const WRITE_BUF_LEN: usize = 64;
/// Default read-prefill window in bytes (bounded by the map extent as well).
pub const READ_WINDOW: usize = 32;

/// Callback invoked with the raw bytes of every received write frame, *before*
/// [`TargetTransport::on_write`] interprets the first byte as a register pointer.
///
/// The callback runs in the exact context that calls
/// [`Stm32I2cTarget::service_once`]. When that method is driven from an Embassy
/// `InterruptExecutor`, the hook therefore runs in interrupt context and can
/// preempt a starved cooperative task. It may legitimately never return (e.g. a
/// bootloader-request hook that performs a system reset).
pub type WriteFrameHook = fn(&[u8]);

/// Context-capable variant of [`WriteFrameHook`]: receives the map handler
/// context (`C` from [`RegServer`]) alongside the raw frame.
///
/// Like [`WriteFrameHook`], this runs in the caller's context (so under an
/// interrupt executor it is interrupt context) and may never return. It is
/// invoked after the context-free hook and before the register server.
pub type WriteFrameHookCtx<C> = fn(&[u8], &mut C);

// `defmt-optional` logging: compiles to nothing without the `defmt` feature.
#[cfg(feature = "defmt")]
macro_rules! log_error {
    ($($arg:tt)*) => {
        ::defmt::error!($($arg)*)
    };
}
#[cfg(not(feature = "defmt"))]
macro_rules! log_error {
    ($($arg:tt)*) => {{}};
}

/// Adapter-level error.
///
/// Register-map (server-level) errors are *not* carried here: they are
/// surfaced per event through [`ServiceOutcome`] (read-phase errors cannot
/// occur under the bounded prefill policy; write-phase rejections are
/// reported there).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The requested target address lies in the reserved I2C/I3C address space
    /// (general call, CBUS / low reserved, I3C fixed/legacy block, device-ID
    /// range, I3C broadcast / controller-start range). Binding is refused
    /// (spec: *Address Validation for Manual Assignment*).
    ReservedAddress,
    /// Bus-level fault surfaced by the upstream embassy driver (arbitration,
    /// bus error, timeout).
    Bus(i2c::Error),
}

impl From<i2c::Error> for Error {
    fn from(value: i2c::Error) -> Self {
        Error::Bus(value)
    }
}

/// One I2C slave event serviced by [`Stm32I2cTarget::service_once`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServiceOutcome {
    /// No master transaction matched this poll cycle (address flag clear).
    Idle,
    /// A write phase was applied to the server (pointer latch and/or payload).
    WritePhase {
        /// Byte count received from the controller.
        bytes: usize,
    },
    /// A write phase was **rejected by the server** (`Unmapped`, `ReadOnly`) —
    /// consumed and ignored on the wire; no register byte moved.
    WriteRejected {
        /// Byte count received from the controller.
        bytes: usize,
        /// Server-side rejection reason.
        error: barefoc_multibus::table::Error,
    },
    /// A read phase was served. `requested` is the prefetched length,
    /// `delivered` the bytes the controller clocked before its final NACK; a
    /// short controller leaves the register pointer re-latched so the next
    /// read transaction continues the stream seamlessly.
    ReadPhase {
        /// Prefetched (window-limited) response length.
        requested: usize,
        /// Bytes actually delivered to the controller.
        delivered: usize,
    },
}

/// Access to the PAC register block behind an embassy I2C instance, so the
/// adapter can *probe* the address-match flag between duty-cycle iterations —
/// making [`Stm32I2cTarget::service_once`] non-blocking while the bus is idle
/// (without the probe, `blocking_listen()` would park in the address-wait
/// loop indefinitely on an idle bus).
///
/// Embassy's `SealedInstance` internals are crate-private, so the mapping is
/// declared per concrete instance (H5 profile: I2C1 / I2C2).
pub trait I2cTargetRegs: i2c::Instance {
    /// PAC register block for this instance.
    fn target_regs() -> pac::i2c::I2c;
}

macro_rules! impl_i2c_target_regs {
    ($peri:ident) => {
        impl I2cTargetRegs for embassy_stm32::peripherals::$peri {
            fn target_regs() -> pac::i2c::I2c {
                // Same register block the embassy driver itself uses; reading
                // STATUS flags is side-effect-free.
                pac::$peri
            }
        }
    };
}

impl_i2c_target_regs!(I2C1);
impl_i2c_target_regs!(I2C2);

/// Blocking-first STM32H5 I2C target adapter around the register-pointer
/// server.
///
/// Owns the embassy `I2c` driver in slave/multimaster mode (clock stretching
/// on, SBC mode on) and a [`RegServer`] by value; the application task
/// interleaves duty-cycle work with [`Stm32I2cTarget::service_once`] calls and
/// publishes telemetry between them via [`Stm32I2cTarget::server_mut`]
/// (design D4: the table is only touched from the single application task).
///
/// Upstream Reference: `ref/embassy/embassy-stm32/src/i2c/v2.rs`
/// (`into_slave_multimaster` / `blocking_listen` /
/// `blocking_respond_to_{write,read}`), `ref/katapult/src/i2c_target.c`
/// (pin/profile-compatible boot-window coexistence).
pub struct Stm32I2cTarget<'d, C, const CAP: usize, const MAX_ENTRIES: usize> {
    i2c: I2c<'d, Blocking, MultiMaster>,
    server: RegServer<C, CAP, MAX_ENTRIES>,
    write_buf: [u8; WRITE_BUF_LEN],
    read_buf: [u8; READ_WINDOW],
    addr7: u8,
    /// Optional raw-frame observer invoked before the register server sees a
    /// write frame (see [`WriteFrameHook`]).
    write_hook: Option<WriteFrameHook>,
    /// Optional context-capable raw-frame observer (see
    /// [`WriteFrameHookCtx`]).
    write_hook_ctx: Option<WriteFrameHookCtx<C>>,
    /// Armed BareBoot target: when set, a one-byte `0xF0` write triggers
    /// handsfree entry before any hook or register-server processing.
    #[cfg(feature = "bareboot")]
    bootloader_target: Option<BootTarget>,
}

impl<'d, C, const CAP: usize, const MAX_ENTRIES: usize> Stm32I2cTarget<'d, C, CAP, MAX_ENTRIES> {
    /// Binds the (already master-initialized) I2C driver to the target address,
    /// switching it into slave/multimaster mode with clock stretching.
    ///
    /// `addr7` is validated against the reserved I2C/I3C address space *before*
    /// any bus state changes (spec: *Address Validation for Manual
    /// Assignment*); the general-call address is always disabled. The
    /// peripheral instance `R` is only needed by the [`service_once`] idle
    /// probe ([`Stm32I2cTarget::service_once`]).
    pub fn new(
        i2c: I2c<'d, Blocking, Master>,
        addr7: u8,
        server: RegServer<C, CAP, MAX_ENTRIES>,
    ) -> Result<Self, Error> {
        validate_address(addr7)?;
        let i2c = i2c.into_slave_multimaster(SlaveAddrConfig::basic(addr7));
        Ok(Self {
            i2c,
            server,
            write_buf: [0u8; WRITE_BUF_LEN],
            read_buf: [0u8; READ_WINDOW],
            addr7,
            write_hook: None,
            write_hook_ctx: None,
            #[cfg(feature = "bareboot")]
            bootloader_target: None,
        })
    }

    /// The validated 7-bit target address the bus answers at.
    pub fn address(&self) -> u8 {
        self.addr7
    }

    /// Installs (or clears with `None`) a raw write-frame callback that is
    /// invoked before the register server interprets the frame.
    ///
    /// See [`WriteFrameHook`] for the calling-context contract.
    pub fn set_write_frame_hook(&mut self, hook: Option<WriteFrameHook>) {
        self.write_hook = hook;
    }

    /// Installs (or clears with `None`) a context-capable raw write-frame
    /// callback. It is invoked after [`WriteFrameHook`] and before the register
    /// server, with a mutable reference to the map handler context.
    ///
    /// See [`WriteFrameHookCtx`] for the calling-context contract.
    pub fn set_write_frame_hook_ctx(&mut self, hook: Option<WriteFrameHookCtx<C>>) {
        self.write_hook_ctx = hook;
    }

    /// Arms BareBoot handsfree entry on a one-byte `0xF0` write frame.
    ///
    /// When armed, [`Stm32I2cTarget::service_once`] checks every received write
    /// frame with `barefoc_boot::trigger_i2c` **before** the user hooks and
    /// before `TargetTransport::on_write`. On a match it writes the cross-reset
    /// cookie and resets (never returns).
    ///
    /// # Interrupt context required
    ///
    /// `service_once` must be driven from interrupt context (see the loud note
    /// in the module docs and section 6 of
    /// `bareBoot/docs/handsfree-app-updates.md`); otherwise the strobe can be
    /// missed while a motor task monopolizes the cooperative executor.
    #[cfg(feature = "bareboot")]
    pub fn enable_bootloader(&mut self, target: BootTarget) {
        self.bootloader_target = Some(target);
    }

    /// Disarms BareBoot handsfree entry previously enabled via
    /// [`Stm32I2cTarget::enable_bootloader`].
    #[cfg(feature = "bareboot")]
    pub fn disable_bootloader(&mut self) {
        self.bootloader_target = None;
    }

    /// Shared view of the register server (telemetry publishing etc.).
    pub fn server(&self) -> &RegServer<C, CAP, MAX_ENTRIES> {
        &self.server
    }

    /// Mutable view of the register server — only between bus events, from
    /// the single application task (design D4).
    pub fn server_mut(&mut self) -> &mut RegServer<C, CAP, MAX_ENTRIES> {
        &mut self.server
    }

    /// `true` when a master address match is currently pending: the bus is
    /// clock-stretched waiting for the event loop to respond.
    pub fn match_pending<R: I2cTargetRegs>(&self) -> bool {
        let _ = &self.i2c;
        R::target_regs().isr().read().addr()
    }

    /// Services exactly one bus event, blocking only while a transaction is
    /// actually on the wire (clock stretching covers the response latency).
    ///
    /// # Interrupt context (BareBoot strobe)
    ///
    /// If [`Stm32I2cTarget::enable_bootloader`] was called, this method MUST be
    /// driven from interrupt context (InterruptExecutor / I2C event ISR) so the
    /// one-byte `0xF0` strobe preempts a starved motor task. See the module
    /// docs and section 6 of `bareBoot/docs/handsfree-app-updates.md`.
    ///
    /// - No address match pending → [`ServiceOutcome::Idle`] immediately.
    /// - Write command → the received frame is applied via
    ///   [`TargetTransport::on_write`]; a server rejection is consumed on the
    ///   wire and surfaced via [`ServiceOutcome::WriteRejected`].
    /// - Read command → the response window is prefetched under the documented
    ///   policy and streamed via [`TargetTransport::on_read`], then
    ///   [`ServiceOutcome::ReadPhase`] reports the delivered count.
    ///
    /// Upstream Reference: `ref/embassy/embassy-stm32/src/i2c/v2.rs`
    /// (`blocking_listen`, `blocking_respond_to_write`,
    /// `blocking_respond_to_read`).
    pub fn service_once<R: I2cTargetRegs>(&mut self) -> Result<ServiceOutcome, Error> {
        // Idle probe: the ADDR flag is set while the master is matched and the
        // bus stretched; reading it is side-effect-free and does not clear it.
        if !R::target_regs().isr().read().addr() {
            return Ok(ServiceOutcome::Idle);
        }

        // ADDR is intentionally NOT cleared here: the respond_* call clears it
        // once the transfer is queued, which releases the clock stretch.
        let command = self.i2c.blocking_listen()?;

        match command.kind {
            SlaveCommandKind::Write => {
                let bytes = self.i2c.blocking_respond_to_write(&mut self.write_buf)?;
                let frame = &self.write_buf[..bytes];

                // BareBoot handsfree strobe. Checked BEFORE the user hooks and
                // before the register server so an in-band strobe is never
                // mistaken for a register-pointer latch. `enter_bareboot`
                // diverges (system reset), so no register state is touched on
                // a match. MUST be serviced in interrupt context (see module
                // docs / doc section 6).
                #[cfg(feature = "bareboot")]
                if let Some(target) = self.bootloader_target {
                    i2c_isr_strobe_hook(frame, target);
                }

                // Raw-frame observer (e.g. handsfree bootloader trigger). Runs in
                // the caller's context, before the register server would mistake
                // a one-byte command frame for a register-pointer latch. A hook
                // may reset the MCU and never return.
                let hook = self.write_hook;
                if let Some(hook) = hook {
                    hook(frame);
                }

                // Context-capable observer, after the plain hook and before
                // the register server interprets the pointer byte.
                let hook_ctx = self.write_hook_ctx;
                if let Some(hook_ctx) = hook_ctx {
                    let ctx = self.server.map_mut().handler();
                    hook_ctx(frame, ctx);
                }

                match self.server.on_write(frame) {
                    Ok(()) => Ok(ServiceOutcome::WritePhase { bytes }),
                    // Rejected phase: consumed, error kept visible to the app.
                    Err(error) => {
                        log_error!("i2c target: server rejected write phase (bytes={})", bytes);
                        Ok(ServiceOutcome::WriteRejected { bytes, error })
                    }
                }
            }
            SlaveCommandKind::Read => {
                // Prefill policy (module docs): bounded by the map extent from
                // the latched pointer, so on_read cannot fail — BufferTooSmall
                // is structurally impossible and out-of-extent requests are
                // window-clamped to 0 (the 0xFF filler then covers the wire).
                let ptr = usize::from(self.server.map().pointer());
                let extent = self.server.map().extent();
                let requested = extent.saturating_sub(ptr).min(self.read_buf.len());
                let _delivered = self
                    .server
                    .on_read(requested, &mut self.read_buf[..requested])
                    .unwrap_or(0);

                // Disjoint field borrows: tx buffer only passed by reference.
                let status = self
                    .i2c
                    .blocking_respond_to_read(&self.read_buf[..requested])?;

                let delivered = match status {
                    i2c::SendStatus::Done => requested,
                    i2c::SendStatus::LeftoverBytes(leftover) => {
                        // Short master: reposition the register pointer so the
                        // *next* read transaction continues the byte stream
                        // exactly where this one ended (pointer-only phase).
                        if leftover > 0 {
                            let next = ptr + requested.saturating_sub(leftover);
                            if next < extent {
                                let _ = self.server.on_write(&[next as u8]);
                            }
                        }
                        requested.saturating_sub(leftover)
                    }
                };

                Ok(ServiceOutcome::ReadPhase { requested, delivered })
            }
        }
    }

    /// Convenience blocking loop for the simplest apps: drives
    /// [`service_once`] forever, logging (when `defmt` is enabled) per-event
    /// bus faults. For motor-duty integration prefer the granular
    /// [`service_once`] inside the single application task loop instead.
    pub fn run_forever<R: I2cTargetRegs>(mut self) -> ! {
        loop {
            if self.service_once::<R>().is_err() {
                // Bus-level fault: state already reset by the embassy driver;
                // log and keep listening (a wedged bus needs a re-init task).
                log_error!("i2c target: bus-level fault in service loop");
            }
        }
    }
}

/// Rejects reserved I2C/I3C addresses at bind time (spec: *Address Validation
/// for Manual Assignment*):
///
/// - `0x00` — general call (also always disabled on the wire),
/// - `0x01`–`0x07` — CBUS / reserved low addresses,
/// - `0x78`–`0x7B` — I3C fixed/legacy block (10-bit header range),
/// - `0x7C`–`0x7D` — I3C device-ID range,
/// - `0x7E` / `0x7F` — I3C broadcast / controller-start addresses.
pub(crate) fn validate_address(addr7: u8) -> Result<(), Error> {
    match addr7 {
        0x00 | 0x01..=0x07 | 0x78..=0x7B | 0x7C..=0x7D | 0x7E..=0x7F => Err(Error::ReservedAddress),
        _ => Ok(()),
    }
}

/// Peripheral ISR strobe hook for I2C targets.
///
/// Evaluates an incoming write frame immediately in ISR context. If the frame is
/// a single-byte `0xF0` or a write of `0xF0` to `RebootCommand` (`0x63` / `0x0063` / `0x0123`),
/// this invokes `enter_bareboot` directly, preempting cooperative application tasks.
#[cfg(feature = "bareboot")]
pub fn i2c_isr_strobe_hook(frame: &[u8], target: BootTarget) -> bool {
    if barefoc::boot::trigger_i2c(frame)
        || (frame.len() >= 2 && frame[0] == 0x63 && frame[1] == barefoc::boot::I2C_CMD_ENTER_BOOTLOADER)
        || (frame.len() >= 3 && frame[0] == 0x00 && frame[1] == 0x63 && frame[2] == barefoc::boot::I2C_CMD_ENTER_BOOTLOADER)
        || (frame.len() >= 3 && frame[0] == 0x01 && frame[1] == 0x23 && frame[2] == barefoc::boot::I2C_CMD_ENTER_BOOTLOADER)
    {
        crate::bootloader::enter_bareboot(target);
    }
    false
}

/// Peripheral ISR strobe hook for I3C CCC and data frames.
#[cfg(feature = "bareboot")]
pub fn i3c_isr_strobe_hook(byte_or_ccc: u8, defining: Option<u8>, target: BootTarget) -> bool {
    let triggered = match defining {
        Some(def) => barefoc::boot::trigger_i3c_ccc(byte_or_ccc, def),
        None => barefoc::boot::trigger_i3c_byte(byte_or_ccc),
    };
    if triggered {
        crate::bootloader::enter_bareboot(target);
    }
    false
}
