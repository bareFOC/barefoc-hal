//! STM32 SPI **target** (slave) adapter for the `barefoc-multibus`
//! register-pointer server core.
//!
//! Binds the upstream embassy `spi` slave configuration surface
//! (`ref/embassy/embassy-stm32/src/spi/mod.rs`: slave register setup, hardware
//! NSS input semantics) plus the metapac `pac::spi::Spi` register block to the
//! bus-agnostic [`barefoc_multibus::transport::TargetTransport`] event surface
//! of the [`barefoc_multibus::transport::RegServer`], so an STM32H5 acts as a
//! register-map server on the wire. The duty loop stays in the **application's
//! single task** — the adapter is "service frames into the map", not an extra
//! executor task (design D4, mirrored from the I2C adapter
//! [`crate::bus_target_i2c`]).
//!
//! # Wire framing (TMC-style reply-in-next-CS-frame)
//!
//! Frames are variable-length chip-select windows matching the controller-side
//! [`barefoc_multibus::client::RegClientSpi`] wire format (one `SpiDevice`
//! transaction == one CS frame):
//!
//! ```text
//! write frame:  [CS↓ reg][CS↑ payload][CS↑]    payload clocked via MOSI,
//!                                             applied via TargetTransport
//!                                             on_write at frame close
//! read frame:   [CS↓ reg][CS↑]                (pointer-latch frame, 1 byte,
//!                                              ARMS the reply state machine)
//!               [CS↓ clock-in len bytes][CS↑] (reply window: MISO streams the
//!                                              prefetched register bytes; the
//!                                              host clocks dummy bytes)
//! ```
//!
//! After a reply window the register pointer is re-positioned to
//! `start + min(bytes clocked, bytes fetched)` so short *and* long masters both
//! resume at the exact byte boundary of the next burst (same policy as the I2C
//! read-prefill / `LeftoverBytes` repositioning in [`crate::bus_target_i2c`]).
//!
//! # MISO release discipline (hardware NSS, Ω configuration)
//!
//! The peripheral is configured with `SSM = 0`, `SSOE = 0`, `SSIOP = ActiveLow`
//! (`SPI_CFG2`), exactly the "hardware NSS input" configuration of the upstream
//! `enable_and_init` slave path: for a device set as slave, the NSS pin acts as
//! a classical NSS input — the slave is selected while NSS is low and, crucially,
//! **MISO is released (high-impedance) whenever NSS is high**, so multiple
//! targets (or a katapult bootloader target or any other MISO talker) can share
//! the bus wire without contention outside a window. The NSS pull and the
//! polarity are adapter configuration (only active-low is supported).
//!
//! # Mid-frame CS deassertion recovery
//!
//! A command window is *dropped*, not partially applied, when the master raises
//! CS before (or exactly when) the frame has not yet been closed: bytes are
//! accumulated whole-frame and [`TargetTransport::on_write`] fires only after
//! CS rises. A truncated one-byte window degrades to a pointer latch (which is
//! precisely what a `RegClient::read_register` first frame is), never touching
//! register data. Stale RXFIFO bytes and underrun/overrun fault flags from an
//! aborted window are flushed and cleared at each window start.
//!
//! # Documented limitations
//!
//! - `SpiDevice` implementations **must keep one transaction == one CS
//!   assertion** (upstream `embedded-hal` contract); `SpiDevice` ZCA/NSS
//!   re-toggling inside one transaction would split a command frame.
//! - An **empty-payload write** (`write_register(reg, &[])`, a 1-byte frame) is
//!   wire-wise indistinguishable from a pointer-latch frame and therefore *arms*
//!   a reply window (a subsequent window would stream a reply). Hosts must use
//!   the pointer-latch frames for reads only.
//! - A reply window longer than the prefetchable map extent streams `0xFF`
//!   filler beyond the extent (same convention as the I2C adapter's overflow);
//!   the host must issue a fresh pointer latch afterwards.
//! - The blocking streaming loop parks (busy-polls) while a CS window is open —
//!   the single application task serves it; for motor-duty integration keep
//!   windows short by closing them on the host side promptly.
//!
//! Upstream Reference: `drivers/barefoc-multibus/src/client.rs`
//! (`RegClientSpi`, reply-in-next-CS-frame), `ref/Miniata/firmware/src/comm_i2c.cpp`
//! (register-pointer idiom), `ref/embassy/embassy-stm32/src/spi/mod.rs`
//! (register init mirroring `enable_and_init`, hardware NSS semantics).

use barefoc_multibus::table::Error as MapError;
use barefoc_multibus::transport::{RegServer, TargetTransport};
use embassy_stm32::gpio::{AfType, Flex, OutputType, Pull, Speed};
use embassy_stm32::pac;
use embassy_stm32::pac::spi::{Spi as SpiRegs, vals};
use embassy_stm32::spi::{
    CsPin, Instance, MisoPin, Mode, MosiPin, Polarity, Phase, SlaveSelectPolarity, SckPin,
};
use embassy_stm32::Peri;


/// DEFMT action: compile to `no-op` without the defmt feature.
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

/// Maximum command-frame length staged by the adapter (pointer byte + payload).
const MAX_CMD_FRAME: usize = 64;
/// Default read-prefetch window in bytes.
pub const READ_WINDOW: usize = 32;
/// TX staging queue size. The per-window prefetch (`read_window`) must fit.
const TX_QUEUE_LEN: usize = 64;

/// Adapter-level error.
///
/// Register-map (server-level) errors are *not* carried here: they are
/// surfaced per event through [`ServiceOutcome`] (write-frame rejections in
/// [`ServiceOutcome::WriteFrame`]; read-phase errors cannot occur under the
/// bounded prefetch policy).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// Active-high NSS was requested. The adapter's documented register
    /// configuration (hardware NSS input, `SSIOP = ActiveLow`) supports only
    /// the active-low chip-select convention currently used by every
    /// `embedded-hal` `SpiDevice` host.
    UnsupportedPolarity,
    /// `read_window` was `0` or exceeded [`TX_QUEUE_LEN`], so a prefetch cannot
    /// be staged into the adapter's bounded TX queue.
    InvalidWindow,
}

/// One SPI chip-select window serviced by [`Stm32SpiTarget::service_once`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServiceOutcome {
    /// No master window was open (NSS deasserted) at poll time.
    Idle,
    /// A command frame was received and closed. `applied` is the server-side
    /// phase result: `Ok(())` for an applied pointer latch/payload (a 1-byte
    /// applied frame additionally **arms** the next reply window), `Err(_)`
    /// for a server rejection (`MapError::Unmapped`, `MapError::ReadOnly`) —
    /// the frame is dropped and no register byte moved.
    WriteFrame {
        /// Byte count received in the (closed) CS frame.
        bytes: usize,
        /// Server-side phase result.
        applied: Result<(), MapError>,
    },
    /// A reply window was streamed (armed pointer-latch frame followed by a
    /// burst window). `served` counts the bytes sourced from the register map,
    /// `clocked` the total bytes the master clocked in (including any `0xFF`
    /// filler beyond the prefetch window).
    ReadFrame {
        /// Bytes actually clocked by the master.
        clocked: usize,
        /// Bytes served from the register map (rest was `0xFF` filler).
        served: usize,
    },
}

/// Access to the PAC register block behind an embassy SPI instance so the
/// adapter can own the target (slave) register configuration *directly on the
/// register surface*.
///
/// Embassy's blocking `Spi<'d, Blocking, Slave>` driver (upstream
/// `new_blocking_slave`) only exposes a constructor — its transfer primitives
/// exist solely on the master/shared path, where a `blocking_transfer` parks
/// forever on an early CS deassertion. A CS-window state machine cannot be
/// built on that surface, so the adapter drives the metapac registers the
/// embassy driver itself configures (H503 profile: `spi_v5_i2s`).
///
/// The mapping is declared per concrete instance (H5 profile:
/// SPI1 / SPI2 / SPI3) including the RCC enable/reset plumbing, which embassy
/// keeps crate-private (`RccInfo::enable_and_reset`). All RCC bus/clock fields
/// are verified against the generated chip metadata
/// (`stm32-data-generated`, STM32H503KB: SPI1 → APB2ENR/APB2RSTR,
/// SPI2/SPI3 → APB1LENR/APB1LRSTR).
pub trait SpiTargetRegs: Instance {
    /// PAC register block for this instance.
    fn target_regs() -> SpiRegs;

    /// Enables the peripheral clock and pulses the reset line.
    ///
    /// # Safety
    ///
    /// Callers must own the exclusive access to this SPI peripheral instance
    /// (the constructor consumes the `Peri<'d, T>` token, which guarantees
    /// that). The method itself only writes RCC enable/reset bits — no shared
    /// register aliasing is possible.
    unsafe fn enable_and_reset();
}

macro_rules! impl_spi_target_regs {
    ($peri:ident, $bus_en:ident, $bus_rst:ident, $en:ident, $rst:ident) => {
        impl SpiTargetRegs for embassy_stm32::peripherals::$peri {
            #[inline]
            fn target_regs() -> SpiRegs {
                // Same register block the embassy driver itself drives; flag
                // reads are side-effect-free (RXFIFO drains excluded).
                pac::$peri
            }

            #[inline]
            unsafe fn enable_and_reset() {
                // SAFETY: documented on the trait method; single-owner token
                // guarantee applies to the RCC bus-port writes too.
                let rcc = pac::RCC;
                rcc.$bus_en().modify(|w| w.$en(true));
                rcc.$bus_rst().modify(|w| w.$rst(true));
                rcc.$bus_rst().modify(|w| w.$rst(false));
            }
        }
    };
}

impl_spi_target_regs!(SPI1, apb2enr, apb2rstr, set_spi1en, set_spi1rst);
impl_spi_target_regs!(SPI2, apb1lenr, apb1lrstr, set_spi2en, set_spi2rst);
impl_spi_target_regs!(SPI3, apb1lenr, apb1lrstr, set_spi3en, set_spi3rst);

/// Configuration for [`Stm32SpiTarget`].
#[derive(Clone, Copy)]
pub struct SpiTargetConfig {
    /// SPI mode (clock polarity/phase). Mode 0 (`MODE_0`) is the protocol
    /// profile of the controller-side client; modes 1–3 are accepted but must
    /// be matched by the host.
    pub mode: Mode,
    /// NSS (chip-select) polarity. Only [`SlaveSelectPolarity::ActiveLow`] is
    /// supported ([`Error::UnsupportedPolarity`]).
    pub nss_polarity: SlaveSelectPolarity,
    /// Pull on the hardware NSS input pin while idle.
    pub nss_pull: Pull,
    /// Pull on the MOSI input pin.
    pub input_pull: Pull,
    /// Read-prefetch window in bytes (`1..=TX_QUEUE_LEN`). Also the byte count
    /// fetched per per-window refill from the register map.
    pub read_window: usize,
}

impl Default for SpiTargetConfig {
    fn default() -> Self {
        Self {
            mode: embassy_stm32::spi::MODE_0,
            nss_polarity: SlaveSelectPolarity::ActiveLow,
            nss_pull: Pull::None,
            input_pull: Pull::None,
            read_window: READ_WINDOW,
        }
    }
}

/// Blocking-first STM32H5 SPI target adapter around the register-pointer
/// server.
///
/// Owns the four target pins as `Flex` handles (SCK/MOSI/MISO/NSS, configured
/// in AF mode mirroring the upstream `new_blocking_slave` pin plan — including
/// the polarity-matched SCK pull) and a [`RegServer`] by value; the application
/// task interleaves duty-cycle work with [`Stm32SpiTarget::service_once`]
/// calls and publishes telemetry between them via
/// [`Stm32SpiTarget::server_mut`] (design D4: the table is only touched from
/// the single application task).
///
/// Upstream Reference: `ref/embassy/embassy-stm32/src/spi/mod.rs`
/// (`enable_and_init`, `new_blocking_slave` pin plan),
/// `drivers/barefoc-multibus/src/client.rs` (`RegClientSpi` CS framing).
pub struct Stm32SpiTarget<'d, C, const CAP: usize, const MAX_ENTRIES: usize> {
    // The AF-configured pins are owned for the whole lifetime so they stay
    // designated to this peripheral.
    _sck: Flex<'d>,
    _mosi: Flex<'d>,
    _miso: Flex<'d>,
    nss: Flex<'d>,
    server: RegServer<C, CAP, MAX_ENTRIES>,
    cfg: SpiTargetConfig,
    /// Bounded TX staging: prefetched register bytes streamed as MISO.
    tx_queue: [u8; TX_QUEUE_LEN],
    /// Index of the next queue byte to stream.
    tx_head: usize,
    /// Remaining streamed count in the queue (after `tx_head`).
    tx_qlen: usize,
    /// Total bytes fetched from the map for the current reply window.
    tx_taken_total: usize,
    /// Map pointer latched before the current reply window's first refill.
    tx_src_ptr: u8,
    /// `true` once a pointer-latch frame (1 command byte, applied) ended a
    /// window: the *next* CS window is serviced as a reply burst.
    armed: bool,
}

impl<'d, C, const CAP: usize, const MAX_ENTRIES: usize> Stm32SpiTarget<'d, C, CAP, MAX_ENTRIES> {
    /// Configures the SPI peripheral as an SPI target (slave) around the
    /// register-pointer server, using **hardware NSS** (`SSM = 0`, `SSOE = 0`,
    /// [`SlaveSelectPolarity::ActiveLow`]) so MISO is released whenever the
    /// chip-select is deasserted.
    ///
    /// Validated against the *chip-select polarity* and the *prefetch window*
    /// bounds **before** any bus state is changed; on error the pins and the
    /// peripheral stay untouched.
    #[allow(clippy::too_many_arguments)]
    pub fn new<T: SpiTargetRegs>(
        _peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        mosi: Peri<'d, impl MosiPin<T>>,
        miso: Peri<'d, impl MisoPin<T>>,
        cs: Peri<'d, impl CsPin<T>>,
        cfg: SpiTargetConfig,
        server: RegServer<C, CAP, MAX_ENTRIES>,
    ) -> Result<Self, Error> {
        if cfg.nss_polarity != SlaveSelectPolarity::ActiveLow {
            return Err(Error::UnsupportedPolarity);
        }
        if cfg.read_window == 0 || cfg.read_window > TX_QUEUE_LEN {
            return Err(Error::InvalidWindow);
        }

        // Pin plan mirrors the upstream `new_blocking_slave` constructor:
        // SCK = AF push-pull input with a polarity-matched weak pull (IdleLow →
        // Pull::Down, IdleHigh → Pull::Up), MISO = AF push-pull output, MOSI =
        // AF input, NSS = AF input (the hardware slave-select gate).
        let (cpha, cpol, sck_pull) = match cfg.mode.polarity {
            Polarity::IdleHigh => {
                let cpha = match cfg.mode.phase {
                    Phase::CaptureOnSecondTransition => vals::Cpha::SecondEdge,
                    Phase::CaptureOnFirstTransition => vals::Cpha::FirstEdge,
                };
                (cpha, vals::Cpol::IdleHigh, Pull::Up)
            }
            Polarity::IdleLow => {
                let cpha = match cfg.mode.phase {
                    Phase::CaptureOnSecondTransition => vals::Cpha::SecondEdge,
                    Phase::CaptureOnFirstTransition => vals::Cpha::FirstEdge,
                };
                (cpha, vals::Cpol::IdleLow, Pull::Down)
            }
        };

        let sck_af = sck.af_num();
        let mut sck_flex = Flex::new(sck);
        // Same AF type the master path derives from `Config::sck_af()`.
        sck_flex.set_as_af_unchecked(
            sck_af,
            AfType::output_pull(OutputType::PushPull, Speed::VeryHigh, sck_pull),
        );

        let mosi_af = mosi.af_num();
        let mut mosi_flex = Flex::new(mosi);
        mosi_flex.set_as_af_unchecked(mosi_af, AfType::input(cfg.input_pull));

        let miso_af = miso.af_num();
        let mut miso_flex = Flex::new(miso);
        miso_flex.set_as_af_unchecked(
            miso_af,
            AfType::output(OutputType::PushPull, Speed::VeryHigh),
        );

        let cs_af = cs.af_num();
        let mut nss_flex = Flex::new(cs);
        nss_flex.set_as_af_unchecked(cs_af, AfType::input(cfg.nss_pull));

        // SAFETY: the constructor exclusively owns the peripheral token, so no
        // concurrent RCC or SPI register access can alias this one.
        unsafe { <T as SpiTargetRegs>::enable_and_reset() };
        init_target_registers(T::target_regs(), cpha, cpol);

        Ok(Self {
            _sck: sck_flex,
            _mosi: mosi_flex,
            _miso: miso_flex,
            nss: nss_flex,
            server,
            cfg,
            tx_queue: [0u8; TX_QUEUE_LEN],
            tx_head: 0,
            tx_qlen: 0,
            tx_taken_total: 0,
            tx_src_ptr: 0,
            armed: false,
        })
    }

    /// Shared view of the register server (telemetry publishing etc.).
    pub fn server(&self) -> &RegServer<C, CAP, MAX_ENTRIES> {
        &self.server
    }

    /// Mutable view of the register server — only between CS windows, from the
    /// single application task (design D4).
    pub fn server_mut(&mut self) -> &mut RegServer<C, CAP, MAX_ENTRIES> {
        &mut self.server
    }

    /// `true` while the master holds the chip-select active.
    fn selected(&self) -> bool {
        if self.nss.is_low() {
            self.cfg.nss_polarity == SlaveSelectPolarity::ActiveLow
        } else {
            false
        }
    }

    /// Services exactly one CS window, blocking only while the window is open
    /// (NSS active). Blocks with no window open only fractionally — a poll on
    /// an idle bus returns [`ServiceOutcome::Idle`] immediately.
    ///
    /// - One-byte applied frame → the window latches the register pointer and
    ///   **arms** the reply state machine; the next window streams bytes.
    /// - Longer command frame → applied as one server write phase (dropped
    ///   whole whenever the server rejects it).
    /// - Armed window → reply burst: MISO streams the prefetched map bytes,
    ///   repositioning the register pointer to the byte the master last
    ///   clocked when the window closes.
    ///
    /// Upstream Reference: `ref/embassy/embassy-stm32/src/spi/mod.rs`
    /// (slave hardware-NSS semantics), `drivers/barefoc-multibus/src/client.rs`
    /// (`RegClientSpi::read_register` reply-in-next-CS-frame).
    pub fn service_once<R: SpiTargetRegs>(&mut self) -> Result<ServiceOutcome, Error> {
        if !self.selected() {
            return Ok(ServiceOutcome::Idle);
        }

        // SAFETY-consumer note: only register *reads* and FIFO/flag registers
        // are touched below; the peripheral token was exclusively owned at
        // construction, so concurrent access cannot race the window.
        let regs = R::target_regs();
        let outcome = if self.armed {
            self.service_reply_window(regs)
        } else {
            self.service_command_window(regs)
        };

        // Window-state hygiene: no partial state may leak into the next window.
        // Freshly *armed* command windows (pointer latches) must keep their
        // prefetch across this boundary — only an already-drained reply window
        // or an unarmed command window resets here.
        if !self.armed {
            self.tx_head = 0;
            self.tx_qlen = 0;
            self.tx_taken_total = 0;
            self.tx_src_ptr = 0;
        }

        Ok(outcome)
    }

    /// Services a command (write / pointer-latch) CS window until the master
    /// raises CS, then applies the whole frame to the server.
    ///
    /// MISO is primed with `0xFF` throughout so a transmit underflow never
    /// registers; the master ignores MISO in a write frame on the wire.
    fn service_command_window(&mut self, regs: SpiRegs) -> ServiceOutcome {
        begin_window(regs);

        let mut frame = [0u8; MAX_CMD_FRAME];
        let mut frame_len = 0usize;
        while self.selected() {
            if regs.sr().read().txp() {
                // Prime the transmitter: underrun filler bytes ignored by the
                // master's write-phase clocking.
                regs.txdr8().write_value(0xFF);
            }
            if regs.sr().read().rxp() {
                let byte = regs.rxdr8().read();
                if frame_len < frame.len() {
                    frame[frame_len] = byte;
                    frame_len += 1;
                }
            }
        }
        // Late bytes clocked between the final poll and CS rising are still
        // part of this frame.
        while regs.sr().read().rxp() {
            let byte = regs.rxdr8().read();
            if frame_len < frame.len() {
                frame[frame_len] = byte;
                frame_len += 1;
            }
        }
        end_window(regs);

        if frame_len == 0 {
            // Mid-frame immediate abort — nothing received, nothing applied.
            return ServiceOutcome::Idle;
        }

        // Whole-frame application keeps the mid-frame aborts atomic: a
        // rejected frame leaves all registers untouched (whole-phase
        // validation in the map core).
        let applied = self.server.on_write(&frame[..frame_len]);
        if let Err(map_error) = applied {
            log_error!("spi target: server rejected command frame (bytes={})", frame_len);
            return ServiceOutcome::WriteFrame { bytes: frame_len, applied: Err(map_error) };
        }
        if frame_len == 1 {
            // Pointer-latch frame: arm the reply state machine and prefetch
            // the response bytes now, so the next window streams without the
            // server having to respond mid-burst.
            self.armed = true;
            self.arm_tx_queue();
        }
        ServiceOutcome::WriteFrame {
            bytes: frame_len,
            applied: Ok(()),
        }
    }

    /// Services a reply (read-burst) CS window until the master raises CS:
    /// streams the prefetched register map bytes on MISO (re-filling the queue
    /// from the advancing map pointer as it drains, `0xFF` filler once the
    /// map extent is exhausted) and discards the master's dummy write bytes.
    fn service_reply_window(&mut self, regs: SpiRegs) -> ServiceOutcome {
        begin_window(regs);

        let mut clocked = 0usize;
        let mut served = 0usize;
        while self.selected() {
            if regs.sr().read().txp() {
                let byte = if self.tx_qlen > 0 {
                    served += 1;
                    self.take_tx_byte()
                } else {
                    0xFF
                };
                regs.txdr8().write_value(byte);
            }
            if regs.sr().read().rxp() {
                // Reply burst: the master's dummy write bytes are discarded.
                let _ = regs.rxdr8().read();
                clocked += 1;
            }
        }
        while regs.sr().read().rxp() {
            let _ = regs.rxdr8().read();
            clocked += 1;
        }
        end_window(regs);

        // Reposition the register pointer for what the master *actually*
        // clocked (speculation from over-eager refills is reverted): a short
        // master therefore resumes the burst at its own byte boundary, a long
        // master has consumed the prefetched bytes exactly.
        let next =
            usize::from(self.tx_src_ptr) + clocked.min(self.tx_taken_total);
        if next < self.server.map().extent() {
            // Pointer-only phase: latch, no register modification.
            let _ = self.server.on_write(&[next as u8]);
        }
        // The reply window is fully consumed: disarm so the next window is
        // again treated as a command frame (the queue state is cleared by
        // `service_once` once this returns).
        self.armed = false;

        ServiceOutcome::ReadFrame { clocked, served }
    }

    /// Arms the TX queue after a pointer-latch frame: latches the map pointer
    /// as the burst source and prefetches the first window of response bytes.
    fn arm_tx_queue(&mut self) {
        self.tx_src_ptr = self.server.map().pointer();
        self.tx_head = 0;
        self.tx_qlen = 0;
        self.tx_taken_total = 0;
        self.top_up_tx();
    }

    /// Refills the TX queue from the register map at the (auto-incremented)
    /// latched pointer, clamped by the configured window and the map extent so
    /// [`TargetTransport::on_read`] cannot fail (`Error::Unmapped` is
    /// structurally impossible; out-of-extent latches prefetch zero bytes).
    fn top_up_tx(&mut self) {
        self.tx_head = 0;
        self.tx_qlen = 0;
        let ptr = usize::from(self.server.map().pointer());
        let extent = self.server.map().extent();
        let want = extent
            .saturating_sub(ptr)
            .min(self.cfg.read_window);
        if want == 0 {
            return;
        }
        self.tx_taken_total = self.tx_taken_total.saturating_add(want);
        // Disjoint-field borrows: the queue is a plain owned slice of `self`.
        if self.server.on_read(want, &mut self.tx_queue[..want]).is_err() {
            self.tx_queue[..want].fill(0xFF);
            log_error!("spi target: read fetch failed, queue filled with filler");
        }
        self.tx_qlen = want;
    }

    /// Streams the next queue byte, refilling from the map when drained
    /// (seamless long bursts up to the map extent; `0xFF` filler afterwards,
    /// matched by the caller's re-positioning of the register pointer).
    fn take_tx_byte(&mut self) -> u8 {
        let byte = self.tx_queue[self.tx_head];
        self.tx_head += 1;
        self.tx_qlen -= 1;
        if self.tx_qlen == 0 {
            self.top_up_tx();
        }
        byte
    }

    /// Convenience blocking loop for the simplest apps: drives
    /// [`service_once`] forever, logging (when `defmt` is enabled) adapter
    /// configuration violations. Because [`Stm32SpiTarget::service_once`]
    /// parks while a CS window is open, this loop yields control back to the
    /// caller only between windows — for motor-duty integration prefer the
    /// granular [`service_once`] inside the single application task instead.
    pub fn run_forever<R: SpiTargetRegs>(mut self) -> ! {
        loop {
            if self.service_once::<R>().is_err() {
                // Adapter-level fault: window state was already reset by the
                // servicing path; log and keep polling (a wedged chip-select
                // stays reported until the fault clears).
                log_error!("spi target: adapter-level fault in service loop");
            }
        }
    }
}

/// Window-start hygiene: clears every pending interrupt/status flag so a
/// stale `OVR`/`UDR` fault condition from an aborted earlier window can
/// never be observed as a fresh fault.
///
/// The RXFIFO itself is deliberately **not** flushed here: the previous
/// window always drains to empty (both service paths drain after NSS rises),
/// so any FIFO content present at window start belongs to the *current*
/// frame (the adapter may be entered mid-window) and must be treated as
/// received data.
fn begin_window(regs: SpiRegs) {
    regs.ifcr().write(|w| w.0 = 0xffff_ffff);
}

/// Window-end hygiene: the flags cleared here (transmit-side state machines
/// carried over) keep the next window's read path clean.
fn end_window(regs: SpiRegs) {
    regs.ifcr().write(|w| w.0 = 0xffff_ffff);
}

/// Configures the metapac `spi_v5_i2s` register surface to the
/// hardware-NSS slave configuration mirrored from the upstream
/// `enable_and_init` `spi_v4+` slave branch.
fn init_target_registers(regs: SpiRegs, cpha: vals::Cpha, cpol: vals::Cpol) {
    regs.cfg2().modify(|w| {
        // Clock mode from the configured SPI timing (CPHA/CPOL), first edge
        // capture default communicated by `SpiTargetConfig::mode`.
        w.set_cpha(cpha);
        w.set_cpol(cpol);
        // Hardware NSS: the classic NSS input (NSS output disabled), exact
        // upstream comment semantics for SSM = 0 / SSOE = 0 on a slave.
        w.set_ssoe(false);
        w.set_ssm(false);
        w.set_ssiop(vals::Ssiop::ActiveLow);
        w.set_master(vals::Master::Slave);
        w.set_comm(vals::Comm::FullDuplex);
        w.set_ssom(vals::Ssom::Asserted);
        w.set_midi(0);
        w.set_mssi(0);
        // Hardware automatic function control disables the AF-control
        // feature-flag mismatch on the slave path (upstream branch).
        w.set_afcntr(true);
    });
    regs.cfg1().modify(|w| {
        w.set_crcen(false);
        // Baud rate is a master-side ratio: irrelevant in slave mode, set to
        // the slowest divisor for exhaustive mode-detection make-up.
        w.set_mbr(vals::Mbr::Div256);
        // 8-bit frames (DSIZE = bits − 1, upstream `SealedWord` for u8).
        w.set_dsize(7);
        w.set_fthlv(vals::Fthlv::OneFrame);
        // Fixed 0x00 underrun data pattern when the master outruns the
        // transmitter (documented overflow semantics mirror the I2C filler).
        w.set_udrcfg(vals::Udrcfg::Constant);
    });
    regs.cr2().modify(|w| {
        // No master-controlled transfer sizing on the slave path.
        w.set_tsize(0);
    });
    regs.cr1().modify(|w| {
        // SSI = false (hardware NSS input drives the internal select line);
        // enable the peripheral last, mirroring the upstream ordering.
        w.set_ssi(false);
        w.set_spe(true);
    });
}
