//! # barefoc-ch32
//!
//! Official CH32 (WCH RISC-V) hardware abstraction bridges for
//! [`barefoc`](https://crates.io/crates/barefoc), built on
//! [`ch32-hal`](https://github.com/ch32-rs/ch32-hal).
//!
//! All hardware modules are gated behind the `ch32-hal` feature. The crate
//! compiles on the host without that feature, enabling `cargo test`.

#![cfg_attr(not(test), no_std)]

pub mod adc;
pub mod clock;
#[cfg(feature = "bareboot")]
pub mod bootloader;
#[cfg(feature = "ch32-hal")]
pub mod driver_6pwm;

#[cfg(feature = "ch32-hal")]
pub use adc::Ch32AdcPin;
#[cfg(feature = "bareboot")]
pub use bootloader::{
    before_reset_hook, clear_before_reset_hook, clear_boot_cookie, enter_bareboot,
    request_bootloader_checked, target_from_sram_top, NotPresent,
};
#[cfg(feature = "embassy-time")]
pub use clock::Ch32Clock;
#[cfg(feature = "ch32-hal")]
pub use driver_6pwm::Ch32BldcDriver6Pwm;

// Always-available helpers.
pub mod util;
pub use util::{raw_to_voltage, voltage_to_current};