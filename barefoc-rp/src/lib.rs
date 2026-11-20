//! # barefoc-rp
//!
//! Official Raspberry Pi RP2040 and RP2350 hardware abstraction bridges for
//! [`barefoc`](https://crates.io/crates/barefoc).

#![cfg_attr(not(test), no_std)]

pub mod uid;
pub use uid::{Rp2350Uid, RpUid};
