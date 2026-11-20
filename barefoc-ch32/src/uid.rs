//! # CH32 Hardware Unique Identifier
//!
//! Provides access to the 96-bit electronic signature (ESIG) unique ID on WCH CH32 RISC-V microcontrollers.

use barefoc::traits::uid::{HardwareUid, MAX_UID_BYTES};
#[cfg(test)]
use barefoc::traits::uid::CANONICAL_HEX_LEN;

/// CH32 ESIG base address in system memory.
pub const CH32_ESIG_UID_ADDR: usize = 0x1FFF_F7E8;

/// CH32 hardware electronic signature unique identifier (96-bit / 12 bytes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ch32Uid {
    bytes: [u8; 12],
}

impl Ch32Uid {
    /// Read the 96-bit electronic signature from CH32 system memory (`0x1FFF_F7E8`).
    pub fn read() -> Self {
        #[cfg(target_arch = "riscv32")]
        {
            let mut bytes = [0u8; 12];
            // SAFETY: 0x1FFF_F7E8 is the factory-programmed 96-bit electronic signature (ESIG)
            // UID address in system memory on CH32 microcontrollers. Reading these registers is
            // read-only and side-effect free.
            unsafe {
                let ptr = CH32_ESIG_UID_ADDR as *const u32;
                let w0 = core::ptr::read_volatile(ptr);
                let w1 = core::ptr::read_volatile(ptr.add(1));
                let w2 = core::ptr::read_volatile(ptr.add(2));
                bytes[0..4].copy_from_slice(&w0.to_le_bytes());
                bytes[4..8].copy_from_slice(&w1.to_le_bytes());
                bytes[8..12].copy_from_slice(&w2.to_le_bytes());
            }
            Self { bytes }
        }
        #[cfg(not(target_arch = "riscv32"))]
        {
            Self { bytes: [0u8; 12] }
        }
    }

    /// Create a `Ch32Uid` instance initialized by reading the hardware ESIG.
    pub fn new() -> Self {
        Self::read()
    }

    /// Create a `Ch32Uid` from explicit 12-byte raw silicon ID.
    pub const fn from_bytes(bytes: [u8; 12]) -> Self {
        Self { bytes }
    }
}

impl Default for Ch32Uid {
    fn default() -> Self {
        Self::read()
    }
}

impl HardwareUid for Ch32Uid {
    fn uid_bytes(&self) -> [u8; MAX_UID_BYTES] {
        self.bytes
    }

    fn uid_len(&self) -> usize {
        12
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ch32_uid_formatting() {
        let raw = [0xAA, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xBB, 0xCC];
        let uid = Ch32Uid::from_bytes(raw);
        assert_eq!(uid.uid_len(), 12);
        assert_eq!(uid.uid_bytes(), raw);

        let mut out = [0u8; CANONICAL_HEX_LEN];
        let n = uid.format_hex(&mut out);
        assert_eq!(n, 24);
        assert_eq!(&out, b"AA112233445566778899BBCC");
    }
}
