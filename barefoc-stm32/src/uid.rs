//! # STM32 Hardware Unique Identifier
//!
//! Provides access to the 96-bit factory electronic signature (UID) on STM32 microcontrollers.

use barefoc::traits::uid::{HardwareUid, MAX_UID_BYTES};
#[cfg(test)]
use barefoc::traits::uid::CANONICAL_HEX_LEN;

/// STM32 hardware electronic signature unique identifier (96-bit / 12 bytes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stm32Uid {
    bytes: [u8; 12],
}

impl Stm32Uid {
    /// Read the STM32 unique ID from the hardware.
    pub fn read() -> Self {
        #[cfg(feature = "embassy-stm32")]
        {
            let id = embassy_stm32::uid::uid();
            Self { bytes: *id }
        }
        #[cfg(not(feature = "embassy-stm32"))]
        {
            Self { bytes: [0u8; 12] }
        }
    }

    /// Create an `Stm32Uid` instance initialized by reading the hardware UID.
    pub fn new() -> Self {
        Self::read()
    }

    /// Create an `Stm32Uid` from explicit 12-byte raw silicon ID.
    pub const fn from_bytes(bytes: [u8; 12]) -> Self {
        Self { bytes }
    }
}

impl Default for Stm32Uid {
    fn default() -> Self {
        Self::read()
    }
}

impl HardwareUid for Stm32Uid {
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
    fn test_stm32_uid_formatting() {
        let raw = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC];
        let uid = Stm32Uid::from_bytes(raw);
        assert_eq!(uid.uid_len(), 12);
        assert_eq!(uid.uid_bytes(), raw);

        let mut out = [0u8; CANONICAL_HEX_LEN];
        let n = uid.format_hex(&mut out);
        assert_eq!(n, 24);
        assert_eq!(&out, b"112233445566778899AABBCC");
    }
}
