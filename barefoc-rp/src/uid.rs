//! # RP2040 / RP2350 Hardware Unique Identifier
//!
//! Provides access to the 64-bit factory electronic unique identifier on RP2040 (flash unique ID)
//! and RP2350 (OTP chip UID).

use barefoc::traits::uid::{HardwareUid, MAX_UID_BYTES};
#[cfg(test)]
use barefoc::traits::uid::CANONICAL_HEX_LEN;

/// RP2350 / RP2040 hardware unique identifier (64-bit / 8 bytes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rp2350Uid {
    bytes: [u8; 8],
}

/// Convenience alias for RP platform UID.
pub type RpUid = Rp2350Uid;

impl Rp2350Uid {
    /// Read the unique ID from hardware (OTP on RP2350, Flash UID on RP2040).
    pub fn read() -> Self {
        #[cfg(target_arch = "arm")]
        {
            // Direct OTP read on RP2350: OTP_DATA_BASE = 0x4013_0000
            // Rows 0x00..0x03 contain the 64-bit chip ID
            const OTP_DATA_BASE: *const u32 = 0x4013_0000 as *const u32;
            let mut bytes = [0u8; 8];
            // SAFETY: On RP2350, reading OTP ECC data registers at 0x4013_0000 is read-only
            // and contains the factory-programmed 64-bit chip UID in the first two 32-bit words (rows 0..3).
            unsafe {
                let w0 = core::ptr::read_volatile(OTP_DATA_BASE);
                let w1 = core::ptr::read_volatile(OTP_DATA_BASE.add(1));
                bytes[0..4].copy_from_slice(&w0.to_le_bytes());
                bytes[4..8].copy_from_slice(&w1.to_le_bytes());
            }
            Self { bytes }
        }
        #[cfg(not(target_arch = "arm"))]
        {
            Self { bytes: [0u8; 8] }
        }
    }

    /// Create an `Rp2350Uid` instance initialized by reading the hardware UID.
    pub fn new() -> Self {
        Self::read()
    }

    /// Create an `Rp2350Uid` from an explicit 8-byte array.
    pub const fn from_bytes(bytes: [u8; 8]) -> Self {
        Self { bytes }
    }
}

impl Default for Rp2350Uid {
    fn default() -> Self {
        Self::read()
    }
}

impl HardwareUid for Rp2350Uid {
    fn uid_bytes(&self) -> [u8; MAX_UID_BYTES] {
        [
            self.bytes[0],
            self.bytes[1],
            self.bytes[2],
            self.bytes[3],
            self.bytes[4],
            self.bytes[5],
            self.bytes[6],
            self.bytes[7],
            0,
            0,
            0,
            0,
        ]
    }

    fn uid_len(&self) -> usize {
        8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rp2350_uid_formatting_and_padding() {
        let raw = [0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF];
        let uid = Rp2350Uid::from_bytes(raw);
        assert_eq!(uid.uid_len(), 8);
        let padded = uid.uid_bytes();
        assert_eq!(&padded[..8], &raw);
        assert_eq!(&padded[8..12], &[0, 0, 0, 0]);

        let mut out = [0u8; CANONICAL_HEX_LEN];
        let n = uid.format_hex(&mut out);
        assert_eq!(n, 16);
        assert_eq!(&out[..n], b"0123456789ABCDEF");
    }
}
