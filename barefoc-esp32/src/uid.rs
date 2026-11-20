//! # ESP32 Hardware Unique Identifier
//!
//! Provides access to the 48-bit base MAC address (eFuse) on ESP32 microcontrollers.

use barefoc::traits::uid::{HardwareUid, MAX_UID_BYTES};
#[cfg(test)]
use barefoc::traits::uid::CANONICAL_HEX_LEN;

/// ESP32 hardware base MAC address unique identifier (48-bit / 6 bytes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Esp32Uid {
    bytes: [u8; 6],
}

impl Esp32Uid {
    /// Read the unique ID from hardware eFuse base MAC address.
    pub fn read() -> Self {
        #[cfg(any(
            feature = "esp32",
            feature = "esp32s3",
            feature = "esp32c3",
            feature = "esp32c6"
        ))]
        {
            let mac = esp_hal::efuse::base_mac_address();
            let b = mac.as_bytes();
            let mut bytes = [0u8; 6];
            bytes.copy_from_slice(b);
            Self { bytes }
        }
        #[cfg(not(any(
            feature = "esp32",
            feature = "esp32s3",
            feature = "esp32c3",
            feature = "esp32c6"
        )))]
        {
            Self { bytes: [0u8; 6] }
        }
    }

    /// Create an `Esp32Uid` instance initialized by reading hardware eFuse.
    pub fn new() -> Self {
        Self::read()
    }

    /// Create an `Esp32Uid` from explicit 6-byte MAC address.
    pub const fn from_mac(bytes: [u8; 6]) -> Self {
        Self { bytes }
    }

    /// Return the raw 6-byte MAC address.
    pub const fn mac_bytes(&self) -> [u8; 6] {
        self.bytes
    }
}

impl Default for Esp32Uid {
    fn default() -> Self {
        Self::read()
    }
}

impl HardwareUid for Esp32Uid {
    fn uid_bytes(&self) -> [u8; MAX_UID_BYTES] {
        [
            self.bytes[0],
            self.bytes[1],
            self.bytes[2],
            self.bytes[3],
            self.bytes[4],
            self.bytes[5],
            0,
            0,
            0,
            0,
            0,
            0,
        ]
    }

    fn uid_len(&self) -> usize {
        6
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_esp32_uid_formatting_and_zero_padding() {
        let mac = [0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33];
        let uid = Esp32Uid::from_mac(mac);
        assert_eq!(uid.uid_len(), 6);
        assert_eq!(uid.mac_bytes(), mac);

        let padded = uid.uid_bytes();
        assert_eq!(&padded[..6], &mac);
        assert_eq!(&padded[6..12], &[0, 0, 0, 0, 0, 0]);

        let mut out = [0u8; CANONICAL_HEX_LEN];
        let n = uid.format_hex(&mut out);
        assert_eq!(n, 12);
        assert_eq!(&out[..n], b"AABBCC112233");
    }
}
