//! STM32H5 non-volatile Flash configuration persistence driver.
//!
//! Implements atomic dual-sector ping-pong storage using STM32H5 Bank 2 Sectors 14 & 15
//! (`0x0801_C000 .. 0x0802_0000`, 16 KiB total) with a 32-byte header, sequence counter,
//! and IEEE 802.3 CRC32 verification.
//!
//! Because Sectors 14 & 15 reside in Bank 2, erase and write operations can proceed
//! while firmware executes from Bank 1 without CPU stalling.

use barefoc::control_table::{error_flags, ControlTable, EEPROM_AREA_SIZE};
use barefoc::traits::storage::ConfigStorage;

/// Magic identifier for valid flash configuration headers ("FISH" in ASCII / hex).
pub const FLASH_MAGIC: u32 = 0x4649_5348;

/// Configuration schema version.
pub const FLASH_SCHEMA_VERSION: u16 = 1;

/// Byte offset of Bank 2 Sector 14 relative to `FLASH_BASE` (`0x0800_0000`).
pub const SECTOR_14_OFFSET: u32 = 0x0001_C000;

/// Byte offset of Bank 2 Sector 15 relative to `FLASH_BASE` (`0x0800_0000`).
pub const SECTOR_15_OFFSET: u32 = 0x0001_E000;

/// Size of each STM32H5 flash sector in bytes (8 KiB).
pub const SECTOR_SIZE: u32 = 8192;

/// Size of the flash persistence header in bytes.
pub const HEADER_SIZE: usize = 32;

/// Total stored record size (32-byte header + 96-byte EEPROM payload = 128 bytes).
pub const RECORD_SIZE: usize = HEADER_SIZE + EEPROM_AREA_SIZE;

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

/// 32-byte sector header structure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SectorHeader {
    pub magic: u32,
    pub version: u16,
    pub payload_len: u16,
    pub seq: u32,
    pub crc32: u32,
}

impl SectorHeader {
    /// Serialize header to 32 bytes (little-endian).
    pub fn to_bytes(&self) -> [u8; HEADER_SIZE] {
        let mut buf = [0u8; HEADER_SIZE];
        buf[0..4].copy_from_slice(&self.magic.to_le_bytes());
        buf[4..6].copy_from_slice(&self.version.to_le_bytes());
        buf[6..8].copy_from_slice(&self.payload_len.to_le_bytes());
        buf[8..12].copy_from_slice(&self.seq.to_le_bytes());
        buf[12..16].copy_from_slice(&self.crc32.to_le_bytes());
        // Remaining 16 bytes are reserved zeros
        buf
    }

    /// Parse header from a 32-byte slice.
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < HEADER_SIZE {
            return None;
        }
        let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let version = u16::from_le_bytes([buf[4], buf[5]]);
        let payload_len = u16::from_le_bytes([buf[6], buf[7]]);
        let seq = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        let crc32 = u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);

        Some(Self {
            magic,
            version,
            payload_len,
            seq,
            crc32,
        })
    }

    /// Check if this header matches the expected magic, schema version, and payload length.
    pub fn is_valid_structure(&self) -> bool {
        self.magic == FLASH_MAGIC
            && self.version == FLASH_SCHEMA_VERSION
            && self.payload_len == (EEPROM_AREA_SIZE as u16)
    }
}

/// Abstract flash storage backend trait to allow both Embassy Flash hardware driver
/// and host test memory buffers.
pub trait FlashBackend {
    /// Read `bytes` starting at flash `offset`.
    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), ()>;

    /// Erase flash region `[from, to)`.
    fn erase(&mut self, from: u32, to: u32) -> Result<(), ()>;

    /// Program `bytes` starting at flash `offset`.
    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), ()>;
}

#[cfg(feature = "embassy-stm32")]
impl<'d> FlashBackend for embassy_stm32::flash::Flash<'d, embassy_stm32::mode::Blocking> {
    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), ()> {
        self.blocking_read(offset, bytes).map_err(|_| ())
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), ()> {
        self.blocking_erase(from, to).map_err(|_| ())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), ()> {
        self.blocking_write(offset, bytes).map_err(|_| ())
    }
}

/// Errors returned by [`H5FlashStorage`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashStorageError {
    /// Read operation from Flash failed.
    ReadError,
    /// Erase operation on Flash sector failed.
    EraseError,
    /// Program / write operation to Flash failed.
    WriteError,
    /// Verification failed after programming.
    VerifyError,
}

/// STM32H5 Dual-Sector Ping-Pong Flash Storage Driver.
pub struct H5FlashStorage<B: FlashBackend> {
    backend: B,
    active_slot: Option<u8>, // 0 for Sector 14, 1 for Sector 15
    next_seq: u32,
}

impl<B: FlashBackend> H5FlashStorage<B> {
    /// Create a new flash storage instance wrapping `backend`.
    pub fn new(backend: B) -> Self {
        Self {
            backend,
            active_slot: None,
            next_seq: 1,
        }
    }

    /// Return the currently active ping-pong slot (0 = Sector 14, 1 = Sector 15).
    pub fn active_slot(&self) -> Option<u8> {
        self.active_slot
    }

    /// Return the next sequence counter to be assigned.
    pub fn next_seq(&self) -> u32 {
        self.next_seq
    }

    fn slot_offset(slot: u8) -> u32 {
        if slot == 0 {
            SECTOR_14_OFFSET
        } else {
            SECTOR_15_OFFSET
        }
    }

    /// Read and validate a slot record.
    fn inspect_slot(&mut self, slot: u8, payload_out: &mut [u8; EEPROM_AREA_SIZE]) -> Option<u32> {
        let offset = Self::slot_offset(slot);
        let mut record = [0u8; RECORD_SIZE];
        if self.backend.read(offset, &mut record).is_err() {
            return None;
        }

        let header = SectorHeader::from_bytes(&record[0..HEADER_SIZE])?;
        if !header.is_valid_structure() {
            return None;
        }

        let payload = &record[HEADER_SIZE..RECORD_SIZE];
        let computed_crc = crc32_ieee(payload);
        if computed_crc != header.crc32 {
            return None;
        }

        payload_out.copy_from_slice(payload);
        Some(header.seq)
    }
}

impl<B: FlashBackend> ConfigStorage for H5FlashStorage<B> {
    type Error = FlashStorageError;

    fn load_config(&mut self, table: &mut ControlTable) -> Result<(), Self::Error> {
        let mut payload_0 = [0u8; EEPROM_AREA_SIZE];
        let mut payload_1 = [0u8; EEPROM_AREA_SIZE];

        let valid_0 = self.inspect_slot(0, &mut payload_0);
        let valid_1 = self.inspect_slot(1, &mut payload_1);

        match (valid_0, valid_1) {
            (Some(seq_0), Some(seq_1)) => {
                if seq_0 >= seq_1 {
                    table.eeprom_bytes_mut().copy_from_slice(&payload_0);
                    self.active_slot = Some(0);
                    self.next_seq = seq_0.wrapping_add(1);
                } else {
                    table.eeprom_bytes_mut().copy_from_slice(&payload_1);
                    self.active_slot = Some(1);
                    self.next_seq = seq_1.wrapping_add(1);
                }
                table.clear_error_flag(error_flags::FLASH_CORRUPT);
                Ok(())
            }
            (Some(seq_0), None) => {
                table.eeprom_bytes_mut().copy_from_slice(&payload_0);
                self.active_slot = Some(0);
                self.next_seq = seq_0.wrapping_add(1);
                table.clear_error_flag(error_flags::FLASH_CORRUPT);
                Ok(())
            }
            (None, Some(seq_1)) => {
                table.eeprom_bytes_mut().copy_from_slice(&payload_1);
                self.active_slot = Some(1);
                self.next_seq = seq_1.wrapping_add(1);
                table.clear_error_flag(error_flags::FLASH_CORRUPT);
                Ok(())
            }
            (None, None) => {
                // Both slots invalid or corrupted; restore defaults and flag error
                table.load_factory_defaults();
                table.set_error_flag(error_flags::FLASH_CORRUPT);
                self.active_slot = None;
                self.next_seq = 1;
                Ok(())
            }
        }
    }

    fn save_config(&mut self, table: &ControlTable) -> Result<(), Self::Error> {
        // Target the inactive ping-pong slot
        let target_slot = match self.active_slot {
            Some(0) => 1,
            _ => 0,
        };

        let target_offset = Self::slot_offset(target_slot);
        let payload = table.eeprom_bytes();
        let crc = crc32_ieee(payload);

        let header = SectorHeader {
            magic: FLASH_MAGIC,
            version: FLASH_SCHEMA_VERSION,
            payload_len: EEPROM_AREA_SIZE as u16,
            seq: self.next_seq,
            crc32: crc,
        };

        let mut record = [0u8; RECORD_SIZE];
        record[0..HEADER_SIZE].copy_from_slice(&header.to_bytes());
        record[HEADER_SIZE..RECORD_SIZE].copy_from_slice(payload);

        // 1. Erase target sector
        self.backend
            .erase(target_offset, target_offset + SECTOR_SIZE)
            .map_err(|_| FlashStorageError::EraseError)?;

        // 2. Program target sector record
        self.backend
            .write(target_offset, &record)
            .map_err(|_| FlashStorageError::WriteError)?;

        // 3. Read back and verify
        let mut readback = [0u8; RECORD_SIZE];
        self.backend
            .read(target_offset, &mut readback)
            .map_err(|_| FlashStorageError::ReadError)?;

        if readback != record {
            return Err(FlashStorageError::VerifyError);
        }

        // Successfully committed to target slot
        self.active_slot = Some(target_slot);
        self.next_seq = self.next_seq.wrapping_add(1);

        Ok(())
    }

    fn factory_reset(&mut self, table: &mut ControlTable) -> Result<(), Self::Error> {
        table.load_factory_defaults();
        table.clear_error_flag(error_flags::FLASH_CORRUPT);

        // Erase both persistence sectors
        let _ = self.backend.erase(SECTOR_14_OFFSET, SECTOR_14_OFFSET + SECTOR_SIZE);
        let _ = self.backend.erase(SECTOR_15_OFFSET, SECTOR_15_OFFSET + SECTOR_SIZE);

        self.active_slot = None;
        self.next_seq = 1;
        Ok(())
    }
}

/// In-memory mock Flash storage backend for host testing.
#[derive(Debug)]
pub struct MockFlashBackend<const SIZE: usize = 0x20000> {
    pub memory: [u8; SIZE],
}

impl<const SIZE: usize> Default for MockFlashBackend<SIZE> {
    fn default() -> Self {
        Self {
            memory: [0xFF; SIZE],
        }
    }
}

impl<const SIZE: usize> MockFlashBackend<SIZE> {
    pub fn new() -> Self {
        Self::default()
    }
}

impl<const SIZE: usize> FlashBackend for MockFlashBackend<SIZE> {
    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), ()> {
        let start = offset as usize;
        let end = start + bytes.len();
        if end <= SIZE {
            bytes.copy_from_slice(&self.memory[start..end]);
            Ok(())
        } else {
            Err(())
        }
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), ()> {
        let start = from as usize;
        let end = to as usize;
        if end <= SIZE && start <= end {
            self.memory[start..end].fill(0xFF);
            Ok(())
        } else {
            Err(())
        }
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), ()> {
        let start = offset as usize;
        let end = start + bytes.len();
        if end <= SIZE {
            // Flash programming can only clear 1s to 0s
            for (i, &b) in bytes.iter().enumerate() {
                self.memory[start + i] &= b;
            }
            Ok(())
        } else {
            Err(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc32_known_vector() {
        // Standard check: CRC32("123456789") == 0xCBF43926
        let input = b"123456789";
        let crc = crc32_ieee(input);
        assert_eq!(crc, 0xCBF4_3926);
    }

    #[test]
    fn test_ping_pong_slot_rotation_and_recovery() {
        let mock = MockFlashBackend::<0x20000>::new();
        let mut storage = H5FlashStorage::new(mock);
        let mut table = ControlTable::new();

        // 1. Initial boot on blank flash -> loads defaults, sets FLASH_CORRUPT
        storage.load_config(&mut table).unwrap();
        assert_eq!(storage.active_slot(), None);
        assert_eq!(storage.next_seq(), 1);
        assert_ne!(table.hardware_error_status() & error_flags::FLASH_CORRUPT, 0);

        // 2. Modify table and save -> should write to Slot 0
        table.set_node_id(0x42);
        table.write_f32(barefoc::control_table::REG_CURRENT_LIMIT, 2.5);
        storage.save_config(&table).unwrap();

        assert_eq!(storage.active_slot(), Some(0));
        assert_eq!(storage.next_seq(), 2);

        // 3. Second save -> should ping-pong to Slot 1
        table.set_node_id(0x43);
        storage.save_config(&table).unwrap();

        assert_eq!(storage.active_slot(), Some(1));
        assert_eq!(storage.next_seq(), 3);

        // 4. Simulate reboot: new storage instance reading from same flash backend
        let mut new_storage = H5FlashStorage::new(storage.backend);
        let mut boot_table = ControlTable::new();
        new_storage.load_config(&mut boot_table).unwrap();

        assert_eq!(new_storage.active_slot(), Some(1));
        assert_eq!(boot_table.node_id(), 0x43);
        assert_eq!(boot_table.current_limit(), 2.5);
        assert_eq!(boot_table.hardware_error_status() & error_flags::FLASH_CORRUPT, 0);

        // 5. Corrupt active slot header CRC and verify fallback to older valid slot 0
        let slot_1_offset = SECTOR_15_OFFSET as usize;
        new_storage.backend.memory[slot_1_offset + 12] ^= 0xFF; // Invert CRC byte

        let mut fallback_storage = H5FlashStorage::new(new_storage.backend);
        let mut fallback_table = ControlTable::new();
        fallback_storage.load_config(&mut fallback_table).unwrap();

        assert_eq!(fallback_storage.active_slot(), Some(0));
        assert_eq!(fallback_table.node_id(), 0x42); // Reverted to slot 0 valid content
    }
}
