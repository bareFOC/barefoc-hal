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

/// Configuration schema version (v2: ControlTable + KinematicsFlashConfig).
pub const FLASH_SCHEMA_VERSION: u16 = 2;

/// Byte offset of Bank 2 Sector 14 relative to `FLASH_BASE` (`0x0800_0000`).
pub const SECTOR_14_OFFSET: u32 = 0x0001_C000;

/// Byte offset of Bank 2 Sector 15 relative to `FLASH_BASE` (`0x0800_0000`).
pub const SECTOR_15_OFFSET: u32 = 0x0001_E000;

/// Size of each STM32H5 flash sector in bytes (8 KiB).
pub const SECTOR_SIZE: u32 = 8192;

/// Size of the flash persistence header in bytes.
pub const HEADER_SIZE: usize = 32;

/// Maximum number of LUT points persisted in flash.
pub const MAX_FLASH_LUT_POINTS: usize = 16;

/// Size of the serialized kinematics configuration area in bytes (multiple of 16 bytes).
pub const KINEMATICS_AREA_SIZE: usize = 192;

/// Combined v2 payload size (ControlTable EEPROM + KinematicsFlashConfig).
pub const PAYLOAD_V2_SIZE: usize = EEPROM_AREA_SIZE + KINEMATICS_AREA_SIZE;

/// Total stored record size (32-byte header + 288-byte v2 payload = 320 bytes).
pub const RECORD_SIZE: usize = HEADER_SIZE + PAYLOAD_V2_SIZE;

/// Flash-persisted runtime kinematics configuration (`TopologyConfig` + `DynamicTransmission` + outer PIDs).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KinematicsFlashConfig {
    pub topology_mode: u8,
    pub coupling_mask: u8,
    pub sensor_map: [u8; 4],
    pub transmission_type: u8,
    pub outer_downsample: u8,
    pub gear_ratio: f32,
    pub backlash_delta: f32,
    pub omega_thresh: f32,
    pub torsional_stiffness: f32,
    pub joint_offset_alpha: f32,
    pub joint_offset_beta: f32,
    pub diff_matrix: [f32; 4],
    pub linkage_lengths: [f32; 4],
    pub lut_mot_min: f32,
    pub lut_mot_max: f32,
    pub lut_points: u16,
    pub lut_table: [f32; MAX_FLASH_LUT_POINTS],
    pub outer_pid_alpha: [f32; 5],
    pub outer_pid_beta: [f32; 5],
}

impl Default for KinematicsFlashConfig {
    fn default() -> Self {
        Self::default_const()
    }
}

impl KinematicsFlashConfig {
    /// Const-evaluable factory default kinematics configuration.
    pub const fn default_const() -> Self {
        Self {
            topology_mode: 0, // Four1Dof
            coupling_mask: 0x00,
            sensor_map: [0, 1, 2, 3],
            transmission_type: 0, // LinearGear
            outer_downsample: 10,
            gear_ratio: 1.0,
            backlash_delta: 0.0,
            omega_thresh: 0.0,
            torsional_stiffness: 0.0,
            joint_offset_alpha: 0.0,
            joint_offset_beta: 0.0,
            diff_matrix: [0.5, 0.5, 0.5, -0.5],
            linkage_lengths: [1.0, 1.0, 1.0, 1.0],
            lut_mot_min: 0.0,
            lut_mot_max: 1.0,
            lut_points: 0,
            lut_table: [0.0; MAX_FLASH_LUT_POINTS],
            outer_pid_alpha: [20.0, 0.0, 0.0, 50.0, 0.0],
            outer_pid_beta: [20.0, 0.0, 0.0, 50.0, 0.0],
        }
    }
    /// Serialize kinematics configuration to 192 little-endian bytes.
    pub fn to_bytes(&self) -> [u8; KINEMATICS_AREA_SIZE] {
        let mut buf = [0u8; KINEMATICS_AREA_SIZE];
        buf[0] = self.topology_mode;
        buf[1] = self.coupling_mask;
        buf[2..6].copy_from_slice(&self.sensor_map);
        buf[6] = self.transmission_type;
        buf[7] = self.outer_downsample;
        buf[8..12].copy_from_slice(&self.gear_ratio.to_le_bytes());
        buf[12..16].copy_from_slice(&self.backlash_delta.to_le_bytes());
        buf[16..20].copy_from_slice(&self.omega_thresh.to_le_bytes());
        buf[20..24].copy_from_slice(&self.torsional_stiffness.to_le_bytes());
        buf[24..28].copy_from_slice(&self.joint_offset_alpha.to_le_bytes());
        buf[28..32].copy_from_slice(&self.joint_offset_beta.to_le_bytes());
        for (i, &v) in self.diff_matrix.iter().enumerate() {
            buf[32 + i * 4..36 + i * 4].copy_from_slice(&v.to_le_bytes());
        }
        for (i, &v) in self.linkage_lengths.iter().enumerate() {
            buf[48 + i * 4..52 + i * 4].copy_from_slice(&v.to_le_bytes());
        }
        buf[64..68].copy_from_slice(&self.lut_mot_min.to_le_bytes());
        buf[68..72].copy_from_slice(&self.lut_mot_max.to_le_bytes());
        buf[72..74].copy_from_slice(&self.lut_points.to_le_bytes());
        for (i, &v) in self.lut_table.iter().enumerate() {
            buf[76 + i * 4..80 + i * 4].copy_from_slice(&v.to_le_bytes());
        }
        for (i, &v) in self.outer_pid_alpha.iter().enumerate() {
            buf[140 + i * 4..144 + i * 4].copy_from_slice(&v.to_le_bytes());
        }
        for (i, &v) in self.outer_pid_beta.iter().enumerate() {
            buf[160 + i * 4..164 + i * 4].copy_from_slice(&v.to_le_bytes());
        }
        buf
    }

    /// Deserialize kinematics configuration from a 192-byte slice.
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < KINEMATICS_AREA_SIZE {
            return None;
        }
        let read_f32 = |off: usize| -> f32 {
            f32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
        };
        let mut diff_matrix = [0.0f32; 4];
        for i in 0..4 {
            diff_matrix[i] = read_f32(32 + i * 4);
        }
        let mut linkage_lengths = [0.0f32; 4];
        for i in 0..4 {
            linkage_lengths[i] = read_f32(48 + i * 4);
        }
        let mut lut_table = [0.0f32; MAX_FLASH_LUT_POINTS];
        for i in 0..MAX_FLASH_LUT_POINTS {
            lut_table[i] = read_f32(76 + i * 4);
        }
        let mut outer_pid_alpha = [0.0f32; 5];
        for i in 0..5 {
            outer_pid_alpha[i] = read_f32(140 + i * 4);
        }
        let mut outer_pid_beta = [0.0f32; 5];
        for i in 0..5 {
            outer_pid_beta[i] = read_f32(160 + i * 4);
        }
        Some(Self {
            topology_mode: buf[0],
            coupling_mask: buf[1],
            sensor_map: [buf[2], buf[3], buf[4], buf[5]],
            transmission_type: buf[6],
            outer_downsample: buf[7],
            gear_ratio: read_f32(8),
            backlash_delta: read_f32(12),
            omega_thresh: read_f32(16),
            torsional_stiffness: read_f32(20),
            joint_offset_alpha: read_f32(24),
            joint_offset_beta: read_f32(28),
            diff_matrix,
            linkage_lengths,
            lut_mot_min: read_f32(64),
            lut_mot_max: read_f32(68),
            lut_points: u16::from_le_bytes([buf[72], buf[73]]),
            lut_table,
            outer_pid_alpha,
            outer_pid_beta,
        })
    }
}

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
            && self.payload_len == (PAYLOAD_V2_SIZE as u16)
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

    /// Read and validate a v2 slot record.
    fn inspect_slot(&mut self, slot: u8, payload_out: &mut [u8; PAYLOAD_V2_SIZE]) -> Option<u32> {
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

    /// Load both `ControlTable` EEPROM and `KinematicsFlashConfig` from Sector 14/15.
    pub fn load_config_v2(
        &mut self,
        table: &mut ControlTable,
        kin: &mut KinematicsFlashConfig,
    ) -> Result<(), FlashStorageError> {
        let mut payload_0 = [0u8; PAYLOAD_V2_SIZE];
        let mut payload_1 = [0u8; PAYLOAD_V2_SIZE];

        let valid_0 = self.inspect_slot(0, &mut payload_0);
        let valid_1 = self.inspect_slot(1, &mut payload_1);

        let chosen = match (valid_0, valid_1) {
            (Some(seq_0), Some(seq_1)) => {
                if seq_0 >= seq_1 {
                    self.active_slot = Some(0);
                    self.next_seq = seq_0.wrapping_add(1);
                    Some(&payload_0)
                } else {
                    self.active_slot = Some(1);
                    self.next_seq = seq_1.wrapping_add(1);
                    Some(&payload_1)
                }
            }
            (Some(seq_0), None) => {
                self.active_slot = Some(0);
                self.next_seq = seq_0.wrapping_add(1);
                Some(&payload_0)
            }
            (None, Some(seq_1)) => {
                self.active_slot = Some(1);
                self.next_seq = seq_1.wrapping_add(1);
                Some(&payload_1)
            }
            (None, None) => None,
        };

        if let Some(payload) = chosen {
            table
                .eeprom_bytes_mut()
                .copy_from_slice(&payload[..EEPROM_AREA_SIZE]);
            if let Some(decoded) =
                KinematicsFlashConfig::from_bytes(&payload[EEPROM_AREA_SIZE..PAYLOAD_V2_SIZE])
            {
                *kin = decoded;
            }
            table.clear_error_flag(error_flags::FLASH_CORRUPT);
        } else {
            table.load_factory_defaults();
            *kin = KinematicsFlashConfig::default();
            table.set_error_flag(error_flags::FLASH_CORRUPT);
            self.active_slot = None;
            self.next_seq = 1;
        }
        Ok(())
    }

    /// Save both `ControlTable` EEPROM and `KinematicsFlashConfig` atomically to the inactive sector.
    pub fn save_config_v2(
        &mut self,
        table: &ControlTable,
        kin: &KinematicsFlashConfig,
    ) -> Result<(), FlashStorageError> {
        let target_slot = match self.active_slot {
            Some(0) => 1,
            _ => 0,
        };

        let target_offset = Self::slot_offset(target_slot);
        let mut payload = [0u8; PAYLOAD_V2_SIZE];
        payload[..EEPROM_AREA_SIZE].copy_from_slice(table.eeprom_bytes());
        payload[EEPROM_AREA_SIZE..PAYLOAD_V2_SIZE].copy_from_slice(&kin.to_bytes());
        let crc = crc32_ieee(&payload);

        let header = SectorHeader {
            magic: FLASH_MAGIC,
            version: FLASH_SCHEMA_VERSION,
            payload_len: PAYLOAD_V2_SIZE as u16,
            seq: self.next_seq,
            crc32: crc,
        };

        let mut record = [0u8; RECORD_SIZE];
        record[0..HEADER_SIZE].copy_from_slice(&header.to_bytes());
        record[HEADER_SIZE..RECORD_SIZE].copy_from_slice(&payload);

        self.backend
            .erase(target_offset, target_offset + SECTOR_SIZE)
            .map_err(|_| FlashStorageError::EraseError)?;

        self.backend
            .write(target_offset, &record)
            .map_err(|_| FlashStorageError::WriteError)?;

        let mut readback = [0u8; RECORD_SIZE];
        self.backend
            .read(target_offset, &mut readback)
            .map_err(|_| FlashStorageError::ReadError)?;

        if readback != record {
            return Err(FlashStorageError::VerifyError);
        }

        self.active_slot = Some(target_slot);
        self.next_seq = self.next_seq.wrapping_add(1);
        Ok(())
    }
}

impl<B: FlashBackend> ConfigStorage for H5FlashStorage<B> {
    type Error = FlashStorageError;

    fn load_config(&mut self, table: &mut ControlTable) -> Result<(), Self::Error> {
        let mut kin = KinematicsFlashConfig::default();
        self.load_config_v2(table, &mut kin)
    }

    fn save_config(&mut self, table: &ControlTable) -> Result<(), Self::Error> {
        self.save_config_v2(table, &KinematicsFlashConfig::default())
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
        table.set_motor_phase_resistance(0, 2.5);
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
        assert_eq!(boot_table.motor_phase_resistance(0), 2.5);
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

    #[test]
    fn test_v2_kinematics_crc32_round_trip() {
        let mock = MockFlashBackend::<0x20000>::new();
        let mut storage = H5FlashStorage::new(mock);
        let mut table = ControlTable::new();
        table.set_node_id(0x45);

        let mut kin = KinematicsFlashConfig::default();
        kin.topology_mode = 1; // Two2Dof
        kin.coupling_mask = 0x03;
        kin.sensor_map = [2, 3, 0, 1];
        kin.transmission_type = 1; // CoupledDifferential
        kin.gear_ratio = 2.5;
        kin.backlash_delta = 0.012;
        kin.joint_offset_alpha = 0.25;
        kin.joint_offset_beta = -0.5;
        kin.diff_matrix = [0.5, 0.5, 0.5, -0.5];
        kin.linkage_lengths = [100.0, 30.0, 90.0, 40.0];
        kin.lut_mot_min = -1.0;
        kin.lut_mot_max = 1.0;
        kin.lut_points = 4;
        kin.lut_table[0..4].copy_from_slice(&[-0.5, -0.1, 0.1, 0.5]);
        kin.outer_pid_alpha = [18.0, 0.5, 0.02, 40.0, 100.0];
        kin.outer_pid_beta = [16.0, 0.2, 0.01, 35.0, 80.0];

        storage.save_config_v2(&table, &kin).unwrap();

        let mut reboot_storage = H5FlashStorage::new(storage.backend);
        let mut reboot_table = ControlTable::new();
        let mut reboot_kin = KinematicsFlashConfig::default();
        reboot_storage
            .load_config_v2(&mut reboot_table, &mut reboot_kin)
            .unwrap();

        assert_eq!(reboot_table.node_id(), 0x45);
        assert_eq!(reboot_kin, kin);
        assert_eq!(
            reboot_table.hardware_error_status() & error_flags::FLASH_CORRUPT,
            0
        );
    }
}
