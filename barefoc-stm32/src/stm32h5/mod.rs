//! STM32H5 specific hardware bridges.

#[cfg(feature = "embassy-stm32")]
pub use crate::clock;
#[cfg(feature = "embassy-stm32")]
pub use crate::clock::DwtClock;
#[cfg(feature = "embassy-stm32")]
pub use crate::driver_6pwm;
#[cfg(feature = "embassy-stm32")]
pub use crate::driver_6pwm::Stm32BldcDriver6Pwm;
#[cfg(feature = "embassy-stm32")]
pub use crate::encoder;
#[cfg(feature = "embassy-stm32")]
pub use crate::encoder::Stm32Encoder;
#[cfg(feature = "embassy-stm32")]
pub use crate::foc_config::FocConfig;

#[cfg(feature = "bareboot")]
pub mod bootloader;
pub mod flash_storage;

#[cfg(feature = "bareboot")]
pub use bootloader::Stm32H5Bootloader;
pub use flash_storage::{H5FlashStorage, SectorHeader, FLASH_MAGIC, FLASH_SCHEMA_VERSION};

