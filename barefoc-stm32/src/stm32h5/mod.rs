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