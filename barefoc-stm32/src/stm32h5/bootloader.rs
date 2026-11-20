//! STM32H5 BareBoot Katapult hands-free bootloader handoff driver.

use barefoc::traits::bootloader::BootloaderHandoff;
use crate::bootloader::{default_target, enter_bareboot};

/// STM32H5 bootloader hand-off driver implementing [`BootloaderHandoff`].
///
/// Prior to invoking the BareBoot reset sequence, invokes the disarm closure or
/// registered `before_reset_hook` to ensure all H-bridge outputs enter high-impedance Coast mode.
pub struct Stm32H5Bootloader {
    disarm_hook: Option<fn()>,
}

impl Default for Stm32H5Bootloader {
    fn default() -> Self {
        Self { disarm_hook: None }
    }
}

impl Stm32H5Bootloader {
    /// Create a new bootloader hand-off instance.
    pub const fn new() -> Self {
        Self { disarm_hook: None }
    }

    /// Create an instance with an explicit driver disarm callback.
    pub fn with_disarm_hook(disarm_hook: fn()) -> Self {
        Self {
            disarm_hook: Some(disarm_hook),
        }
    }
}

impl BootloaderHandoff for Stm32H5Bootloader {
    fn enter_bootloader(&mut self) -> ! {
        if let Some(hook) = self.disarm_hook {
            hook();
        }
        enter_bareboot(default_target())
    }
}
