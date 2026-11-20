//! Cortex-M DWT-based microsecond clock implementation.
//!
//! The DWT (Data Watchpoint and Trace) cycle counter is a 32-bit register
//! that increments at the core clock frequency. It is the most accurate
//! and lowest-overhead way to measure time on Cortex-M4/M7 devices (STM32F4/G4).

use barefoc::common::time::Clock;

/// A microsecond clock based on the Cortex-M DWT cycle counter.
pub struct DwtClock {
    /// Core clock frequency in Hz (e.g., 168_000_000 for STM32F405, 170_000_000 for STM32G431).
    core_clock_hz: u32,
}

impl DwtClock {
    /// Create a new `DwtClock`.
    ///
    /// # Arguments
    /// * `core_clock_hz` - The system core clock frequency in Hz.
    pub fn new(core_clock_hz: u32) -> Self {
        #[cfg(all(target_arch = "arm", feature = "embassy-stm32"))]
        unsafe {
            let dwt = &*cortex_m::peripheral::DWT::PTR;
            dwt.ctrl.modify(|r| r | 1); // Enable CYCCNT
        }

        Self { core_clock_hz }
    }

    /// Get the configured core clock frequency in Hz.
    pub fn core_clock_hz(&self) -> u32 {
        self.core_clock_hz
    }
}

impl Clock for DwtClock {
    fn now_us(&self) -> u32 {
        #[cfg(all(target_arch = "arm", feature = "embassy-stm32"))]
        unsafe {
            let dwt = &*cortex_m::peripheral::DWT::PTR;
            let cycles = dwt.cyccnt.read();
            // Convert cycles to microseconds: (cycles * 1_000_000) / core_clock_hz
            ((cycles as u64 * 1_000_000) / self.core_clock_hz as u64) as u32
        }
        #[cfg(not(all(target_arch = "arm", feature = "embassy-stm32")))]
        0
    }

    fn delay_ms(&self, ms: u32) {
        #[cfg(all(target_arch = "arm", feature = "embassy-stm32"))]
        {
            let target_us = self.now_us().wrapping_add(ms * 1000);
            while self.now_us() < target_us {
                cortex_m::asm::nop();
            }
        }
        #[cfg(not(all(target_arch = "arm", feature = "embassy-stm32")))]
        let _ = ms;
    }
}
