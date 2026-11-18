//! # ESP32 Clock and Timing Bridges
//!
//! Provides microsecond and millisecond timekeeping implementations for BareFOC,
//! integrating with `esp-hal` timers, `esp-rtos`, and `embassy-time`.
//!
//! Upstream reference:
//! - `source/Arduino-FOC/src/common/time_utils.h`
//! - `source/Arduino-FOC/src/common/time_utils.cpp`

use barefoc::common::time::Clock;
use core::sync::atomic::{AtomicU32, Ordering};

/// Convert microseconds to milliseconds.
#[inline]
pub const fn us_to_ms(us: u32) -> u32 {
    us / 1000
}

/// Convert milliseconds to microseconds.
#[inline]
pub const fn ms_to_us(ms: u32) -> u32 {
    ms.saturating_mul(1000)
}

/// Calculate elapsed time (dt) in seconds between two microsecond timestamps,
/// properly handling 32-bit timestamp wraparound.
#[inline]
pub fn calc_dt(now_us: u32, prev_us: u32) -> f32 {
    let delta = now_us.wrapping_sub(prev_us);
    delta as f32 * 1e-6_f32
}

/// Monotonic microsecond clock for ESP32 systems.
///
/// Can operate either as a software/hardware-backed monotonic counter,
/// or hook into target hardware timers via `esp-hal` / `esp-rtos`.
pub struct Esp32Clock {
    counter: AtomicU32,
}

impl Esp32Clock {
    /// Create a new `Esp32Clock` initialized at zero.
    pub const fn new() -> Self {
        Self {
            counter: AtomicU32::new(0),
        }
    }

    /// Advance the internal tick counter by a specific number of microseconds.
    /// Useful for host testing and simulated execution.
    pub fn advance_us(&self, us: u32) {
        self.counter.fetch_add(us, Ordering::Relaxed);
    }

    /// Set the microsecond counter value directly.
    pub fn set_us(&self, us: u32) {
        self.counter.store(us, Ordering::Relaxed);
    }

    /// Return the current elapsed time in milliseconds.
    pub fn now_ms(&self) -> u32 {
        us_to_ms(self.now_us())
    }

    /// Return elapsed microseconds since `start_us`, accounting for wraparound.
    pub fn elapsed_us(&self, start_us: u32) -> u32 {
        self.now_us().wrapping_sub(start_us)
    }

    /// Return elapsed milliseconds since `start_ms`, accounting for wraparound.
    pub fn elapsed_ms(&self, start_ms: u32) -> u32 {
        self.now_ms().wrapping_sub(start_ms)
    }
}

impl Default for Esp32Clock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for Esp32Clock {
    fn now_us(&self) -> u32 {
        #[cfg(all(feature = "esp32", not(test)))]
        {
            // On target with esp-hal, query the hardware SystemTimer or Instant
            esp_hal::time::Instant::now().as_micros() as u32
        }
        #[cfg(not(all(feature = "esp32", not(test))))]
        {
            self.counter.load(Ordering::Relaxed)
        }
    }

    fn delay_ms(&self, ms: u32) {
        let start = self.now_us();
        let target_us = ms.saturating_mul(1000);
        while self.now_us().wrapping_sub(start) < target_us {
            core::hint::spin_loop();
        }
    }
}

/// An Embassy time-driver clock bridge utilizing `embassy-time` and `esp-rtos`.
#[cfg(feature = "embassy")]
pub struct EmbassyClock;

#[cfg(feature = "embassy")]
impl EmbassyClock {
    /// Create a new `EmbassyClock`.
    pub const fn new() -> Self {
        Self
    }
}

#[cfg(feature = "embassy")]
impl Default for EmbassyClock {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "embassy")]
impl Clock for EmbassyClock {
    fn now_us(&self) -> u32 {
        embassy_time::Instant::now().as_micros() as u32
    }

    fn delay_ms(&self, ms: u32) {
        embassy_time::block_for(embassy_time::Duration::from_millis(ms as u64));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_us_to_ms_and_ms_to_us() {
        assert_eq!(us_to_ms(0), 0);
        assert_eq!(us_to_ms(999), 0);
        assert_eq!(us_to_ms(1000), 1);
        assert_eq!(us_to_ms(2500), 2);
        assert_eq!(us_to_ms(1_000_000), 1000);

        assert_eq!(ms_to_us(0), 0);
        assert_eq!(ms_to_us(1), 1000);
        assert_eq!(ms_to_us(42), 42_000);
        assert_eq!(ms_to_us(u32::MAX), u32::MAX);
    }

    #[test]
    fn test_calc_dt_normal_and_wrapping() {
        // Normal elapsed time: 1000 us = 1 ms = 0.001 s
        let dt = calc_dt(2000, 1000);
        assert!((dt - 0.001_f32).abs() < 1e-7);

        // 50 us elapsed
        let dt_fast = calc_dt(1050, 1000);
        assert!((dt_fast - 0.00005_f32).abs() < 1e-7);

        // Wraparound across u32::MAX
        let prev = u32::MAX - 500;
        let now = 499; // elapsed is (500 + 1 + 499) = 1000 us
        let dt_wrap = calc_dt(now, prev);
        assert!((dt_wrap - 0.001_f32).abs() < 1e-7);
    }

    #[test]
    fn test_esp32_clock_operations() {
        let clock = Esp32Clock::new();
        assert_eq!(clock.now_us(), 0);
        assert_eq!(clock.now_ms(), 0);

        clock.advance_us(15_500);
        assert_eq!(clock.now_us(), 15_500);
        assert_eq!(clock.now_ms(), 15);
        assert_eq!(clock.elapsed_us(10_000), 5500);
        assert_eq!(clock.elapsed_ms(10), 5);

        clock.set_us(100_000);
        assert_eq!(clock.now_us(), 100_000);
        assert_eq!(clock.now_ms(), 100);
    }

    #[cfg(feature = "embassy")]
    #[test]
    fn test_embassy_clock_creation() {
        let _clock = EmbassyClock::new();
    }
}
