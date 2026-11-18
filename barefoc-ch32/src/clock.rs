//! CH32 clock and timing bridge.

use barefoc::common::time::Clock;
use core::cell::Cell;

pub fn us_to_ms(us: u32) -> u32 {
    us / 1000
}

pub fn ms_to_us(ms: u32) -> u32 {
    ms.saturating_mul(1000)
}

pub fn calc_dt(now_us: u32, prev_us: u32) -> f32 {
    let delta = now_us.wrapping_sub(prev_us);
    delta as f32 * 1e-6_f32
}

/// Monotonic microsecond clock for CH32.
pub struct Ch32Clock {
    counter: Cell<u32>,
}

impl Ch32Clock {
    pub const fn new() -> Self {
        Self {
            counter: Cell::new(0),
        }
    }

    pub fn advance_us(&self, us: u32) {
        self.counter.set(self.counter.get().wrapping_add(us));
    }

    pub fn set_us(&self, us: u32) {
        self.counter.set(us);
    }

    pub fn now_ms(&self) -> u32 {
        us_to_ms(self.now_us())
    }
}

impl Default for Ch32Clock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for Ch32Clock {
    fn now_us(&self) -> u32 {
        #[cfg(feature = "embassy-time")]
        {
            embassy_time::Instant::now().as_micros() as u32
        }
        #[cfg(not(feature = "embassy-time"))]
        {
            self.counter.get()
        }
    }

    fn delay_ms(&self, ms: u32) {
        #[cfg(feature = "embassy-time")]
        {
            embassy_time::block_for(embassy_time::Duration::from_millis(ms as u64));
        }
        #[cfg(not(feature = "embassy-time"))]
        {
            let start = self.now_us();
            let target = ms.saturating_mul(1000);
            while self.now_us().wrapping_sub(start) < target {
                core::hint::spin_loop();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conversion() {
        assert_eq!(us_to_ms(1000), 1);
        assert_eq!(ms_to_us(2), 2000);
    }

    #[test]
    fn counter_fallback() {
        let c = Ch32Clock::new();
        c.advance_us(1000);
        assert_eq!(c.now_us(), 1000);
    }
}