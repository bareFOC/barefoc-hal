//! # ESP32 PCNT Quadrature Encoder Sensor
//!
//! Provides hardware quadrature encoder decoding using the ESP32 Pulse Counter (PCNT) peripheral.
//! PCNT tracks 4x quadrature edges in hardware without CPU interrupt overhead.
//!
//! Upstream reference:
//! - `source/Arduino-FOC/src/sensors/Encoder.h`
//! - `source/Arduino-FOC/src/sensors/Encoder.cpp`
//! - `source/Arduino-FOC/src/common/base_classes/Sensor.h`

use barefoc::common::time::Clock;
use barefoc::traits::sensor::{Direction, Sensor, SensorBase};

/// Calculate shaft mechanical angle in `[0, 2π)` from accumulated pulse count and CPR.
#[inline]
pub fn calc_pcnt_angle(count: i32, cpr: u32) -> f32 {
    if cpr == 0 {
        return 0.0;
    }
    let cnt_f = count as f32;
    let cpr_f = cpr as f32;
    let norm = cnt_f - libm::floorf(cnt_f / cpr_f) * cpr_f;
    let angle = barefoc::math::_2PI * norm / cpr_f;
    if angle < 0.0 {
        angle + barefoc::math::_2PI
    } else {
        angle
    }
}

/// ESP32 PCNT-backed quadrature encoder sensor.
///
/// Implements `barefoc::traits::sensor::Sensor` using pulse counter hardware tracking.
pub struct Esp32PcntSensor {
    base: SensorBase,
    cpr: u32,
    accumulated_count: i32,
    last_hw_count: i16,
    direction: Direction,
}

impl Esp32PcntSensor {
    /// Create a new PCNT encoder sensor from pulses per revolution (PPR).
    ///
    /// In 4x quadrature decoding, CPR (counts per revolution) is `4 * PPR`.
    pub fn new(ppr: u32) -> Self {
        Self::from_cpr(ppr.saturating_mul(4))
    }

    /// Create a new PCNT encoder sensor from counts per revolution (CPR).
    pub fn from_cpr(cpr: u32) -> Self {
        Self {
            base: SensorBase::new(),
            cpr,
            accumulated_count: 0,
            last_hw_count: 0,
            direction: Direction::UNKNOWN,
        }
    }

    /// Get configured Counts Per Revolution (CPR = 4 * PPR).
    pub fn cpr(&self) -> u32 {
        self.cpr
    }

    /// Get current total accumulated count.
    pub fn count(&self) -> i32 {
        self.accumulated_count
    }

    /// Get most recent rotation direction.
    pub fn direction(&self) -> Direction {
        self.direction
    }

    /// Get raw mechanical shaft angle in `[0, 2π)`.
    pub fn get_sensor_angle(&self) -> f32 {
        calc_pcnt_angle(self.accumulated_count, self.cpr)
    }

    /// Reset internal accumulated position count to zero.
    pub fn reset_count(&mut self) {
        self.accumulated_count = 0;
        self.last_hw_count = 0;
        self.direction = Direction::UNKNOWN;
    }

    /// Feed a raw 16-bit hardware counter reading from the PCNT peripheral.
    ///
    /// Handles 16-bit hardware wraparound and accumulates into 32-bit position.
    pub fn update_hardware_count(&mut self, hw_count: i16) {
        let delta = hw_count.wrapping_sub(self.last_hw_count);
        self.last_hw_count = hw_count;
        self.accumulated_count = self.accumulated_count.wrapping_add(delta as i32);

        if delta > 0 {
            self.direction = Direction::CW;
        } else if delta < 0 {
            self.direction = Direction::CCW;
        }
    }
}

impl<C: Clock> Sensor<C> for Esp32PcntSensor {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    fn get_sensor_angle(&self) -> f32 {
        calc_pcnt_angle(self.accumulated_count, self.cpr)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use barefoc::common::time::Clock;

    struct TestClock(core::sync::atomic::AtomicU32);
    impl Clock for TestClock {
        fn now_us(&self) -> u32 {
            self.0.load(core::sync::atomic::Ordering::Relaxed)
        }
    }

    #[test]
    fn test_calc_pcnt_angle_positive_and_negative() {
        let cpr = 4000;

        // Count = 0 -> 0.0 rad
        assert_eq!(calc_pcnt_angle(0, cpr), 0.0);

        // Count = 1000 (quarter rotation) -> π / 2
        let angle_quarter = calc_pcnt_angle(1000, cpr);
        assert!((angle_quarter - (core::f32::consts::PI / 2.0)).abs() < 1e-4);

        // Count = 2000 (half rotation) -> π
        let angle_half = calc_pcnt_angle(2000, cpr);
        assert!((angle_half - core::f32::consts::PI).abs() < 1e-4);

        // Count = 4000 (full rotation) -> 0.0
        let angle_full = calc_pcnt_angle(4000, cpr);
        assert!(angle_full < 1e-4);

        // Count = -1000 (-quarter rotation) -> 3π / 2
        let angle_neg = calc_pcnt_angle(-1000, cpr);
        assert!((angle_neg - (3.0 * core::f32::consts::PI / 2.0)).abs() < 1e-4);
    }

    #[test]
    fn test_pcnt_hardware_accumulation() {
        let mut sensor = Esp32PcntSensor::new(1000); // 4000 CPR
        assert_eq!(sensor.cpr(), 4000);
        assert_eq!(sensor.count(), 0);
        assert_eq!(sensor.direction(), Direction::UNKNOWN);

        sensor.update_hardware_count(100);
        assert_eq!(sensor.count(), 100);
        assert_eq!(sensor.direction(), Direction::CW);

        sensor.update_hardware_count(250);
        assert_eq!(sensor.count(), 250);
        assert_eq!(sensor.direction(), Direction::CW);

        sensor.update_hardware_count(200);
        assert_eq!(sensor.count(), 200);
        assert_eq!(sensor.direction(), Direction::CCW);

        // Test 16-bit wraparound (e.g. from 32700 to -32700)
        sensor.last_hw_count = 32700;
        sensor.accumulated_count = 32700;
        // Wrapping forward past i16::MAX: 32700 + 100 = 32800 -> represented as i16 is -32736
        let next_hw: i16 = 32700i16.wrapping_add(100);
        sensor.update_hardware_count(next_hw);
        assert_eq!(sensor.count(), 32800);
        assert_eq!(sensor.direction(), Direction::CW);
    }

    #[test]
    fn test_pcnt_sensor_trait_angle() {
        let mut sensor = Esp32PcntSensor::from_cpr(4000);
        let clock = TestClock(core::sync::atomic::AtomicU32::new(1000));

        sensor.update_hardware_count(2000);
        let angle = sensor.get_sensor_angle();
        assert!((angle - core::f32::consts::PI).abs() < 1e-4);

        sensor.update(&clock);
        let mech_angle = <Esp32PcntSensor as Sensor<TestClock>>::get_mechanical_angle(&sensor);
        assert!((mech_angle - core::f32::consts::PI).abs() < 1e-4);
    }
}
