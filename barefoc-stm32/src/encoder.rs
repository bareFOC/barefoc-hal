//! Hardware Quadrature Encoder Interface (QEI) implementation using `embassy-stm32`'s `Qei`.
//!
//! Provides zero-CPU-overhead hardware position tracking adhering to `barefoc::traits::Sensor`.
//!
//! Upstream reference:
//! - `source/Arduino-FOC/src/sensors/Encoder.cpp`
//! - `source/Arduino-FOC/src/common/base_classes/Sensor.h`

use barefoc::common::time::Clock;
use barefoc::traits::sensor::{Direction, Sensor, SensorBase};
use embassy_stm32::timer::qei::{Config, Direction as QeiDirection, Qei};
use embassy_stm32::timer::{Ch1, Ch2, GeneralInstance4Channel, TimerPin};
use embassy_stm32::Peri;

/// Hardware Quadrature Encoder sensor backed by STM32 timer hardware QEI decoding.
pub struct Stm32Encoder<'d, T: GeneralInstance4Channel> {
    base: SensorBase,
    qei: Qei<'d, T>,
    cpr: u32,
}

impl<'d, T: GeneralInstance4Channel> Stm32Encoder<'d, T> {
    /// Create a new hardware encoder instance.
    ///
    /// # Arguments
    /// * `tim` - The timer peripheral (e.g., TIM2, TIM3, TIM4, TIM5).
    /// * `ch1` - Channel 1 pin (Encoder phase A).
    /// * `ch2` - Channel 2 pin (Encoder phase B).
    /// * `ppr` - Pulses per revolution (CPR will be `4 * ppr` in hardware 4x quadrature mode).
    pub fn new(
        tim: Peri<'d, T>,
        ch1: Peri<'d, impl TimerPin<T, Ch1>>,
        ch2: Peri<'d, impl TimerPin<T, Ch2>>,
        ppr: u32,
    ) -> Self {
        let qei = Qei::new(tim, ch1, ch2, Config::default());
        let cpr = ppr * 4;

        Self {
            base: SensorBase::new(),
            qei,
            cpr,
        }
    }

    /// Construct an encoder from a pre-configured `Qei` instance and known CPR.
    pub fn from_qei(qei: Qei<'d, T>, cpr: u32) -> Self {
        Self {
            base: SensorBase::new(),
            qei,
            cpr,
        }
    }

    /// Read raw hardware timer counter value.
    pub fn count(&self) -> u32 {
        self.qei.count()
    }

    /// Get configured Counts Per Revolution (CPR = 4 * PPR).
    pub fn cpr(&self) -> u32 {
        self.cpr
    }

    /// Get current rotation direction from the hardware timer counter.
    pub fn direction(&self) -> Direction {
        match self.qei.read_direction() {
            QeiDirection::Upcounting => Direction::CW,
            QeiDirection::Downcounting => Direction::CCW,
        }
    }
}

impl<'d, T: GeneralInstance4Channel, C: Clock> Sensor<C> for Stm32Encoder<'d, T> {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    fn get_sensor_angle(&self) -> f32 {
        calc_encoder_angle(self.qei.count(), self.cpr)
    }
}

/// Helper to calculate mechanical shaft angle in `[0, 2π)` from raw encoder counter and CPR.
pub fn calc_encoder_angle(count: u32, cpr: u32) -> f32 {
    if cpr == 0 {
        return 0.0;
    }
    let cnt = count as f32;
    let cpr_f = cpr as f32;
    let m = cnt - libm::floorf(cnt / cpr_f) * cpr_f;
    barefoc::math::_2PI * m / cpr_f
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calc_encoder_angle() {
        let cpr = 4000; // 1000 PPR * 4
        // Zero count -> 0.0 rad
        assert!((calc_encoder_angle(0, cpr) - 0.0).abs() < 1e-6);

        // Quarter turn -> π/2
        let angle_quarter = calc_encoder_angle(1000, cpr);
        assert!((angle_quarter - barefoc::math::_PI_2).abs() < 1e-4);

        // Half turn -> π
        let angle_half = calc_encoder_angle(2000, cpr);
        assert!((angle_half - barefoc::math::_PI).abs() < 1e-4);

        // Three-quarter turn -> 3π/2
        let angle_3quarter = calc_encoder_angle(3000, cpr);
        assert!((angle_3quarter - barefoc::math::_3PI_2).abs() < 1e-4);

        // Modulo wrap at CPR -> 0.0 rad
        let angle_full = calc_encoder_angle(4000, cpr);
        assert!(angle_full.abs() < 1e-4);
    }
}
