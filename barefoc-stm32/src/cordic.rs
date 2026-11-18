//! STM32G4 Hardware CORDIC trigonometric acceleration.
//!
//! Provides hardware trigonometric calculation using the STM32G4 CORDIC coprocessor,
//! implementing [`barefoc::math::cordic::CordicTrig`].
//!
//! Upstream reference:
//! - STM32G4 Reference Manual RM0440 (CORDIC Co-processor)

use barefoc::math::cordic::{angle_to_q1_31, q1_31_to_f32, CordicTrig};
use dsp_fixedpoint::Q32;
use embassy_stm32::cordic::{AccessCount, Config, Cordic, Function, Instance, Precision, Scale};
use embassy_stm32::Peri;

/// STM32G4 hardware CORDIC driver for accelerated field oriented control trigonometric math.
pub struct Stm32G4Cordic<'d, T: Instance> {
    cordic: Cordic<'d, T>,
}

impl<'d, T: Instance> Stm32G4Cordic<'d, T> {
    /// Create a new hardware CORDIC accelerator instance configured for sine/cosine computation.
    pub fn new(peri: Peri<'d, T>) -> Self {
        let config =
            Config::new(Function::Cos, Precision::Iters4, Scale::Arg1Res1).expect("valid CORDIC config");
        let cordic = Cordic::new(peri, &config);

        Self { cordic }
    }

    /// Access the underlying `embassy_stm32::cordic::Cordic` driver.
    pub fn inner_mut(&mut self) -> &mut Cordic<'d, T> {
        &mut self.cordic
    }
}

impl<'d, T: Instance> CordicTrig for Stm32G4Cordic<'d, T> {
    fn sincos(&mut self, angle_rad: f32) -> (f32, f32) {
        let q_angle = Q32::<31>::from_bits(angle_to_q1_31(angle_rad));
        // ARG1 = angle, ARG2 = 1.0 (magnitude, set by `Cordic::new`); with
        // `AccessCount::One` arguments are reused, and `AccessCount::Two`
        // results delivers RES1 = cos * mag, RES2 = sin * mag.
        let mut res = [Q32::<31>::from_bits(0); 2];
        if self
            .cordic
            .q1_31(AccessCount::One, AccessCount::Two)
            .blocking_calc(&[q_angle], &mut res)
            .is_ok()
        {
            let cos_f = q1_31_to_f32(res[0].into_bits());
            let sin_f = q1_31_to_f32(res[1].into_bits());
            (sin_f, cos_f)
        } else {
            barefoc::math::sincos(angle_rad)
        }
    }
}
