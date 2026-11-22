//! # temperature — STM32 factory-calibrated internal temperature sensor
//!
//! Provides [`Stm32InternalTemp`], a zero-heap adapter that converts the
//! internal temperature-sensor ADC channel using the factory-programmed
//! `TS_CAL1` / `TS_CAL2` words of the target silicon. No fixed nominal slope is
//! assumed and no MCU-specific register address is encoded in a library crate.
//!
//! Contract sources:
//! - `bareFOC/src/traits/temperature.rs` — `TemperatureSensor` interface.
//! - `bareFOC/src/traits/adc.rs` — `AdcPin` raw-sample source.
//! - `bareFOC/src/control_table.rs` — `REG_PRESENT_TEMP_MCU` telemetry consumer.
//!
//! The two calibration words are supplied as const-generic parameters. A HAL
//! application reads them from the family's system-memory calibration region
//! (for example with an `embassy-stm32` PAC / device-family crate) and passes
//! them at construction, keeping address knowledge at the HAL boundary:
//!
//! ```ignore
//! let cal1 = /* factory TS_CAL1 half-word read by the family HAL */;
//! let cal2 = /* factory TS_CAL2 half-word read by the family HAL */;
//! let mut sensor = Stm32InternalTemp::<_, CAL1, CAL2>::new(adc_pin);
//! let deg_c = sensor.read_temperature().unwrap();
//! ```
//!
//! The raw ADC sample and calibration words must share the same converter
//! resolution (12-bit on most STM32 families).

use barefoc::traits::adc::AdcPin;
use barefoc::traits::temperature::TemperatureSensor;

/// Temperature at the factory `TS_CAL1` calibration point (deg C).
pub const TS_CAL1_TEMPERATURE_C: f32 = 30.0;
/// Temperature at the factory `TS_CAL2` calibration point (deg C).
pub const TS_CAL2_TEMPERATURE_C: f32 = 110.0;

/// Factory-calibrated STM32 internal temperature sensor.
///
/// `TS_CAL1` and `TS_CAL2` are the raw ADC words programmed into system memory
/// at the 30 °C and 110 °C reference points for the specific silicon batch.
/// They are const parameters so the conversion is monomorphized with no RAM
/// or heap cost.
pub struct Stm32InternalTemp<P: AdcPin, const TS_CAL1: u16, const TS_CAL2: u16> {
    pin: P,
}

impl<P: AdcPin, const TS_CAL1: u16, const TS_CAL2: u16>
    Stm32InternalTemp<P, TS_CAL1, TS_CAL2>
{
    /// Wrap an ADC channel that samples the internal temperature sensor.
    pub const fn new(pin: P) -> Self {
        Self { pin }
    }

    /// Borrow the underlying ADC pin.
    pub fn pin_mut(&mut self) -> &mut P {
        &mut self.pin
    }

    /// Factory calibration words used by this instance `(TS_CAL1, TS_CAL2)`.
    pub const fn calibration_words() -> (u16, u16) {
        (TS_CAL1, TS_CAL2)
    }

    /// Convert a raw internal-sensor sample to degrees Celsius.
    ///
    /// Uses the two-point line through `(TS_CAL1, 30 °C)` and
    /// `(TS_CAL2, 110 °C)`:
    ///
    /// $$T = 30 + (110 - 30)\,\frac{n - TS_{CAL1}}{TS_{CAL2} - TS_{CAL1}}$$
    ///
    /// Returns `f32::NAN` if the calibration words are not strictly increasing
    /// (a corrupt or unprogrammed calibration region).
    pub fn temperature_from_raw(raw: u16) -> f32 {
        if TS_CAL2 <= TS_CAL1 {
            return f32::NAN;
        }
        let numerator = raw as i32 - TS_CAL1 as i32;
        let denominator = TS_CAL2 as i32 - TS_CAL1 as i32;
        TS_CAL1_TEMPERATURE_C
            + (TS_CAL2_TEMPERATURE_C - TS_CAL1_TEMPERATURE_C)
                * (numerator as f32 / denominator as f32)
    }
}

impl<P: AdcPin, const TS_CAL1: u16, const TS_CAL2: u16> TemperatureSensor
    for Stm32InternalTemp<P, TS_CAL1, TS_CAL2>
{
    type Error = ();

    /// Sample the internal channel and convert to degrees Celsius.
    fn read_temperature(&mut self) -> Result<f32, Self::Error> {
        Ok(Self::temperature_from_raw(self.pin.read()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adc::Stm32AdcPinFixed;

    type Sensor = Stm32InternalTemp<Stm32AdcPinFixed, 1650, 2010>;

    #[test]
    fn test_calibration_point_conversion() {
        assert_eq!(Sensor::temperature_from_raw(1650), 30.0);
        assert_eq!(Sensor::temperature_from_raw(2010), 110.0);
        let midpoint = Sensor::temperature_from_raw(1830);
        assert!((midpoint - 70.0).abs() < 1e-6, "got {}", midpoint);
        assert_eq!(Sensor::calibration_words(), (1650, 2010));
    }

    #[test]
    fn test_trait_read_and_corrupt_calibration() {
        let mut sensor = Sensor::new(Stm32AdcPinFixed::new(1650));
        assert_eq!(sensor.read_temperature(), Ok(30.0));
        sensor.pin_mut().set_value(2010);
        assert_eq!(sensor.read_temperature(), Ok(110.0));

        type Corrupt = Stm32InternalTemp<Stm32AdcPinFixed, 2000, 2000>;
        assert!(Corrupt::temperature_from_raw(2000).is_nan());
    }
}
