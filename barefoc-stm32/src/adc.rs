//! # adc — STM32 ADC pin abstraction
//!
//! Provides [`Stm32AdcPin`] satisfying the [`barefoc::traits::adc::AdcPin`] contract
//! for phase current, bus voltage, and analog sensor acquisition on STM32 microcontrollers.

use barefoc::traits::adc::AdcPin;

/// STM32 ADC pin wrapper implementing [`AdcPin`].
///
/// Generic over a reader closure or function `R: FnMut() -> u16`.
/// Defaults to a function pointer `fn() -> u16`.
pub struct Stm32AdcPin<R = fn() -> u16> {
    reader: R,
}

impl<R: FnMut() -> u16> Stm32AdcPin<R> {
    /// Create a new `Stm32AdcPin` from a reading function or closure.
    pub const fn new(reader: R) -> Self {
        Self { reader }
    }

    /// Read the current raw ADC value directly.
    pub fn read_pin(&mut self) -> u16 {
        (self.reader)()
    }

    /// Convert a raw ADC count to millivolts/volts using a given reference voltage.
    pub fn to_voltage(&self, raw: u16, vref: f32, max_count: u16) -> f32 {
        if max_count == 0 {
            0.0
        } else {
            (raw as f32 / max_count as f32) * vref
        }
    }

    /// Convert a raw ADC count to voltage using STM32 factory calibrated VREFINT.
    ///
    /// - `raw`: Channel ADC reading.
    /// - `vref_cal`: Factory calibration value at 3.3V (e.g. from `VREFINT_CAL_ADDR`).
    /// - `raw_vref`: Live ADC reading of internal VREFINT channel.
    pub fn to_calibrated_voltage(&self, raw: u16, vref_cal: u16, raw_vref: u16) -> f32 {
        if raw_vref == 0 {
            0.0
        } else {
            3.3 * (vref_cal as f32 * raw as f32) / (raw_vref as f32 * 4095.0)
        }
    }
}

/// Helper struct for static or fixed value ADC pins.
pub struct Stm32AdcPinFixed {
    value: u16,
}

impl Stm32AdcPinFixed {
    /// Create a new fixed-value ADC pin.
    pub const fn new(value: u16) -> Self {
        Self { value }
    }

    /// Update the value.
    pub fn set_value(&mut self, value: u16) {
        self.value = value;
    }
}

impl AdcPin for Stm32AdcPinFixed {
    fn read(&mut self) -> u16 {
        self.value
    }
}

impl<R: FnMut() -> u16> AdcPin for Stm32AdcPin<R> {
    fn read(&mut self) -> u16 {
        (self.reader)()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stm32_adc_pin_closure() {
        let mut count = 100u16;
        let mut pin = Stm32AdcPin::new(move || {
            let val = count;
            count += 10;
            val
        });

        assert_eq!(pin.read(), 100);
        assert_eq!(pin.read(), 110);
        assert_eq!(pin.read(), 120);
    }

    #[test]
    fn test_stm32_adc_pin_voltage_conversion() {
        let pin = Stm32AdcPinFixed::new(2048);
        let adc = Stm32AdcPin::new(|| 2048);

        // 2048 / 4096 * 3.3V = 1.65V
        let v = adc.to_voltage(2048, 3.3, 4096);
        assert!((v - 1.65).abs() < 1e-3);

        let v_cal = adc.to_calibrated_voltage(2048, 1650, 1650);
        assert!((v_cal - 1.65).abs() < 1e-2);
        assert_eq!(pin.value, 2048);
    }
}
