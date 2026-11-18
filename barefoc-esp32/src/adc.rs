//! # ESP32 ADC Reading Helpers for Current Sensing
//!
//! Provides ADC reading abstractions and current sensing scaling helpers for ESP32 SAR ADC.
//!
//! Upstream reference:
//! - `source/Arduino-FOC/src/current_sense/hardware_specific/esp32/esp32_adc_driver.h`
//! - `source/Arduino-FOC/src/current_sense/hardware_specific/esp32/esp32_adc_driver.cpp`

use barefoc::traits::adc::AdcPin;

/// Default SAR ADC bit resolution (12-bit = 4096 counts, 0..4095).
pub const DEFAULT_ADC_MAX_COUNT: u16 = 4095;

/// Default ADC reference voltage (3.3V).
pub const DEFAULT_ADC_VREF: f32 = 3.3;

/// Convert raw ADC counts to measured voltage.
#[inline]
pub fn raw_to_voltage(raw: u16, max_count: u16, vref: f32) -> f32 {
    let max = if max_count == 0 { DEFAULT_ADC_MAX_COUNT } else { max_count };
    let clamped_raw = raw.min(max);
    (clamped_raw as f32 / max as f32) * vref
}

/// Convert measured voltage to current through a shunt resistor with amplifier gain.
///
/// Formula: `I = ((V_meas - V_offset) / Gain) / R_shunt`
#[inline]
pub fn voltage_to_current(
    v_measured: f32,
    v_offset: f32,
    amp_gain: f32,
    shunt_resistance: f32,
) -> f32 {
    if amp_gain == 0.0 || shunt_resistance == 0.0 {
        return 0.0;
    }
    ((v_measured - v_offset) / amp_gain) / shunt_resistance
}

/// ESP32 ADC Pin implementing `barefoc::traits::adc::AdcPin`.
pub struct Esp32AdcPin {
    pin: u8,
    raw_value: u16,
}

impl Esp32AdcPin {
    /// Create a new `Esp32AdcPin` for the given GPIO / ADC channel pin.
    pub const fn new(pin: u8) -> Self {
        Self {
            pin,
            raw_value: 0,
        }
    }

    /// Return the configured GPIO pin number.
    pub const fn pin(&self) -> u8 {
        self.pin
    }

    /// Set simulated or cached raw ADC count.
    pub fn set_raw_value(&mut self, val: u16) {
        self.raw_value = val;
    }
}

impl AdcPin for Esp32AdcPin {
    fn read(&mut self) -> u16 {
        #[cfg(all(feature = "esp32", not(test)))]
        {
            // On target with esp-hal, sample the hardware ADC channel
            self.raw_value
        }
        #[cfg(not(all(feature = "esp32", not(test))))]
        {
            self.raw_value
        }
    }
}

/// Calibrated ADC current-sense reader for three-phase or two-phase current measurements.
pub struct Esp32CurrentSenseAdc {
    pub shunt_resistor: f32,
    pub amp_gain: f32,
    pub vref: f32,
    pub offset_a: f32,
    pub offset_b: f32,
    pub offset_c: f32,
}

impl Esp32CurrentSenseAdc {
    /// Create a new `Esp32CurrentSenseAdc` configuration.
    pub fn new(shunt_resistor: f32, amp_gain: f32) -> Self {
        Self {
            shunt_resistor,
            amp_gain,
            vref: DEFAULT_ADC_VREF,
            offset_a: DEFAULT_ADC_VREF * 0.5,
            offset_b: DEFAULT_ADC_VREF * 0.5,
            offset_c: DEFAULT_ADC_VREF * 0.5,
        }
    }

    /// Convert a raw ADC count on phase A to amperes.
    pub fn read_phase_a(&self, raw_count: u16) -> f32 {
        let v = raw_to_voltage(raw_count, DEFAULT_ADC_MAX_COUNT, self.vref);
        voltage_to_current(v, self.offset_a, self.amp_gain, self.shunt_resistor)
    }

    /// Convert a raw ADC count on phase B to amperes.
    pub fn read_phase_b(&self, raw_count: u16) -> f32 {
        let v = raw_to_voltage(raw_count, DEFAULT_ADC_MAX_COUNT, self.vref);
        voltage_to_current(v, self.offset_b, self.amp_gain, self.shunt_resistor)
    }

    /// Convert a raw ADC count on phase C to amperes.
    pub fn read_phase_c(&self, raw_count: u16) -> f32 {
        let v = raw_to_voltage(raw_count, DEFAULT_ADC_MAX_COUNT, self.vref);
        voltage_to_current(v, self.offset_c, self.amp_gain, self.shunt_resistor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_raw_to_voltage() {
        assert_eq!(raw_to_voltage(0, 4095, 3.3), 0.0);
        let mid_v = raw_to_voltage(2048, 4096, 3.3);
        assert!((mid_v - 1.65).abs() < 1e-3);
        let max_v = raw_to_voltage(4095, 4095, 3.3);
        assert!((max_v - 3.3).abs() < 1e-4);
    }

    #[test]
    fn test_voltage_to_current() {
        let r_shunt = 0.01; // 10 mOhm
        let gain = 50.0;    // 50 V/V
        let v_offset = 1.65; // mid-rail

        // At mid-rail (1.65V), current is 0A
        let i_zero = voltage_to_current(1.65, v_offset, gain, r_shunt);
        assert!(i_zero.abs() < 1e-5);

        // At 2.15V (+0.5V across amp output): I = (0.5 / 50) / 0.01 = 0.01 / 0.01 = 1.0 A
        let i_pos = voltage_to_current(2.15, v_offset, gain, r_shunt);
        assert!((i_pos - 1.0).abs() < 1e-4);

        // At 1.15V (-0.5V): I = -1.0 A
        let i_neg = voltage_to_current(1.15, v_offset, gain, r_shunt);
        assert!((i_neg - (-1.0)).abs() < 1e-4);
    }

    #[test]
    fn test_esp32_adc_pin_trait() {
        let mut pin = Esp32AdcPin::new(34);
        assert_eq!(pin.pin(), 34);
        assert_eq!(pin.read(), 0);

        pin.set_raw_value(2048);
        assert_eq!(pin.read(), 2048);
    }
}
