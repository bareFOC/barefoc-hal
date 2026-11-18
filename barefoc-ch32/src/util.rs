//! Conversion helpers for CH32 ADC current sensing.

/// Default ADC max count for 12-bit CH32 ADCs.
pub const DEFAULT_ADC_MAX_COUNT: u16 = 4095;
/// Default ADC reference voltage.
pub const DEFAULT_ADC_VREF: f32 = 3.3;

/// Convert raw ADC counts to voltage.
#[inline]
pub fn raw_to_voltage(raw: u16, max_count: u16, vref: f32) -> f32 {
    let max = if max_count == 0 {
        DEFAULT_ADC_MAX_COUNT
    } else {
        max_count
    };
    let clamped = raw.min(max);
    (clamped as f32 / max as f32) * vref
}

/// Convert measured voltage to current.
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_to_voltage_basic() {
        assert!((raw_to_voltage(0, 4095, 3.3) - 0.0).abs() < 1e-6);
        assert!((raw_to_voltage(4095, 4095, 3.3) - 3.3).abs() < 1e-6);
    }

    #[test]
    fn voltage_to_current_basic() {
        let i = voltage_to_current(1.65, 1.65, 50.0, 0.01);
        assert!((i - 0.0).abs() < 1e-6);
    }
}