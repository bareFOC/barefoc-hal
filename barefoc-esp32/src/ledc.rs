//! # ESP32 LEDC BLDC Driver
//!
//! Universal 3-PWM BLDC driver implementation using the ESP32 LED PWM Controller (LEDC).
//! Available across all ESP32 variants including RISC-V targets (ESP32-C3, ESP32-C6, ESP32-H2)
//! as well as Xtensa targets (ESP32, ESP32-S2, ESP32-S3).
//!
//! Supports phase-centered modulation using the hardware `hpoint` register.
//!
//! Upstream reference:
//! - `source/Arduino-FOC/src/drivers/hardware_specific/esp32/esp32_ledc_mcu.cpp`

use barefoc::traits::driver::{BLDCDriver, DriverType, FOCDriver, FocDriverParams, PhaseState};

/// Default LEDC PWM frequency (25 kHz), matching upstream `_PWM_FREQUENCY`.
pub const DEFAULT_LEDC_PWM_FREQUENCY_HZ: u32 = 25_000;

/// Default PWM bit resolution (10 bits = 1024 levels, 0..1023), matching upstream `_PWM_RES_BIT`.
pub const DEFAULT_LEDC_RESOLUTION_BITS: u8 = 10;

/// Calculate the maximum duty counter value for a given bit resolution: `(1 << bits) - 1`.
#[inline]
pub const fn calc_ledc_max_duty(resolution_bits: u8) -> u32 {
    if resolution_bits == 0 || resolution_bits > 20 {
        1023 // fallback to 10-bit default
    } else {
        (1 << resolution_bits) - 1
    }
}

/// Convert normalized duty cycle `[0.0, 1.0]` to LEDC duty register ticks.
#[inline]
pub fn calc_ledc_duty(duty: f32, max_duty: u32) -> u32 {
    let dc_clamped = duty.clamp(0.0, 1.0);
    (dc_clamped * max_duty as f32) as u32
}

/// Calculate the LEDC hardware `hpoint` tick offset to center the PWM pulse within the period.
///
/// Upstream SimpleFOC calculation:
/// `hpoint = _PWM_RES / 2.0 * (1.0 - dc)`
#[inline]
pub fn calc_ledc_hpoint(duty: f32, max_duty: u32) -> u32 {
    let dc_clamped = duty.clamp(0.0, 1.0);
    ((max_duty as f32 * 0.5_f32) * (1.0_f32 - dc_clamped)) as u32
}

/// Convert commanded phase voltage into a normalized duty cycle in `[0.0, 1.0]`.
#[inline]
pub fn voltage_to_duty(voltage: f32, voltage_limit: f32, voltage_power_supply: f32) -> f32 {
    let limit = if voltage_limit > 0.0 {
        voltage_limit
    } else {
        voltage_power_supply
    };
    let v_clamped = if voltage < 0.0 {
        0.0
    } else if voltage > limit {
        limit
    } else {
        voltage
    };

    if voltage_power_supply > 0.0 {
        let dc = v_clamped / voltage_power_supply;
        if dc > 1.0 {
            1.0
        } else {
            dc
        }
    } else {
        0.0
    }
}

/// Universal 3-PWM BLDC motor driver using ESP32 LEDC peripheral.
///
/// Drives 3 independent channels (A, B, C) with configurable resolution and hardware hpoint
/// centering to minimize harmonic current distortion on budget drivers or RISC-V ESP32s.
pub struct Esp32LedcDriver {
    params: FocDriverParams,
    resolution_bits: u8,
    max_duty: u32,
    phase_states: [PhaseState; 3],
    duties: [f32; 3],
    channel_duties: [u32; 3],
    channel_hpoints: [u32; 3],
}

impl Esp32LedcDriver {
    /// Create a new `Esp32LedcDriver` with 10-bit resolution.
    pub fn new(pwm_frequency_hz: u32) -> Self {
        Self::with_resolution(pwm_frequency_hz, DEFAULT_LEDC_RESOLUTION_BITS)
    }

    /// Create a new `Esp32LedcDriver` with custom bit resolution (e.g. 8 to 14 bits).
    pub fn with_resolution(pwm_frequency_hz: u32, resolution_bits: u8) -> Self {
        let freq = if pwm_frequency_hz > 0 {
            pwm_frequency_hz
        } else {
            DEFAULT_LEDC_PWM_FREQUENCY_HZ
        };
        let max_duty = calc_ledc_max_duty(resolution_bits);

        let mut params = FocDriverParams::new();
        params.pwm_frequency = freq;

        Self {
            params,
            resolution_bits,
            max_duty,
            phase_states: [PhaseState::PHASE_OFF; 3],
            duties: [0.0; 3],
            channel_duties: [0; 3],
            channel_hpoints: [0; 3],
        }
    }

    /// Get configured bit resolution.
    pub fn resolution_bits(&self) -> u8 {
        self.resolution_bits
    }

    /// Get maximum duty register value.
    pub fn max_duty(&self) -> u32 {
        self.max_duty
    }

    /// Get computed duty register values for the 3 channels.
    pub fn channel_duties(&self) -> [u32; 3] {
        self.channel_duties
    }

    /// Get computed hpoint register values for the 3 channels.
    pub fn channel_hpoints(&self) -> [u32; 3] {
        self.channel_hpoints
    }
}

impl FOCDriver for Esp32LedcDriver {
    type Error = core::convert::Infallible;

    fn init(&mut self) -> Result<(), Self::Error> {
        if self.params.voltage_limit <= 0.0 || self.params.voltage_limit > self.params.voltage_power_supply {
            self.params.voltage_limit = self.params.voltage_power_supply;
        }

        self.max_duty = calc_ledc_max_duty(self.resolution_bits);
        self.params.initialized = true;
        Ok(())
    }

    fn enable(&mut self) {
        self.phase_states = [PhaseState::PHASE_ON; 3];
        self.set_pwm(0.0, 0.0, 0.0);
    }

    fn disable(&mut self) {
        self.set_pwm(0.0, 0.0, 0.0);
        self.phase_states = [PhaseState::PHASE_OFF; 3];
    }

    fn driver_type(&self) -> DriverType {
        DriverType::BLDC
    }

    fn params(&self) -> &FocDriverParams {
        &self.params
    }

    fn params_mut(&mut self) -> &mut FocDriverParams {
        &mut self.params
    }
}

impl BLDCDriver for Esp32LedcDriver {
    fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32) {
        let dc_a = voltage_to_duty(ua, self.params.voltage_limit, self.params.voltage_power_supply);
        let dc_b = voltage_to_duty(ub, self.params.voltage_limit, self.params.voltage_power_supply);
        let dc_c = voltage_to_duty(uc, self.params.voltage_limit, self.params.voltage_power_supply);

        self.duties = [dc_a, dc_b, dc_c];

        for i in 0..3 {
            let dc = self.duties[i];
            if self.phase_states[i] == PhaseState::PHASE_OFF {
                self.channel_duties[i] = 0;
                self.channel_hpoints[i] = 0;
            } else {
                self.channel_duties[i] = calc_ledc_duty(dc, self.max_duty);
                self.channel_hpoints[i] = calc_ledc_hpoint(dc, self.max_duty);
            }
        }
    }

    fn set_phase_state(&mut self, sa: PhaseState, sb: PhaseState, sc: PhaseState) {
        self.phase_states = [sa, sb, sc];
        let [dc_a, dc_b, dc_c] = self.duties;
        let v_sup = self.params.voltage_power_supply;
        self.set_pwm(dc_a * v_sup, dc_b * v_sup, dc_c * v_sup);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calc_ledc_max_duty_and_hpoint() {
        assert_eq!(calc_ledc_max_duty(10), 1023);
        assert_eq!(calc_ledc_max_duty(8), 255);
        assert_eq!(calc_ledc_max_duty(12), 4095);

        // At 0% duty, hpoint is max_duty / 2
        let hp_zero = calc_ledc_hpoint(0.0, 1023);
        assert_eq!(hp_zero, 511);

        // At 100% duty, hpoint is 0
        let hp_full = calc_ledc_hpoint(1.0, 1023);
        assert_eq!(hp_full, 0);

        // At 50% duty, hpoint is max_duty / 4
        let hp_half = calc_ledc_hpoint(0.5, 1023);
        assert_eq!(hp_half, 255);
    }

    #[test]
    fn test_esp32_ledc_driver_pwm_and_states() {
        let mut driver = Esp32LedcDriver::new(25_000);
        driver.params_mut().voltage_power_supply = 12.0;
        driver.params_mut().voltage_limit = 12.0;
        assert!(driver.init().is_ok());

        driver.enable();
        assert_eq!(driver.phase_states, [PhaseState::PHASE_ON; 3]);

        // 6V -> 50% duty, 12V -> 100% duty, 0V -> 0% duty
        driver.set_pwm(6.0, 12.0, 0.0);
        assert_eq!(driver.channel_duties()[0], 511);
        assert_eq!(driver.channel_duties()[1], 1023);
        assert_eq!(driver.channel_duties()[2], 0);

        // Turn phase B off
        driver.set_phase_state(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON);
        driver.set_pwm(6.0, 12.0, 6.0);
        assert_eq!(driver.channel_duties()[0], 511);
        assert_eq!(driver.channel_duties()[1], 0); // turned off
        assert_eq!(driver.channel_duties()[2], 511);

        // Disable driver
        driver.disable();
        assert_eq!(driver.channel_duties(), [0, 0, 0]);
        assert_eq!(driver.phase_states, [PhaseState::PHASE_OFF; 3]);
    }
}
