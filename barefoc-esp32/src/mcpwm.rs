//! # ESP32 MCPWM BLDC Driver
//!
//! Complementary 6-PWM BLDC driver implementation using the ESP32 Motor Control Pulse Width Modulator
//! (MCPWM) peripheral. Supports center-aligned UP-DOWN counting mode, hardware dead-time generator,
//! and emergency hardware fault/brake trip protection.
//!
//! Target microcontrollers: ESP32, ESP32-S3, ESP32-P4.
//!
//! Upstream reference:
//! - `source/Arduino-FOC/src/drivers/hardware_specific/esp32/esp32_driver_mcpwm.h`
//! - `source/Arduino-FOC/src/drivers/hardware_specific/esp32/esp32_driver_mcpwm.cpp`

use barefoc::traits::driver::{BLDCDriver, DriverType, FOCDriver, FocDriverParams, PhaseState};

/// Default MCPWM clock resolution (160 MHz timebase), matching upstream SimpleFOC `_MCPWM_FREQ`.
pub const DEFAULT_MCPWM_TIMEBASE_HZ: u32 = 160_000_000;

/// Default PWM frequency (25 kHz), matching upstream SimpleFOC `_PWM_FREQUENCY`.
pub const DEFAULT_PWM_FREQUENCY_HZ: u32 = 25_000;

/// Maximum allowed PWM frequency (50 kHz).
pub const MAX_PWM_FREQUENCY_HZ: u32 = 50_000;

/// Calculate the MCPWM period ticks from desired PWM frequency and clock resolution.
///
/// In Center-Aligned (UP-DOWN) counting mode, the counter counts up to `period_ticks`
/// and back down to 0, giving a full symmetric carrier period of `2 * period_ticks`.
#[inline]
pub fn calculate_mcpwm_period(pwm_frequency_hz: u32, resolution_hz: u32) -> u32 {
    let freq = if pwm_frequency_hz == 0 {
        DEFAULT_PWM_FREQUENCY_HZ
    } else {
        pwm_frequency_hz.min(MAX_PWM_FREQUENCY_HZ)
    };
    let res = if resolution_hz == 0 {
        DEFAULT_MCPWM_TIMEBASE_HZ
    } else {
        resolution_hz
    };
    // period ticks for up-down counting mode: resolution / (2 * freq)
    res / (2 * freq)
}

/// Convert a commanded phase voltage into a normalized duty cycle in `[0.0, 1.0]`.
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

/// Convert normalized duty cycle to MCPWM comparator register ticks.
#[inline]
pub fn duty_to_compare(duty: f32, period_ticks: u32) -> u32 {
    let dc_clamped = duty.clamp(0.0, 1.0);
    (dc_clamped * period_ticks as f32) as u32
}

/// Calculate hardware dead-time generator ticks from nanoseconds and clock frequency.
#[inline]
pub fn deadtime_to_ticks(dead_time_ns: u32, clock_freq_hz: u32) -> u16 {
    if dead_time_ns == 0 || clock_freq_hz == 0 {
        return 0;
    }
    ((dead_time_ns as u64 * clock_freq_hz as u64) / 1_000_000_000) as u16
}

/// ESP32 MCPWM complementary 6-PWM BLDC motor driver.
///
/// Manages 3 operator channels (A, B, C) driving complementary high-side and low-side FETs
/// with automatic hardware dead-time generator insertion and fault-protection tripping.
pub struct Esp32McpwmDriver {
    params: FocDriverParams,
    dead_time_ns: u32,
    dead_time_ticks: u16,
    resolution_hz: u32,
    period_ticks: u32,
    fault_pin_enabled: bool,
    fault_tripped: bool,
    phase_states: [PhaseState; 3],
    duties: [f32; 3],
    compare_values: [u32; 3],
}

impl Esp32McpwmDriver {
    /// Create a new `Esp32McpwmDriver` with configured frequency and dead-time.
    pub fn new(pwm_frequency_hz: u32, dead_time_ns: u32) -> Self {
        let freq = if pwm_frequency_hz > 0 {
            pwm_frequency_hz
        } else {
            DEFAULT_PWM_FREQUENCY_HZ
        };
        let period = calculate_mcpwm_period(freq, DEFAULT_MCPWM_TIMEBASE_HZ);
        let dt_ticks = deadtime_to_ticks(dead_time_ns, DEFAULT_MCPWM_TIMEBASE_HZ);

        let mut params = FocDriverParams::new();
        params.pwm_frequency = freq;

        Self {
            params,
            dead_time_ns,
            dead_time_ticks: dt_ticks,
            resolution_hz: DEFAULT_MCPWM_TIMEBASE_HZ,
            period_ticks: period,
            fault_pin_enabled: false,
            fault_tripped: false,
            phase_states: [PhaseState::PHASE_OFF; 3],
            duties: [0.0; 3],
            compare_values: [0; 3],
        }
    }

    /// Enable hardware emergency fault trip input (break pin).
    pub fn with_fault_pin(mut self) -> Self {
        self.fault_pin_enabled = true;
        self
    }

    /// Get configured dead-time in nanoseconds.
    pub fn dead_time_ns(&self) -> u32 {
        self.dead_time_ns
    }

    /// Get configured dead-time in hardware ticks.
    pub fn dead_time_ticks(&self) -> u16 {
        self.dead_time_ticks
    }

    /// Set dead-time duration in nanoseconds.
    pub fn set_dead_time_ns(&mut self, dead_time_ns: u32) {
        self.dead_time_ns = dead_time_ns;
        self.dead_time_ticks = deadtime_to_ticks(dead_time_ns, self.resolution_hz);
    }

    /// Get MCPWM carrier period ticks.
    pub fn period_ticks(&self) -> u32 {
        self.period_ticks
    }

    /// Get current compare values for phases A, B, and C.
    pub fn compare_values(&self) -> [u32; 3] {
        self.compare_values
    }

    /// Check if hardware fault has been tripped.
    pub fn is_fault_tripped(&self) -> bool {
        self.fault_tripped
    }

    /// Trigger emergency hardware fault trip (shuts down all 6 outputs).
    pub fn trip_fault(&mut self) {
        self.fault_tripped = true;
        self.compare_values = [0, 0, 0];
        self.phase_states = [PhaseState::PHASE_OFF; 3];
    }

    /// Reset emergency fault trip state.
    pub fn reset_fault(&mut self) {
        self.fault_tripped = false;
        self.phase_states = [PhaseState::PHASE_ON; 3];
    }
}

impl FOCDriver for Esp32McpwmDriver {
    type Error = core::convert::Infallible;

    fn init(&mut self) -> Result<(), Self::Error> {
        if self.params.voltage_limit <= 0.0 || self.params.voltage_limit > self.params.voltage_power_supply {
            self.params.voltage_limit = self.params.voltage_power_supply;
        }

        self.period_ticks = calculate_mcpwm_period(self.params.pwm_frequency, self.resolution_hz);
        self.dead_time_ticks = deadtime_to_ticks(self.dead_time_ns, self.resolution_hz);
        self.params.initialized = true;
        Ok(())
    }

    fn enable(&mut self) {
        if self.fault_tripped {
            return;
        }
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

impl BLDCDriver for Esp32McpwmDriver {
    fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32) {
        if self.fault_tripped {
            self.compare_values = [0, 0, 0];
            return;
        }

        let dc_a = voltage_to_duty(ua, self.params.voltage_limit, self.params.voltage_power_supply);
        let dc_b = voltage_to_duty(ub, self.params.voltage_limit, self.params.voltage_power_supply);
        let dc_c = voltage_to_duty(uc, self.params.voltage_limit, self.params.voltage_power_supply);

        self.duties = [dc_a, dc_b, dc_c];

        self.compare_values[0] = match self.phase_states[0] {
            PhaseState::PHASE_OFF => 0,
            _ => duty_to_compare(dc_a, self.period_ticks),
        };
        self.compare_values[1] = match self.phase_states[1] {
            PhaseState::PHASE_OFF => 0,
            _ => duty_to_compare(dc_b, self.period_ticks),
        };
        self.compare_values[2] = match self.phase_states[2] {
            PhaseState::PHASE_OFF => 0,
            _ => duty_to_compare(dc_c, self.period_ticks),
        };
    }

    fn set_phase_state(&mut self, sa: PhaseState, sb: PhaseState, sc: PhaseState) {
        self.phase_states = [sa, sb, sc];
        // Re-apply PWM with updated phase states
        let [dc_a, dc_b, dc_c] = self.duties;
        let v_sup = self.params.voltage_power_supply;
        self.set_pwm(dc_a * v_sup, dc_b * v_sup, dc_c * v_sup);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calculate_mcpwm_period() {
        // 160 MHz timebase, 25 kHz -> 160_000_000 / (2 * 25_000) = 3200 ticks
        let period = calculate_mcpwm_period(25_000, 160_000_000);
        assert_eq!(period, 3200);

        // 160 MHz timebase, 20 kHz -> 160_000_000 / (2 * 20_000) = 4000 ticks
        let period_20k = calculate_mcpwm_period(20_000, 160_000_000);
        assert_eq!(period_20k, 4000);
    }

    #[test]
    fn test_deadtime_to_ticks() {
        // 500 ns at 160 MHz: (500 * 160_000_000) / 1_000_000_000 = 80 ticks
        let ticks = deadtime_to_ticks(500, 160_000_000);
        assert_eq!(ticks, 80);

        // 1000 ns at 160 MHz: 160 ticks
        let ticks_1u = deadtime_to_ticks(1000, 160_000_000);
        assert_eq!(ticks_1u, 160);
    }

    #[test]
    fn test_voltage_to_duty_clamping() {
        let v_supply = 12.0;
        let v_limit = 10.0;

        assert_eq!(voltage_to_duty(0.0, v_limit, v_supply), 0.0);
        assert_eq!(voltage_to_duty(6.0, v_limit, v_supply), 0.5);
        assert_eq!(voltage_to_duty(10.0, v_limit, v_supply), 10.0 / 12.0);
        assert_eq!(voltage_to_duty(12.0, v_limit, v_supply), 10.0 / 12.0); // clamped to limit
        assert_eq!(voltage_to_duty(-5.0, v_limit, v_supply), 0.0);
    }

    #[test]
    fn test_esp32_mcpwm_driver_pwm_and_states() {
        let mut driver = Esp32McpwmDriver::new(25_000, 500);
        driver.params_mut().voltage_power_supply = 12.0;
        driver.params_mut().voltage_limit = 12.0;
        assert!(driver.init().is_ok());
        assert!(driver.params().initialized);

        driver.enable();
        assert_eq!(driver.phase_states, [PhaseState::PHASE_ON; 3]);

        // Command 6V, 3V, 0V
        driver.set_pwm(6.0, 3.0, 0.0);
        let period = driver.period_ticks();
        assert_eq!(driver.compare_values()[0], period / 2); // 50% duty
        assert_eq!(driver.compare_values()[1], period / 4); // 25% duty
        assert_eq!(driver.compare_values()[2], 0);          // 0% duty

        // Change phase state to turn off phase C
        driver.set_phase_state(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF);
        driver.set_pwm(6.0, 3.0, 6.0);
        assert_eq!(driver.compare_values()[0], period / 2);
        assert_eq!(driver.compare_values()[1], period / 4);
        assert_eq!(driver.compare_values()[2], 0); // phase C forced off

        // Fault trip
        driver.trip_fault();
        assert!(driver.is_fault_tripped());
        driver.set_pwm(6.0, 6.0, 6.0);
        assert_eq!(driver.compare_values(), [0, 0, 0]);

        // Reset fault
        driver.reset_fault();
        assert!(!driver.is_fault_tripped());
        driver.set_pwm(6.0, 6.0, 6.0);
        assert_eq!(driver.compare_values()[0], period / 2);
    }
}
