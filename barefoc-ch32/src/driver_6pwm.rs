//! CH32 complementary 6-PWM BLDC driver.

use barefoc::traits::driver::{BLDCDriver, DriverType, FOCDriver, FocDriverParams, PhaseState};
use ch32_hal::time::Hertz;
use ch32_hal::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin};
use ch32_hal::timer::simple_pwm::PwmPin;
use ch32_hal::timer::low_level::CountingMode;
use ch32_hal::timer::{AdvancedInstance, Channel};
use ch32_hal::Peri;

/// Duty conversion helper.
pub fn duty_to_ccr(voltage: f32, voltage_limit: f32, voltage_power_supply: f32, max_duty: u32) -> u32 {
    let v_clamped = voltage.clamp(0.0, voltage_limit);
    let dc = if voltage_power_supply > 0.0 {
        (v_clamped / voltage_power_supply).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (dc * max_duty as f32) as u32
}

pub struct Ch32BldcDriver6Pwm<'d, T: AdvancedInstance> {
    params: FocDriverParams,
    pwm: ComplementaryPwm<'d, T>,
    dead_time_ns: u32,
}

// Constructor for the common (no-remap) case.
impl<'d, T: AdvancedInstance> Ch32BldcDriver6Pwm<'d, T> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        tim: Peri<'d, T>,
        ch1: Peri<'d, impl ch32_hal::timer::Channel1Pin<T, 0>>,
        ch1n: Peri<'d, impl ch32_hal::timer::Channel1ComplementaryPin<T, 0>>,
        ch2: Peri<'d, impl ch32_hal::timer::Channel2Pin<T, 0>>,
        ch2n: Peri<'d, impl ch32_hal::timer::Channel2ComplementaryPin<T, 0>>,
        ch3: Peri<'d, impl ch32_hal::timer::Channel3Pin<T, 0>>,
        ch3n: Peri<'d, impl ch32_hal::timer::Channel3ComplementaryPin<T, 0>>,
        pwm_frequency_hz: u32,
        dead_time_ns: u32,
    ) -> Self {
        let p1 = PwmPin::new_ch1(ch1);
        let p1n = ComplementaryPwmPin::new_ch1(ch1n);
        let p2 = PwmPin::new_ch2(ch2);
        let p2n = ComplementaryPwmPin::new_ch2(ch2n);
        let p3 = PwmPin::new_ch3(ch3);
        let p3n = ComplementaryPwmPin::new_ch3(ch3n);

        let pwm = ComplementaryPwm::new(
            tim,
            Some(p1),
            Some(p1n),
            Some(p2),
            Some(p2n),
            Some(p3),
            Some(p3n),
            None,
            Hertz(pwm_frequency_hz),
            CountingMode::CenterAlignedDownInterrupts,
        );

        let mut driver = Self::from_pwm(pwm, dead_time_ns);
        driver.params.pwm_frequency = pwm_frequency_hz;
        driver
    }
}


impl<'d, T: AdvancedInstance> Ch32BldcDriver6Pwm<'d, T> {
    pub fn from_pwm(pwm: ComplementaryPwm<'d, T>, dead_time_ns: u32) -> Self {
        let mut params = FocDriverParams::new();
        params.pwm_frequency = 25_000;
        params.voltage_power_supply = 12.0;
        params.voltage_limit = 12.0;

        let mut driver = Self {
            params,
            pwm,
            dead_time_ns,
        };
        driver.apply_dead_time();
        driver
    }

    pub fn with_voltage_power_supply(mut self, v_supply: f32) -> Self {
        self.params.voltage_power_supply = v_supply;
        self
    }

    pub fn with_voltage_limit(mut self, v_limit: f32) -> Self {
        self.params.voltage_limit = v_limit;
        self
    }

    pub fn with_pwm_frequency(mut self, freq_hz: u32) -> Self {
        self.params.pwm_frequency = freq_hz;
        self.pwm.set_frequency(Hertz(freq_hz));
        self.apply_dead_time();
        self
    }

    pub fn with_dead_time_ns(mut self, dead_time_ns: u32) -> Self {
        self.dead_time_ns = dead_time_ns;
        self.apply_dead_time();
        self
    }

    pub fn set_dead_time_ns(&mut self, dead_time_ns: u32) {
        self.dead_time_ns = dead_time_ns;
        self.apply_dead_time();
    }

    pub fn dead_time_ns(&self) -> u32 {
        self.dead_time_ns
    }

    pub fn max_duty(&self) -> u32 {
        self.pwm.get_max_duty() as u32
    }

    fn apply_dead_time(&mut self) {
        let max_duty = self.pwm.get_max_duty();
        if max_duty > 0 && self.dead_time_ns > 0 {
            let timer_clock_hz = 2 * (max_duty as u64) * (self.params.pwm_frequency as u64);
            let dt_ticks = ((self.dead_time_ns as u64 * timer_clock_hz) / 1_000_000_000) as u16;
            self.pwm.set_dead_time(dt_ticks);
        }
    }
}

impl<'d, T: AdvancedInstance> FOCDriver for Ch32BldcDriver6Pwm<'d, T> {
    type Error = core::convert::Infallible;

    fn init(&mut self) -> Result<(), Self::Error> {
        if self.params.voltage_limit <= 0.0 || self.params.voltage_limit > self.params.voltage_power_supply {
            self.params.voltage_limit = self.params.voltage_power_supply;
        }
        self.apply_dead_time();
        self.params.initialized = true;
        Ok(())
    }

    fn enable(&mut self) {
        self.set_pwm(0.0, 0.0, 0.0);
        self.pwm.enable(Channel::Ch1);
        self.pwm.enable(Channel::Ch2);
        self.pwm.enable(Channel::Ch3);
    }

    fn disable(&mut self) {
        self.set_pwm(0.0, 0.0, 0.0);
        self.pwm.disable(Channel::Ch1);
        self.pwm.disable(Channel::Ch2);
        self.pwm.disable(Channel::Ch3);
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

impl<'d, T: AdvancedInstance> BLDCDriver for Ch32BldcDriver6Pwm<'d, T> {
    fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32) {
        let max_duty = self.pwm.get_max_duty() as u32;
        let ccr1 = duty_to_ccr(ua, self.params.voltage_limit, self.params.voltage_power_supply, max_duty);
        let ccr2 = duty_to_ccr(ub, self.params.voltage_limit, self.params.voltage_power_supply, max_duty);
        let ccr3 = duty_to_ccr(uc, self.params.voltage_limit, self.params.voltage_power_supply, max_duty);

        self.pwm.set_duty(Channel::Ch1, ccr1.min(max_duty) as u16);
        self.pwm.set_duty(Channel::Ch2, ccr2.min(max_duty) as u16);
        self.pwm.set_duty(Channel::Ch3, ccr3.min(max_duty) as u16);
    }

    fn set_phase_state(&mut self, _sa: PhaseState, _sb: PhaseState, _sc: PhaseState) {
        // Not implemented for complementary driver.
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duty_conversion() {
        let max_duty = 1000u32;
        assert_eq!(duty_to_ccr(0.0, 12.0, 12.0, max_duty), 0);
        assert_eq!(duty_to_ccr(12.0, 12.0, 12.0, max_duty), 1000);
    }
}