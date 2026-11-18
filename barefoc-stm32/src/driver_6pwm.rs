//! Unified STM32 6-PWM BLDC driver implementation using `embassy-stm32`'s `ComplementaryPwm`.
//!
//! Provides identical complementary 6-PWM generation with center-aligned mode, hardware
//! dead-time insertion, and opt-in break input (BKIN) across STM32F4 and STM32G4 advanced
//! timers (TIM1, TIM8, TIM20).
//!
//! Upstream reference:
//! - `source/Arduino-FOC/src/drivers/hardware_specific/stm32/stm32_mcu.cpp`
//! - `source/Arduino-FOC/src/drivers/BLDCDriver6PWM.cpp`

use barefoc::traits::driver::{BLDCDriver, DriverType, FOCDriver, FocDriverParams, PhaseState};
use embassy_stm32::gpio::{AfType, OutputType, Pull};
use embassy_stm32::pac::timer::TimAdv;
use embassy_stm32::pac::timer::vals;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::{
    AdvancedInstance4Channel, BreakInput, BreakInputPin, Ch1, Ch2, Ch3, Channel,
    TimerComplementaryPin, TimerPin,
};
use embassy_stm32::Peri;

/// Calculate the Auto-Reload (ARR) and Prescaler (PSC) for center-aligned PWM mode.
///
/// In Center-Aligned Mode, the counter counts up from 0 to ARR then down to 0,
/// meaning the total PWM period contains `2 * (ARR + 1) * (PSC + 1)` timer clock ticks.
pub fn calculate_arr_psc(timer_clock_hz: u32, pwm_frequency_hz: u32) -> (u16, u32) {
    let freq = if pwm_frequency_hz > 0 { pwm_frequency_hz } else { 25_000 };
    let half_period_ticks = timer_clock_hz / (2 * freq);
    if half_period_ticks > 65535 {
        let p = (half_period_ticks / 65536) as u16;
        let a = (half_period_ticks / (p as u32 + 1)).saturating_sub(1);
        (p, a)
    } else {
        (0u16, half_period_ticks.saturating_sub(1))
    }
}

/// Convert a commanded phase voltage to a timer compare register (CCR) value.
pub fn duty_to_ccr(voltage: f32, voltage_limit: f32, voltage_power_supply: f32, max_duty: u32) -> u32 {
    let v_clamped = voltage.clamp(0.0, voltage_limit);
    let dc = if voltage_power_supply > 0.0 {
        (v_clamped / voltage_power_supply).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (dc * max_duty as f32) as u32
}

/// Unified complementary 6-PWM BLDC driver for STM32 advanced control timers (TIM1, TIM8, TIM20).
///
/// Features:
/// - Complementary outputs with hardware dead-time insertion.
/// - Center-Aligned Mode 1 (`CMS = 01`) for symmetric Space Vector PWM.
/// - TRGO event generation at carrier valley (`RCR = 1`, `MMS = 010`) for injected ADC synchronization.
/// - Opt-in Break Input (`BKIN`) emergency hardware safety trip.
pub struct Stm32BldcDriver6Pwm<'d, T: AdvancedInstance4Channel> {
    params: FocDriverParams,
    pwm: ComplementaryPwm<'d, T>,
    dead_time_ns: u32,
    break_enabled: bool,
}

impl<'d, T: AdvancedInstance4Channel> Stm32BldcDriver6Pwm<'d, T> {
    /// Create a new 6-PWM driver instance from peripheral pins.
    ///
    /// The timer is automatically configured in center-aligned mode (`CountingMode::CenterAlignedDownInterrupts`).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        tim: Peri<'d, T>,
        ch1: Peri<'d, impl TimerPin<T, Ch1>>,
        ch1n: Peri<'d, impl TimerComplementaryPin<T, Ch1>>,
        ch2: Peri<'d, impl TimerPin<T, Ch2>>,
        ch2n: Peri<'d, impl TimerComplementaryPin<T, Ch2>>,
        ch3: Peri<'d, impl TimerPin<T, Ch3>>,
        ch3n: Peri<'d, impl TimerComplementaryPin<T, Ch3>>,
        pwm_frequency_hz: u32,
        dead_time_ns: u32,
    ) -> Self {
        let p1 = PwmPin::new(ch1, OutputType::PushPull);
        let p1n = ComplementaryPwmPin::new(ch1n, OutputType::PushPull);
        let p2 = PwmPin::new(ch2, OutputType::PushPull);
        let p2n = ComplementaryPwmPin::new(ch2n, OutputType::PushPull);
        let p3 = PwmPin::new(ch3, OutputType::PushPull);
        let p3n = ComplementaryPwmPin::new(ch3n, OutputType::PushPull);

        let pwm = ComplementaryPwm::new(
            tim,
            Some(p1),
            Some(p1n),
            Some(p2),
            Some(p2n),
            Some(p3),
            Some(p3n),
            None,
            None,
            Hertz(pwm_frequency_hz),
            CountingMode::CenterAlignedDownInterrupts,
        );

        let mut driver = Self::from_pwm(pwm, dead_time_ns);
        driver.params.pwm_frequency = pwm_frequency_hz;
        driver
    }

    /// Construct a 6-PWM driver from a pre-configured `ComplementaryPwm` instance.
    pub fn from_pwm(pwm: ComplementaryPwm<'d, T>, dead_time_ns: u32) -> Self {
        let mut params = FocDriverParams::new();
        params.pwm_frequency = 25_000;
        params.voltage_power_supply = 12.0;
        params.voltage_limit = 12.0;

        let mut driver = Self {
            params,
            pwm,
            dead_time_ns,
            break_enabled: false,
        };
        driver.apply_dead_time();
        driver
    }

    /// Arm the hardware Break Input (BKIN) pin for emergency output shutdown.
    pub fn with_break_pin<M: BreakInput>(mut self, pin: Peri<'d, impl BreakInputPin<T, M>>) -> Self {
        let af = pin.af_num();
        let mut flex = embassy_stm32::gpio::Flex::new(pin);
        flex.set_as_af_unchecked(af, AfType::input(Pull::None));
        core::mem::forget(flex);
        self.break_enabled = true;
        self
    }

    /// Configure the power supply voltage.
    pub fn with_voltage_power_supply(mut self, v_supply: f32) -> Self {
        self.params.voltage_power_supply = v_supply;
        self
    }

    /// Configure the voltage limit.
    pub fn with_voltage_limit(mut self, v_limit: f32) -> Self {
        self.params.voltage_limit = v_limit;
        self
    }

    /// Configure the PWM frequency in Hz.
    pub fn with_pwm_frequency(mut self, freq_hz: u32) -> Self {
        self.params.pwm_frequency = freq_hz;
        self.pwm.set_frequency(Hertz(freq_hz));
        self.apply_dead_time();
        self
    }

    /// Configure the hardware dead-time in nanoseconds.
    pub fn with_dead_time_ns(mut self, dead_time_ns: u32) -> Self {
        self.dead_time_ns = dead_time_ns;
        self.apply_dead_time();
        self
    }

    /// Update the hardware dead-time setting dynamically.
    pub fn set_dead_time_ns(&mut self, dead_time_ns: u32) {
        self.dead_time_ns = dead_time_ns;
        self.apply_dead_time();
    }

    /// Get the configured dead-time in nanoseconds.
    pub fn dead_time_ns(&self) -> u32 {
        self.dead_time_ns
    }

    /// Get maximum duty counter value.
    pub fn max_duty(&self) -> u32 {
        self.pwm.get_max_duty()
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

impl<'d, T: AdvancedInstance4Channel> FOCDriver for Stm32BldcDriver6Pwm<'d, T> {
    type Error = core::convert::Infallible;

    fn init(&mut self) -> Result<(), Self::Error> {
        if self.params.voltage_limit <= 0.0 || self.params.voltage_limit > self.params.voltage_power_supply {
            self.params.voltage_limit = self.params.voltage_power_supply;
        }

        self.apply_dead_time();

        let regs = unsafe { TimAdv::from_ptr(T::regs()) };

        // 1. Configure Master Mode Selection: Update event as TRGO (MMS = 010)
        regs.cr2().modify(|w| {
            w.set_mms(vals::Mms::Update);
        });

        // 2. Set Repetition counter = 1 so update/TRGO fires strictly at carrier valley (underflow)
        regs.rcr().modify(|w| {
            w.set_rep(1);
        });

        // 3. Configure hardware Break Input if enabled
        if self.break_enabled {
            regs.bdtr().modify(|w| {
                w.set_bke(0, true);
            });
        }

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

impl<'d, T: AdvancedInstance4Channel> BLDCDriver for Stm32BldcDriver6Pwm<'d, T> {
    fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32) {
        let max_duty = self.pwm.get_max_duty();
        let ccr1 = duty_to_ccr(ua, self.params.voltage_limit, self.params.voltage_power_supply, max_duty);
        let ccr2 = duty_to_ccr(ub, self.params.voltage_limit, self.params.voltage_power_supply, max_duty);
        let ccr3 = duty_to_ccr(uc, self.params.voltage_limit, self.params.voltage_power_supply, max_duty);

        self.pwm.set_duty(Channel::Ch1, ccr1.min(max_duty));
        self.pwm.set_duty(Channel::Ch2, ccr2.min(max_duty));
        self.pwm.set_duty(Channel::Ch3, ccr3.min(max_duty));
    }

    fn set_phase_state(&mut self, sa: PhaseState, sb: PhaseState, sc: PhaseState) {
        let regs = unsafe { TimAdv::from_ptr(T::regs()) };
        regs.ccer().modify(|w| {
            match sa {
                PhaseState::PHASE_ON => {
                    w.set_cce(0, true);
                    w.set_ccne(0, true);
                }
                PhaseState::PHASE_OFF => {
                    w.set_cce(0, false);
                    w.set_ccne(0, false);
                }
                PhaseState::PHASE_HI => {
                    w.set_cce(0, true);
                    w.set_ccne(0, false);
                }
                PhaseState::PHASE_LO => {
                    w.set_cce(0, false);
                    w.set_ccne(0, true);
                }
            }
            match sb {
                PhaseState::PHASE_ON => {
                    w.set_cce(1, true);
                    w.set_ccne(1, true);
                }
                PhaseState::PHASE_OFF => {
                    w.set_cce(1, false);
                    w.set_ccne(1, false);
                }
                PhaseState::PHASE_HI => {
                    w.set_cce(1, true);
                    w.set_ccne(1, false);
                }
                PhaseState::PHASE_LO => {
                    w.set_cce(1, false);
                    w.set_ccne(1, true);
                }
            }
            match sc {
                PhaseState::PHASE_ON => {
                    w.set_cce(2, true);
                    w.set_ccne(2, true);
                }
                PhaseState::PHASE_OFF => {
                    w.set_cce(2, false);
                    w.set_ccne(2, false);
                }
                PhaseState::PHASE_HI => {
                    w.set_cce(2, true);
                    w.set_ccne(2, false);
                }
                PhaseState::PHASE_LO => {
                    w.set_cce(2, false);
                    w.set_ccne(2, true);
                }
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calculate_arr_psc_standard_frequencies() {
        let (psc, arr) = calculate_arr_psc(168_000_000, 25_000);
        assert_eq!(psc, 0);
        assert_eq!(arr, 3359);

        let (psc, arr) = calculate_arr_psc(84_000_000, 20_000);
        assert_eq!(psc, 0);
        assert_eq!(arr, 2099);
    }

    #[test]
    fn test_duty_to_ccr_proportions() {
        let max_duty = 3359;
        let v_supply = 12.0;
        let v_limit = 12.0;

        assert_eq!(duty_to_ccr(0.0, v_limit, v_supply, max_duty), 0);
        assert_eq!(duty_to_ccr(6.0, v_limit, v_supply, max_duty), 1679);
        assert_eq!(duty_to_ccr(12.0, v_limit, v_supply, max_duty), 3359);
        assert_eq!(duty_to_ccr(15.0, v_limit, v_supply, max_duty), 3359);
        assert_eq!(duty_to_ccr(-2.0, v_limit, v_supply, max_duty), 0);
    }
}
