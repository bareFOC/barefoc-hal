//! Configuration builder for STM32 FOC hardware.

use crate::clock::DwtClock;
use crate::driver_6pwm::Stm32BldcDriver6Pwm;
use crate::encoder::Stm32Encoder;
use embassy_stm32::timer::{
    AdvancedInstance4Channel, Ch1, Ch2, Ch3, GeneralInstance4Channel, TimerComplementaryPin,
    TimerPin,
};
use embassy_stm32::Peri;

/// Configuration builder for STM32 FOC hardware components.
#[derive(Clone, Debug)]
pub struct FocConfig {
    pub shunt_resistor: f32,
    pub amp_gain: f32,
    pub dead_time_ns: u32,
    pub pwm_frequency: u32,
    pub voltage_power_supply: f32,
    pub voltage_limit: f32,
    pub vref: f32,
}

impl Default for FocConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl FocConfig {
    /// Create a new FOC hardware configuration with sensible defaults.
    pub fn new() -> Self {
        Self {
            shunt_resistor: 0.01,   // Default 10 mOhm
            amp_gain: 50.0,         // Default 50x gain
            dead_time_ns: 500,      // Default 500 ns
            pwm_frequency: 25_000,  // Default 25 kHz
            voltage_power_supply: 12.0,
            voltage_limit: 12.0,
            vref: 3.3,
        }
    }

    /// Set the shunt resistor value in ohms.
    pub fn shunt_resistor(mut self, value: f32) -> Self {
        self.shunt_resistor = value;
        self
    }

    /// Set the current-sense op-amp gain.
    pub fn amp_gain(mut self, value: f32) -> Self {
        self.amp_gain = value;
        self
    }

    /// Set the PWM dead-time in nanoseconds.
    pub fn dead_time_ns(mut self, value: u32) -> Self {
        self.dead_time_ns = value;
        self
    }

    /// Set the PWM carrier frequency in Hz.
    pub fn pwm_frequency(mut self, value: u32) -> Self {
        self.pwm_frequency = value;
        self
    }

    /// Set the power supply voltage.
    pub fn voltage_power_supply(mut self, value: f32) -> Self {
        self.voltage_power_supply = value;
        self
    }

    /// Set the maximum driver voltage limit.
    pub fn voltage_limit(mut self, value: f32) -> Self {
        self.voltage_limit = value;
        self
    }

    /// Set the ADC analog reference voltage.
    pub fn vref(mut self, value: f32) -> Self {
        self.vref = value;
        self
    }

    /// Initialize a Cortex-M DWT-based microsecond clock.
    pub fn init_clock(&self, core_clock_hz: u32) -> DwtClock {
        DwtClock::new(core_clock_hz)
    }

    /// Build a complementary 6-PWM BLDC driver configured according to this `FocConfig`.
    #[allow(clippy::too_many_arguments)]
    pub fn build_6pwm_driver<'d, T: AdvancedInstance4Channel>(
        &self,
        tim: Peri<'d, T>,
        ch1: Peri<'d, impl TimerPin<T, Ch1>>,
        ch1n: Peri<'d, impl TimerComplementaryPin<T, Ch1>>,
        ch2: Peri<'d, impl TimerPin<T, Ch2>>,
        ch2n: Peri<'d, impl TimerComplementaryPin<T, Ch2>>,
        ch3: Peri<'d, impl TimerPin<T, Ch3>>,
        ch3n: Peri<'d, impl TimerComplementaryPin<T, Ch3>>,
    ) -> Stm32BldcDriver6Pwm<'d, T> {
        Stm32BldcDriver6Pwm::new(
            tim,
            ch1,
            ch1n,
            ch2,
            ch2n,
            ch3,
            ch3n,
            self.pwm_frequency,
            self.dead_time_ns,
        )
        .with_voltage_power_supply(self.voltage_power_supply)
        .with_voltage_limit(self.voltage_limit)
    }

    /// Build a hardware QEI encoder.
    pub fn build_encoder<'d, T: GeneralInstance4Channel>(
        &self,
        tim: Peri<'d, T>,
        ch1: Peri<'d, impl TimerPin<T, Ch1>>,
        ch2: Peri<'d, impl TimerPin<T, Ch2>>,
        ppr: u32,
    ) -> Stm32Encoder<'d, T> {
        Stm32Encoder::new(tim, ch1, ch2, ppr)
    }
}
