//! STM32G4 internal Operational Amplifier (OPAMP) configuration.
//!
//! Provides support for configuring STM32G4 internal OPAMPs in Programmable Gain Amplifier (PGA)
//! mode with outputs directly routed into internal ADC channels for zero-external-component
//! current sensing (e.g. on ST B-G431B-ESC1).
//!
//! Upstream reference:
//! - STM32G4 Reference Manual RM0440 (OPAMP)
//! - ST B-G431B-ESC1 Discovery kit user manual UM2516

use embassy_stm32::opamp::{
    Instance, NonInvertingPin, OpAmp, OpAmpGain, OpAmpInternalOutput, OpAmpSpeed,
};
use embassy_stm32::Peri;

/// STM32G4 internal OPAMP wrapper configured in PGA mode for internal ADC sensing.
pub struct Stm32G4OpAmp<'d, T: Instance> {
    opamp: OpAmp<'d, T>,
}

impl<'d, T: Instance> Stm32G4OpAmp<'d, T> {
    /// Create a new OPAMP driver in high-speed mode.
    pub fn new(opamp: Peri<'d, T>) -> Self {
        Self {
            opamp: OpAmp::new(opamp, OpAmpSpeed::HighSpeed),
        }
    }

    /// Configure the OPAMP in PGA mode with internal connection directly to ADC.
    ///
    /// The input pin is configured for analog mode, and the output is routed
    /// internally to the corresponding ADC channel without requiring an external output pin.
    pub fn buffer_internal(
        &mut self,
        pin: Peri<'_, impl NonInvertingPin<T> + embassy_stm32::gpio::Pin>,
        gain: OpAmpGain,
    ) -> OpAmpInternalOutput<'_, T> {
        self.opamp.pga_int(pin, gain)
    }

    /// Access the underlying `embassy_stm32::opamp::OpAmp` driver.
    pub fn inner_mut(&mut self) -> &mut OpAmp<'d, T> {
        &mut self.opamp
    }
}
