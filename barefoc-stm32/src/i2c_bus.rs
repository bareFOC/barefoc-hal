//! # i2c_bus — Runtime STM32 I2C Bus Frequency Control & Recovery
//!
//! Implements [`barefoc::traits::sensor::SensorBusControl`] for STM32 I2C v2
//! peripherals (`STM32H5`, `STM32G4`) so applications can tune `TIMINGR`
//! frequency or recover from stuck `BUSY`/`BERR`/`ARLO` states at runtime
//! without tearing down the peripheral.

use barefoc::traits::sensor::BusConfigError;
#[cfg(all(
    feature = "embassy-stm32",
    any(feature = "_stm32h5", feature = "_stm32g4")
))]
use barefoc::traits::sensor::SensorBusControl;

/// Computed STM32 I2C v2 `TIMINGR` register fields.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct I2cTimingFields {
    pub presc: u8,
    pub scll: u8,
    pub sclh: u8,
    pub sdadel: u8,
    pub scldel: u8,
}

/// Compute STM32 I2C v2 `TIMINGR` fields for a given peripheral kernel clock (`i2cclk_hz`)
/// and target bus clock (`target_hz`).
pub fn compute_timingr_fields(
    i2cclk_hz: u32,
    target_hz: u32,
) -> Result<I2cTimingFields, BusConfigError> {
    if target_hz == 0 || target_hz > i2cclk_hz / 4 {
        return Err(BusConfigError::UnsupportedFrequency);
    }
    let ratio = i2cclk_hz / target_hz;
    if ratio < 4 {
        return Err(BusConfigError::UnsupportedFrequency);
    }
    let presc = ratio.saturating_sub(1) / 512;
    let presc_reg = core::cmp::min(presc, 15) as u8;
    let presc_val = (presc_reg as u32) + 1;
    let half = (ratio / presc_val).saturating_sub(2) / 2;
    let sclh = core::cmp::min(half, 255) as u8;
    let scll = sclh;
    let sdadel = core::cmp::min(
        core::cmp::max((i2cclk_hz / 2_000_000 / presc_val) as u8, 2),
        15,
    );
    let scldel = core::cmp::min(
        core::cmp::max((i2cclk_hz / 500_000 / presc_val).saturating_sub(1) as u8, 4),
        15,
    );

    Ok(I2cTimingFields {
        presc: presc_reg,
        scll,
        sclh,
        sdadel,
        scldel,
    })
}

/// Runtime I2C bus controller for STM32 I2C v2 peripherals (`STM32H5`, `STM32G4`).
#[cfg(all(
    feature = "embassy-stm32",
    any(feature = "_stm32h5", feature = "_stm32g4")
))]
pub struct Stm32I2cBusControl {
    regs: embassy_stm32::pac::i2c::I2c,
    pclk_hz: u32,
    active_hz: u32,
}

#[cfg(all(
    feature = "embassy-stm32",
    any(feature = "_stm32h5", feature = "_stm32g4")
))]
impl Stm32I2cBusControl {
    /// Create a new bus controller for the given PAC `I2c` register block,
    /// kernel clock frequency (`pclk_hz`), and initial active bus frequency (`initial_hz`).
    pub const fn new(
        regs: embassy_stm32::pac::i2c::I2c,
        pclk_hz: u32,
        initial_hz: u32,
    ) -> Self {
        Self {
            regs,
            pclk_hz,
            active_hz: initial_hz,
        }
    }

    /// Return the peripheral kernel clock frequency in Hz.
    #[inline]
    pub const fn pclk_hz(&self) -> u32 {
        self.pclk_hz
    }

    /// Probe candidate frequencies in descending order and select the fastest
    /// frequency for which `probe_fn()` returns `true`.
    ///
    /// Leaves the bus configured at the winning frequency (or `fallback_hz` if
    /// all candidates fail).
    pub fn probe_max_frequency_hz<F>(
        &mut self,
        candidates_hz: &[u32],
        fallback_hz: u32,
        mut probe_fn: F,
    ) -> u32
    where
        F: FnMut(u32) -> bool,
    {
        for &hz in candidates_hz {
            if self.set_bus_frequency_hz(hz).is_ok() && probe_fn(hz) {
                return hz;
            }
        }
        let _ = self.set_bus_frequency_hz(fallback_hz);
        fallback_hz
    }
}

#[cfg(all(
    feature = "embassy-stm32",
    any(feature = "_stm32h5", feature = "_stm32g4")
))]
impl SensorBusControl for Stm32I2cBusControl {
    fn bus_frequency_hz(&self) -> u32 {
        self.active_hz
    }

    fn set_bus_frequency_hz(&mut self, hz: u32) -> Result<u32, BusConfigError> {
        let fields = compute_timingr_fields(self.pclk_hz, hz)?;

        self.regs.cr1().modify(|w| w.set_pe(false));
        // STM32 I2C v2 requires PE=0 for at least 3 APB clock cycles before re-enabling.
        while self.regs.cr1().read().pe() {}

        self.regs.timingr().write(|w| {
            w.set_presc(fields.presc);
            w.set_scll(fields.scll);
            w.set_sclh(fields.sclh);
            w.set_sdadel(fields.sdadel);
            w.set_scldel(fields.scldel);
        });
        self.regs.cr1().modify(|w| w.set_pe(true));

        self.active_hz = hz;
        Ok(hz)
    }

    fn recover_bus(&mut self) -> Result<(), BusConfigError> {
        self.regs.cr1().modify(|w| w.set_pe(false));
        while self.regs.cr1().read().pe() {}

        // Clear latched bus/arbitration/overrun/stop/nack flags.
        self.regs.icr().write(|w| {
            w.set_berrcf(true);
            w.set_arlocf(true);
            w.set_ovrcf(true);
            w.set_stopcf(true);
            w.set_nackcf(true);
        });

        self.regs.cr1().modify(|w| w.set_pe(true));
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compute_timingr_fields_accepts_400k_and_1m_at_64mhz() {
        let f400 = compute_timingr_fields(64_000_000, 400_000).unwrap();
        assert_eq!(f400.presc, 0);
        assert_eq!(f400.sclh, 79);
        assert_eq!(f400.scll, 79);

        let f1m = compute_timingr_fields(64_000_000, 1_000_000).unwrap();
        assert_eq!(f1m.presc, 0);
        assert_eq!(f1m.sclh, 31);
        assert_eq!(f1m.scll, 31);
    }

    #[test]
    fn compute_timingr_fields_rejects_zero_and_overclock() {
        assert_eq!(
            compute_timingr_fields(64_000_000, 0),
            Err(BusConfigError::UnsupportedFrequency)
        );
        assert_eq!(
            compute_timingr_fields(64_000_000, 32_000_000),
            Err(BusConfigError::UnsupportedFrequency)
        );
    }
}
