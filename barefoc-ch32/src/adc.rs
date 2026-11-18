//! CH32 ADC bridge.

#[cfg(feature = "ch32-hal")]
use barefoc::traits::adc::AdcPin;

#[cfg(feature = "ch32-hal")]
use ch32_hal::adc::{Adc, AdcChannel, SampleTime};

/// CH32 ADC pin implementing `barefoc::traits::adc::AdcPin`.
#[cfg(feature = "ch32-hal")]
pub struct Ch32AdcPin<'d, T, C>
where
    T: ch32_hal::adc::Instance,
    C: AdcChannel<T>,
{
    adc: Adc<'d, T>,
    channel: C,
    sample_time: SampleTime,
    max_count: u16,
}

#[cfg(feature = "ch32-hal")]
impl<'d, T, C> Ch32AdcPin<'d, T, C>
where
    T: ch32_hal::adc::Instance,
    C: AdcChannel<T>,
{
    pub fn new(adc: Adc<'d, T>, channel: C, sample_time: SampleTime) -> Self {
        Self {
            adc,
            channel,
            sample_time,
            max_count: 4095,
        }
    }

    pub fn with_max_count(mut self, max_count: u16) -> Self {
        self.max_count = max_count;
        self
    }

    pub fn max_count(&self) -> u16 {
        self.max_count
    }

    pub fn read_voltage(&mut self) -> f32 {
        let raw = self.read();
        crate::util::raw_to_voltage(raw, self.max_count, crate::util::DEFAULT_ADC_VREF)
    }
}

#[cfg(feature = "ch32-hal")]
impl<'d, T, C> AdcPin for Ch32AdcPin<'d, T, C>
where
    T: ch32_hal::adc::Instance,
    C: AdcChannel<T>,
{
    fn read(&mut self) -> u16 {
        #[cfg(feature = "adc-v3")]
        {
            self.adc.convert(&mut self.channel, self.sample_time, ch32_hal::pac::adc::vals::Pga::X1)
        }
        #[cfg(not(feature = "adc-v3"))]
        {
            self.adc.convert(&mut self.channel, self.sample_time)
        }
    }
}

#[cfg(test)]
mod tests {
    use barefoc::traits::adc::AdcPin;

    struct MockAdc {
        value: u16,
    }
    impl AdcPin for MockAdc {
        fn read(&mut self) -> u16 {
            self.value
        }
    }

    #[test]
    fn mock_adc_read() {
        let mut a = MockAdc { value: 123 };
        assert_eq!(a.read(), 123);
    }
}