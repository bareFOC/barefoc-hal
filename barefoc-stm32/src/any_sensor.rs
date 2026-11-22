//! # any_sensor — multi-interface dynamic enum dispatcher
//!
//! Provides the zero-heap, `#![no_std]` [`AnySensor`] enum dispatcher for runtime-configurable
//! sensor selection in BareFOC actuators.
//!
//! Actuator boards may feature fixed onboard magnetic sensors (e.g. MT6701 on SPI or AS5600 on I2C),
//! expansion headers for external optical/magnetic quadrature encoders via timer QEI, Hall sensors,
//! or virtual angle testing modes.
//!
//! By wrapping all interface variants into an enum that implements [`Sensor<C>`], actuator firmware
//! can be flashed once at manufacturing time and dynamically switch interfaces at runtime driven
//! by the ControlTable `REG_MOTORx_SENSOR_TYPE` register without recompiling or reflashing.

use barefoc::common::time::Clock;
use barefoc::sensors::hall::HallSensor;
use barefoc::sensors::magnetic_i2c::{MagneticSensorI2C, AS5048_I2C, AS5600_I2C};
use barefoc::sensors::magnetic_spi::{MagneticSensorSPI, AS5047_SPI, MT6701_SPI};
use barefoc::traits::sensor::{Sensor, SensorBase};

use crate::clock::DwtClock;
use crate::encoder::Stm32Encoder;

#[cfg(feature = "embassy-stm32")]
use embassy_stm32::timer::GeneralInstance4Channel;
#[cfg(not(feature = "embassy-stm32"))]
use crate::encoder::GeneralInstance4Channel;

/// Virtual dummy sensor for open-loop diagnostics, testing, and sensorless operation.
#[derive(Clone, Debug, Default)]
pub struct DummySensor {
    base: SensorBase,
    angle: f32,
}

impl DummySensor {
    /// Create a new dummy sensor with 0.0 rad initial angle.
    pub const fn new() -> Self {
        Self {
            base: SensorBase::new(),
            angle: 0.0,
        }
    }

    /// Create a dummy sensor with a preset angle in `[0, 2π)`.
    pub const fn with_angle(angle: f32) -> Self {
        Self {
            base: SensorBase::new(),
            angle,
        }
    }

    /// Update the virtual mechanical angle.
    pub fn set_angle(&mut self, angle: f32) {
        self.angle = angle;
    }

    /// Get the current virtual mechanical angle.
    pub fn angle(&self) -> f32 {
        self.angle
    }
}

impl<C: Clock> Sensor<C> for DummySensor {
    #[inline]
    fn base(&self) -> &SensorBase {
        &self.base
    }

    #[inline]
    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    #[inline]
    fn get_sensor_angle(&self) -> f32 {
        self.angle
    }
}

/// Numerical sensor type IDs matching `ControlTable`'s `REG_MOTORx_SENSOR_TYPE` register.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum SensorType {
    /// 0: Board default / onboard sensor selected by target BSP
    DefaultOnboard = 0,
    /// 1: AS5600 12-bit I2C sensor (default addr 0x36)
    As5600 = 1,
    /// 2: AS5048B 14-bit I2C sensor (default addr 0x40)
    As5048b = 2,
    /// 3: MT6701 14-bit SSI/SPI sensor
    Mt6701 = 3,
    /// 4: AS5047P or MA730 14-bit SPI sensor
    As5047p = 4,
    /// 5: Quadrature Encoder Interface via hardware timer
    EncoderQei = 5,
    /// 6: 3-phase Hall effect sensors
    Hall = 6,
    /// 7: Virtual dummy sensor for open-loop diagnostics
    Virtual = 7,
    /// 8: TMAG5273 3D magnetic angle sensor via I2C
    Tmag5273 = 8,
}

impl TryFrom<u8> for SensorType {
    type Error = u8;

    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::DefaultOnboard),
            1 => Ok(Self::As5600),
            2 => Ok(Self::As5048b),
            3 => Ok(Self::Mt6701),
            4 => Ok(Self::As5047p),
            5 => Ok(Self::EncoderQei),
            6 => Ok(Self::Hall),
            7 => Ok(Self::Virtual),
            8 => Ok(Self::Tmag5273),
            other => Err(other),
        }
    }
}

/// Error returned when constructing an `AnySensor` from a `ControlTable` sensor type ID.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnySensorBuildError {
    /// Required I2C peripheral was not provided for the requested sensor type.
    MissingI2cPeripheral,
    /// Required SPI peripheral was not provided for the requested sensor type.
    MissingSpiPeripheral,
    /// Required timer QEI encoder peripheral was not provided for the requested sensor type.
    MissingEncoderPeripheral,
    /// Required Hall sensor was not provided for the requested sensor type.
    MissingHallPeripheral,
    /// Unrecognized sensor type ID (not in 0..=7).
    InvalidSensorType(u8),
}

/// Unified zero-heap dynamic sensor enum dispatcher.
///
/// Wraps I2C, SPI, timer QEI encoder, Hall, and virtual sensors to allow runtime selection
/// without memory reallocation or generic monomorphization recompilation.
pub enum AnySensor<
    I2C: embedded_hal::i2c::I2c,
    SPI: embedded_hal::spi::SpiDevice<u16>,
    QEI: GeneralInstance4Channel,
    C: Clock = DwtClock,
> {
    /// I2C magnetic sensor (AS5600, AS5048B, TMAG5273, etc.)
    I2c(MagneticSensorI2C<C, I2C>),
    /// SPI magnetic sensor (MT6701, AS5047P, MA730, etc.)
    Spi(MagneticSensorSPI<C, SPI>),
    /// Optical/magnetic quadrature encoder via hardware timer QEI
    Encoder(Stm32Encoder<'static, QEI>),
    /// 3-phase digital Hall effect position sensor
    Hall(HallSensor<C>),
    /// Virtual sensor for open-loop testing and diagnostics
    Virtual(DummySensor),
    /// TMAG5273 3D magnetic angle sensor via I2C
    #[cfg(feature = "tmag")]
    Tmag5273(tmag_rs::barefoc::Tmag5273Sensor<I2C>),
    /// Safe unconfigured fallback returning 0.0 rad
    Unconfigured(SensorBase),
}

impl<
    I2C: embedded_hal::i2c::I2c,
    SPI: embedded_hal::spi::SpiDevice<u16>,
    QEI: GeneralInstance4Channel,
    C: Clock,
> Default for AnySensor<I2C, SPI, QEI, C> {
    fn default() -> Self {
        Self::unconfigured()
    }
}

impl<
    I2C: embedded_hal::i2c::I2c,
    SPI: embedded_hal::spi::SpiDevice<u16>,
    QEI: GeneralInstance4Channel,
    C: Clock,
> Sensor<C> for AnySensor<I2C, SPI, QEI, C> {
    #[inline]
    fn get_sensor_angle(&self) -> f32 {
        match self {
            Self::I2c(s) => s.get_sensor_angle(),
            Self::Spi(s) => s.get_sensor_angle(),
            Self::Encoder(s) => <Stm32Encoder<'static, QEI> as Sensor<C>>::get_sensor_angle(s),
            Self::Hall(s) => s.get_sensor_angle(),
            Self::Virtual(s) => <DummySensor as Sensor<C>>::get_sensor_angle(s),
            Self::Unconfigured(_) => 0.0,
            #[cfg(feature = "tmag")]
            Self::Tmag5273(s) => <tmag_rs::barefoc::Tmag5273Sensor<I2C> as Sensor<C>>::get_sensor_angle(s),
        }
    }

    #[inline]
    fn base(&self) -> &SensorBase {
        match self {
            Self::I2c(s) => s.base(),
            Self::Spi(s) => s.base(),
            Self::Encoder(s) => <Stm32Encoder<'static, QEI> as Sensor<C>>::base(s),
            Self::Hall(s) => s.base(),
            Self::Virtual(s) => <DummySensor as Sensor<C>>::base(s),
            Self::Unconfigured(b) => b,
            #[cfg(feature = "tmag")]
            Self::Tmag5273(s) => <tmag_rs::barefoc::Tmag5273Sensor<I2C> as Sensor<C>>::base(s),
        }
    }

    #[inline]
    fn base_mut(&mut self) -> &mut SensorBase {
        match self {
            Self::I2c(s) => s.base_mut(),
            Self::Spi(s) => s.base_mut(),
            Self::Encoder(s) => <Stm32Encoder<'static, QEI> as Sensor<C>>::base_mut(s),
            Self::Hall(s) => s.base_mut(),
            Self::Virtual(s) => <DummySensor as Sensor<C>>::base_mut(s),
            Self::Unconfigured(b) => b,
            #[cfg(feature = "tmag")]
            Self::Tmag5273(s) => <tmag_rs::barefoc::Tmag5273Sensor<I2C> as Sensor<C>>::base_mut(s),
        }
    }

    #[inline]
    fn init(&mut self, clock: &C) {
        match self {
            Self::I2c(s) => s.init(clock),
            Self::Spi(s) => s.init(clock),
            Self::Encoder(s) => <Stm32Encoder<'static, QEI> as Sensor<C>>::init(s, clock),
            Self::Hall(s) => <HallSensor<C> as Sensor<C>>::init(s, clock),
            Self::Virtual(s) => <DummySensor as Sensor<C>>::init(s, clock),
            Self::Unconfigured(_) => {},
            #[cfg(feature = "tmag")]
            Self::Tmag5273(s) => <tmag_rs::barefoc::Tmag5273Sensor<I2C> as Sensor<C>>::init(s, clock),
        }
    }

    #[inline]
    fn update(&mut self, clock: &C) {
        match self {
            Self::I2c(s) => s.update(clock),
            Self::Spi(s) => s.update(clock),
            Self::Encoder(s) => <Stm32Encoder<'static, QEI> as Sensor<C>>::update(s, clock),
            Self::Hall(s) => s.update(clock),
            Self::Virtual(s) => <DummySensor as Sensor<C>>::update(s, clock),
            Self::Unconfigured(_) => {},
            #[cfg(feature = "tmag")]
            Self::Tmag5273(s) => <tmag_rs::barefoc::Tmag5273Sensor<I2C> as Sensor<C>>::update(s, clock),
        }
    }

    #[inline]
    fn get_mechanical_angle(&self) -> f32 {
        match self {
            Self::I2c(s) => s.get_mechanical_angle(),
            Self::Spi(s) => s.get_mechanical_angle(),
            Self::Encoder(s) => <Stm32Encoder<'static, QEI> as Sensor<C>>::get_mechanical_angle(s),
            Self::Hall(s) => s.get_mechanical_angle(),
            Self::Virtual(s) => <DummySensor as Sensor<C>>::get_mechanical_angle(s),
            Self::Unconfigured(_) => 0.0,
            #[cfg(feature = "tmag")]
            Self::Tmag5273(s) => <tmag_rs::barefoc::Tmag5273Sensor<I2C> as Sensor<C>>::get_mechanical_angle(s),
        }
    }

    #[inline]
    fn get_angle(&self) -> f32 {
        match self {
            Self::I2c(s) => s.get_angle(),
            Self::Spi(s) => s.get_angle(),
            Self::Encoder(s) => <Stm32Encoder<'static, QEI> as Sensor<C>>::get_angle(s),
            Self::Hall(s) => s.get_angle(),
            Self::Virtual(s) => <DummySensor as Sensor<C>>::get_angle(s),
            Self::Unconfigured(_) => 0.0,
            #[cfg(feature = "tmag")]
            Self::Tmag5273(s) => <tmag_rs::barefoc::Tmag5273Sensor<I2C> as Sensor<C>>::get_angle(s),
        }
    }

    #[inline]
    fn get_precise_angle(&self) -> f64 {
        match self {
            Self::I2c(s) => s.get_precise_angle(),
            Self::Spi(s) => s.get_precise_angle(),
            Self::Encoder(s) => <Stm32Encoder<'static, QEI> as Sensor<C>>::get_precise_angle(s),
            Self::Hall(s) => s.get_precise_angle(),
            Self::Virtual(s) => <DummySensor as Sensor<C>>::get_precise_angle(s),
            Self::Unconfigured(_) => 0.0,
            #[cfg(feature = "tmag")]
            Self::Tmag5273(s) => <tmag_rs::barefoc::Tmag5273Sensor<I2C> as Sensor<C>>::get_precise_angle(s),
        }
    }

    #[inline]
    fn get_velocity(&mut self) -> f32 {
        match self {
            Self::I2c(s) => s.get_velocity(),
            Self::Spi(s) => s.get_velocity(),
            Self::Encoder(s) => <Stm32Encoder<'static, QEI> as Sensor<C>>::get_velocity(s),
            Self::Hall(s) => s.get_velocity(),
            Self::Virtual(s) => <DummySensor as Sensor<C>>::get_velocity(s),
            Self::Unconfigured(_) => 0.0,
            #[cfg(feature = "tmag")]
            Self::Tmag5273(s) => <tmag_rs::barefoc::Tmag5273Sensor<I2C> as Sensor<C>>::get_velocity(s),
        }
    }

    #[inline]
    fn get_full_rotations(&self) -> i32 {
        match self {
            Self::I2c(s) => s.get_full_rotations(),
            Self::Spi(s) => s.get_full_rotations(),
            Self::Encoder(s) => <Stm32Encoder<'static, QEI> as Sensor<C>>::get_full_rotations(s),
            Self::Hall(s) => s.get_full_rotations(),
            Self::Virtual(s) => <DummySensor as Sensor<C>>::get_full_rotations(s),
            Self::Unconfigured(b) => b.full_rotations,
            #[cfg(feature = "tmag")]
            Self::Tmag5273(s) => <tmag_rs::barefoc::Tmag5273Sensor<I2C> as Sensor<C>>::get_full_rotations(s),
        }
    }

    #[inline]
    fn needs_search(&self) -> i32 {
        match self {
            Self::I2c(s) => s.needs_search(),
            Self::Spi(s) => s.needs_search(),
            Self::Encoder(s) => <Stm32Encoder<'static, QEI> as Sensor<C>>::needs_search(s),
            Self::Hall(s) => s.needs_search(),
            Self::Virtual(s) => <DummySensor as Sensor<C>>::needs_search(s),
            Self::Unconfigured(_) => 0,
            #[cfg(feature = "tmag")]
            Self::Tmag5273(s) => <tmag_rs::barefoc::Tmag5273Sensor<I2C> as Sensor<C>>::needs_search(s),
        }
    }
}

impl<
    I2C: embedded_hal::i2c::I2c,
    SPI: embedded_hal::spi::SpiDevice<u16>,
    QEI: GeneralInstance4Channel,
    C: Clock,
> AnySensor<I2C, SPI, QEI, C> {
    /// Create an unconfigured sensor fallback returning 0.0 rad.
    pub const fn unconfigured() -> Self {
        Self::Unconfigured(SensorBase::new())
    }

    /// Construct a virtual dummy sensor with initial angle 0.0.
    pub const fn new_virtual() -> Self {
        Self::Virtual(DummySensor::new())
    }

    /// Construct a virtual dummy sensor with a preset angle.
    pub const fn new_virtual_with_angle(angle: f32) -> Self {
        Self::Virtual(DummySensor::with_angle(angle))
    }

    /// Construct an I2C sensor variant.
    pub fn new_i2c(sensor: MagneticSensorI2C<C, I2C>) -> Self {
        Self::I2c(sensor)
    }

    /// Construct an SPI sensor variant.
    pub fn new_spi(sensor: MagneticSensorSPI<C, SPI>) -> Self {
        Self::Spi(sensor)
    }

    /// Construct an encoder variant.
    pub fn new_encoder(encoder: Stm32Encoder<'static, QEI>) -> Self {
        Self::Encoder(encoder)
    }

    /// Construct a Hall sensor variant.
    pub fn new_hall(hall: HallSensor<C>) -> Self {
        Self::Hall(hall)
    }

    /// Helper constructor for AS5600 I2C magnetic sensor (address 0x36).
    pub fn from_i2c_as5600(i2c: I2C) -> Self {
        Self::I2c(MagneticSensorI2C::as5600(i2c))
    }

    /// Helper constructor for AS5048B I2C magnetic sensor (address 0x40).
    pub fn from_i2c_as5048b(i2c: I2C) -> Self {
        Self::I2c(MagneticSensorI2C::as5048b(i2c))
    }

    /// Helper constructor for MT6701 SPI magnetic sensor.
    pub fn from_spi_mt6701(spi: SPI) -> Self {
        Self::Spi(MagneticSensorSPI::mt6701(spi))
    }

    /// Helper constructor for AS5047P SPI magnetic sensor.
    pub fn from_spi_as5047p(spi: SPI) -> Self {
        Self::Spi(MagneticSensorSPI::as5047p(spi))
    }

    /// Helper constructor for hardware QEI encoder.
    pub fn from_encoder(encoder: Stm32Encoder<'static, QEI>) -> Self {
        Self::Encoder(encoder)
    }

    /// Helper constructor for 3-phase Hall effect sensor.
    pub fn from_hall(hall: HallSensor<C>) -> Self {
        Self::Hall(hall)
    }

    /// Helper constructor for TMAG5273 I2C sensor adapter.
    #[cfg(feature = "tmag")]
    pub fn new_tmag5273(sensor: tmag_rs::barefoc::Tmag5273Sensor<I2C>) -> Self {
        Self::Tmag5273(sensor)
    }

    /// Helper constructor for TMAG5273 from a configured Tmag5273 driver.
    #[cfg(feature = "tmag")]
    pub fn from_tmag5273(driver: tmag_rs::Tmag5273<I2C>) -> Self {
        Self::Tmag5273(tmag_rs::barefoc::Tmag5273Sensor::new(driver))
    }

    /// Helper constructor for TMAG5273 from an I2C bus and default candidate address 0x35.
    #[cfg(feature = "tmag")]
    pub fn from_i2c_tmag5273(i2c: I2C) -> Self {
        Self::from_tmag5273(tmag_rs::Tmag5273::new_uninit(i2c, 0x35))
    }

    /// Borrow the TMAG5273 sensor if active.
    #[cfg(feature = "tmag")]
    pub fn as_tmag5273(&self) -> Option<&tmag_rs::barefoc::Tmag5273Sensor<I2C>> {
        match self {
            Self::Tmag5273(s) => Some(s),
            _ => None,
        }
    }

    /// Mutably borrow the TMAG5273 sensor if active.
    #[cfg(feature = "tmag")]
    pub fn as_tmag5273_mut(&mut self) -> Option<&mut tmag_rs::barefoc::Tmag5273Sensor<I2C>> {
        match self {
            Self::Tmag5273(s) => Some(s),
            _ => None,
        }
    }

    /// Switch the active variant in place at runtime (zero-heap).
    pub fn switch_variant(&mut self, new_variant: Self) {
        *self = new_variant;
    }

    /// Switch in place to virtual dummy sensor.
    pub fn switch_to_virtual(&mut self, angle: f32) {
        *self = Self::new_virtual_with_angle(angle);
    }

    /// Switch in place to unconfigured safe fallback.
    pub fn switch_to_unconfigured(&mut self) {
        *self = Self::unconfigured();
    }

    /// Returns true if this sensor instance is configured (not Unconfigured).
    pub fn is_configured(&self) -> bool {
        !matches!(self, Self::Unconfigured(_))
    }

    /// Returns the active SensorType corresponding to REG_MOTORx_SENSOR_TYPE if mapped.
    pub fn sensor_type(&self) -> Option<SensorType> {
        match self {
            Self::I2c(s) => {
                if s.conf == AS5600_I2C {
                    Some(SensorType::As5600)
                } else if s.conf == AS5048_I2C {
                    Some(SensorType::As5048b)
                } else {
                    Some(SensorType::DefaultOnboard)
                }
            }
            Self::Spi(s) => {
                if s.conf == MT6701_SPI {
                    Some(SensorType::Mt6701)
                } else if s.conf == AS5047_SPI {
                    Some(SensorType::As5047p)
                } else {
                    Some(SensorType::DefaultOnboard)
                }
            }
            Self::Encoder(_) => Some(SensorType::EncoderQei),
            Self::Hall(_) => Some(SensorType::Hall),
            Self::Virtual(_) => Some(SensorType::Virtual),
            Self::Unconfigured(_) => None,
            #[cfg(feature = "tmag")]
            Self::Tmag5273(_) => Some(SensorType::Tmag5273),
        }
    }

    /// Borrow the virtual dummy sensor if active.
    pub fn as_virtual(&self) -> Option<&DummySensor> {
        match self {
            Self::Virtual(s) => Some(s),
            _ => None,
        }
    }

    /// Mutably borrow the virtual dummy sensor if active.
    pub fn as_virtual_mut(&mut self) -> Option<&mut DummySensor> {
        match self {
            Self::Virtual(s) => Some(s),
            _ => None,
        }
    }

    /// Borrow the I2C sensor if active.
    pub fn as_i2c(&self) -> Option<&MagneticSensorI2C<C, I2C>> {
        match self {
            Self::I2c(s) => Some(s),
            _ => None,
        }
    }

    /// Borrow the SPI sensor if active.
    pub fn as_spi(&self) -> Option<&MagneticSensorSPI<C, SPI>> {
        match self {
            Self::Spi(s) => Some(s),
            _ => None,
        }
    }

    /// Borrow the QEI encoder if active.
    pub fn as_encoder(&self) -> Option<&Stm32Encoder<'static, QEI>> {
        match self {
            Self::Encoder(s) => Some(s),
            _ => None,
        }
    }

    /// Borrow the Hall sensor if active.
    pub fn as_hall(&self) -> Option<&HallSensor<C>> {
        match self {
            Self::Hall(s) => Some(s),
            _ => None,
        }
    }

    /// Factory builder linking `ControlTable`'s `REG_MOTORx_SENSOR_TYPE` register value (0..7)
    /// to an `AnySensor` variant.
    pub fn build_from_type(
        sensor_type: u8,
        i2c: Option<I2C>,
        spi: Option<SPI>,
        encoder: Option<Stm32Encoder<'static, QEI>>,
        hall: Option<HallSensor<C>>,
    ) -> Result<Self, AnySensorBuildError> {
        match sensor_type {
            0 => {
                if let Some(i) = i2c {
                    Ok(Self::from_i2c_as5600(i))
                } else if let Some(s) = spi {
                    Ok(Self::from_spi_mt6701(s))
                } else if let Some(e) = encoder {
                    Ok(Self::from_encoder(e))
                } else if let Some(h) = hall {
                    Ok(Self::from_hall(h))
                } else {
                    Ok(Self::unconfigured())
                }
            }
            1 => {
                let i = i2c.ok_or(AnySensorBuildError::MissingI2cPeripheral)?;
                Ok(Self::from_i2c_as5600(i))
            }
            2 => {
                let i = i2c.ok_or(AnySensorBuildError::MissingI2cPeripheral)?;
                Ok(Self::from_i2c_as5048b(i))
            }
            3 => {
                let s = spi.ok_or(AnySensorBuildError::MissingSpiPeripheral)?;
                Ok(Self::from_spi_mt6701(s))
            }
            4 => {
                let s = spi.ok_or(AnySensorBuildError::MissingSpiPeripheral)?;
                Ok(Self::from_spi_as5047p(s))
            }
            5 => {
                let e = encoder.ok_or(AnySensorBuildError::MissingEncoderPeripheral)?;
                Ok(Self::from_encoder(e))
            }
            6 => {
                let h = hall.ok_or(AnySensorBuildError::MissingHallPeripheral)?;
                Ok(Self::from_hall(h))
            }
            7 => Ok(Self::new_virtual()),
            #[cfg(feature = "tmag")]
            8 => {
                let i = i2c.ok_or(AnySensorBuildError::MissingI2cPeripheral)?;
                Ok(Self::from_i2c_tmag5273(i))
            }
            other => Err(AnySensorBuildError::InvalidSensorType(other)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mock clock for deterministic host tests.
    #[derive(Debug, Default)]
    struct TestClock {
        now: core::cell::Cell<u32>,
    }

    impl TestClock {
        fn new() -> Self {
            Self {
                now: core::cell::Cell::new(0),
            }
        }
        fn set_us(&self, us: u32) {
            self.now.set(us);
        }
        fn advance_us(&self, us: u32) {
            self.now.set(self.now.get().wrapping_add(us));
        }
    }

    impl Clock for TestClock {
        fn now_us(&self) -> u32 {
            self.now.get()
        }
    }

    /// Mock I2C device implementing embedded-hal v1.0.
    #[derive(Clone, Copy)]
    struct MockI2c {
        angle_word: u16,
    }

    impl embedded_hal::i2c::ErrorType for MockI2c {
        type Error = core::convert::Infallible;
    }

    impl embedded_hal::i2c::I2c for MockI2c {
        fn transaction(
            &mut self,
            _address: u8,
            operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            for op in operations {
                if let embedded_hal::i2c::Operation::Read(buf) = op {
                    if buf.len() >= 2 {
                        buf[0] = ((self.angle_word >> 8) & 0xFF) as u8;
                        buf[1] = (self.angle_word & 0xFF) as u8;
                    }
                }
            }
            Ok(())
        }
    }

    /// Mock SPI device implementing embedded-hal v1.0.
    struct MockSpi {
        angle_word: u16,
    }

    impl embedded_hal::spi::ErrorType for MockSpi {
        type Error = core::convert::Infallible;
    }

    impl embedded_hal::spi::SpiDevice<u16> for MockSpi {
        fn transaction(
            &mut self,
            operations: &mut [embedded_hal::spi::Operation<'_, u16>],
        ) -> Result<(), Self::Error> {
            for op in operations {
                match op {
                    embedded_hal::spi::Operation::Read(read_buf) => {
                        if !read_buf.is_empty() {
                            read_buf[0] = self.angle_word;
                        }
                    }
                    embedded_hal::spi::Operation::Transfer(read_buf, _write_buf) => {
                        if !read_buf.is_empty() {
                            read_buf[0] = self.angle_word;
                        }
                    }
                    _ => {}
                }
            }
            Ok(())
        }
    }

    #[cfg(feature = "embassy-stm32")]
    type TestQei = embassy_stm32::peripherals::TIM2;

    #[cfg(not(feature = "embassy-stm32"))]
    type TestQei = ();

    type TestAnySensor = AnySensor<MockI2c, MockSpi, TestQei, TestClock>;

    #[test]
    fn test_unconfigured_defaults() {
        let clock = TestClock::new();
        let mut sensor = TestAnySensor::unconfigured();

        assert!(!sensor.is_configured());
        assert_eq!(sensor.sensor_type(), None);
        assert_eq!(sensor.get_sensor_angle(), 0.0);
        assert_eq!(sensor.get_mechanical_angle(), 0.0);
        assert_eq!(sensor.get_angle(), 0.0);
        assert_eq!(sensor.get_velocity(), 0.0);
        assert_eq!(sensor.get_full_rotations(), 0);
        assert_eq!(sensor.needs_search(), 0);

        // Update/init on unconfigured must be safe no-op
        sensor.init(&clock);
        sensor.update(&clock);
        assert_eq!(sensor.get_sensor_angle(), 0.0);
    }

    #[test]
    fn test_virtual_sensor_delegation_and_wrap() {
        let clock = TestClock::new();
        clock.set_us(100_000);

        let mut sensor = TestAnySensor::new_virtual_with_angle(1.5);
        assert!(sensor.is_configured());
        assert_eq!(sensor.sensor_type(), Some(SensorType::Virtual));
        assert!((sensor.get_sensor_angle() - 1.5).abs() < 1e-4);

        sensor.init(&clock);

        // Advance time and change angle to 2.5
        clock.advance_us(20_000);
        sensor.as_virtual_mut().unwrap().set_angle(2.5);
        sensor.update(&clock);

        assert!((sensor.get_sensor_angle() - 2.5).abs() < 1e-4);
        assert!((sensor.get_mechanical_angle() - 2.5).abs() < 1e-4);
        assert!((sensor.get_angle() - 2.5).abs() < 1e-4);
        assert_eq!(sensor.get_full_rotations(), 0);

        // Simulate full rotation forward wrap (angle from near 2pi to near 0)
        // 2.5 rad -> 6.0 rad
        clock.advance_us(20_000);
        sensor.as_virtual_mut().unwrap().set_angle(6.0);
        sensor.update(&clock);

        // 6.0 rad -> 0.2 rad (jump across 2pi wrap: delta = 0.2 - 6.0 = -5.8 < -0.8*2pi)
        // When wrapping positive direction, full_rotations increments
        clock.advance_us(20_000);
        sensor.as_virtual_mut().unwrap().set_angle(0.2);
        sensor.update(&clock);

        assert_eq!(sensor.get_full_rotations(), 1);
        let expected_total_angle = barefoc::math::_2PI + 0.2;
        assert!((sensor.get_angle() - expected_total_angle).abs() < 1e-3);
    }

    #[test]
    fn test_i2c_sensor_delegation() {
        // AS5600 12-bit: 1024 / 4096 = 0.25 rev = pi/2 rad
        let mock_i2c = MockI2c { angle_word: 1024 };
        let sensor = TestAnySensor::from_i2c_as5600(mock_i2c);

        assert!(sensor.is_configured());
        assert_eq!(sensor.sensor_type(), Some(SensorType::As5600));

        let angle = sensor.get_sensor_angle();
        assert!((angle - barefoc::math::_PI_2).abs() < 1e-3);
    }

    #[test]
    fn test_spi_sensor_delegation() {
        // AS5047P 14-bit: 4096 / 16384 = 0.25 rev = pi/2 rad
        let mock_spi = MockSpi { angle_word: 4096 };
        let sensor = TestAnySensor::from_spi_as5047p(mock_spi);

        assert!(sensor.is_configured());
        assert_eq!(sensor.sensor_type(), Some(SensorType::As5047p));

        let angle = sensor.get_sensor_angle();
        assert!((angle - barefoc::math::_PI_2).abs() < 1e-3);
    }

    #[test]
    fn test_encoder_sensor_delegation() {
        // 1000 CPR, count 250 -> 0.25 rev = pi/2 rad
        let encoder = Stm32Encoder::new_mock(250, 1000);
        let sensor = TestAnySensor::from_encoder(encoder);

        assert!(sensor.is_configured());
        assert_eq!(sensor.sensor_type(), Some(SensorType::EncoderQei));

        let angle = sensor.get_sensor_angle();
        assert!((angle - barefoc::math::_PI_2).abs() < 1e-3);
    }

    #[test]
    fn test_hall_sensor_delegation() {
        let hall = HallSensor::new(1, 2, 3, 1);
        let sensor = TestAnySensor::from_hall(hall);

        assert!(sensor.is_configured());
        assert_eq!(sensor.sensor_type(), Some(SensorType::Hall));
    }

    #[test]
    fn test_runtime_switching_without_heap_allocation() {
        let mut sensor = TestAnySensor::unconfigured();
        assert_eq!(sensor.sensor_type(), None);

        // Switch to virtual
        sensor.switch_to_virtual(1.0);
        assert_eq!(sensor.sensor_type(), Some(SensorType::Virtual));
        assert!((sensor.get_sensor_angle() - 1.0).abs() < 1e-4);

        // Switch to AS5600 I2C
        sensor.switch_variant(TestAnySensor::from_i2c_as5600(MockI2c { angle_word: 2048 }));
        assert_eq!(sensor.sensor_type(), Some(SensorType::As5600));
        assert!((sensor.get_sensor_angle() - barefoc::math::_PI).abs() < 1e-3);

        // Switch to AS5047P SPI
        sensor.switch_variant(TestAnySensor::from_spi_as5047p(MockSpi { angle_word: 8192 }));
        assert_eq!(sensor.sensor_type(), Some(SensorType::As5047p));
        assert!((sensor.get_sensor_angle() - barefoc::math::_PI).abs() < 1e-3);

        // Switch to Encoder QEI
        sensor.switch_variant(TestAnySensor::from_encoder(Stm32Encoder::new_mock(500, 1000)));
        assert_eq!(sensor.sensor_type(), Some(SensorType::EncoderQei));
        assert!((sensor.get_sensor_angle() - barefoc::math::_PI).abs() < 1e-3);

        // Switch to Unconfigured
        sensor.switch_to_unconfigured();
        assert_eq!(sensor.sensor_type(), None);
        assert_eq!(sensor.get_sensor_angle(), 0.0);

        // Size check: sized statically on stack, zero heap allocation
        let size = core::mem::size_of::<TestAnySensor>();
        assert!(size > 0 && size < 128);
    }

    #[test]
    fn test_build_from_type_matrix() {
        let i2c = MockI2c { angle_word: 1024 };
        let spi = MockSpi { angle_word: 4096 };
        let encoder = Stm32Encoder::new_mock(250, 1000);
        let hall = HallSensor::new(1, 2, 3, 1);

        // Test 1: AS5600
        let s1 = TestAnySensor::build_from_type(1, Some(i2c), None, None, None).unwrap();
        assert_eq!(s1.sensor_type(), Some(SensorType::As5600));

        // Test 3: MT6701
        let s3 = TestAnySensor::build_from_type(3, None, Some(spi), None, None).unwrap();
        assert_eq!(s3.sensor_type(), Some(SensorType::Mt6701));

        // Test 5: Encoder QEI
        let s5 = TestAnySensor::build_from_type(5, None, None, Some(encoder), None).unwrap();
        assert_eq!(s5.sensor_type(), Some(SensorType::EncoderQei));

        // Test 6: Hall
        let s6 = TestAnySensor::build_from_type(6, None, None, None, Some(hall)).unwrap();
        assert_eq!(s6.sensor_type(), Some(SensorType::Hall));

        // Test 7: Virtual
        let s7 = TestAnySensor::build_from_type(7, None, None, None, None).unwrap();
        assert_eq!(s7.sensor_type(), Some(SensorType::Virtual));

        // Test 8: TMAG5273
        #[cfg(feature = "tmag")]
        {
            let s8 = TestAnySensor::build_from_type(8, Some(i2c), None, None, None).unwrap();
            assert_eq!(s8.sensor_type(), Some(SensorType::Tmag5273));
        }

        // Missing peripheral errors
        let err1 = TestAnySensor::build_from_type(1, None, None, None, None);
        assert_eq!(err1.err(), Some(AnySensorBuildError::MissingI2cPeripheral));

        let err3 = TestAnySensor::build_from_type(3, None, None, None, None);
        assert_eq!(err3.err(), Some(AnySensorBuildError::MissingSpiPeripheral));

        let err5 = TestAnySensor::build_from_type(5, None, None, None, None);
        assert_eq!(err5.err(), Some(AnySensorBuildError::MissingEncoderPeripheral));

        let err6 = TestAnySensor::build_from_type(6, None, None, None, None);
        assert_eq!(err6.err(), Some(AnySensorBuildError::MissingHallPeripheral));

        // Invalid sensor type
        let err99 = TestAnySensor::build_from_type(99, None, None, None, None);
        assert_eq!(err99.err(), Some(AnySensorBuildError::InvalidSensorType(99)));
    }
}
