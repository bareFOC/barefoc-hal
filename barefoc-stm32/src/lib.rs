//! # barefoc-stm32
//!
//! Unified STM32 hardware abstraction bridges for [`barefoc`](https://crates.io/crates/barefoc)
//! utilizing [`embassy-stm32`](https://crates.io/crates/embassy-stm32) across STM32F4 and STM32G4.
//!
//! Features:
//! - Complementary 6-PWM generation with hardware dead-time and center-aligned mode via `embassy-stm32::timer::complementary_pwm::ComplementaryPwm`.
//! - Hardware Quadrature Encoder decoding via `embassy-stm32::timer::qei::Qei`.
//! - STM32G4 hardware CORDIC trigonometric acceleration via `embassy-stm32::cordic::Cordic`.
//! - STM32G4 internal OPAMP configuration in PGA mode for internal shunt sensing.
//! - High-precision Cortex-M DWT cycle counter microsecond clock.

#![cfg_attr(not(test), no_std)]

#[cfg(feature = "bareboot")]
pub mod bootloader;
#[cfg(feature = "bareboot")]
pub mod katapult;

pub mod adc;
pub mod any_sensor;
pub mod clock;
pub mod encoder;
pub mod i2c_bus;

#[cfg(feature = "embassy-stm32")]
pub mod driver_6pwm;
#[cfg(feature = "embassy-stm32")]
pub mod foc_config;

/// Blocking `embedded_io` bridge over embassy-usb CDC ACM for passthrough.
#[cfg(feature = "usb-cdc-passthrough")]
pub mod usb_cdc;

#[cfg(all(feature = "embassy-stm32", feature = "_stm32g4"))]
pub mod cordic;
#[cfg(all(feature = "embassy-stm32", feature = "_stm32g4"))]
pub mod opamp;

#[cfg(all(feature = "embassy-stm32", feature = "_stm32h5", feature = "multibus"))]
pub mod bus_target_i2c;
#[cfg(all(feature = "embassy-stm32", feature = "_stm32h5", feature = "multibus"))]
pub mod bus_target_i3c;
#[cfg(all(feature = "embassy-stm32", feature = "_stm32h5", feature = "multibus"))]
pub mod bus_target_spi;

/// Factory-calibrated internal temperature sensor; available on every
/// supported STM32 family through the `internal-temp` feature (enabled
/// automatically by the `stm32f4` / `stm32g4` / `stm32h5` family features).
#[cfg(any(
    feature = "internal-temp",
    feature = "_stm32f4",
    feature = "_stm32g4",
    feature = "_stm32h5"
))]
pub mod temperature;

#[cfg(feature = "bareboot")]
#[allow(deprecated)]
pub use katapult::{
    clear_boot_cookie, request_bootloader, CANBOOT_MAGIC_ADDR, FLASH_BOOT_ADDRESS, REQUEST_CANBOOT,
};

#[cfg(feature = "bareboot")]
pub use bootloader::{
    barefoc_failsafe_recovery, before_reset_hook, clear_before_reset_hook, clear_boot_loop_guard,
    default_target, enter_bareboot, enter_bareboot_with_node_id, feed_iwdg, get_active_bus_type,
    get_active_dynamic_node_id, read_soft_reset_mailbox, request_bootloader_checked,
    request_bootloader_checked_with_node_id, set_active_bus_type, set_active_dynamic_node_id,
    NotPresent, SoftResetMailbox, BOOT_LOOP_GUARD_ADDR, BOOT_LOOP_MAGIC, DEFAULT_APP_BASE,
    DEFAULT_FLASH_BASE, DEFAULT_SRAM_TOP, SOFT_RESET_MAILBOX_ADDR,
};

pub use adc::{Stm32AdcPin, Stm32AdcPinFixed};
pub use any_sensor::{AnySensor, AnySensorBuildError, DummySensor, SensorType};
pub use clock::DwtClock;
pub use encoder::Stm32Encoder;
#[cfg(all(
    feature = "embassy-stm32",
    any(feature = "_stm32h5", feature = "_stm32g4")
))]
pub use i2c_bus::Stm32I2cBusControl;
#[cfg(any(
    feature = "internal-temp",
    feature = "_stm32f4",
    feature = "_stm32g4",
    feature = "_stm32h5"
))]
pub use temperature::Stm32InternalTemp;

#[cfg(feature = "embassy-stm32")]
pub use driver_6pwm::Stm32BldcDriver6Pwm;
#[cfg(feature = "embassy-stm32")]
pub use foc_config::FocConfig;

#[cfg(all(feature = "embassy-stm32", feature = "_stm32g4"))]
pub use cordic::Stm32G4Cordic;
#[cfg(all(feature = "embassy-stm32", feature = "_stm32g4"))]
pub use opamp::Stm32G4OpAmp;

pub mod uid;
pub use uid::Stm32Uid;

pub mod stm32f4;
pub mod stm32g4;
pub mod stm32h5;
