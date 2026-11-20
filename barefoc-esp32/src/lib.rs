//! # barefoc-esp32
//!
//! Official ESP32 hardware abstraction bridges for [`barefoc`](https://crates.io/crates/barefoc),
//! ported from and inspired by the [Arduino SimpleFOC](https://simplefoc.com) project.
//!
//! Features:
//! - Complementary 6-PWM with hardware dead-time generator and fault trip pin (`Esp32McpwmDriver`)
//!   for ESP32 and ESP32-S3.
//! - Universal 3-PWM fallback with hardware `hpoint` pulse centering (`Esp32LedcDriver`)
//!   for all ESP32 variants including RISC-V ESP32-C3 and ESP32-C6.
//! - Pulse Counter quadrature encoder sensor (`Esp32PcntSensor`) implementing `Sensor`.
//! - Microsecond and millisecond monotonic clocks (`Esp32Clock`, `EmbassyClock`) implementing `Clock`.
//! - ADC reading helpers and current sense scaling (`Esp32AdcPin`, `Esp32CurrentSenseAdc`).

#![cfg_attr(not(test), no_std)]

pub mod adc;
#[cfg(all(
    feature = "bareboot",
    any(
        feature = "esp32",
        feature = "esp32s3",
        feature = "esp32c3",
        feature = "esp32c6"
    )
))]
pub mod bootloader;
pub mod clock;
pub mod ledc;
pub mod mcpwm;
pub mod pcnt;
pub mod uid;

pub use adc::{raw_to_voltage, voltage_to_current, Esp32AdcPin, Esp32CurrentSenseAdc};
#[cfg(all(
    feature = "bareboot",
    any(
        feature = "esp32",
        feature = "esp32s3",
        feature = "esp32c3",
        feature = "esp32c6"
    )
))]
pub use bootloader::{
    before_reset_hook, clear_before_reset_hook, clear_boot_cookie, enter_bareboot,
    request_bootloader_checked, target_from_sram_top, NotPresent,
};
#[cfg(feature = "embassy")]
pub use clock::EmbassyClock;
pub use clock::{calc_dt, ms_to_us, us_to_ms, Esp32Clock};
pub use ledc::{calc_ledc_duty, calc_ledc_hpoint, calc_ledc_max_duty, Esp32LedcDriver};
pub use mcpwm::{
    calculate_mcpwm_period, deadtime_to_ticks, duty_to_compare, voltage_to_duty, Esp32McpwmDriver,
};
pub use pcnt::{calc_pcnt_angle, Esp32PcntSensor};
pub use uid::Esp32Uid;
