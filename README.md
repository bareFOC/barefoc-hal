# barefoc-hal

Hardware abstraction bridges for [barefoc](../barefoc), providing platform-specific
implementations of the `embedded-hal` / BareFOC traits (PWM drivers, encoders,
clocks, ADC current sense, etc.) for supported microcontrollers.

## Sub-crates

| Crate | Target | Backend |
|---|---|---|
| `barefoc-stm32` | STM32F4, STM32G4, STM32H5 | `embassy-stm32` |
| `barefoc-esp32` | ESP32, ESP32-S3, ESP32-C3, ESP32-C6 | `esp-hal` |
| `barefoc-ch32` | WCH CH32V307 (RISC-V) | `ch32-hal` |

## barefoc-stm32

Features:
- Complementary 6-PWM generation with hardware dead-time and center-aligned mode (`Stm32BldcDriver6Pwm`)
- Hardware quadrature encoder decoding (`Stm32Encoder`)
- STM32G4 hardware CORDIC trigonometric acceleration
- STM32G4 internal OPAMP in PGA mode for shunt current sensing
- High-precision Cortex-M DWT cycle-counter microsecond clock (`DwtClock`)
- Optional Katapult bootloader re-entry helper

Enable a target chip via features, e.g. `stm32g431cb`, `stm32f405rg`, `stm32h503kb`.

## barefoc-esp32

Features:
- Complementary 6-PWM with hardware dead-time + fault trip (`Esp32McpwmDriver`) — ESP32/S3
- Universal 3-PWM fallback with `hpoint` pulse centering (`Esp32LedcDriver`) — all variants
- Pulse Counter quadrature encoder sensor (`Esp32PcntSensor`)
- Microsecond/millisecond monotonic clocks (`Esp32Clock`, `EmbassyClock`)
- ADC reading helpers and current-sense scaling (`Esp32AdcPin`, `Esp32CurrentSenseAdc`)

## Usage

Add the relevant sub-crate to your firmware project and enable the target chip feature:

```toml
[dependencies]
barefoc-stm32 = { path = "path/to/barefoc-hal/barefoc-stm32", features = ["stm32g431cb"] }
```

All code is `#![no_std]` and depends on `barefoc` + `embedded-hal` v1.0.

## BareBoot handsfree entry support

All BareBoot client code is built on the portable `drivers/barefoc-boot` crate.
Every new feature is default-off.

| Sub-crate | Feature | Reset primitive | Mailbox | Notes |
|---|---|---|---|---|
| `barefoc-stm32` | `bareboot` (alias `katapult`) | `SCB_AIRCR` SYSRESETREQ | `BootTarget::mailbox_addr()` | Presence-checked entry via `enter_bareboot`/`request_bootloader_checked`; fallback mailbox `sram_top - 8` when no vector table is found. |
| `barefoc-stm32` | `dcache-m7` | (same as above) | (same) | Enables CP15 DCache clean; enable only on Cortex-M7. |
| `barefoc-ch32` | `bareboot` | PFIC `0xE000_E048` system reset | `sram_top - 8` | `csrci mstatus, 8` masks interrupts. |
| `barefoc-esp32` | `bareboot` + chip feature | `esp_hal::system::software_reset()` | `sram_top - 8` | RISC-V masks interrupts; Xtensa gap documented in the module. |

### Documented gaps

- **RP2040 / RP2350**: no `barefoc-rp` HAL sub-crate exists yet, so there is no
  BareBoot adapter for RP parts. The portable `barefoc-boot` client core can be
  used directly with a Cortex-M reset helper; a dedicated HAL crate is deferred
  because its scope is too large for the current change.
- **ESP32 Xtensa interrupt masking**: the pinned esp-hal has no portable
  "mask all interrupts" helper; the module leaves this to the caller's
  driver-disarm hook (see `barefoc-esp32/src/bootloader.rs`).
- **Cortex-M7 DCache**: gated behind the default-off `dcache-m7` feature because
  there is no automatic target cfg; verify on real M7 hardware before enabling.
- **Interrupt context required**: the STM32 I2C target's `0xF0` strobe decoder
  must be serviced from interrupt context (InterruptExecutor or I2C ISR); see
  `bareBoot/docs/handsfree-app-updates.md` section 6.
