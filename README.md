# barefoc-hal

Hardware abstraction bridge crates for the [BareFOC](https://github.com/bareFOC/bareFOC) ecosystem.

This workspace provides platform-specific implementations of BareFOC and `embedded-hal` traits (complementary PWM generation, hardware encoder decoding, monotonic clocks, and ADC current sensing) across supported microcontroller architectures.

---

## Workspace Structure

```
barefoc-hal/
├── barefoc-stm32/    # STM32 platform support (F4, G4, H5) via embassy-stm32
├── barefoc-esp32/    # ESP32 platform support (ESP32, S3, C3, C6) via esp-hal
└── barefoc-ch32/     # WCH CH32 RISC-V platform support via ch32-hal
```

---

## Sub-crates & Capabilities

### 1. `barefoc-stm32`
Built on `embassy-stm32`:
- **PWM Driver**: Complementary 6-PWM generation with hardware dead-time and break input (`BKIN`) protection (`Stm32BldcDriver6Pwm`).
- **Encoders**: Hardware quadrature decoding using advanced timer encoder mode (`Stm32Encoder`).
- **Hardware Acceleration**: STM32G4 hardware CORDIC integration for trigonometric operations (`Stm32G4Cordic`) and internal OPAMP programmable gain amplifier setup (`Stm32G4OpAmp`).
- **Clock**: High-resolution monotonic microsecond clock based on the ARM Cortex-M DWT cycle counter (`DwtClock`).
- **Bus Transports**: Hardware I2C and SPI target adapters implementing `barefoc-multibus::TargetTransport` for the STM32H5 series.

### 2. `barefoc-esp32`
Built on `esp-hal`:
- **PWM Drivers**:
  - Complementary 6-PWM using the MCPWM peripheral with hardware dead-time and fault trip (`Esp32McpwmDriver`) on ESP32 / ESP32-S3.
  - Universal 3-PWM fallback using the LEDC peripheral with `hpoint` center-alignment (`Esp32LedcDriver`).
- **Encoders**: Hardware pulse counter quadrature decoding (`Esp32PcntSensor`).
- **Clock**: Monotonic microsecond clock options (`Esp32Clock`, `EmbassyClock`).
- **Current Sensing**: ADC sampling helpers and current-sense scaling adapters.

### 3. `barefoc-ch32`
Built on `ch32-hal` for WCH RISC-V microcontrollers:
- Complementary 6-PWM generation with hardware dead-time insertion.
- System timer monotonic clocking.
- Phase current measurement adapters.

---

## Bootloader Soft-Reset & Entry

Hardware reset coordination integrates with Katapult and BareBoot bootloaders using mailbox cookies provided by `barefoc::boot`:

| Sub-crate | Feature Flag | Mechanism | Target Mailbox |
|---|---|---|---|
| `barefoc-stm32` | `bareboot` / `katapult` | ARM `SCB_AIRCR` SYSRESETREQ | `BootTarget::mailbox_addr()` (default `sram_top - 8`) |
| `barefoc-stm32` | `dcache-m7` | Cache maintenance before reset | ARM Cortex-M7 data cache clean |
| `barefoc-esp32` | `bareboot` | `esp_hal::system::software_reset()` | Top of SRAM mailbox |
| `barefoc-ch32` | `bareboot` | PFIC system reset (`0xE000_E048`) | Top of SRAM mailbox |

---

## Usage Example

Add the appropriate platform crate to your firmware manifest and select the target chip feature:

```toml
[dependencies]
barefoc = { version = "0.1", default-features = false }
barefoc-stm32 = { path = "path/to/barefoc-hal/barefoc-stm32", features = ["stm32g431cb"] }
```

---

## Testing

Verify compilation across target platform feature gates:

```bash
cargo check -p barefoc-stm32 --features stm32g4
cargo check -p barefoc-stm32 --features "bareboot,stm32h5"
```

---

## License

This project is licensed under the MIT License. See individual crate directories for license files.
