# barefoc-ch32

Official CH32 (WCH RISC-V) hardware abstraction bridges for BareFOC,
built on `ch32-hal`.

Part of the BareFOC project. `#![no_std]` compatible.

## Features

- `ch32-hal` — enable hardware bridges (requires a chip feature on `ch32-hal`).
- `embassy-time` — enable `Ch32Clock` using `embassy-time`.

## Provided bridges

- `Ch32Clock` (feature `embassy-time`)
- `Ch32BldcDriver6Pwm` (feature `ch32-hal`)
- `Ch32AdcPin` (feature `ch32-hal`)

I2C/SPI/SSI magnetic sensors are reused from `barefoc`/`barefoc-encoders`.

## Notes

- No hardware quadrature encoder support; use magnetic sensors.
- Hardware builds require a RISC-V target and nightly/esp toolchain.
EOF
echo done
## SSI / rx-only SPI sensors

CH32 SPI supports rx-only mode via `ch32_hal::spi::Spi::new_blocking_rxonly`,
which is suitable for SSI encoders (e.g. MT6701) where MOSI is unconnected.
Wrap the bus with `embedded-hal-bus::spi::SpiDeviceWithCs` to provide the
`SpiDevice` required by BareFOC SSI drivers.
