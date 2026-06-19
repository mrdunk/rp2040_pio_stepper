# rp2040_pio_stepper

**rp2040_pio_stepper** is an Ethernet-based stepper motor controller for [LinuxCNC](https://linuxcnc.org/), built on the [RP2040](https://www.raspberrypi.com/documentation/microcontrollers/microcontroller-chips.html) or [RP2350](https://www.raspberrypi.com/documentation/microcontrollers/microcontroller-chips.html) microcontroller.

Most hobby CNC projects use GRBL — simple, cheap, but limited: no real-time feedback to the host, no encoder support, and no path to industrial-grade control. Entry-level industrial controllers (Mesa cards and similar) work well but cost significantly more and assume a level of familiarity with LinuxCNC internals. Meanwhile, the parallel port — long the standard interface for LinuxCNC — is increasingly hard to find on modern hardware.

rp2040_pio_stepper sits between these worlds: low-cost, open hardware with a real-time Ethernet link to LinuxCNC, accurate step generation via the RP2040's PIO state machines, and a HAL driver that integrates cleanly with LinuxCNC's existing toolchain. No parallel port required.

The recommended board for new users is the [W6100-EVB-Pico2](https://docs.wiznet.io/Product/Chip/Ethernet/W6100/w6100-evb-pico2) (RP2350 + W6100, IPv4/IPv6). Hardware-validated boards:

| Board | MCU | Ethernet | Status |
|-------|-----|----------|--------|
| [W6100-EVB-Pico2](https://docs.wiznet.io/Product/Chip/Ethernet/W6100/w6100-evb-pico2) _(recommended)_ | RP2350 | W6100 | Validated |
| [W6100-EVB-Pico](https://docs.wiznet.io/Product/Chip/Ethernet/W6100/w6100-evb-pico) | RP2040 | W6100 | Validated |
| [W5500-EVB-Pico2](https://docs.wiznet.io/Product/Chip/Ethernet/W5500/w5500-evb-pico2) | RP2350 | W5500 | Validated |
| [W5500-EVB-Pico](https://docs.wiznet.io/Product/Chip/Ethernet/W5500/w5500-evb-pico) | RP2040 | W5500 | Validated |
| W5100S-EVB-Pico | RP2040 | W5100S | Validated |
| W6300-EVB-Pico2 | RP2350 | W6300 | WIP ([#36](https://github.com/mrdunk/rp2040_pio_stepper/issues/36)) |
| [W55RP20-EVB-Pico](https://docs.wiznet.io/Product/Chip/MCU/W55RP20/w55rp20-evb-pico) | RP2040 | W5500 | WIP |
| [W55RP20-Arduino](https://docs.wiznet.io/Product/Chip/MCU/W55RP20/w55rp20-arduino) | RP2040 | W5500 | WIP |

See [Setup](docs/setup.md) to build the firmware, install the driver, and configure LinuxCNC. Working HAL and INI configs for 3, 4, 6, and 8-axis machines are in `config/`.

# Specifications

| Property | Value |
|----------|-------|
| Joints | 4 (current); 6 on RP2350 ([#53](https://github.com/mrdunk/rp2040_pio_stepper/issues/53)); 8/12 with SM sharing ([#48](https://github.com/mrdunk/rp2040_pio_stepper/issues/48)) |
| Step rate — theoretical max | ~380 kHz per joint (STEP HIGH pulse is a fixed 2.53 µs; minimum step period is 2.63 µs) |
| Step rate — practical max | ~131 kHz per joint (both HIGH and LOW meet TB6600/DM860 2.5 µs minimums) |
| STEP pulse width | Fixed 2.53 µs at all step rates (independent of velocity) |
| Step command modes | Position or velocity, configurable per joint |
| Position feedback | Step counter via second PIO state machine |
| GPIO channels | 32 by default (compile-time limit; see [Advanced GPIO](docs/setup.md#advanced-increasing-gpio-and-i2c-expander-counts)) |
| MCP23017 expanders supported | 4 by default, up to 8 (compile-time limit; see above) |
| Spindle controllers | 1 RS-485 VFD (Huanyang, Fuling, Weiken); multiple planned |
| MCU | RP2040 or RP2350 |
| Network | UDP/Ethernet, W5500, W5100S, W6100, or W6300 |
| Host connection | Direct point-to-point to LinuxCNC NIC (no switch needed) |

# Docs
- [Setup](docs/setup.md) — build, flash, network tuning, LinuxCNC config
- [Architecture](docs/architecture.md) — component overview, message flow, clock sync, PIO step generation
- [HAL reference](docs/hal_reference.md) — all HAL pins and parameters

