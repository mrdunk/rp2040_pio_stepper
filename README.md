# rp2040_pio_stepper

**rp2040_pio_stepper** is an Ethernet-based stepper motor controller for [LinuxCNC](https://linuxcnc.org/), built on the [RP2040](https://www.raspberrypi.com/documentation/microcontrollers/microcontroller-chips.html) microcontroller.

Most hobby CNC projects use GRBL — simple, cheap, but limited: no real-time feedback to the host, no encoder support, and no path to industrial-grade control. Entry-level industrial controllers (Mesa cards and similar) work well but cost significantly more and assume a level of familiarity with LinuxCNC internals. Meanwhile, the parallel port — long the standard interface for LinuxCNC — is increasingly hard to find on modern hardware.

rp2040_pio_stepper sits between these worlds: low-cost, open hardware with a real-time Ethernet link to LinuxCNC, accurate step generation via the RP2040's PIO state machines, and a HAL driver that integrates cleanly with LinuxCNC's existing toolchain. No parallel port required.

A verified starting point is the [W5500-EVB-Pico](https://docs.wiznet.io/Product/Chip/Ethernet/W5500/w5500-evb-pico) — a dev board combining the RP2040 with a W5500 Ethernet chip. Three-axis operation has been tested and confirmed; four-axis should work but has not yet been tested.

The RP2040 is at `192.168.12.2`; the LinuxCNC host NIC is `192.168.12.1`.

# Specifications

| Property | Value |
|----------|-------|
| Joints (stepper axes) | Up to 4 (3 tested) |
| Step rate — theoretical max | 6.65 MHz per joint (PIO ceiling at 133 MHz clock) |
| Step rate — practical max | ~500 kHz per joint (limited by stepper driver minimum pulse width, typically 1–2 µs) |
| Step command modes | Position or velocity, configurable per joint |
| Position feedback | Step counter via second PIO state machine |
| GPIO channels | 32 by default (compile-time limit; see [Advanced GPIO](docs/setup.md#advanced-increasing-gpio-and-i2c-expander-counts)) |
| MCP23017 expanders supported | 4 by default, up to 8 (compile-time limit; see above) |
| Spindle controllers | Up to 4 RS-485 VFD (Huanyang, Fuling, Weiken) |
| Network | UDP/Ethernet, W5500 or W5100S |
| Host connection | Direct point-to-point to LinuxCNC NIC (no switch needed) |

# Docs
- [Setup](docs/setup.md) — build, flash, network tuning, LinuxCNC config
- [Architecture](docs/architecture.md) — component overview, message flow, clock sync, PIO step generation
- [HAL reference](docs/hal_reference.md) — all HAL pins and parameters

# Wishlist
  * RS-485 support.
  * Test with all 4 axis. (Only 3 tested so far.)

# Stretch goals
  * SPI support.
  * SPI attached child RP2040 for more joints.
  * Test with multiple RP2040 on same network segment. Would need some driver work.
