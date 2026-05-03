# rp2040_pio_stepper
Stepper motor control from the rp2040 for LinuxCNC.

The RP2040 is at `192.168.12.2`; the LinuxCNC host NIC is `192.168.12.1`.

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
