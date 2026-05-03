# rp2040_pio_stepper
Stepper motor control from the rp2040 for LinuxCNC.

# Building
See BUILD.txt for details on building the RP firmware and the LinuxCNC driver.

The IP address of the RP2040 is 192.168.12.2.
I use a straight point-to-point ethernet connecton to an empty network interface on the LinuxCNC host.
I use 192.168.12.1 for the LinuxCNC host.

# Host network tuning

For lowest latency, disable interrupt coalescing on the network interface connected
to the RP2040. Add a udev rule so it applies automatically on boot and link-up:

```
# /etc/udev/rules.d/60-linuxcnc-coalesce.rules
ACTION=="add", SUBSYSTEM=="net", KERNEL=="eth0", \
  RUN+="/sbin/ethtool -C eth0 rx-usecs 0 rx-frames 1"
```

Replace `eth0` with the actual interface name. Without this, the NIC may coalesce
several incoming packets before raising an interrupt, adding variable latency to
the `pos-fb` round-trip.

# Architecture
See [docs/architecture.md](docs/architecture.md) for a component overview and links to detailed pages covering message flow, clock sync, and PIO step generation.

# Configuration
See [docs/hal_reference.md](docs/hal_reference.md) for a full reference of all HAL pins and parameters, types, directions, and setup examples.

Machine-specific LinuxCNC config files live in `config/<machine-name>/`:

```
config/
  frankencnc/
    frankencnc.ini    -- LinuxCNC machine config
    frankencnc.hal    -- HAL wiring for joints, GPIO, spindle
```

To launch LinuxCNC with a config: `linuxcnc config/frankencnc/frankencnc.ini`

`rp2040_gpio_types.ini` lives in `src/driver/` and is included by the machine `.ini` via `#INCLUDE`. LinuxCNC resolves `#INCLUDE` paths relative to the including file's directory.

# Wishlist
  * RS-485 support.
  * Test with all 4 axis. (Only 3 tested so far.)

# Stretch goals
  * SPI support.
  * SPI attached child RP2040 for more joints.
  * Test with multiple RP2040 on same network segment. Would need some driver work.
