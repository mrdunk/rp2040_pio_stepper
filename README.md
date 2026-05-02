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

# Configuration
See [docs/hal_pins.md](docs/hal_pins.md) for a full reference of all HAL pins, types, directions, and setup examples.

Machine-specific LinuxCNC config files live in `config/<machine-name>/`:

```
config/
  frankencnc/
    frankencnc.ini    -- LinuxCNC machine config
    frankencnc.hal    -- HAL wiring for joints, GPIO, spindle
```

To launch LinuxCNC with a config: `linuxcnc config/frankencnc/frankencnc.ini`

`rp2040_gpio_types.ini` lives in `src/driver/` and is included by the machine `.ini` via `#INCLUDE`. LinuxCNC resolves `#INCLUDE` paths relative to the including file's directory.

The KP value in the .ini file governs the amount of software smoothing applied to step generation.
Technically valid ranges are from 1.0 - 0.0 but at 1.0 oscillations are too large to be usable and at 0.0 no actual update will occur.
`KP = 0.2` is a good balance between smoothing with very little added latency.

# Wishlist
  * Configurable step pulse polartiy.
  * GPIO native
  * GPIO i2c
  * RS-485 support.
  * RP side max speed and acceleration. Clamp values up to a set threashold. Error if over threashold.
  * LinuxCNC side feedback checking.
  * RP configuration confirmation. Currently there's no way to prove a UDP configuration packet arrived on the RP.
  * Test with dropped packets.
  * Test with mis-timed packets.
  * Test with all 4 axis. (Only 3 tested so far.)

# Stretch goals
  * Diagram documenting SW layout.
  * SPI support.
  * SPI attached child RP2040 for more joints.
  * Test with multiple RP2040 on same network segment. Would need some driver work.
