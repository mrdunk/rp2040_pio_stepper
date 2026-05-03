# Setup

From-zero-to-running walkthrough: build the firmware, flash it, install the
LinuxCNC driver, tune the network, and configure your machine.

## Prerequisites

- `arm-none-eabi-gcc` and the [pico-sdk](https://github.com/raspberrypi/pico-sdk)
- `halcompile` (part of LinuxCNC — install LinuxCNC first)
- `ethtool` (usually pre-installed on Linux)

Edit `CMakeLists.txt` to match your Ethernet chip. Find the section:

```cmake
# Set ethernet chip
set(WIZNET_CHIP W5500)
```

Change `W5500` to `W5100S` if you have that chip instead.

## Build the Firmware

```bash
cmake -B build_rp -S . -DBUILD_RP=ON
make -C build_rp stepper_control
```

Without `-DBUILD_RP=ON`, CMake configures successfully but produces an empty
Makefile with no firmware targets — no error, no warning.

The output image:

```
build_rp/src/rp2040/stepper_control.uf2
```

## Flash the Firmware

Find the RPI-RP2 mount point:

```bash
sudo blkid -o list | grep RPI-RP2
```

Hold BOOTSEL on the RP2040 while plugging USB. The device appears as a mass
storage drive. Copy the `.uf2`:

```bash
cp build_rp/src/rp2040/stepper_control.uf2 /media/$USER/RPI-RP2/
```

The board reboots automatically once the file is written.

## Build and Run Tests

Tests run on the host — no hardware needed.

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON
make -C build_tests
ctest --test-dir build_tests --output-on-failure
```

## Install the LinuxCNC Driver

```bash
cd src/driver
sudo halcompile --compile --install ./hal_rp2040_eth.c
```

Rerun this command any time you change `src/driver/hal_rp2040_eth.c` or
`src/driver/messages.h`. A stale `.so` against a reflashed firmware produces
the warning: `WARN: Unconsumed RX buffer remainder: N bytes`.

## Network Setup

Connect the RP2040 directly to a spare Ethernet interface on the LinuxCNC host
(point-to-point, no switch needed).

Assign addresses:

| Host | Address |
|------|---------|
| LinuxCNC host NIC | `192.168.12.1` |
| RP2040 | `192.168.12.2` |

### Reduce NIC latency

Disable interrupt coalescing on the interface to minimise round-trip latency.
Add a udev rule so it applies on every boot and link-up:

```
# /etc/udev/rules.d/60-linuxcnc-coalesce.rules
ACTION=="add", SUBSYSTEM=="net", KERNEL=="eth0", \
  RUN+="/sbin/ethtool -C eth0 rx-usecs 0 rx-frames 1"
```

Replace `eth0` with your actual interface name. Without this the NIC may batch
incoming packets before raising an interrupt, adding variable latency to
position-feedback round-trips.

## LinuxCNC Machine Config

Machine-specific files live in `config/<machine-name>/`:

```
config/
  pico-eth-cnc-breakout/
    pico-eth-cnc-breakout.ini    -- LinuxCNC machine config
    pico-eth-cnc-breakout.hal    -- HAL wiring for joints, GPIO, spindle
    custom.hal                   -- site-specific overrides
    postgui_call_list.hal        -- post-GUI HAL commands
  shared/
    rp2040_gpio_types.ini        -- shared HAL type definitions
```

Launch LinuxCNC with a config:

```bash
linuxcnc config/pico-eth-cnc-breakout/pico-eth-cnc-breakout.ini
```

`rp2040_gpio_types.ini` lives in `config/shared/` and is included by the machine
`.ini` via `#INCLUDE`. LinuxCNC resolves `#INCLUDE` paths relative to the
including file's directory:

```ini
#INCLUDE ../shared/rp2040_gpio_types.ini
```

See [hal_reference.md](hal_reference.md) for a full reference of all HAL pins
and parameters.
