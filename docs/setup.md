# Setup

From-zero-to-running walkthrough: clone the repo, build the firmware, flash it,
install the [LinuxCNC](https://linuxcnc.org/) driver, tune the network, and
configure your machine.

## Get the Source

```bash
git clone https://github.com/mrdunk/rp2040_pio_stepper.git
cd rp2040_pio_stepper
```

## Prerequisites

```bash
sudo apt install git cmake gcc-arm-none-eabi ethtool
```

- [pico-sdk](https://github.com/raspberrypi/pico-sdk)
- `halcompile` (part of [LinuxCNC](https://linuxcnc.org/) — install LinuxCNC first)

The [Getting Started with Raspberry Pi Pico](https://rptl.io/pico-get-started)
guide covers installing the toolchain and pico-sdk, and also explains how to
connect a UART serial console for debug output from the firmware (see [Advanced: Connecting to the RP2040 UART](#advanced-connecting-to-the-rp2040-uart)).

## Build the Firmware

Two cache variables control the build:

| Variable | Default | Description |
|----------|---------|-------------|
| `WIZNET_CHIP` | `W5500` | Ethernet chip — `W5500` or `W5100S` |
| `MAX_JOINT` | `8` | Number of stepper joints (1–8) |

```bash
cmake -B build_rp -S . -DBUILD_RP=ON -DWIZNET_CHIP=W5500 -DMAX_JOINT=6
make -C build_rp stepper_control
```

Without `-DBUILD_RP=ON`, CMake configures successfully but produces an empty
Makefile with no firmware targets — no error, no warning.

`MAX_JOINT` determines the PIO layout and feedback channel allocation. It must
match the number of joints configured in LinuxCNC. Rebuilding with a different
value requires reflashing the firmware.

The output image:

```
build_rp/src/rp2040/stepper_control.uf2
```

See [Getting Started with Raspberry Pi Pico](https://rptl.io/pico-get-started)
for full toolchain setup and build troubleshooting.

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

For detail on BOOTSEL mode and UF2 flashing, see [Getting Started with Raspberry Pi Pico](https://rptl.io/pico-get-started).
For connecting a serial console to read firmware debug output, see
[Advanced: Connecting to the RP2040 UART](#advanced-connecting-to-the-rp2040-uart).

## Install the LinuxCNC Driver

```bash
cd src/driver
sudo halcompile --compile --install ./hal_rp2040_eth.c
```

A single driver binary works with any firmware `MAX_JOINT` (4–8). The driver
detects the firmware's joint count from the first reply packet and logs it:
`INFO: firmware reports N joints`. All HAL configs use:

```
loadrt hal_rp2040_eth num_joints=[KINS]JOINTS
```

`num_joints` is required — omitting it causes an error at load time and the
driver defaults to `MAX_JOINT`. The firmware also prints its `MAX_JOINT` and
protocol version at startup alongside branch, commit, and build info — connect
a [UART serial console](#advanced-connecting-to-the-rp2040-uart) to read it.

Rerun this command any time you change `src/driver/hal_rp2040_eth.c` or
`src/driver/messages.h`. A version mismatch between the driver and firmware
(e.g. a reflashed RP2040 without reinstalling the driver) can cause erratic
behaviour — incorrect position feedback, unexpected faults, or garbled data.
`WARN: Unconsumed RX buffer remainder: N bytes` is one possible symptom but
not the only one.

## Network Setup

Connect the RP2040 point-to-point to a spare Ethernet interface on the
[LinuxCNC](https://linuxcnc.org/) host. Ideally no switch or other equipment
between the LinuxCNC PC and the RP2040 — each hop adds latency and jitter.

Assign addresses:

| Host | Address |
|------|---------|
| LinuxCNC host NIC | `192.168.12.1` |
| RP2040 | `192.168.12.2` |

The RP2040 address is hardcoded in the firmware. The LinuxCNC host NIC address
must be set to match. See [Advanced: Changing the IP Addresses](#advanced-changing-the-ip-addresses)
if the defaults conflict with your network.

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

## [LinuxCNC](https://linuxcnc.org/) Machine Config

Machine-specific files live in `config/<machine-name>/`:

```
config/
  pico-eth-cnc-3axis/
    pico-eth-cnc-3axis.ini    -- LinuxCNC machine config (3-joint)
    pico-eth-cnc-3axis.hal    -- HAL wiring for joints, GPIO, spindle
    custom.hal                   -- site-specific overrides
    postgui_call_list.hal        -- post-GUI HAL commands
  pico-eth-cnc-4axis/
    pico-eth-cnc-4axis.ini       -- LinuxCNC machine config (4-joint XYZA)
    pico-eth-cnc-4axis.hal       -- HAL wiring: X(J2), Y(J3), Z(J4), A(J5)
    custom.hal                   -- site-specific overrides
    postgui_call_list.hal        -- post-GUI HAL commands
  pico-eth-cnc-6axis/
    pico-eth-cnc-6axis.ini       -- LinuxCNC machine config (6-joint XYZABC)
    pico-eth-cnc-6axis.hal       -- HAL wiring for 6 joints
    custom.hal                   -- site-specific overrides
    postgui_call_list.hal        -- post-GUI HAL commands
  pico-eth-cnc-8axis/
    pico-eth-cnc-8axis.ini       -- LinuxCNC machine config (8-joint XYZABCUV)
    pico-eth-cnc-8axis.hal       -- HAL wiring for 8 joints (open-loop, no feedback)
    custom.hal                   -- site-specific overrides
    postgui_call_list.hal        -- post-GUI HAL commands
  shared/
    rp2040_types.ini        -- shared HAL type definitions
```

Launch [LinuxCNC](https://linuxcnc.org/) with a config:

```bash
linuxcnc config/pico-eth-cnc-3axis/pico-eth-cnc-3axis.ini
```

`rp2040_types.ini` lives in `config/shared/` and is included by the machine
`.ini` via `#INCLUDE`. [LinuxCNC](https://linuxcnc.org/) resolves `#INCLUDE`
paths relative to the including file's directory:

```ini
#INCLUDE ../shared/rp2040_types.ini
```

See [hal_reference.md](hal_reference.md) for a full reference of all HAL pins
and parameters.

## GPIO

GPIO channels are numbered sequentially (`gpio.00`, `gpio.01`, …) regardless
of whether they are native RP2040 pins or I2C expander pins. Each channel is
configured by setting HAL params before the driver starts.

### Native GPIO (RP2040 GP pins)

Native pins are fast (updated every servo cycle) and directly driven by the
RP2040.

| Param | Value |
|-------|-------|
| `rp2040_eth.0.gpio.NN.type` | `[RP2040_ETH_GPIO_TYPES]GPIO_TYPE_NATIVE_IN` or `GPIO_TYPE_NATIVE_OUT` |
| `rp2040_eth.0.gpio.NN.index` | GP pin number (e.g. `8` = GP8) |

HAL pins available: `.in`, `.in-not`, `.out`, `.out-invert`.

Example from `config/pico-eth-cnc-3axis/pico-eth-cnc-3axis.hal` — the
J6 header exposes GP8–GP15. Direction is not fixed by hardware; set `type` to
`NATIVE_IN` or `NATIVE_OUT` for your application:

```hal
# GPIO — J6 GPIO_FAST (native RP2040 GPIO, GP8-GP15)
setp rp2040_eth.0.gpio.00.type      [RP2040_ETH_GPIO_TYPES]GPIO_TYPE_NOT_SET
setp rp2040_eth.0.gpio.00.index     8

setp rp2040_eth.0.gpio.01.type      [RP2040_ETH_GPIO_TYPES]GPIO_TYPE_NOT_SET
setp rp2040_eth.0.gpio.01.index     9
# ... and so on for gpio.02–07 (GP10–GP15)
```

A fully configured native input wired to a signal:

```hal
setp rp2040_eth.0.gpio.00.type   [RP2040_ETH_GPIO_TYPES]GPIO_TYPE_NATIVE_IN
setp rp2040_eth.0.gpio.00.index  8
net my-input  rp2040_eth.0.gpio.00.in
```

### I2C GPIO (MCP23017 expander)

I2C pins are slower than native (latency of one extra servo cycle) and allow
up to 16 pins per MCP23017 expander. Up to 8 expanders can share one I2C bus
by setting hardware address pins A0–A2.

| Param | Value |
|-------|-------|
| `rp2040_eth.0.gpio.NN.type` | `GPIO_TYPE_I2C_MCP_IN` or `GPIO_TYPE_I2C_MCP_OUT` |
| `rp2040_eth.0.gpio.NN.index` | Pin on the MCP23017 (0–15) |
| `rp2040_eth.0.gpio.NN.address` | I2C address of the expander (decimal) |

**Addressing:** The MCP23017 base address is 0x20 (decimal 32). Hardware pins
A0–A2 add 0–7 to that base, giving addresses 32–39 and supporting up to 8
devices on one bus.

Declare the address in the INI to avoid duplicating it in the HAL:

```ini
# config/pico-eth-cnc-3axis/pico-eth-cnc-3axis.ini
[I2C_0]
TYPE = MCP
ADDRESS = 32
```

Then reference it in the HAL:

```hal
setp rp2040_eth.0.gpio.NN.address  [I2C_0]ADDRESS
```

Example from `config/pico-eth-cnc-3axis/pico-eth-cnc-3axis.hal` — fully
configured I2C inputs and outputs (J8–J11):

```hal
# J8:0 probe — input from probe/tool-setter (MCP23017 pin 13)
setp rp2040_eth.0.gpio.16.type      [RP2040_ETH_GPIO_TYPES]GPIO_TYPE_I2C_MCP_IN
setp rp2040_eth.0.gpio.16.index     13
setp rp2040_eth.0.gpio.16.address   [I2C_0]ADDRESS
net probe-in  rp2040_eth.0.gpio.16.in

# J10:0 pump — output (coolant pump, MCP23017 pin 10)
setp rp2040_eth.0.gpio.20.type      [RP2040_ETH_GPIO_TYPES]GPIO_TYPE_I2C_MCP_OUT
setp rp2040_eth.0.gpio.20.index     10
setp rp2040_eth.0.gpio.20.address   [I2C_0]ADDRESS
net pump-out  rp2040_eth.0.gpio.20.out
```

## Spindle

The driver supports RS-485 VFD spindle control. HAL pins are under
`rp2040_eth.0.spindle.0.*`:

| Pin | Direction | Description |
|-----|-----------|-------------|
| `fwd` | in | Run spindle forward |
| `rev` | in | Run spindle reverse |
| `speed-cmd` | in | Commanded speed (RPM) |
| `speed-fb` | out | Actual speed feedback (RPM) |
| `at-speed` | out | Speed reached signal |

Key params:

| Param | Description |
|-------|-------------|
| `vfd-type` | VFD protocol: 0=disabled, 1=Huanyang, 2=Fuling, 3=Weiken, 4=Loopback |
| `address` | Modbus address of the VFD |
| `bitrate` | RS-485 baud rate |
| `poles` | Motor pole count (for speed scaling) |

### Loopback mode (testing without hardware)

Set `vfd-type = 4` to echo `speed-cmd` back as `speed-fb` and assert
`at-speed`. Use this to verify HAL wiring before connecting a VFD:

```hal
setp rp2040_eth.0.spindle.0.vfd-type  4
```

### Connecting to a VFD

Set `vfd-type` to match your VFD (1=Huanyang, 2=Fuling, 3=Weiken), set the
Modbus address and baud rate, then wire the spindle signals:

```hal
setp rp2040_eth.0.spindle.0.vfd-type  1       # Huanyang
setp rp2040_eth.0.spindle.0.address   1        # Modbus address
setp rp2040_eth.0.spindle.0.bitrate   9600

net spindle-fwd       spindle.0.forward   => rp2040_eth.0.spindle.0.fwd
net spindle-rev       spindle.0.reverse   => rp2040_eth.0.spindle.0.rev
net spindle-speed-cmd spindle.0.speed-out => rp2040_eth.0.spindle.0.speed-cmd
net spindle-speed-fb  spindle.0.speed-in  <= rp2040_eth.0.spindle.0.speed-fb
net spindle-at-speed  spindle.0.at-speed  <= rp2040_eth.0.spindle.0.at-speed
```

See [hal_reference.md](hal_reference.md) for a full list of spindle pins and
parameters.

## Advanced: Connecting to the RP2040 UART

The firmware writes startup diagnostics and runtime log messages to UART0 at
**115200 baud**. The pins are fixed by the pico-sdk default:

| Signal | RP2040 pin |
|--------|-----------|
| TX     | GP0       |
| RX     | GP1       |

Connect a 3.3 V USB-to-serial adapter (CP2102, CH340, or similar) with TX→RX
and RX→TX crossed. **Do not connect 5 V adapters directly — the RP2040 GPIO is
not 5 V tolerant.**

Then open a terminal at 115200 8N1. Examples:

```bash
# minicom
minicom -b 115200 -D /dev/ttyUSB0

# screen
screen /dev/ttyUSB0 115200

# picocom
picocom -b 115200 /dev/ttyUSB0
```

On power-up or reset the firmware prints a startup block:

```
--------------------------------
UART up.
Branch: main
Commit: abc1234
Built:  2026-05-11 12:00:00 by user
Joints: 6
Version: 0.2.4
--------------------------------
```

`Version` is the protocol version shared between the firmware and the
LinuxCNC driver. The driver logs `INFO: version OK (M.N.P)` on first contact,
or `ERROR: version mismatch` if the firmware and driver were built from
different source revisions — reflash the firmware and reinstall the driver from
the same commit to resolve it.

## Advanced: Increasing GPIO and I2C Expander Counts

`MAX_GPIO` and `MAX_I2C_MCP` in `src/shared/messages.h` are compile-time limits
that can be raised if your hardware supports more channels.

```c
// src/shared/messages.h
#define MAX_GPIO     32   // must be a multiple of 32
#define MAX_I2C_MCP   4   // hardware maximum is 8 (MCP23017 address range 0x20–0x27)
```

**`MAX_GPIO`** controls the total number of GPIO channels (native RP2040 pins
plus MCP23017 pins combined). It must be a multiple of 32 — the driver packs
GPIO state into 32-bit banks (`MAX_GPIO_BANK = MAX_GPIO / 32`).

The practical ceiling is the number of native GP pins available on your board
(varies by hardware) plus `MAX_I2C_MCP × 16` (16 pins per MCP23017 expander).

**`MAX_I2C_MCP`** controls how many MCP23017 expanders are tracked. The
MCP23017 I2C address is set by hardware pins A0–A2, giving addresses 0x20–0x27
on a single I2C bus — a hard limit of 8 expanders (128 I2C GPIO pins).

After changing either constant, recompile and reflash the firmware, then
recompile and reinstall the driver. Both share `src/shared/messages.h` — a
mismatch between firmware and driver will corrupt the wire format; the symptom
is `WARN: Unconsumed RX buffer remainder: N bytes` or garbled GPIO data.

## Advanced: Changing the IP Addresses

The IP addresses and UDP port are hardcoded in two source files. To change
them, edit both files and recompile both the firmware and the driver.

**RP2040 firmware** — `src/rp2040/stepper_control.c`:

```c
static wiz_NetInfo g_net_info = {
    .ip = {192, 168, 12, 2},   // RP2040 address
    .gw = {192, 168, 12, 1},   // host/gateway
    ...
};
```

**LinuxCNC driver** — `src/driver/rp2040_network.c:31`:

```c
char *hostname = "192.168.12.2";
int portno = 5002;
```

**UDP port** — `src/rp2040/stepper_control.h` (`NW_PORT`) must match `portno`
in the driver. After editing: recompile and reflash the firmware, then
recompile and reinstall the driver.

## Advanced: Tuning

### Verifying sync with seq-out and seq-in

The driver stamps each outgoing packet with a sequence number
(`rp2040_eth.0.seq-out`) and the RP2040 echoes it back
(`rp2040_eth.0.seq-in`). The difference `seq-out − seq-in` is the round-trip
latency measured in servo cycles.

The value is system-dependent — establish a baseline on your hardware, then
watch for instability or sustained drift upward, which indicates network or
host load problems.

Observe both values live with halmeter:

```bash
halmeter pin rp2040_eth.0.seq-out &
halmeter pin rp2040_eth.0.seq-in &
```

Or connect them to signals for halscope logging. The example config wires
`seq-in` to a named signal:

```hal
net seq-in  rp2040_eth.0.seq-in
```

### Setting FERROR with ferror-suggest

[LinuxCNC](https://linuxcnc.org/)'s `FERROR` setting in the `[JOINT.N]` INI
section is the maximum allowed following error. If exceeded, LinuxCNC faults
with "Follow error".

`rp2040_eth.0.joint.N.ferror-suggest` gives a calculated minimum safe value
based on current round-trip latency:

```
ferror-suggest = vel-limit × (seq-out − seq-in) × packet-interval × 1e-9
```

**Workflow:**
1. Run the machine at full speed.
2. Read `ferror-suggest` via halmeter: `halmeter pin rp2040_eth.0.joint.0.ferror-suggest`
3. Set `FERROR` in the INI to that value plus a small safety margin.

`FERROR` is parsed at startup before HAL runs — it cannot be auto-populated.
Read the pin at runtime and update the INI manually. The example INI includes a
comment showing how to calculate it from first principles:

```ini
# FERROR = 2 * MAX_VELOCITY * SERVO_PERIOD = 2 * 25mm/s * 0.001s = 0.1mm
# Factor of 2: round-trip latency (command→steps→feedback = 2 servo periods).
# Doubled again to tolerate one dropped packet.
FERROR = 0.2
MIN_FERROR = 0.2
```

### Max velocity and acceleration

`rp2040_eth.0.joint.N.vel-limit` and `rp2040_eth.0.joint.N.accel-limit` cap
what the driver will command. In the example config these are set from the INI:

```hal
setp rp2040_eth.0.joint.0.vel-limit   [JOINT_0]MAX_VELOCITY
setp rp2040_eth.0.joint.0.accel-limit [JOINT_0]MAX_ACCELERATION
```

Start with conservative values and increase while monitoring `ferror-suggest`
and following error in halscope. See [hal_reference.md](hal_reference.md) for
the full list of joint pins and params.

## Development

Tests run on the host — no hardware needed.

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON
make -C build_tests
ctest --test-dir build_tests --output-on-failure
```
