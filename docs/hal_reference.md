# HAL Reference

All pins and parameters are registered under the component name `rp2040_eth`. With the default
device index of 0, the full prefix is `rp2040_eth.0`.

Name format:
- Scalar: `rp2040_eth.0.<name>`
- Per-joint: `rp2040_eth.0.joint.<N>.<name>` (N = 0–3)
- Per-GPIO: `rp2040_eth.0.gpio.<NN>.<name>` (NN = 00–63, zero-padded)
- Per-spindle: `rp2040_eth.0.spindle.<N>.<name>` (N = 0–3)

**HAL pins** can be connected to signals with `net`. **HAL parameters** are set once at
config time with `setp` and cannot be connected to signals.

---

## Scalar Pins

| Pin | Type | Dir | Description |
|-----|------|-----|-------------|
| `core1-period` | u32 | OUT | RP2040 core1 measured time between loop iterations (µs) |
| `core1-tick` | u32 | OUT | RP2040 core1 loop iteration counter; a frozen value indicates firmware hang |
| `eth-up` | bit | OUT | Ethernet link state as seen by the driver |
| `machine-on` | bit | OUT | True when the RP2040 Ethernet link is established and communicating |
| `packet-interval` | s32 | OUT | Time between consecutive packets computed from LinuxCNC timestamps (ns); nominally equals the servo period |
| `rx-miss-count` | u32 | OUT | Consecutive cycles without a response from RP2040; resets to 0 on success; triggers network-down handling at MAX_SKIPPED_PACKETS |
| `seq-in` | u32 | OUT | Sequence number echoed back by RP2040; `seq-out − seq-in` gives round-trip latency in servo cycles |
| `seq-out` | u32 | OUT | Sequence number stamped on each packet sent to RP2040 |
| `update-overrun` | float | OUT | Exponential moving average of cycles where Core1 received more than one update from Core0 per period |
| `update-underrun` | float | OUT | Exponential moving average of cycles where Core1 found no new update from Core0 |

---

## Joint Pins

Per-joint pins are indexed 0–3. Replace `<N>` with the joint number.

| Pin | Type | Dir | Description |
|-----|------|-----|-------------|
| `accel-limit` | float | IN | Maximum acceleration (units/sec²) |
| `enable-cmd` | bit | IN | Enable joint (LinuxCNC command) |
| `enable-fb` | bit | OUT | RP2040's actual enabled state; may remain false after network recovery until the protocol explicitly re-enables the joint |
| `ferror-suggest` | float | OUT | Expected following error at `vel-limit` given current round-trip latency (`vel-limit × (seq-out − seq-in) × packet-interval × 1e-9`); set FERROR above this value |
| `pos-cmd` | float | IN | Position command from LinuxCNC trajectory planner (sent to RP2040 but not consumed by firmware; used locally to compute `pos-error-fb`) |
| `pos-error-fb` | s32 | OUT | Difference between commanded and actual step count (raw steps, unscaled) |
| `pos-fb` | float | OUT | Position feedback (cumulative step count ÷ scale) |
| `scale` | float | IN | Steps per unit; applied to both position and velocity commands |
| `vel-calculated` | float | OUT | Velocity the RP2040 computed after applying `vel-limit` and `accel-limit` |
| `vel-cmd` | float | IN | Velocity command from LinuxCNC |
| `vel-fb` | float | OUT | Velocity feedback (raw steps per servo period, unscaled) |
| `vel-limit` | float | IN | Maximum velocity (units/sec) |

### Joint Parameters

Hardware wiring — set once at config time.

| Parameter | Type | Description |
|-----------|------|-------------|
| `gpio-dir` | s32 | RP2040 GPIO pin number for the direction signal |
| `gpio-step` | s32 | RP2040 GPIO pin number for the step signal |

### Joint setup example

```hal
# config/frankencnc/frankencnc.hal

# Hardware wiring (parameters)
setp rp2040_eth.0.joint.0.gpio-step   0         # RP2040 GP0
setp rp2040_eth.0.joint.0.gpio-dir    1         # RP2040 GP1

# Motion limits and scale (pins — can be driven by signals)
setp rp2040_eth.0.joint.0.scale       800       # 800 steps/mm
setp rp2040_eth.0.joint.0.vel-limit   [JOINT_0]MAX_VELOCITY
setp rp2040_eth.0.joint.0.accel-limit [JOINT_0]MAX_ACCELERATION

# Connect to LinuxCNC motion controller
net xpos-cmd  joint.0.motor-pos-cmd => rp2040_eth.0.joint.0.pos-cmd
net xvel-cmd  joint.0.vel-cmd       => rp2040_eth.0.joint.0.vel-cmd
net xpos-fb   rp2040_eth.0.joint.0.pos-fb => joint.0.motor-pos-fb
net x-enable  joint.0.amp-enable-out => rp2040_eth.0.joint.0.enable-cmd
```

---

## GPIO Pins

Per-GPIO pins are indexed 00–63 (zero-padded to two digits). Replace `<NN>` with
the GPIO channel number.

| Pin | Type | Dir | Description |
|-----|------|-----|-------------|
| `in` | bit | OUT | Input state read from hardware |
| `in-not` | bit | OUT | Inverted input state |
| `out` | bit | IN | Output command |
| `out-invert` | bit | IN | Invert output before writing to hardware |

### GPIO Parameters

Hardware wiring — set once at config time.

| Parameter | Type | Description |
|-----------|------|-------------|
| `address` | u32 | I2C address of the GPIO device (MCP23017 only) |
| `index` | u32 | Pin index within the GPIO device |
| `type` | u32 | GPIO device type — see `config/shared/rp2040_gpio_types.ini` for values |

GPIO type values (from `rp2040_gpio_types.ini`):

| Constant | Meaning |
|----------|---------|
| `GPIO_TYPE_NOT_SET` | Unused channel |
| `GPIO_TYPE_NATIVE_IN` | Native RP2040 GPIO input |
| `GPIO_TYPE_NATIVE_OUT` | Native RP2040 GPIO output |
| `GPIO_TYPE_I2C_MCP_IN` | MCP23017 I2C expander input |
| `GPIO_TYPE_I2C_MCP_OUT` | MCP23017 I2C expander output |

### GPIO setup examples

**Native RP2040 input (e.g. probe)**
```hal
# config/frankencnc/frankencnc.hal
setp rp2040_eth.0.gpio.00.type  [RP2040_ETH_GPIO_TYPES]GPIO_TYPE_NATIVE_IN
setp rp2040_eth.0.gpio.00.index 8    # GP8
net probe-in  rp2040_eth.0.gpio.00.in
```

**Native RP2040 output (e.g. coolant relay)**
```hal
# config/frankencnc/frankencnc.hal
setp rp2040_eth.0.gpio.01.type      [RP2040_ETH_GPIO_TYPES]GPIO_TYPE_NATIVE_OUT
setp rp2040_eth.0.gpio.01.index     9    # GP9
setp rp2040_eth.0.gpio.01.out-invert false
net coolant-out  rp2040_eth.0.gpio.01.out
```

**MCP23017 I2C expander input**
```hal
# config/frankencnc/frankencnc.hal
setp rp2040_eth.0.gpio.08.type    [RP2040_ETH_GPIO_TYPES]GPIO_TYPE_I2C_MCP_IN
setp rp2040_eth.0.gpio.08.index   0        # MCP23017 pin GPA0
setp rp2040_eth.0.gpio.08.address [I2C_0]ADDRESS
net hold-in  rp2040_eth.0.gpio.08.in
```

**MCP23017 I2C expander output**
```hal
# config/frankencnc/frankencnc.hal
setp rp2040_eth.0.gpio.09.type    [RP2040_ETH_GPIO_TYPES]GPIO_TYPE_I2C_MCP_OUT
setp rp2040_eth.0.gpio.09.index   1        # MCP23017 pin GPA1
setp rp2040_eth.0.gpio.09.address [I2C_0]ADDRESS
net pump-out  rp2040_eth.0.gpio.09.out
```

---

## Spindle Pins

Per-spindle pins are indexed 0–3. Replace `<N>` with the spindle number.

| Pin | Type | Dir | Description |
|-----|------|-----|-------------|
| `at-speed` | bit | OUT | Spindle has reached commanded speed |
| `fwd` | bit | IN | Run spindle forward |
| `rev` | bit | IN | Run spindle reverse |
| `speed-cmd` | float | IN | Commanded speed |
| `speed-fb` | float | OUT | Actual speed feedback |

### Spindle Parameters

Hardware wiring — set once at config time.

| Parameter | Type | Description |
|-----------|------|-------------|
| `address` | u32 | Modbus/RS-485 device address |
| `bitrate` | u32 | Serial link bitrate |
| `poles` | float | Motor pole count |
| `vfd-type` | u32 | VFD protocol (0=disabled, 1=Huanyang, 2=Fuling, 3=Weiken, 4=Loopback) |

### Spindle setup example

```hal
# config/frankencnc/frankencnc.hal

# VFD parameters
setp rp2040_eth.0.spindle.0.vfd-type 1     # Huanyang
setp rp2040_eth.0.spindle.0.address  1
setp rp2040_eth.0.spindle.0.poles    4
setp rp2040_eth.0.spindle.0.bitrate  9600

# Connect to LinuxCNC spindle component
net spindle-fwd       spindle.0.forward   => rp2040_eth.0.spindle.0.fwd
net spindle-rev       spindle.0.reverse   => rp2040_eth.0.spindle.0.rev
net spindle-speed-cmd spindle.0.speed-out => rp2040_eth.0.spindle.0.speed-cmd
net spindle-speed-fb  spindle.0.speed-in  <= rp2040_eth.0.spindle.0.speed-fb
net spindle-at-speed  spindle.0.at-speed  <= rp2040_eth.0.spindle.0.at-speed
```
