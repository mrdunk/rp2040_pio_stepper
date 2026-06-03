# Stepper Driver Compatibility

The `step_gen2` PIO program generates step/direction pulses with a configurable HIGH
pulse width and symmetric LOW phases either side. This page covers the timing
parameters that matter for driver compatibility and how to configure them.

---

## Timing parameters

```
  DIR  ──────────────────────────────────────────────────
  STEP ─────╮          ╭────────╮          ╭────────╮
            │          │        │          │        │
            ╰──────────╯        ╰──────────╯        ╰──
            ← low1 →   ← HIGH → ← low2 →  ← low1 → ...
```

| Parameter | Description |
|-----------|-------------|
| **low1** | LOW phase before the rising edge. Sets direction setup time. Duration = `step_low_half` PIO cycles. |
| **HIGH** | Step pulse width. Fixed per joint; set by `high_count`. Default ≈ 2.53 µs. |
| **low2** | LOW phase after the falling edge. Sets hold time. Duration = `step_low_half` PIO cycles. |

`low1` and `low2` are equal (`step_low_half` cycles each). The inter-pulse LOW
(falling edge of step N to rising edge of step N+1) = `low2 + low1 + overhead` ≈
`2 × step_low_half + 10 cycles` ≈ `2 × step_low_half + 75 ns`.

`step_low_half` shrinks as step rate increases. `high_count` is per-joint and
constant regardless of step rate (see [Configuring high_count](#configuring-high_count)).

---

## Driver timing requirements

Specs from manufacturer datasheets. "Min LOW" = minimum time from falling edge to
next rising edge (covers both low2 and low1). DIR setup = minimum time DIR must be
stable before the step rising edge (= minimum low1 after a direction change).

| Driver | Min LOW | Min HIGH (pulse) | DIR setup | Max rate |
|--------|---------|------------------|-----------|----------|
| **TMC2208 / TMC2209** | 100 ns | 100 ns | 20 ns | 2.1 MHz |
| **TMC5160** | 100 ns | 100 ns | 20 ns | 2.5 MHz |
| **A4988** | 1.0 µs | 1.0 µs | 200 ns | ~500 kHz |
| **DRV8825** | 1.9 µs | 1.9 µs | 650 ns | ~250 kHz |
| **TB6600** (opto) | 2.2 µs | 2.2 µs | 5 µs | ~20 kHz† |
| **DM542 / DM556** | 2.5 µs | 2.5 µs | 5 µs | 200 kHz |
| **DM860H** | 2.5 µs | 2.5 µs | 5 µs | 300 kHz |
| **AM882** | 2.5 µs | 2.5 µs | 5 µs | 200 kHz |

† TB6600 opto-coupler modules are typically limited to ~20 kHz regardless of spec;
  the underlying IC can run faster but the coupler RC time constant limits bandwidth.

---

## Maximum step rate from step_gen2

The PIO runs at 133 MHz. Minimum achievable period (when `step_low_half = 0`) is
≈ 348 cycles ≈ 2.62 µs → **absolute maximum ≈ 383 kHz**. Driver requirements reduce
this further:

Step rate is limited by whichever constraint is binding — the LOW requirement or the
DIR setup time. At steady velocity (no direction change) DIR setup does not apply.

| Driver | Steady-state max | After direction change |
|--------|-----------------|----------------------|
| TMC2208/2209/5160 | ~376 kHz (PIO limit) | ~376 kHz |
| A4988 | ~282 kHz | ~282 kHz |
| DRV8825 | ~225 kHz | ~225 kHz |
| TB6600 | ~79 kHz (opto: ~20 kHz) | ~79 kHz (opto: ~20 kHz) |
| DM542/DM556 | ~199 kHz | ~79 kHz |
| DM860H / AM882 | ~199 kHz | ~79 kHz |

After a direction change DM542-class drivers are limited to ~79 kHz because the
5 µs DIR setup requirement forces `step_low_half ≥ 665 cycles`. This recovers
on the next step once `step_low_half` shrinks back to the steady-state value.

In practice LinuxCNC's servo period (typically 1 ms) and acceleration limits keep
step rates well below these PIO maxima.

---

## Configuring `high_count`

`high_count` sets the step pulse HIGH width:

```
HIGH duration = 1 + (high_count + 1) × 21  PIO cycles
              = (1 + (high_count + 1) × 21) / 133  µs
```

| high_count | HIGH width | Suitable for |
|-----------|-----------|--------------|
| 0 | 165 ns | TMC only (careful — no margin) |
| 4 | 2.21 µs | TMC, A4988, DRV8825, TB6600 |
| 6 | 2.35 µs | TMC, A4988, DRV8825, TB6600 |
| 11 | 1.92 µs | TMC, A4988, DRV8825 |
| **15** | **2.53 µs** | **All drivers above (default)** |
| 20 | 3.33 µs | DM542-class with extra margin |
| 30 | 4.94 µs | Very conservative / long-cable setups |

The default `high_count = 15` (≈ 2.53 µs) covers every driver in the table above
with margin. Only reduce it if minimising step overhead matters (e.g. TMC at very
high step rates using LinuxCNC-side interpolation).

`high_count` can be set at runtime per joint:

```c
pio_set_step_high_count(joint, high_count);
```

Valid range is 0–63. Values `high_count = 0` map Y to 0 before the loop,
resulting in a single iteration (≈ 165 ns) — safe for TMC but below spec for all
other drivers.

---

## Minimum LOW enforcement

`step_low_half` is computed from velocity and shrinks toward zero at maximum step
rate. There is no hardware clamp. To ensure the driver's minimum LOW requirement is
met, set `MAX_VELOCITY` in the LinuxCNC INI so the resulting step rate stays within
the driver's maximum:

```
max_steps_per_second = MAX_VELOCITY × SCALE   (SCALE = steps per unit)
```

Use the steady-state max rate column above for the limiting figure. For example,
with a DM542 and SCALE = 400 steps/mm, set `MAX_VELOCITY ≤ 199000 / 400 = 497 mm/s`.

---

## Notes

- **Opto-isolated drivers** (TB6600, DM542, AM882): the optocoupler RC filter limits
  high-frequency response regardless of the IC's stated max. If steps are missed at
  rates well below the spec maximum, try reducing `high_count` slightly (shortening
  HIGH so the duty cycle is lower) or reducing `MAX_VELOCITY`.

- **TMC drivers in UART/SPI mode**: the step/dir interface is still active alongside
  the serial interface. Pulse specs above apply. Use the default `high_count = 15`
  unless step rate needs to exceed ~200 kHz, in which case reducing to `high_count = 4`
  (≈ 2.21 µs) frees overhead without violating the 100 ns spec.

- **Long cables or differential drivers**: signal rise time degrades with cable
  length. If in doubt, increase `high_count` to give the driver more time to register
  the pulse.
