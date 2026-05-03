# PIO step generation

How the RP2040 PIO hardware generates step pulses and counts position.

![PIO step generation diagram](diagrams/pio-stepgen.svg)

---

## Two state machines per joint

Each joint uses one state machine on each PIO block:

| SM | PIO block | Program | Role |
|----|-----------|---------|------|
| `sm0` | PIO0 | `step_gen` | Reads commands from TX FIFO, drives step and direction GPIO pins |
| `sm1` | PIO1 | `step_count` | Counts rising edges on the step pin, pushes position to RX FIFO |

With four joints the firmware occupies all four state machines on PIO0 and all four on
PIO1. Both programs run at the full 133 MHz system clock (clkdiv = 1.0).

---

## `step_gen` program (PIO0)

`step_gen` waits for a packed 32-bit word in the TX FIFO. The encoding is:

| Bits | Field |
|------|-------|
| 31..1 | pulse-length counter (half-period in PIO clock cycles) |
| 0 | direction (1 = positive, 0 = negative) |

On receipt it sets the direction pin immediately, then generates one square-wave step
pulse: step pin high for `pulse_len` cycles, step pin low for `pulse_len` cycles.  A
guard prevents issuing a second step before the first completes across the servo period
boundary.

---

## `step_count` program (PIO1)

`step_count` monitors the step pin for rising edges. On each edge it reads the current
state of the direction pin and either increments or decrements a 32-bit counter. The
counter is pushed to the RX FIFO unconditionally on every edge. Core1 drains the FIFO at
the start of each tick and uses the last value as `abs_pos_achieved`.

---

## Velocity → pulse length

Core1 calls `calculate_step_len()` to convert the Q16.16 fixed-point velocity into a PIO
pulse-length in 133 MHz clock ticks:

```
period_ticks = update_period_us × 133          (servo period in clock cycles)
pulse_len    = period_ticks × 65536
               ─────────────────────── − 9
               step_count_q × 2
```

where `step_count_q` is the Q16.16 magnitude of the requested velocity (steps per servo
period × 65536) and 9 is `STEP_PIO_LEN_OVERHEAD` — the fixed instruction overhead per
half-cycle in the `step_gen` PIO program.

`plan_steps()` then calculates how many complete step pulses fit in the remaining servo
period and returns that count to `do_steps()`, which pushes the packed command word to
the TX FIFO.

---

## Minimum step count guard

If `step_count_q` is below `MIN_STEP_COUNT_Q` (= 4096, i.e. 0.0625 steps/period in
Q16.16), `calculate_step_len()` returns 0 and no steps are generated. This prevents
generating a pulse so wide it would span into the next servo period.

---

## State machine allocation

`init_pio(joint)` is called at firmware startup for each configured joint:

1. Load `step_gen` program into PIO0 (shared offset, loaded once).
2. Load `step_count` program into PIO1 (shared offset, loaded once).
3. Claim SM `joint` on PIO0; configure `step_gen` with the joint's step/dir GPIO pins.
4. Claim SM `joint` on PIO1; configure `step_count` with the same GPIO pins.
5. Enable both SMs.
