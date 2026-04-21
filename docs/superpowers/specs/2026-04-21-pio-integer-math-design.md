# PIO Integer Math (Q16.16 fixed-point) — Design

## Goal

Eliminate software double-precision float from Core1's hot path to reduce
`fb-overrun-ratio` during motion (~0.3–0.5 observed; target near zero).

## Context

Two overrun contributors were identified:

1. **Mutex ordering** (idle 0.2 overrun) — fixed in commit `6f01560`:
   `serialise_joint_movement()` and `serialise_joint_metrics()` now run before
   `packet_generation++`, so Core1 wakes only after Core0 has released all
   `mtx_joint[]` locks.

2. **Software float** (motion-correlated delta) — addressed by this spec:
   `get_velocity()`, `clamp_accel()`, `calculate_step_len()`, and `plan_steps()`
   currently perform ~30 double-precision ops per joint per tick. On Cortex-M0+
   (no FPU) each 64-bit software-float op costs ~60–100 cycles; with 4 joints
   that is ~10,000 cycles per tick (~7% of the 133,000-cycle budget), enough to
   push Core1 over the 1 ms boundary.

## Design

### Fixed-point format

**Q16.16 signed** (`int32_t`): 15-bit integer part, 16-bit fractional part.

- Range: ±32767
- Resolution: 1/65536 ≈ 0.000015 steps/period
- At 1 kHz that is 0.015 steps/second — negligible for any stepper application
- Multiplications that would overflow int32_t use `int64_t` intermediates

### Conversion boundary

`ConfigAxis` fields (`velocity_requested`, `abs_pos_requested`, `max_velocity`,
`max_accel`) remain `double` — they are written by Core0 from the network layer
and shared with the driver. Conversion to Q16.16 happens once at the top of
`do_steps()`:

```c
int32_t pos_diff_q  = (int32_t)((abs_pos_requested - abs_pos_achieved) * 65536.0);
int32_t vel_req_q   = (int32_t)((velocity_requested / update_period_us) * 65536.0);
int32_t max_vel_q   = (int32_t)((max_velocity / update_period_us) * 65536.0);
int32_t max_accel_q = (int32_t)((max_accel / update_period_us) * 65536.0);
int32_t period_ticks = update_period_us * RP2040_CLOCK_MHZ;  /* plain integer */
```

This reduces double ops from ~30 to ~6 per joint per tick.

### JointPioState changes

```c
// Before:
double  last_velocity;
double  step_accumulator;

// After:
int32_t last_velocity_q;
int32_t step_accumulator_q;
```

### Function-by-function

**`get_velocity()`** — signature changes to accept Q16.16 integers:

- `< 0.001` position threshold → `abs(pos_diff_q) < 66`  (0.001 × 65536 = 65.5)
- Direction disagreement check uses sign of int32_t operands directly
- Bias blend uses int64_t intermediates:
  `((int64_t)pos_diff_q * 6554 + (int64_t)vel_req_q * 55706) >> 16`
  where 6554 = round(0.1 × 65536), 55706 = round(0.85 × 65536)

**`clamp_accel()`** — pure int32_t arithmetic, no Q tricks needed:

```c
int32_t delta = velocity_q - last_velocity_q;
if (delta >  max_accel_q) return last_velocity_q + max_accel_q;
if (delta < -max_accel_q) return last_velocity_q - max_accel_q;
return velocity_q;
```

**`calculate_step_len()`** — the division `period_ticks / (step_count × 2)`:

```c
int32_t len = (int32_t)((int64_t)period_ticks * 65536
                        / ((int64_t)step_count_q << 1) - STEP_PIO_LEN_OVERHEAD);
```

`min_len` uses `max_vel_q` in place of `step_count_q`; `max_len` is plain integer:

```c
int32_t max_len = period_ticks / 2 - STEP_PIO_LEN_OVERHEAD;
```

`MIN_STEP_COUNT` threshold (0.0625) → `step_count_q <= 4096` (0.0625 × 65536).

**`plan_steps()`** — accumulator in Q16.16:

```c
step_accumulator_q += abs(velocity_q);           /* Q16.16 add */
int32_t n_steps = step_accumulator_q >> 16;      /* integer part */
step_accumulator_q -= n_steps << 16;             /* keep fractional */
int32_t step_period = 2 * (step_len + (int32_t)STEP_PIO_LEN_OVERHEAD);
int32_t max_steps   = (step_period > 0) ? period_ticks / step_period : 0;
```

**`STOP_THRESHOLD`** — `1.0` in Q16.16 = `65536`. Replace `fabs(last_velocity) < 1.0`
with `abs(last_velocity_q) < 65536`.

### Constants

```c
#define POSITION_BIAS_Q  6554   /* round(0.1  * 65536) */
#define VELOCITY_BIAS_Q  55706  /* round(0.85 * 65536) */
#define MIN_STEP_COUNT_Q 4096   /* round(0.0625 * 65536) */
#define STOP_THRESHOLD_Q 65536  /* 1.0 * 65536 */
```

The float constants `POSITION_BIAS`, `VELOCITY_BIAS`, `MIN_STEP_COUNT`,
`STOP_THRESHOLD` and `STEP_PIO_MULTIPLIER` are removed.

`STEP_PIO_LEN_OVERHEAD` stays as a plain integer constant (value 9).

### Test impact

The 27 existing `rpPioTest` tests drive `do_steps()` end-to-end. Most will pass
unchanged — the step count and PIO FIFO outputs are the same for the same inputs.
Tests that call the internal functions (`get_velocity`, `calculate_step_len`,
`plan_steps`, `clamp_accel`) directly need their argument and return types updated
to Q16.16. Any test that compares velocity values directly needs checking for
Q16.16 equivalents.

## Files

- Modify: `src/rp2040/pio.c`
- Modify: `src/test/rp_pio_test.c`
