# Fractional Step Timing (partial steps) — Design Spec

**Branch**: `dunk_partial_steps`
**Date**: 2026-05-25

## Problem

At non-integer velocities above 1 step/period, the Bresenham accumulator in `plan_steps`
alternates between floor(v) and ceil(v) steps per period.  The second
`calculate_step_len` call in `commit_steps` derives `step_len` from that integer output,
so the PIO half-period jumps between two different values every 1 ms.  For example at
v = 2.3 steps/period, `step_len` alternates between the value for 2 steps/period and the
value for 3 steps/period, causing a velocity discontinuity at every period boundary.

The PIO accepts a new step-length command after the current step completes, so there is no
technical barrier to holding `step_len` constant at the exact fractional rate.

## Goal

Replace the integer-quantised PIO step_len with a fractional value encoding the exact
commanded velocity.  Steps naturally span 1 ms period boundaries.  Bresenham position
tracking is unchanged.

## Scope

- **In scope**: PIO timing for v > 1 step/period.
- **Not in scope (this branch)**: sub-1-step behaviour, direction-alternation hunting at
  near-zero velocity (see Known Limitations).

## Architecture

`commit_steps` currently makes two `calculate_step_len` calls:

1. `step_len_ceil = calculate_step_len(step_count_q, ...)` — sizes the half-period for
   `v_ceil` steps; passed to `plan_steps` so the Bresenham gets `max_steps = v_ceil`.
   **Unchanged.**
2. `step_len_ticks = calculate_step_len(n_steps * 65536, ...)` — sizes the half-period
   for the integer Bresenham output; sent to the PIO.  **Replaced.**

`plan_steps`, `calculate_step_len`, `issue_pio_step`, and position tracking (`n_steps`
added to `abs_pos_achieved`) are all unchanged.

## New function: `calculate_step_len_frac`

```c
/* Exact half-period for fractional velocity v = step_count_q/65536 steps/period.
 * Complements calculate_step_len (which sizes for ceil(v)) — this function is
 * used for PIO timing so the step rate matches the commanded fractional velocity. */
int32_t calculate_step_len_frac(int32_t step_count_q,
                                int32_t period_ticks,
                                int32_t max_vel_q);
```

### Formula

```
len = period_ticks * 65536 / (2 * step_count_q) - STEP_PIO_LEN_OVERHEAD
```

`int64_t` is required for the multiply: `133000 * 65536 ≈ 8.7e9`, which overflows int32.

### Clamping

| Condition | Action |
|-----------|--------|
| `step_count_q <= 0` | return 0 |
| `len > max_len` (`period_ticks/2 - overhead`) | clamp to `max_len` |
| `max_vel_q > 0` and `step_count_q > max_vel_q` | clamp to `min_len = period_ticks * 65536 / (2 * max_vel_q) - overhead` |

The `max_len` clamp preserves the existing invariant that a step must complete within one
servo period so the PIO FIFO is not blocked for the following Core1 tick.

The `min_len` clamp enforces the configured velocity ceiling using the same fractional
formula, consistent with the step_len formula itself.

### Behaviour at boundary velocities

- **v < 1 step/period**: formula gives `len > max_len` → clamped to `max_len`.
  Combined with the `n_steps > 0` gate below, sub-1-step behaviour is identical to today.
- **v = integer**: formula gives the same result as `calculate_step_len(v * 65536, ...)`.
  No behavioural change at integer velocities.
- **v > 1, non-integer**: `step_len` is stable at the fractional value; no longer
  alternates between floor/ceil step_lens.

## Change in `commit_steps`

```c
/* BEFORE */
int32_t step_len_ceil  = calculate_step_len(step_count_q, dq->period_ticks, dq->max_vel_q);
int32_t n_steps        = plan_steps(plan_vel_q, joint, dq->period_ticks, step_len_ceil);
int32_t step_len_ticks = calculate_step_len(n_steps * 65536, dq->period_ticks, dq->max_vel_q);
...
issue_pio_step(joint, step_len_ticks, direction);

/* AFTER */
int32_t step_len_ceil  = calculate_step_len(step_count_q, dq->period_ticks, dq->max_vel_q);
int32_t n_steps        = plan_steps(plan_vel_q, joint, dq->period_ticks, step_len_ceil);
int32_t step_len_ticks = (n_steps > 0)
    ? calculate_step_len_frac(step_count_q, dq->period_ticks, dq->max_vel_q)
    : 0;
...
issue_pio_step(joint, step_len_ticks, direction);
```

The `n_steps > 0` gate replaces the implicit zero that arose when
`calculate_step_len(0, ...)` returned 0.  When the Bresenham says no step this period
(sub-1-step case), `step_len_ticks = 0` is sent to `issue_pio_step`, pausing the PIO —
identical to existing behaviour.

## Position tracking note

`n_steps` (Bresenham integer output) still drives `abs_pos_achieved`.  At non-integer
velocities the PIO step count and the Bresenham count may differ by ±1 in any given
period, but average to the same value over time.  For open-loop joints this introduces
sub-step tracking noise; for joints with hardware step_count feedback the hardware counter
is authoritative and the Bresenham is not used.

## Testing

### New unit tests for `calculate_step_len_frac`

| Case | Expected |
|------|----------|
| `step_count_q = 0` | 0 |
| Integer v (e.g. v = 2.0) | same as `calculate_step_len(2*65536, ...)` |
| Fractional v > 1 (e.g. v = 2.3) | `period_ticks * 65536 / (2 * step_count_q) - overhead` |
| v < 1 (e.g. v = 0.3) | clamped to `max_len` |
| v > max_vel | clamped to `min_len` for max_vel |

### Updated `do_steps` / `commit_steps` tests

Tests that assert the step_len written to the PIO mock at non-integer velocities
(e.g. v = 1.5, v = 2.3) will now expect the fractional value.  These failures are
expected and intentional — update expected values to match the new formula.

Tests at integer velocities and sub-1-step: **no changes required**.

## Known limitations

### Direction alternation at sub-1-step (not fixed by this branch)

At very low speeds in position mode, the position correction term can oscillate between
small positive and negative values, causing the Bresenham to fire alternating-direction
steps several periods apart.  This is independent of step_len computation and requires
a separate fix (e.g. a dead zone in the correction term, or tighter interaction between
`apply_at_target_snap` and the Bresenham accumulator).

## Deferred: Approach B refactor

After this branch is merged, consider refactoring `plan_steps` to accept `v_ceil`
directly instead of inferring it from `step_len`:

```c
/* Current */
int32_t plan_steps(int32_t velocity_q, uint8_t joint,
                   int32_t period_ticks, int32_t step_len);

/* Proposed */
int32_t plan_steps(int32_t velocity_q, uint8_t joint, int32_t max_steps);
```

This decouples the Bresenham max-steps calculation from PIO timing entirely, making the
separation of concerns explicit at the API level.  `calculate_step_len` would then be
purely the fractional formula (no internal v_ceil logic).
