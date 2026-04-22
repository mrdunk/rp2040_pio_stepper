# Remove get_velocity() — Design

## Goal

Simplify `pio.c` by removing the position-blended `get_velocity()` function and
driving the motor directly from `velocity_requested` sent by the LinuxCNC driver.

## Context

`get_velocity()` currently blends 10% position-correction with 85% requested
velocity:

```c
return (int32_t)(((int64_t)pos_diff_q * POSITION_BIAS_Q
                  + (int64_t)vel_req_q * VELOCITY_BIAS_Q) >> 16);
```

It also has two early-return guards:

- Returns 0 if `abs(pos_diff_q) < 66` (motor within 0.001 steps of target)
- Returns 0 if position direction and velocity direction disagree

The position-correction term is redundant: `abs_pos_achieved` is fed back to
LinuxCNC each tick, so LinuxCNC can see any drift and issue a corrective
`velocity_requested` in the following packet. Correcting position on the RP2040
as well means double-correction, and the direction-disagreement guard can
incorrectly suppress motion during LinuxCNC-commanded deceleration through zero.

## Design

### do_steps() change

Remove `pos_diff_q` and the `get_velocity()` call. Set velocity directly:

```c
// Remove:
int32_t pos_diff_q = (int32_t)((abs_pos_requested - (double)abs_pos_achieved) * 65536.0);
int32_t velocity_q = get_velocity(pos_diff_q, vel_req_q);

// Replace with:
int32_t velocity_q = vel_req_q;
```

`abs_pos_requested` is still read from config (still needed to compute
`last_pos_requested` and `position_error`).

### Deletions

- `get_velocity()` function body and declaration removed from `pio.c` / `pio.h`
- `POSITION_BIAS_Q` and `VELOCITY_BIAS_Q` constants removed (dead code)
- 3 unit tests removed from `rp_pio_test.c`:
  `test_get_velocity_zero_pos_diff`, `test_get_velocity_direction_disagreement`,
  `test_get_velocity_normal_forward`

### What is unchanged

`clamp_accel`, `calculate_step_len`, `plan_steps`, the underrun PIO-stop guard,
and direction logic (`velocity_q > 0`) are all unaffected.

## Files

- Modify: `src/rp2040/pio.c`
- Modify: `src/rp2040/pio.h`
- Modify: `src/test/rp_pio_test.c`
