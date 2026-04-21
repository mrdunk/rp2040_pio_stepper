# pio.c Accel Limiting + Cleanup

**Date:** 2026-04-21
**Branch:** dunk_stepgen_refactor
**Files affected:** `src/rp2040/pio.c`, `src/rp2040/pio.h`, `src/test/rp_pio_test.c`

---

## Dead state to remove

Three file-scope statics declared and reset but never written (post-refactor):

- `static uint32_t count` — incremented but never read; remove declaration and `count++`
- `static size_t dir_change_count[MAX_JOINT]` — never written; remove
- `static uint32_t last_direction[MAX_JOINT]` — never written; remove
- Remove their reset entries in `pio_reset_for_test()`

## Minor fixes

- `if(updated <= 0)` → `if(updated == 0)` — `updated` is `uint32_t`, it cannot be negative
- `static const uint32_t clock_multiplier = 133` inside `do_steps()` → `#define RP2040_CLOCK_MHZ 133` at file top

## New clamp_accel() function (Task 3)

```c
double clamp_accel(double velocity, double last_velocity, double max_accel);
```

Pure function. Clamps `|velocity - last_velocity| <= max_accel`. Returns velocity unchanged if `max_accel <= 0`.

## Wiring (Task 3)

Add `static double last_velocity[MAX_JOINT]`. In `do_steps()`, after `get_velocity()` and normalisation:

```c
velocity = clamp_accel(velocity, last_velocity[joint], max_accel);
last_velocity[joint] = velocity;
```

## Tests (Task 2 + 3)

Five pure unit tests for `clamp_accel()` + one `do_steps()` integration test confirming accel clamping activates.
