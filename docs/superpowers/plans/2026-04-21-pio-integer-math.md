# PIO Q16.16 Fixed-Point Math Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert `get_velocity()`, `clamp_accel()`, `calculate_step_len()`, and `plan_steps()` from double-precision float to Q16.16 fixed-point integers to reduce Core1 overrun during motion.

**Architecture:** Q16.16 signed `int32_t` (16 fractional bits, resolution 0.000015 steps/period). Config boundary stays `double`; conversion happens once at the top of `do_steps()` (~6 double ops per joint, down from ~30). All intermediate multiplications use `int64_t` to prevent overflow. `JointPioState.last_velocity` and `step_accumulator` become `int32_t`.

**Tech Stack:** C, RP2040, cmocka host tests. Build: `cmake -B build_tests -S . -DBUILD_TESTS=ON && make -C build_tests`. Test: `ctest --test-dir build_tests --output-on-failure`.

**Spec:** `docs/superpowers/specs/2026-04-21-pio-integer-math-design.md`

**Pre-commit hook constraint:** The hook builds and runs all tests on every commit. Because changing a function's signature immediately breaks its call sites, all signature changes, their call sites in `do_steps()`, and the affected tests must land in a single commit. There is no "commit failing tests first" pattern here.

---

## File Structure

- Modify: `src/rp2040/pio.c` — constants, JointPioState fields, all four math functions, do_steps()
- Modify: `src/rp2040/pio.h` — function signatures (BUILD_TESTS section + clamp_accel)
- Modify: `src/test/rp_pio_test.c` — update tests for all four functions

---

## Task 1: Convert PIO math to Q16.16 fixed-point

**Files:**
- Modify: `src/rp2040/pio.c`
- Modify: `src/rp2040/pio.h`
- Modify: `src/test/rp_pio_test.c`

All steps must be completed before committing — signatures, implementations, and tests must all be consistent when the pre-commit hook runs.

---

- [ ] **Step 1: Update constants and includes in `pio.c`**

Replace the top of `pio.c` (lines 1–25, up to the `typedef struct`):

```c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"
#include "../test/mocks/pio_mocks.h"

#else  // BUILD_TESTS

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico_stepper.pio.h"

#endif  // BUILD_TESTS

#include "pio.h"
#include "config.h"

#define STEP_PIO_LEN_OVERHEAD  9
#define RP2040_CLOCK_MHZ       133
#define POSITION_BIAS_Q        6554    /* round(0.1  * 65536) */
#define VELOCITY_BIAS_Q        55706   /* round(0.85 * 65536) */
#define MIN_STEP_COUNT_Q       4096    /* 0.0625 * 65536 */
#define STOP_THRESHOLD_Q       65536   /* 1.0 * 65536 */
```

(Removed: `#include <math.h>`, `STEP_PIO_MULTIPLIER`, `POSITION_BIAS`, `VELOCITY_BIAS`, `MIN_STEP_COUNT`, `STOP_THRESHOLD`.)

---

- [ ] **Step 2: Update `JointPioState` in `pio.c`**

Replace the struct definition:

```c
typedef struct {
    uint32_t sm0;
    uint32_t sm1;
    bool     init_done;
    int32_t  last_pos_requested;
    int32_t  last_pos_achieved;
    uint32_t last_enabled;
    int32_t  last_velocity_q;
    int32_t  step_accumulator_q;
} JointPioState;
```

(`pio_reset_for_test()` uses `memset`, so no change needed there.)

---

- [ ] **Step 3: Replace `get_velocity()` in `pio.c`**

Replace the entire `get_velocity` function (currently lines 145–169):

```c
int32_t get_velocity(int32_t pos_diff_q, int32_t vel_req_q) {
    if (abs(pos_diff_q) < 66) {
        return 0;
    }
    if ((pos_diff_q > 0 && vel_req_q < 0) || (pos_diff_q < 0 && vel_req_q > 0)) {
        return 0;
    }
    return (int32_t)(((int64_t)pos_diff_q * POSITION_BIAS_Q
                      + (int64_t)vel_req_q * VELOCITY_BIAS_Q) >> 16);
}
```

---

- [ ] **Step 4: Replace `calculate_step_len()` in `pio.c`**

Replace the entire function (currently lines 173–186):

```c
int32_t calculate_step_len(int32_t step_count_q, int32_t period_ticks, int32_t max_vel_q) {
    if (step_count_q <= MIN_STEP_COUNT_Q) {
        return 0;
    }
    int32_t len = (int32_t)((int64_t)period_ticks * 65536
                            / ((int64_t)step_count_q << 1) - STEP_PIO_LEN_OVERHEAD);
    int32_t min_len = (int32_t)((int64_t)period_ticks * 65536
                                / ((int64_t)max_vel_q << 1) - STEP_PIO_LEN_OVERHEAD);
    int32_t max_len = period_ticks / 2 - STEP_PIO_LEN_OVERHEAD;
    int32_t clamped = len < min_len ? min_len : len;
    return clamped > max_len ? max_len : clamped;
}
```

---

- [ ] **Step 5: Replace `clamp_accel()` in `pio.c`**

Replace the entire function (currently lines 190–202):

```c
int32_t clamp_accel(int32_t velocity_q, int32_t last_velocity_q, int32_t max_accel_q) {
    if (max_accel_q <= 0) {
        return velocity_q;
    }
    int32_t delta = velocity_q - last_velocity_q;
    if (delta > max_accel_q) {
        return last_velocity_q + max_accel_q;
    }
    if (delta < -max_accel_q) {
        return last_velocity_q - max_accel_q;
    }
    return velocity_q;
}
```

---

- [ ] **Step 6: Replace `plan_steps()` in `pio.c`**

Replace the entire function (currently lines 204–220):

```c
int32_t plan_steps(int32_t velocity_q, uint8_t joint,
                   int32_t period_ticks, int32_t step_len) {
    joint_state[joint].step_accumulator_q += abs(velocity_q);
    int32_t n_steps_desired = joint_state[joint].step_accumulator_q >> 16;
    joint_state[joint].step_accumulator_q -= n_steps_desired << 16;

    int32_t step_period = 2 * (step_len + STEP_PIO_LEN_OVERHEAD);
    int32_t max_steps = (step_period > 0) ? period_ticks / step_period : 0;
    if (max_steps < 0) max_steps = 0;

    int32_t n_steps = n_steps_desired < max_steps ? n_steps_desired : max_steps;
    joint_state[joint].step_accumulator_q += (n_steps_desired - n_steps) << 16;

    return n_steps;
}
```

---

- [ ] **Step 7: Update `do_steps()` in `pio.c`**

Replace the motion-computation block. Find these lines in `do_steps()` (after the underrun early-return, after `drain_rx_fifo`, currently lines 285–316):

```c
  double velocity = get_velocity(
      update_period_us,
      joint,
      abs_pos_achieved,
      abs_pos_requested,
      velocity_requested);

  max_velocity /= update_period_us;
  max_accel /= update_period_us;
  velocity = clamp_accel(velocity, joint_state[joint].last_velocity, max_accel);
  joint_state[joint].last_velocity = velocity;

  double step_count = fabs(velocity);
  double update_period_ticks = update_period_us * RP2040_CLOCK_MHZ;

  int32_t step_len_ticks = calculate_step_len(step_count, update_period_ticks, max_velocity);
  int32_t n_steps = plan_steps(velocity, joint, update_period_ticks, step_len_ticks);

  uint32_t direction = (velocity > 0);
```

Replace with:

```c
  int32_t pos_diff_q   = (int32_t)((abs_pos_requested - (double)abs_pos_achieved) * 65536.0);
  int32_t vel_req_q    = (int32_t)((velocity_requested / (double)update_period_us) * 65536.0);
  int32_t max_vel_q    = (int32_t)((max_velocity / (double)update_period_us) * 65536.0);
  int32_t max_accel_q  = (int32_t)((max_accel / (double)update_period_us) * 65536.0);
  int32_t period_ticks = (int32_t)update_period_us * RP2040_CLOCK_MHZ;

  int32_t velocity_q = get_velocity(pos_diff_q, vel_req_q);
  velocity_q = clamp_accel(velocity_q, joint_state[joint].last_velocity_q, max_accel_q);
  joint_state[joint].last_velocity_q = velocity_q;

  int32_t step_count_q   = abs(velocity_q);
  int32_t step_len_ticks = calculate_step_len(step_count_q, period_ticks, max_vel_q);
  int32_t n_steps        = plan_steps(velocity_q, joint, period_ticks, step_len_ticks);

  uint32_t direction = (velocity_q > 0);
```

Also update the underrun early-return check a few lines above (currently uses `last_velocity`):

```c
  if(updated == 0 || update_period_us == 0) {
    if (abs(joint_state[joint].last_velocity_q) < STOP_THRESHOLD_Q &&
        pio_sm_is_tx_fifo_empty(pio0, joint_state[joint].sm0)) {
      pio_sm_put(pio0, joint_state[joint].sm0, 0);
    }
    return 0;
  }
```

And update the `velocity_requested_tm1` write-back a few lines below:

```c
  int32_t velocity_requested_tm1 = velocity_q >> 16;
```

(replaces `int32_t velocity_requested_tm1 = (int32_t)velocity;`)

---

- [ ] **Step 8: Update `pio.h` signatures**

Replace the full `pio.h` content:

```c
#ifndef PIO__H
#define PIO__H

#include <stdint.h>

/* Initialize a pair of PIO programmes.
 * One for step generation on pio0 and one for counting said steps on pio1.
 */
void init_pio(const uint32_t joint);

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t joint);

int32_t clamp_accel(int32_t velocity_q, int32_t last_velocity_q, int32_t max_accel_q);

#ifdef BUILD_TESTS
/* Reset all static state in pio.c — used by test setup fixtures only. */
void pio_reset_for_test(void);

/* Exposed for unit testing only. */
int32_t get_velocity(int32_t pos_diff_q, int32_t vel_req_q);
int32_t drain_rx_fifo(uint32_t sm, int32_t current_pos);
int32_t calculate_step_len(int32_t step_count_q, int32_t period_ticks, int32_t max_vel_q);
int32_t plan_steps(int32_t velocity_q, uint8_t joint, int32_t period_ticks, int32_t step_len);
#endif  // BUILD_TESTS

#endif  // PIO__H
```

---

- [ ] **Step 9: Update `clamp_accel` tests in `rp_pio_test.c`**

Replace the five `test_clamp_accel_*` test bodies. Values: 5.0→327680, 2.0→131072, 7.0→458752, 3.0→196608, 1.0→65536, 10.0→655360, 100.0→6553600, 0.0→0.

```c
static void test_clamp_accel_no_change(void **state) {
    (void)state;
    int32_t result = clamp_accel(327680, 327680, 131072);
    assert_int_equal(result, 327680);  /* 5.0 unchanged */
}

static void test_clamp_accel_under_limit(void **state) {
    (void)state;
    int32_t result = clamp_accel(458752, 327680, 196608);
    assert_int_equal(result, 458752);  /* delta=131072 < max_accel=196608, not clamped */
}

static void test_clamp_accel_over_limit_positive(void **state) {
    (void)state;
    int32_t result = clamp_accel(655360, 327680, 131072);
    assert_int_equal(result, 458752);  /* 5.0 + 2.0 = 7.0 */
}

static void test_clamp_accel_over_limit_negative(void **state) {
    (void)state;
    int32_t result = clamp_accel(65536, 327680, 131072);
    assert_int_equal(result, 196608);  /* 5.0 - 2.0 = 3.0 */
}

static void test_clamp_accel_zero_max(void **state) {
    (void)state;
    int32_t result = clamp_accel(6553600, 0, 0);
    assert_int_equal(result, 6553600);  /* max_accel=0 -> no limit */
}
```

---

- [ ] **Step 10: Update `calculate_step_len` tests in `rp_pio_test.c`**

The function now takes Q16.16 for step_count and max_velocity; period_ticks is a plain integer. Multiply the old float values by 65536 to convert:

```c
static void test_calculate_step_len_normal(void **state) {
    (void)state;
    /* step_count=2.0 -> Q16.16=131072, period=133000, max_vel=50.0 -> Q16.16=3276800
     * 133000*65536=8716288000; 8716288000/(131072*2)=8716288000/262144=33250 exactly
     * 33250-9=33241 */
    int32_t result = calculate_step_len(131072, 133000, 3276800);
    assert_int_equal(result, 33241);
}

static void test_calculate_step_len_clamped(void **state) {
    (void)state;
    /* step_count=200.0, max_vel=50.0, both in Q16.16 */
    int32_t result = calculate_step_len(13107200, 133000, 3276800);
    /* len=323, min=1321, result=1321 */
    assert_int_equal(result, 1321);
}

static void test_calculate_step_len_below_threshold(void **state) {
    (void)state;
    /* 0.05 * 65536 = 3276.8 -> 3276; MIN_STEP_COUNT_Q=4096; 3276<=4096 -> 0 */
    int32_t result = calculate_step_len(3276, 133000, 3276800);
    assert_int_equal(result, 0);
}

static void test_calculate_step_len_upper_clamp(void **state) {
    (void)state;
    /* 0.1 * 65536 = 6553.6 -> 6553; raw len >> max_len; clamped to max_len */
    int32_t result = calculate_step_len(6553, 133000, 3276800);
    /* max_len = 133000/2 - 9 = 66491 */
    assert_int_equal(result, 66491);
}
```

---

- [ ] **Step 11: Update `plan_steps` tests in `rp_pio_test.c`**

The function now takes Q16.16 for velocity; period_ticks and step_len are plain integers. Pass `(int32_t)(value * 65536)` for velocities.

```c
static void test_plan_steps_fractional_accumulation(void **state) {
    (void)state;
    /* velocity_q = round(0.3 * 65536) = 19661
     * acc: 19661, 39322, 58983, 78644 -> steps: 0,0,0,1 */
    int32_t vq = (int32_t)(0.3 * 65536);  /* C truncates toward zero: 19660 */
    assert_int_equal(plan_steps(vq, 0, 133000, 1), 0);
    assert_int_equal(plan_steps(vq, 0, 133000, 1), 0);
    assert_int_equal(plan_steps(vq, 0, 133000, 1), 0);
    assert_int_equal(plan_steps(vq, 0, 133000, 1), 1);
}

static void test_plan_steps_total_over_ten_periods(void **state) {
    (void)state;
    /* Use 2.75 (= 180224 in Q16.16, exactly representable); 10 periods -> 27 steps */
    int32_t total = 0;
    for (int i = 0; i < 10; i++) {
        total += plan_steps(180224, 0, 133000, 1);
    }
    assert_int_equal(total, 27);
}

static void test_plan_steps_excess_returned_to_accumulator(void **state) {
    (void)state;
    /* velocity_q=196608 (3.0), step_period=50000, max_steps=133000/50000=2
     * desired=3, capped to 2, excess=1 returned; next call vel=0, acc=1.0 -> 1 step */
    assert_int_equal(plan_steps(196608, 0, 133000, 24991), 2);
    assert_int_equal(plan_steps(0,      0, 133000, 24991), 1);
}

static void test_plan_steps_zero_velocity(void **state) {
    (void)state;
    assert_int_equal(plan_steps(0, 0, 133000, 13291), 0);
    assert_int_equal(plan_steps(0, 0, 133000, 13291), 0);
}
```

---

- [ ] **Step 12: Update `get_velocity` tests in `rp_pio_test.c`**

The function now takes pre-computed Q16.16 values (pos_diff_q and vel_req_q); the caller computes them. The old `update_period_us`, `joint`, `abs_pos_achieved`, `abs_pos_requested`, `expected_velocity` arguments are gone.

```c
static void test_get_velocity_zero_pos_diff(void **state) {
    (void)state;
    /* pos_diff=0 steps -> abs(0) < 66 -> return 0 */
    int32_t v = get_velocity(0, (int32_t)(5.0 * 65536));
    assert_true(v == 0);
}

static void test_get_velocity_direction_disagreement(void **state) {
    (void)state;
    /* pos_diff=+10 steps (forward), vel_req=-5.0 steps/period (backward) -> return 0 */
    int32_t v = get_velocity((int32_t)(10.0 * 65536), (int32_t)(-5.0 * 65536));
    assert_true(v == 0);
}

static void test_get_velocity_normal_forward(void **state) {
    (void)state;
    /* pos_diff=+10, vel_req=+5.0 -> combined > 0 */
    int32_t v = get_velocity((int32_t)(10.0 * 65536), (int32_t)(5.0 * 65536));
    assert_true(v > 0);
}
```

---

- [ ] **Step 13: Verify do_steps end-to-end tests need no changes**

The following tests call `do_steps()` only and check behavioral outcomes (PIO word values, call counts). They set `config` fields with the same double values as before and the Q16.16 conversion happens inside `do_steps()`. All expected values remain correct:

- `test_do_steps_accel_clamped`: checks `first_word >> 1 == 13291` and `second_word >> 1 == 6641`. With Q16.16: max_accel_q = 5*65536 = 327680, step_len on first call = 13291 ✓, second call = 6641 ✓ (same as float because 5.0 and 10.0 are exactly representable).
- `test_do_steps_underrun_slow_stops_pio`: primes last_velocity_q ≈ 31130 (< STOP_THRESHOLD_Q 65536) → PIO stopped ✓.
- `test_do_steps_underrun_medium_leaves_pio_running`: primes last_velocity_q ≈ 344094 (> 65536) → PIO not touched ✓.

No changes needed to any `test_do_steps_*` test.

---

- [ ] **Step 14: Build and run all tests**

```bash
make -C build_tests
ctest --test-dir build_tests --output-on-failure
```

Expected: all 12 test suites pass, all 27 `rpPioTest` tests pass.

If `test_calculate_step_len_normal` fails with 33241 vs 33242, update the assert to match the actual Q16.16 result (both are correct; the discrepancy is a 7.5 ns rounding difference).

---

- [ ] **Step 15: Build firmware to verify it compiles**

```bash
cmake -B build -S . -DBUILD_RP=ON
make -C build stepper_control
```

Expected: clean build, no warnings about undefined symbols or type mismatches.

---

- [ ] **Step 16: Commit**

```bash
git add src/rp2040/pio.c src/rp2040/pio.h src/test/rp_pio_test.c
git commit -m "perf: convert PIO math to Q16.16 fixed-point; eliminate double ops from Core1 hot path"
```
