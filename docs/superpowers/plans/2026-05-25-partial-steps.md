# Fractional Step Timing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace integer-quantised PIO step_len with a fractional formula so the PIO runs at the exact commanded velocity, eliminating the step-rate jump that occurs every 1 ms at non-integer velocities above 1 step/period.

**Architecture:** Add `calculate_step_len_frac` alongside the existing `calculate_step_len`. In `commit_steps`, replace the second `calculate_step_len(n_steps * 65536, ...)` call with `calculate_step_len_frac(step_count_q, ...)` gated on `n_steps > 0`. Bresenham accumulator and position tracking are untouched.

**Tech Stack:** C11, cmocka, RP2040 PIO. Build: `cmake -B build_tests -S . -DBUILD_TESTS=ON && make -C build_tests`. Test run: `ctest --test-dir build_tests --output-on-failure`. Pre-commit hook runs full build+tests automatically on every commit.

**Spec:** `docs/superpowers/specs/2026-05-25-partial-steps-design.md`

---

## Files

| File | Change |
|------|--------|
| `src/rp2040/pio.c` | Add `calculate_step_len_frac`; update `commit_steps` |
| `src/rp2040/pio.h` | Expose `calculate_step_len_frac` under `BUILD_TESTS` guard |
| `src/test/rp_pio_test.c` | Add unit tests for new function and new PIO timing assertion |

---

## Task 1: Create the feature branch

- [ ] **Step 1: Create and switch to the branch**

```bash
cd /home/duncan/Working/rp2040/rp2040_pio_stepper
git checkout -b dunk_partial_steps
```

Expected: `Switched to a new branch 'dunk_partial_steps'`

---

## Task 2: Add `calculate_step_len_frac` with tests

The pre-commit hook builds and runs all tests, so tests and implementation must be in the **same commit**. Write both, verify manually, then commit.

**Files:**
- Modify: `src/rp2040/pio.h` (add declaration)
- Modify: `src/rp2040/pio.c` (add implementation)
- Modify: `src/test/rp_pio_test.c` (add 5 unit tests)

### Step 1: Add declaration to `pio.h`

In `src/rp2040/pio.h`, inside the `#ifdef BUILD_TESTS` block (after line 57 which declares `calculate_step_len`), add:

```c
int32_t calculate_step_len_frac(int32_t step_count_q, int32_t period_ticks, int32_t max_vel_q);
```

The block should now read:

```c
#ifdef BUILD_TESTS
void pio_reset_for_test(void);
int32_t drain_rx_fifo(uint32_t sm, int32_t current_pos);
int32_t calculate_step_len(int32_t step_count_q, int32_t period_ticks, int32_t max_vel_q);
int32_t calculate_step_len_frac(int32_t step_count_q, int32_t period_ticks, int32_t max_vel_q);
int32_t plan_steps(int32_t velocity_q, uint8_t joint, int32_t period_ticks, int32_t step_len);
#endif  // BUILD_TESTS
```

### Step 2: Implement `calculate_step_len_frac` in `pio.c`

Insert the new function immediately after `calculate_step_len` (after line 214, before `clamp_accel`):

```c
/* Compute the PIO step half-period for exact fractional velocity
 * v = step_count_q / 65536 steps/period.
 *
 * Unlike calculate_step_len (which sizes for ceil(v) to give the Bresenham
 * the right max_steps), this function encodes the exact fractional velocity
 * so the PIO runs at a stable rate and steps span period boundaries smoothly.
 *
 * Uses int64 for the multiply: 133000 * 65536 ≈ 8.7e9, overflows int32. */
int32_t calculate_step_len_frac(int32_t step_count_q, int32_t period_ticks, int32_t max_vel_q) {
    if (step_count_q <= 0) {
        return 0;
    }
    int32_t max_len = period_ticks / 2 - STEP_PIO_LEN_OVERHEAD;
    int32_t len = (int32_t)((int64_t)period_ticks * 65536 / (2 * step_count_q))
                  - STEP_PIO_LEN_OVERHEAD;
    if (len > max_len) len = max_len;
    if (len <= 0) len = 1;
    if (max_vel_q > 0 && step_count_q > max_vel_q) {
        int32_t min_len = (int32_t)((int64_t)period_ticks * 65536 / (2 * max_vel_q))
                          - STEP_PIO_LEN_OVERHEAD;
        if (min_len > max_len) min_len = max_len;
        if (len < min_len) len = min_len;
    }
    return len;
}
```

### Step 3: Add 5 unit tests to `rp_pio_test.c`

Insert the following block after the last `test_calculate_step_len_*` function (around line 160, before `test_clamp_accel_no_change`):

```c
/* calculate_step_len_frac: zero velocity -> 0 */
static void test_calculate_step_len_frac_zero(void **state) {
    (void)state;
    assert_int_equal(calculate_step_len_frac(0, 133000, 0), 0);
    assert_int_equal(calculate_step_len_frac(0, 133000, 3276800), 0);
}

/* calculate_step_len_frac: integer velocity matches calculate_step_len result.
 * v=2.0 (step_count_q=131072): 133000*65536/(2*131072)-9 = 33250-9 = 33241. */
static void test_calculate_step_len_frac_integer_v(void **state) {
    (void)state;
    int32_t result = calculate_step_len_frac(131072, 133000, 0);
    assert_int_equal(result, 33241);
    /* Must equal the ceil formula at integer velocity. */
    assert_int_equal(result, calculate_step_len(131072, 133000, 0));
}

/* calculate_step_len_frac: fractional velocity gives value between floor/ceil results.
 * v=2.3 (step_count_q=150732): 133000*65536/(2*150732)-9 = 28913-9 = 28904.
 * Old formula for n=2 gives 33241, for n=3 gives 22157. New is between them. */
static void test_calculate_step_len_frac_fractional_v(void **state) {
    (void)state;
    /* step_count_q for 2.3 steps/period at 1ms: (int32_t)(2.3 * 65536) = 150732 */
    int32_t result = calculate_step_len_frac(150732, 133000, 0);
    assert_int_equal(result, 28904);
    /* Must differ from both floor and ceil step_lens. */
    assert_int_not_equal(result, calculate_step_len(2 * 65536, 133000, 0)); /* 33241 */
    assert_int_not_equal(result, calculate_step_len(3 * 65536, 133000, 0)); /* 22157 */
}

/* calculate_step_len_frac: sub-1-step velocity clamps to max_len.
 * v=0.3 (step_count_q=19660): raw len >> max_len=66491, so clamped. */
static void test_calculate_step_len_frac_sub1step(void **state) {
    (void)state;
    int32_t max_len = 133000 / 2 - 9;  /* 66491 */
    assert_int_equal(calculate_step_len_frac(19660, 133000, 0), max_len);
    assert_int_equal(calculate_step_len_frac(1,     133000, 0), max_len);
}

/* calculate_step_len_frac: velocity ceiling clamps to min_len.
 * v=3.0 > max_vel=2.0: len=22157 < min_len=33241 -> clamped to min_len. */
static void test_calculate_step_len_frac_vel_ceiling(void **state) {
    (void)state;
    /* step_count_q=3*65536=196608, max_vel_q=2*65536=131072 */
    int32_t result  = calculate_step_len_frac(196608, 133000, 131072);
    int32_t min_len = 33241;  /* 133000*65536/(2*131072)-9 = 33250-9 */
    assert_int_equal(result, min_len);
}
```

### Step 4: Register the 5 tests in `main()`

In `rp_pio_test.c`, find the `cmocka_unit_test_setup(test_calculate_step_len_clamped_allows_v_ceil_max_steps, ...)` line and add the new tests immediately after:

```c
cmocka_unit_test_setup(test_calculate_step_len_clamped_allows_v_ceil_max_steps, test_setup),
cmocka_unit_test_setup(test_calculate_step_len_frac_zero,         test_setup),
cmocka_unit_test_setup(test_calculate_step_len_frac_integer_v,    test_setup),
cmocka_unit_test_setup(test_calculate_step_len_frac_fractional_v, test_setup),
cmocka_unit_test_setup(test_calculate_step_len_frac_sub1step,     test_setup),
cmocka_unit_test_setup(test_calculate_step_len_frac_vel_ceiling,  test_setup),
```

### Step 5: Build and run tests

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON
make -C build_tests
ctest --test-dir build_tests --output-on-failure
```

Expected: all tests pass, including the 5 new `test_calculate_step_len_frac_*` tests.

### Step 6: Commit

```bash
git add src/rp2040/pio.c src/rp2040/pio.h src/test/rp_pio_test.c
git commit -m "feat: add calculate_step_len_frac for exact fractional PIO timing"
```

---

## Task 3: Update `commit_steps` to use fractional step_len

**Files:**
- Modify: `src/rp2040/pio.c` (change `commit_steps`)
- Modify: `src/test/rp_pio_test.c` (add 2 new `do_steps` tests)

### Step 1: Add 2 new tests to `rp_pio_test.c`

Insert the following immediately before `test_do_steps_velmode_frac_1_5` (around line 1026):

```c
/* do_steps at fractional velocity sends fractional step_len to PIO.
 * v=2.3 steps/period (2300 steps/s at 1ms): step_count_q=150732.
 * calculate_step_len_frac gives 28904.
 * Old code: n_steps=2 → step_len=33241, or n_steps=3 → step_len=22157. */
static void test_do_steps_fractional_uses_frac_step_len(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].enabled            = 1;
    config.joint[0].velocity_requested = 2300.0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;

    do_steps(0);
    /* step_len must equal 133000*65536/(2*150732)-9 = 28904, not 33241 or 22157. */
    assert_int_equal(last_pio_put_value >> 1, 28904);
}

/* step_len is stable across periods regardless of Bresenham output (2 or 3 steps).
 * v=2.3: Bresenham gives 2 steps for periods 1-3, then 3 steps for period 4.
 * With fractional formula, step_len is 28904 every period. */
static void test_do_steps_fractional_step_len_stable(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].enabled            = 1;
    config.joint[0].velocity_requested = 2300.0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;

    /* Run 4 periods; Bresenham gives 2,2,2,3 steps respectively. */
    for (int i = 0; i < 4; i++) {
        last_pio_put_value = 0;
        do_steps(0);
        assert_int_equal(last_pio_put_value >> 1, 28904);
    }
}
```

### Step 2: Register the 2 new tests in `main()`

Add immediately before `cmocka_unit_test_setup(test_do_steps_velmode_frac_1_5, ...)`:

```c
cmocka_unit_test_setup(test_do_steps_fractional_uses_frac_step_len,  test_setup),
cmocka_unit_test_setup(test_do_steps_fractional_step_len_stable,      test_setup),
```

### Step 3: Verify new tests fail before the implementation change

```bash
make -C build_tests rpPioTest
./build_tests/src/test/rpPioTest
```

Expected: `test_do_steps_fractional_uses_frac_step_len` and `test_do_steps_fractional_step_len_stable` **FAIL** (step_len will be 33241 or 22157 from old code, not 28904).

### Step 4: Update `commit_steps` in `pio.c`

In `commit_steps` (around line 436-444), replace:

```c
    int32_t step_len_ceil  = calculate_step_len(step_count_q, dq->period_ticks, dq->max_vel_q);
    int32_t n_steps        = plan_steps(plan_vel_q, joint, dq->period_ticks, step_len_ceil);
    /* Derive step_len for the exact n_steps this period (floor or ceil of v),
     * so the PIO pulse rate matches the intended physical step count. */
    int32_t step_len_ticks = calculate_step_len(n_steps * 65536, dq->period_ticks, dq->max_vel_q);
```

with:

```c
    int32_t step_len_ceil  = calculate_step_len(step_count_q, dq->period_ticks, dq->max_vel_q);
    int32_t n_steps        = plan_steps(plan_vel_q, joint, dq->period_ticks, step_len_ceil);
    /* Use exact fractional step_len for PIO timing so the step rate matches the
     * commanded fractional velocity and steps span period boundaries smoothly.
     * When n_steps==0 (sub-1-step, no step this period), send 0 to pause PIO. */
    int32_t step_len_ticks = (n_steps > 0)
        ? calculate_step_len_frac(step_count_q, dq->period_ticks, dq->max_vel_q)
        : 0;
```

The rest of `commit_steps` is unchanged.

### Step 5: Build and run all tests

```bash
make -C build_tests
ctest --test-dir build_tests --output-on-failure
```

Expected: all 75 tests pass. In particular:
- `test_do_steps_fractional_uses_frac_step_len` — PASS (step_len=28904)
- `test_do_steps_fractional_step_len_stable` — PASS (28904 across all 4 periods)
- `test_do_steps_velmode_frac_1_5` — PASS (still 150 steps in 100 periods)
- `test_do_steps_velmode_frac_10_5` — PASS (still 1050 steps in 100 periods)
- All existing integer-velocity step_len tests — PASS (formula identical at integer v)

### Step 6: Commit

```bash
git add src/rp2040/pio.c src/test/rp_pio_test.c
git commit -m "feat: use fractional step_len for PIO timing at non-integer velocities"
```
