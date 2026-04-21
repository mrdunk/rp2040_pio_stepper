# Whole-Step Accumulator with Carry Tracking — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the holdoff-based slow-speed suppression in `pio.c` with a fractional step accumulator, add an upper clamp to step length, and introduce `plan_steps()` to produce whole-step counts with carry tracking.

**Architecture:** Four TDD commits: (1) upper clamp on `calculate_step_len()`, (2) holdoff removal from `get_velocity()`, (3) new `plan_steps()` function, (4) wire `plan_steps()` into `do_steps()`. All changes confined to three files.

**Tech Stack:** C11, cmocka, existing `BUILD_TESTS` guard pattern for test-only declarations.

---

## File map

| File | Role |
|---|---|
| `src/rp2040/pio.c` | All production changes |
| `src/rp2040/pio.h` | Expose `plan_steps()` under `BUILD_TESTS` |
| `src/test/rp_pio_test.c` | Add/update/remove tests |

---

## Build and test command

Run after every change:

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON 2>/dev/null && \
make -C build_tests -j$(nproc) 2>&1 | grep -E "error:|warning:|Built|failed" && \
ctest --test-dir build_tests --output-on-failure
```

All 12 test suites must pass before committing.

---

## Task 1: Add upper clamp to `calculate_step_len()`

**Files:**
- Modify: `src/rp2040/pio.c:185-195`
- Modify: `src/test/rp_pio_test.c` (add one test, append to `tests[]` array)

### Design note

`calculate_step_len()` currently clamps `len` to `[min_len, ∞)`. A very slow velocity
produces an astronomically large `len`. Adding `max_len = period_ticks / 2 − overhead`
caps the step half-period to at most one full loop period, ensuring `plan_steps()`
carry is always in `[0, period_ticks)`.

- [ ] **Step 1: Write the failing test**

Add this test function to `src/test/rp_pio_test.c`, after `test_calculate_step_len_below_threshold`:

```c
/* calculate_step_len: very slow step_count -> clamped to max_len */
static void test_calculate_step_len_upper_clamp(void **state) {
    (void)state;
    /* step_count=0.1, period_ticks=133000, max_velocity=50.0
     * raw     = (133000 / (0.1 * 2.0)) - 9.0 = 665000 - 9 = 664991
     * min_len = (133000 / (50.0 * 2.0)) - 9.0 = 1330 - 9 = 1321
     * max_len = (133000 / 2.0) - 9.0 = 66500 - 9 = 66491
     * result  = 66491 (clamped to max_len) */
    int32_t result = calculate_step_len(0.1, 133000.0, 50.0);
    assert_int_equal(result, 66491);
}
```

Register it in `main()` inside the `tests[]` array:

```c
cmocka_unit_test_setup(test_calculate_step_len_upper_clamp, test_setup),
```

- [ ] **Step 2: Run — confirm only the new test fails**

```
rpPioTest: test_calculate_step_len_upper_clamp FAILED
```

- [ ] **Step 3: Implement the upper clamp in `calculate_step_len()`**

Current function (`src/rp2040/pio.c:185-195`):

```c
int32_t calculate_step_len(double step_count, double update_period_ticks,
                           double max_velocity) {
    if (step_count <= MIN_STEP_COUNT) {
        return 0;
    }
    int32_t len = (int32_t)((update_period_ticks / (step_count * STEP_PIO_MULTIPLIER))
                            - STEP_PIO_LEN_OVERHEAD);
    int32_t min_len = (int32_t)((update_period_ticks / (fabs(max_velocity) * STEP_PIO_MULTIPLIER))
                                - STEP_PIO_LEN_OVERHEAD);
    return len < min_len ? min_len : len;
}
```

Replace with:

```c
int32_t calculate_step_len(double step_count, double update_period_ticks,
                           double max_velocity) {
    if (step_count <= MIN_STEP_COUNT) {
        return 0;
    }
    int32_t len = (int32_t)((update_period_ticks / (step_count * STEP_PIO_MULTIPLIER))
                            - STEP_PIO_LEN_OVERHEAD);
    int32_t min_len = (int32_t)((update_period_ticks / (fabs(max_velocity) * STEP_PIO_MULTIPLIER))
                                - STEP_PIO_LEN_OVERHEAD);
    int32_t max_len = (int32_t)(update_period_ticks / STEP_PIO_MULTIPLIER
                                - STEP_PIO_LEN_OVERHEAD);
    int32_t clamped = len < min_len ? min_len : len;
    return clamped > max_len ? max_len : clamped;
}
```

- [ ] **Step 4: Run — all 12 suites pass**

- [ ] **Step 5: Commit**

```bash
git add src/rp2040/pio.c src/test/rp_pio_test.c
git commit -m "feat: add upper clamp to calculate_step_len(); cap at one loop period"
```

---

## Task 2: Remove `holdoff` from `get_velocity()`

**Files:**
- Modify: `src/rp2040/pio.c` (remove static, remove code in `get_velocity()`, remove reset in `pio_reset_for_test()`)
- Modify: `src/test/rp_pio_test.c` (remove one test)

### Design note

The holdoff counter suppresses steps for several cycles when velocity is very slow,
then issues a burst. The fractional accumulator (Task 3) replaces this: fractional
steps accumulate every period until a whole step is due. Holdoff can be deleted.

- [ ] **Step 1: Remove the holdoff test**

Delete the entire `test_get_velocity_holdoff` function from `src/test/rp_pio_test.c`:

```c
/* get_velocity: slow step triggers holdoff; next call returns 0 */
static void test_get_velocity_holdoff(void **state) {
    (void)state;
    /* combined = 0.5*0.1 + 0.001*0.85 = 0.05085 < 1.0 -> holdoff set */
    double v1 = get_velocity(1000, 0, 0, 0.5, 1.0);
    assert_true(v1 > 0.0);    /* first call returns combined_vel */
    double v2 = get_velocity(1000, 0, 0, 0.5, 1.0);
    assert_true(v2 == 0.0);   /* holdoff active */
}
```

Also remove its entry from the `tests[]` array in `main()`:

```c
cmocka_unit_test_setup(test_get_velocity_holdoff,                test_setup),
```

- [ ] **Step 2: Remove holdoff static declaration from `pio.c`**

In `src/rp2040/pio.c`, delete this line (currently around line 37):

```c
static int32_t  holdoff[MAX_JOINT]            = {0, 0, 0, 0};
```

- [ ] **Step 3: Remove holdoff logic from `get_velocity()`**

Current holdoff block in `get_velocity()` (`src/rp2040/pio.c`, after the direction check):

```c
  // Short steps that span multiple cycles are inclined to "clump" together.
  // If a single step is calculated to take more than one cycle, don't allow
  // any steps in the following cycles.
  if(fabs(combined_vel) < 1.0) {
    if(holdoff[joint] > 0) {
      holdoff[joint]--;
      return 0.0;
    }
    holdoff[joint] = 0.75 / fabs(combined_vel);
  } else if(holdoff[joint] > 0) {
    holdoff[joint]--;
  }
```

Delete all of it. The function should end with:

```c
  return combined_vel;
}
```

- [ ] **Step 4: Remove holdoff reset from `pio_reset_for_test()`**

In `src/rp2040/pio.c`, inside the `#ifdef BUILD_TESTS` block, delete:

```c
        holdoff[j]            = 0;
```

- [ ] **Step 5: Run — all 12 suites pass (one fewer test in rpPioTest)**

- [ ] **Step 6: Commit**

```bash
git add src/rp2040/pio.c src/test/rp_pio_test.c
git commit -m "refactor: replace holdoff with fractional accumulator (remove holdoff)"
```

---

## Task 3: Implement `plan_steps()`

**Files:**
- Modify: `src/rp2040/pio.c` (add statics, implement function, update `pio_reset_for_test()`)
- Modify: `src/rp2040/pio.h` (expose under `BUILD_TESTS`)
- Modify: `src/test/rp_pio_test.c` (add tests)

### Design note

`plan_steps()` owns two file-scope statics: `step_accumulator[MAX_JOINT]` (fractional
steps not yet issued) and `carry_ticks[MAX_JOINT]` (ticks of in-progress step at
period boundary). Each call: add `|velocity|` to accumulator, extract whole steps,
reduce by how many fit in `available_ticks = period_ticks − carry`, return excess
to accumulator, update carry.

**Carry is always 0 in normal operation** (provable: `n_steps ≤ max_steps` ensures
`n_steps * step_period ≤ available_ticks`). The carry machinery is correct and
decrements when `n_steps = 0`, but will not activate in practice given the
`max_len` cap from Task 1. It is included for correctness.

- [ ] **Step 1: Write failing tests**

Add these test functions to `src/test/rp_pio_test.c` after the `clamp_accel` tests:

```c
/* plan_steps: fractional accumulation -> step fires on 4th call */
static void test_plan_steps_fractional_accumulation(void **state) {
    (void)state;
    /* velocity=0.3, step_len=1 (tiny: max_steps >> desired, not the limiter)
     * acc after calls: 0.3, 0.6, 0.9, 1.2 -> n=0,0,0,1 */
    assert_int_equal(plan_steps(0.3, 0, 133000.0, 1), 0);
    assert_int_equal(plan_steps(0.3, 0, 133000.0, 1), 0);
    assert_int_equal(plan_steps(0.3, 0, 133000.0, 1), 0);
    assert_int_equal(plan_steps(0.3, 0, 133000.0, 1), 1);
}

/* plan_steps: correct total over 10 periods for fractional velocity */
static void test_plan_steps_total_over_ten_periods(void **state) {
    (void)state;
    /* velocity=2.7, step_len=1 -> 10 periods should produce 27 steps total */
    int32_t total = 0;
    for (int i = 0; i < 10; i++) {
        total += plan_steps(2.7, 0, 133000.0, 1);
    }
    assert_int_equal(total, 27);
}

/* plan_steps: excess steps returned to accumulator when max_steps limits output */
static void test_plan_steps_excess_returned_to_accumulator(void **state) {
    (void)state;
    /* step_len=24991: step_period=2*(24991+9)=50000
     * max_steps = floor(133000/50000) = 2
     * velocity=3.0 -> desired=3, capped to 2, excess 1 returned
     * next call velocity=0.0 -> acc=1.0 -> n=1 */
    assert_int_equal(plan_steps(3.0, 0, 133000.0, 24991), 2);
    assert_int_equal(plan_steps(0.0, 0, 133000.0, 24991), 1);
}

/* plan_steps: zero velocity -> no steps, accumulator stays 0 */
static void test_plan_steps_zero_velocity(void **state) {
    (void)state;
    assert_int_equal(plan_steps(0.0, 0, 133000.0, 13291), 0);
    assert_int_equal(plan_steps(0.0, 0, 133000.0, 13291), 0);
}
```

Register all four in `main()`:

```c
cmocka_unit_test_setup(test_plan_steps_fractional_accumulation,        test_setup),
cmocka_unit_test_setup(test_plan_steps_total_over_ten_periods,         test_setup),
cmocka_unit_test_setup(test_plan_steps_excess_returned_to_accumulator, test_setup),
cmocka_unit_test_setup(test_plan_steps_zero_velocity,                  test_setup),
```

- [ ] **Step 2: Run — confirm the four new tests fail to compile** (symbol not found)

- [ ] **Step 3: Expose `plan_steps()` in `pio.h`**

In `src/rp2040/pio.h`, inside the `#ifdef BUILD_TESTS` block, add after `int32_t drain_rx_fifo(...)`:

```c
int32_t plan_steps(double velocity, uint8_t joint,
                   double period_ticks, int32_t step_len);
```

- [ ] **Step 4: Add statics to `pio.c`**

In `src/rp2040/pio.c`, add these two lines alongside the other file-scope statics
(after `last_velocity[]`):

```c
static double  step_accumulator[MAX_JOINT] = {0.0, 0.0, 0.0, 0.0};
static int32_t carry_ticks[MAX_JOINT]      = {0, 0, 0, 0};
```

- [ ] **Step 5: Implement `plan_steps()`**

Add after `clamp_accel()` and before the `#ifdef BUILD_TESTS` block in `src/rp2040/pio.c`:

```c
int32_t plan_steps(double velocity, uint8_t joint,
                   double period_ticks, int32_t step_len) {
    step_accumulator[joint] += fabs(velocity);
    int32_t n_steps_desired = (int32_t)floor(step_accumulator[joint]);
    step_accumulator[joint] -= (double)n_steps_desired;

    int32_t step_period = 2 * (step_len + (int32_t)STEP_PIO_LEN_OVERHEAD);
    double available_ticks = period_ticks - (double)carry_ticks[joint];
    int32_t max_steps = (step_period > 0)
        ? (int32_t)(available_ticks / (double)step_period)
        : 0;
    if (max_steps < 0) max_steps = 0;

    int32_t n_steps = n_steps_desired < max_steps ? n_steps_desired : max_steps;
    step_accumulator[joint] += (double)(n_steps_desired - n_steps);

    int32_t new_carry = carry_ticks[joint] + n_steps * step_period - (int32_t)period_ticks;
    carry_ticks[joint] = new_carry > 0 ? new_carry : 0;

    return n_steps;
}
```

- [ ] **Step 6: Update `pio_reset_for_test()`**

In `src/rp2040/pio.c`, inside `pio_reset_for_test()`, add within the `for` loop:

```c
        step_accumulator[j] = 0.0;
        carry_ticks[j]      = 0;
```

- [ ] **Step 7: Run — all 12 suites pass**

- [ ] **Step 8: Commit**

```bash
git add src/rp2040/pio.c src/rp2040/pio.h src/test/rp_pio_test.c
git commit -m "feat: add plan_steps() with fractional step accumulator and carry tracking"
```

---

## Task 4: Wire `plan_steps()` into `do_steps()`

**Files:**
- Modify: `src/rp2040/pio.c` (`do_steps()` body only)
- Modify: `src/test/rp_pio_test.c` (add one test for n_steps=0 path)

### Design note

`do_steps()` currently calls `issue_pio_step()` unconditionally. After this task,
it calls `plan_steps()` and only issues a step if `n_steps > 0`; otherwise it
writes 0 to the FIFO to stop the PIO. The `step_len_ticks` variable is unchanged
(still passed to `update_joint_config`).

- [ ] **Step 1: Write the failing test for the n_steps=0 path**

Add to `src/test/rp_pio_test.c`:

```c
/* do_steps: enabled, no position error -> n_steps=0 -> puts 0 to FIFO */
static void test_do_steps_no_motion(void **state) {
    (void)state;
    /* All positions at zero and velocity_requested=0 -> get_velocity returns 0.0
     * -> step_len=0 -> plan_steps returns 0 -> PIO should receive 0. */
    config.joint[0].enabled            = 1;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].velocity_requested = 0.0;
    config.joint[0].max_velocity       = 50.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                  = 1;
    mock_rx_fifo_level                  = 0;

    uint8_t result = do_steps(0);

    assert_true(result > 0);
    assert_int_equal(last_pio_put_value, 0);
}
```

Register it in `main()`:

```c
cmocka_unit_test_setup(test_do_steps_no_motion, test_setup),
```

- [ ] **Step 2: Run — confirm the new test fails**

With the old code, `do_steps()` calls `issue_pio_step()` unconditionally, which only
writes to FIFO if empty. With velocity=0, step_len=0, direction=0, the call is
`issue_pio_step(joint, 0, 0)` → puts `(0 << 1) | 0 = 0`. So `last_pio_put_value = 0`.

The test actually passes with the old code too! Proceed — the test documents correct
behaviour and verifies it is preserved after the refactor.

- [ ] **Step 3: Rewrite the step-issue block in `do_steps()`**

Locate the section in `src/rp2040/pio.c` starting at `double step_count = fabs(velocity);`
and ending at `issue_pio_step(joint, step_len_ticks, direction);`. Replace it with:

```c
  double step_count = fabs(velocity);
  double update_period_ticks = update_period_us * RP2040_CLOCK_MHZ;

  int32_t step_len_ticks = calculate_step_len(step_count, update_period_ticks, max_velocity);
  int32_t n_steps = plan_steps(velocity, joint, update_period_ticks, step_len_ticks);

  uint32_t direction = (velocity > 0);

  if (n_steps > 0) {
    issue_pio_step(joint, step_len_ticks, direction);
  } else if (pio_sm_is_tx_fifo_empty(pio0, sm0[joint])) {
    pio_sm_put(pio0, sm0[joint], 0);
  }
```

- [ ] **Step 4: Run — all 12 suites pass**

Verify specifically:
- `test_do_steps_normal_step` still passes (non-zero put, direction bit = 1)
- `test_do_steps_accel_clamped` still passes (step_len values 13291 and 6641 unchanged)
- `test_do_steps_no_motion` passes

- [ ] **Step 5: Commit**

```bash
git add src/rp2040/pio.c src/test/rp_pio_test.c
git commit -m "feat: wire plan_steps() into do_steps(); gate issue_pio_step on n_steps > 0"
```

---

## Self-review

**Spec coverage check:**

| Spec requirement | Task |
|---|---|
| Only whole steps issued | Task 3 (`plan_steps` accumulator) + Task 4 (gate on `n_steps > 0`) |
| Fractional steps accumulate and carry forward | Task 3 |
| Carry reduces budget of next period | Task 3 (carry_ticks) |
| Step rate from velocity, not spread across period | Task 1 + Task 4 (unchanged `calculate_step_len` call) |
| `calculate_step_len` upper-clamped to one period | Task 1 |
| Holdoff removed | Task 2 |
| `pio_reset_for_test()` updated | Task 3 |
| `plan_steps()` exposed under `BUILD_TESTS` | Task 3 |

**Type consistency check:** `plan_steps()` signature in `pio.h` and `pio.c` match. `step_len_ticks` (int32_t) passed to both `plan_steps()` and `update_joint_config()`.

**Placeholder scan:** No TBD/TODO. All code blocks are complete and buildable.
