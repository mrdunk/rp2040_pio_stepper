# pio.c Readability and Testability Refactor — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract three sub-functions from `do_steps()`, add a test-reset hook, fix two latent bugs, and write a full CMocka test suite for `src/rp2040/pio.c`.

**Architecture:** All extracted helpers (`drain_rx_fifo`, `calculate_step_len`) live in `pio.c` and are exposed in `pio.h` only under `#ifdef BUILD_TESTS`, following the pattern of `timing_reset_for_test()`. `do_steps()` becomes a coordinator. Tests use the existing CMocka + linker-wrap pattern; PIO mock functions that need per-test control are wrapped via `--wrap=`. Runtime behaviour is unchanged.

**Tech Stack:** C99, CMocka, CMake, RP2040 SDK (mocked for tests via `BUILD_TESTS`), linker `--wrap=` interception.

---

## File Map

| File | Action | Purpose |
|------|--------|---------|
| `src/rp2040/pio.c` | Modify | Extract helpers, add reset hook, fix bugs |
| `src/rp2040/pio.h` | Modify | Expose test-only declarations under `#ifdef BUILD_TESTS` |
| `src/test/rp_pio_test.c` | Create | CMocka tests for pio.c |
| `src/test/CMakeLists.txt` | Modify | Add `rpPioTest` target |

---

## Task 1: Fix two latent bugs in pio.c

**Files:**
- Modify: `src/rp2040/pio.c:170-175` (static declarations inside `do_steps`)
- Modify: `src/rp2040/pio.c:292` (the `velocity_requested_tm1` assignment)

- [ ] **Step 1: Remove the dead `last_velocity` variable**

In `src/rp2040/pio.c`, inside `do_steps()`, find and remove:
```c
static double last_velocity[MAX_JOINT] = {0, 0, 0, 0};
```
And remove the write at the bottom of `do_steps()`:
```c
last_velocity[joint] = velocity;
```
(`last_velocity` is written but never read — it is dead code.)

- [ ] **Step 2: Make the narrowing cast explicit**

Find:
```c
int32_t velocity_requested_tm1 = velocity;
```
Replace with:
```c
int32_t velocity_requested_tm1 = (int32_t)velocity;
```

- [ ] **Step 3: Verify the firmware still builds**

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON && make -C build_tests 2>&1 | tail -20
```
Expected: zero errors, zero new warnings.

- [ ] **Step 4: Commit**

```bash
git add src/rp2040/pio.c
git commit -m "fix: remove dead last_velocity variable; make velocity cast explicit"
```

---

## Task 2: Add pio_reset_for_test() and expose helpers in pio.h

This task adds the test-reset hook and declares the helpers that later tasks will implement.

**Files:**
- Modify: `src/rp2040/pio.c` (add reset function at bottom)
- Modify: `src/rp2040/pio.h` (add BUILD_TESTS section)

- [ ] **Step 1: Add BUILD_TESTS section to pio.h**

The current `src/rp2040/pio.h` ends with:
```c
/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t joint);

#endif  // PIO__H
```

Replace those last three lines with:
```c
/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t joint);

#ifdef BUILD_TESTS
/* Reset all static state in pio.c — used by test setup fixtures only. */
void pio_reset_for_test(void);

/* Exposed for unit testing only. */
double get_velocity(uint32_t update_period_us, uint8_t joint,
                    int32_t abs_pos_achieved, double abs_pos_requested,
                    double expected_velocity);
int32_t drain_rx_fifo(uint32_t sm, int32_t current_pos);
int32_t calculate_step_len(double step_count, double update_period_ticks,
                           double max_velocity);
#endif  // BUILD_TESTS

#endif  // PIO__H
```

- [ ] **Step 2: Add pio_reset_for_test() to pio.c**

At the very bottom of `src/rp2040/pio.c` (after `do_steps`), add:

```c
#ifdef BUILD_TESTS
void pio_reset_for_test(void) {
    /* sm arrays — file-scope statics */
    for (int j = 0; j < MAX_JOINT; j++) {
        sm0[j] = 0;
        sm1[j] = 0;
    }

    /* init_pio statics — reset by reinitialising through a compound literal
     * trick is not possible for static-locals; use sentinel approach instead.
     * We expose a flag: set programs_loaded sentinel via a helper. */

    /* get_velocity static — holdoff */
    /* do_steps statics — last_pos_*, last_enabled, dir_change, last_direction, count */

    /* These static-local variables cannot be accessed directly from outside
     * the function. Use a flag to tell each function to self-reset on next call. */
}
#endif  // BUILD_TESTS
```

**Wait — static-local variables cannot be reset from outside their containing function.**
The correct pattern (used in `timing.c`) is to make the reset-needed state accessible.
The simplest approach: convert the relevant static locals into file-scope statics so
`pio_reset_for_test()` can zero them directly. Do this now:

In `src/rp2040/pio.c`, move these static-local declarations from inside their functions
to file scope (just below the `sm0`/`sm1` declarations at line 28):

From inside `init_pio()`, move:
```c
static bool init_done[MAX_JOINT] = {false, false, false, false};
static uint32_t offset_pio0 = 0;
static uint32_t offset_pio1 = 0;
static uint8_t programs_loaded = 0;
```

From inside `get_velocity()`, move:
```c
static int32_t holdoff[MAX_JOINT] = {0, 0, 0, 0};
```

From inside `do_steps()`, move:
```c
static uint32_t count = 0;
static int32_t last_pos_requested[MAX_JOINT] = {0, 0, 0, 0};
static int32_t last_pos_achieved[MAX_JOINT] = {0, 0, 0, 0};
static uint32_t last_enabled[MAX_JOINT] = {0, 0, 0, 0};
static size_t dir_change_count[MAX_JOINT] = {0, 0, 0, 0};
static uint32_t last_direction[MAX_JOINT] = {0, 0, 0, 0};
```

Leave `clock_multiplier` (it is `const`) inside `do_steps()`.

Now replace the skeleton `pio_reset_for_test()` with the real implementation:

```c
#ifdef BUILD_TESTS
void pio_reset_for_test(void) {
    for (int j = 0; j < MAX_JOINT; j++) {
        sm0[j]                = 0;
        sm1[j]                = 0;
        init_done[j]          = false;
        holdoff[j]            = 0;
        last_pos_requested[j] = 0;
        last_pos_achieved[j]  = 0;
        last_enabled[j]       = 0;
        dir_change_count[j]   = 0;
        last_direction[j]     = 0;
    }
    offset_pio0     = 0;
    offset_pio1     = 0;
    programs_loaded = 0;
    count           = 0;
}
#endif  // BUILD_TESTS
```

- [ ] **Step 3: Verify the firmware still compiles**

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON && make -C build_tests 2>&1 | tail -20
```
Expected: zero errors.

- [ ] **Step 4: Commit**

```bash
git add src/rp2040/pio.c src/rp2040/pio.h
git commit -m "refactor: promote pio.c statics to file scope; add pio_reset_for_test()"
```

---

## Task 3: Wire up CMakeLists and create empty test skeleton

**Files:**
- Modify: `src/test/CMakeLists.txt`
- Create: `src/test/rp_pio_test.c`

- [ ] **Step 1: Add rpPioTest target to CMakeLists.txt**

Append the following block at the end of `src/test/CMakeLists.txt` (before the final blank line):

```cmake
add_executable(
  rpPioTest
  ${CMAKE_CURRENT_SOURCE_DIR}/rp_pio_test.c
  ${CMAKE_SOURCE_DIR}/src/rp2040/pio.c
  ${CMAKE_SOURCE_DIR}/src/rp2040/config.c
  ${CMAKE_CURRENT_SOURCE_DIR}/mocks/pio_mocks.c
  ${CMAKE_CURRENT_SOURCE_DIR}/mocks/rp_mocks.c
)
target_link_libraries(
  rpPioTest
  cmocka
  m
  -Wl,--wrap=pio_sm_get_rx_fifo_level
  -Wl,--wrap=pio_sm_get_blocking
  -Wl,--wrap=pio_sm_put
  -Wl,--wrap=pio_sm_is_tx_fifo_empty
)
add_test(
  rpPioTest
  rpPioTest
)
```

- [ ] **Step 2: Create the test file skeleton**

Create `src/test/rp_pio_test.c`:

```c
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <string.h>
#include <cmocka.h>

#include "../rp2040/pio.h"
#include "../rp2040/config.h"

extern volatile struct ConfigGlobal config;

/* ── PIO mock state ── */
static size_t  mock_rx_fifo_level     = 0;
static int32_t mock_rx_values[8]      = {0};
static size_t  mock_rx_index          = 0;
static uint32_t last_pio_put_value    = 0;
static int     mock_tx_fifo_empty     = 0;

size_t __wrap_pio_sm_get_rx_fifo_level(size_t pio, size_t sm) {
    (void)pio; (void)sm;
    return mock_rx_fifo_level;   /* drain_rx_fifo reads level once, loops itself */
}

size_t __wrap_pio_sm_get_blocking(size_t pio, size_t sm) {
    (void)pio; (void)sm;
    if (mock_rx_index < 8) {
        return (size_t)mock_rx_values[mock_rx_index++];
    }
    return 0;
}

void __wrap_pio_sm_put(size_t pio, size_t sm, size_t data) {
    (void)pio; (void)sm;
    last_pio_put_value = (uint32_t)data;
}

int __wrap_pio_sm_is_tx_fifo_empty(size_t pio, size_t sm) {
    (void)pio; (void)sm;
    return mock_tx_fifo_empty;
}

/* ── Setup / Teardown ── */
static int test_setup(void **state) {
    (void)state;
    pio_reset_for_test();
    init_config();
    config.update_time_us = 1000;  /* init_config() does not reset this */
    for (size_t j = 0; j < MAX_JOINT; j++) {
        config.joint[j].updated_from_c0  = 0;
        config.joint[j].updated_from_c1  = 0;
        config.joint[j].overrun_count    = 0;
        config.joint[j].underrun_count   = 0;
        config.joint[j].enabled          = 0;
        config.joint[j].io_pos_step      = 1;  /* valid pin (0–31) */
        config.joint[j].io_pos_dir       = 2;  /* valid pin (0–31) */
        config.joint[j].abs_pos_requested  = 0;
        config.joint[j].abs_pos_achieved   = 0;
        config.joint[j].velocity_requested = 0;
        config.joint[j].max_velocity       = 50.0;
    }
    mock_rx_fifo_level  = 0;
    mock_rx_index       = 0;
    last_pio_put_value  = 0;
    mock_tx_fifo_empty  = 0;
    memset(mock_rx_values, 0, sizeof(mock_rx_values));
    return 0;
}

/* placeholder — tests added in subsequent tasks */
static void test_placeholder(void **state) {
    (void)state;
    assert_true(1);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_placeholder, test_setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
```

- [ ] **Step 3: Build and run to verify skeleton compiles and passes**

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON && make -C build_tests rpPioTest
ctest --test-dir build_tests -R rpPioTest --output-on-failure
```
Expected: 1 test passes.

- [ ] **Step 4: Commit**

```bash
git add src/test/rp_pio_test.c src/test/CMakeLists.txt
git commit -m "test: add rpPioTest skeleton and CMakeLists entry"
```

---

## Task 4: Extract drain_rx_fifo() — TDD

**Files:**
- Modify: `src/rp2040/pio.c`
- Modify: `src/test/rp_pio_test.c`

The goal: extract lines 237–241 of `do_steps()` into a named helper.

### 4a — Write failing tests

- [ ] **Step 1: Add drain_rx_fifo tests to rp_pio_test.c**

Replace the `test_placeholder` function and its entry in `main()` with the following.
Add these three test functions before `main()`:

```c
/* drain_rx_fifo: FIFO is empty → returns current_pos unchanged */
static void test_drain_rx_fifo_empty_returns_current(void **state) {
    (void)state;
    mock_rx_fifo_level = 0;
    int32_t result = drain_rx_fifo(0, 42);
    assert_int_equal(result, 42);
}

/* drain_rx_fifo: single entry → returns that value */
static void test_drain_rx_fifo_single_entry(void **state) {
    (void)state;
    mock_rx_fifo_level  = 1;
    mock_rx_values[0]   = 99;
    int32_t result = drain_rx_fifo(0, 0);
    assert_int_equal(result, 99);
}

/* drain_rx_fifo: multiple entries → returns only the last */
static void test_drain_rx_fifo_keeps_last(void **state) {
    (void)state;
    mock_rx_fifo_level  = 3;
    mock_rx_values[0]   = 10;
    mock_rx_values[1]   = 20;
    mock_rx_values[2]   = 30;
    int32_t result = drain_rx_fifo(0, 0);
    assert_int_equal(result, 30);
}
```

Update `main()` to:
```c
int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_drain_rx_fifo_empty_returns_current, test_setup),
        cmocka_unit_test_setup(test_drain_rx_fifo_single_entry,          test_setup),
        cmocka_unit_test_setup(test_drain_rx_fifo_keeps_last,            test_setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
```

- [ ] **Step 2: Run — expect compile failure (drain_rx_fifo not defined yet)**

```bash
make -C build_tests rpPioTest 2>&1 | tail -10
```
Expected: linker error — `undefined reference to drain_rx_fifo`.

### 4b — Implement

- [ ] **Step 3: Add drain_rx_fifo() to pio.c**

Add this function immediately before `get_velocity()` in `src/rp2040/pio.c`:

```c
/* Drain PIO1's RX FIFO and return the last feedback position received.
 * Returns current_pos unchanged if the FIFO is empty. */
int32_t drain_rx_fifo(uint32_t sm, int32_t current_pos) {
    uint8_t fifo_len = pio_sm_get_rx_fifo_level(pio1, sm);
    while (fifo_len > 0) {
        current_pos = pio_sm_get_blocking(pio1, sm);
        fifo_len--;
    }
    return current_pos;
}
```

Replace the original inlined FIFO drain in `do_steps()` (lines 237–241 of the original
file — the block that starts with `uint8_t fifo_len = ...`) with:

```c
abs_pos_achieved = drain_rx_fifo(sm1[joint], abs_pos_achieved);
```

- [ ] **Step 4: Run tests — expect all three to pass**

```bash
make -C build_tests rpPioTest && ctest --test-dir build_tests -R rpPioTest --output-on-failure
```
Expected: `3 tests passed`.

- [ ] **Step 5: Commit**

```bash
git add src/rp2040/pio.c src/test/rp_pio_test.c
git commit -m "refactor: extract drain_rx_fifo(); add 3 unit tests"
```

---

## Task 5: Extract calculate_step_len() — TDD

**Files:**
- Modify: `src/rp2040/pio.c`
- Modify: `src/test/rp_pio_test.c`

### 5a — Write failing tests

- [ ] **Step 1: Add calculate_step_len tests to rp_pio_test.c**

Add these test functions before `main()`:

```c
/* calculate_step_len: normal step count → positive result */
static void test_calculate_step_len_normal(void **state) {
    (void)state;
    /* step_count=2.0, period_ticks=133000, max_velocity=50.0
     * expected = (133000 / (2.0 * 2.0)) - 9.0 = 33241
     * min      = (133000 / (50.0 * 2.0)) - 9.0 = 1321
     * result   = max(33241, 1321) = 33241 */
    int32_t result = calculate_step_len(2.0, 133000.0, 50.0);
    assert_int_equal(result, 33241);
}

/* calculate_step_len: step_count exceeds max_velocity → clamped to min */
static void test_calculate_step_len_clamped(void **state) {
    (void)state;
    /* step_count=200.0 (very fast), period_ticks=133000, max_velocity=50.0
     * unclamped = (133000 / (200.0 * 2.0)) - 9.0 = 323.5
     * min       = (133000 / (50.0 * 2.0)) - 9.0  = 1321
     * result    = max(323, 1321) = 1321 */
    int32_t result = calculate_step_len(200.0, 133000.0, 50.0);
    assert_int_equal(result, 1321);
}

/* calculate_step_len: below MIN_STEP_COUNT threshold → 0 */
static void test_calculate_step_len_below_threshold(void **state) {
    (void)state;
    /* MIN_STEP_COUNT = 0.0625; 0.05 < 0.0625 → returns 0 */
    int32_t result = calculate_step_len(0.05, 133000.0, 50.0);
    assert_int_equal(result, 0);
}
```

Add all three to `main()`:
```c
cmocka_unit_test_setup(test_calculate_step_len_normal,          test_setup),
cmocka_unit_test_setup(test_calculate_step_len_clamped,         test_setup),
cmocka_unit_test_setup(test_calculate_step_len_below_threshold, test_setup),
```

- [ ] **Step 2: Run — expect compile failure**

```bash
make -C build_tests rpPioTest 2>&1 | tail -10
```
Expected: linker error — `undefined reference to calculate_step_len`.

### 5b — Implement

- [ ] **Step 3: Add calculate_step_len() to pio.c**

Add this function immediately before `do_steps()` in `src/rp2040/pio.c`:

```c
/* Compute the PIO step-timer length in clock ticks.
 * Returns 0 if step_count is below MIN_STEP_COUNT (too slow to drive PIO). */
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

Replace the original inlined step-length block in `do_steps()` (the `if(step_count > MIN_STEP_COUNT)` block, lines 261–269 of the original file) with:

```c
int32_t step_len_ticks = calculate_step_len(step_count, update_period_ticks, max_velocity);
```

Remove the now-redundant `int32_t step_len_ticks = 0;` declaration that preceded it.

- [ ] **Step 4: Run tests — expect all six to pass**

```bash
make -C build_tests rpPioTest && ctest --test-dir build_tests -R rpPioTest --output-on-failure
```
Expected: `6 tests passed`.

- [ ] **Step 5: Commit**

```bash
git add src/rp2040/pio.c src/test/rp_pio_test.c
git commit -m "refactor: extract calculate_step_len(); add 3 unit tests"
```

---

## Task 6: Extract issue_pio_step() — no separate tests (tested via do_steps)

This extraction is for readability only. `issue_pio_step` is a 1-line wrapper; it is
implicitly exercised by the `do_steps` tests in Task 8.

**Files:**
- Modify: `src/rp2040/pio.c`

- [ ] **Step 1: Add issue_pio_step() to pio.c**

Add this static function immediately before `do_steps()`:

```c
/* Write the packed step command to PIO0's TX FIFO if it is empty.
 * Encoding: lower bit = direction, upper bits = half-period in ticks. */
static void issue_pio_step(uint32_t joint, int32_t step_len_ticks,
                           uint32_t direction) {
    if (pio_sm_is_tx_fifo_empty(pio0, sm0[joint])) {
        pio_sm_put(pio0, sm0[joint], (uint32_t)((step_len_ticks << 1) | (int32_t)direction));
    }
}
```

Replace the original two-line block in `do_steps()`:
```c
if(pio_sm_is_tx_fifo_empty(pio0, sm0[joint])) {
    pio_sm_put(pio0, sm0[joint], (step_len_ticks << 1) | direction);
}
```
with:
```c
issue_pio_step(joint, step_len_ticks, direction);
```

- [ ] **Step 2: Run tests — all six still pass**

```bash
make -C build_tests rpPioTest && ctest --test-dir build_tests -R rpPioTest --output-on-failure
```
Expected: `6 tests passed`.

- [ ] **Step 3: Commit**

```bash
git add src/rp2040/pio.c
git commit -m "refactor: extract issue_pio_step() for clarity"
```

---

## Task 7: Tests for get_velocity()

**Files:**
- Modify: `src/test/rp_pio_test.c`

`get_velocity()` is a stateful function (static `holdoff[]` is now file-scope).
`pio_reset_for_test()` resets `holdoff[]`, so each test starts clean.

- [ ] **Step 1: Add get_velocity tests to rp_pio_test.c**

Add these four test functions before `main()`:

```c
/* get_velocity: near-zero position diff → returns 0 */
static void test_get_velocity_zero_pos_diff(void **state) {
    (void)state;
    /* abs_pos_requested == abs_pos_achieved → position_diff = 0.0 */
    double v = get_velocity(1000, 0, 10, 10.0, 5.0);
    assert_true(v == 0.0);
}

/* get_velocity: direction disagreement → returns 0 */
static void test_get_velocity_direction_disagreement(void **state) {
    (void)state;
    /* position_diff = +10 (move forward), velocity component = -5/1000 (backward) */
    double v = get_velocity(1000, 0, 0, 10.0, -5000.0);
    assert_true(v == 0.0);
}

/* get_velocity: normal forward motion → returns positive combined velocity */
static void test_get_velocity_normal_forward(void **state) {
    (void)state;
    /* position_diff = 10, velocity = 5000/1000 = 5.0
     * combined = 10 * 0.1 + 5.0 * 0.85 = 1.0 + 4.25 = 5.25 */
    double v = get_velocity(1000, 0, 0, 10.0, 5000.0);
    assert_true(v > 0.0);
}

/* get_velocity: slow step triggers holdoff; next call returns 0 */
static void test_get_velocity_holdoff(void **state) {
    (void)state;
    /* combined_vel = 0.5 * 0.1 + 0.001 * 0.85 = 0.05085 → holdoff set
     * first call returns the combined_vel (not 0), second call returns 0 */
    double v1 = get_velocity(1000, 0, 0, 0.5, 1.0);
    assert_true(v1 > 0.0);     /* first call returns combined_vel */
    double v2 = get_velocity(1000, 0, 0, 0.5, 1.0);
    assert_true(v2 == 0.0);    /* holdoff active: returns 0 */
}
```

Add all four to `main()`:
```c
cmocka_unit_test_setup(test_get_velocity_zero_pos_diff,        test_setup),
cmocka_unit_test_setup(test_get_velocity_direction_disagreement, test_setup),
cmocka_unit_test_setup(test_get_velocity_normal_forward,       test_setup),
cmocka_unit_test_setup(test_get_velocity_holdoff,              test_setup),
```

- [ ] **Step 2: Run tests — expect all 10 to pass**

```bash
make -C build_tests rpPioTest && ctest --test-dir build_tests -R rpPioTest --output-on-failure
```
Expected: `10 tests passed`.

- [ ] **Step 3: Commit**

```bash
git add src/test/rp_pio_test.c
git commit -m "test: add 4 get_velocity() unit tests"
```

---

## Task 8: Integration tests for do_steps()

**Files:**
- Modify: `src/test/rp_pio_test.c`

These tests drive `do_steps()` end-to-end using real `config.c` and wrapped PIO mocks.
Each test calls `test_setup()` (via `cmocka_unit_test_setup`) which resets all state.

- [ ] **Step 1: Add do_steps tests to rp_pio_test.c**

Add these four test functions before `main()`:

```c
/* do_steps: update_period == 0 → puts 0 to FIFO, returns 0 */
static void test_do_steps_zero_period(void **state) {
    (void)state;
    config.update_time_us = 0;   /* get_period() returns 0 */
    /* pio_sm_is_tx_fifo_full (mock, not wrapped) returns 0 — FIFO not full */
    uint8_t result = do_steps(0);
    assert_int_equal(result, 0);
    assert_int_equal(last_pio_put_value, 0);
}

/* do_steps: joint disabled → puts 0 to FIFO when empty, returns 0 */
static void test_do_steps_disabled(void **state) {
    (void)state;
    config.update_time_us        = 1000;
    config.joint[0].enabled      = 0;
    config.joint[0].updated_from_c0 = 1;
    mock_tx_fifo_empty           = 1;
    uint8_t result = do_steps(0);
    assert_int_equal(result, 0);
    assert_int_equal(last_pio_put_value, 0);
}

/* do_steps: no new core0 data (updated == 0) → returns 0, no PIO write */
static void test_do_steps_no_update(void **state) {
    (void)state;
    config.update_time_us           = 1000;
    config.joint[0].enabled         = 1;
    config.joint[0].io_pos_step     = 1;
    config.joint[0].io_pos_dir      = 2;
    config.joint[0].updated_from_c0 = 0;   /* no new data */
    mock_tx_fifo_empty              = 1;
    /* First call: enabled transition 0→1 triggers init_pio; updated==0 → returns 0 */
    uint8_t result = do_steps(0);
    assert_int_equal(result, 0);
}

/* do_steps: valid position request with empty FIFO → non-zero step written to PIO */
static void test_do_steps_normal_step(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].io_pos_step        = 1;
    config.joint[0].io_pos_dir         = 2;
    config.joint[0].abs_pos_requested  = 10.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].velocity_requested = 5000.0;  /* 5 steps/period at 1000µs */
    config.joint[0].max_velocity       = 50.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;    /* no PIO feedback, use config value */

    uint8_t result = do_steps(0);

    assert_true(result > 0);
    assert_true(last_pio_put_value != 0);
    /* direction bit (LSB) should be 1 (forward) */
    assert_int_equal(last_pio_put_value & 1, 1);
}
```

Add all four to `main()`:
```c
cmocka_unit_test_setup(test_do_steps_zero_period,  test_setup),
cmocka_unit_test_setup(test_do_steps_disabled,     test_setup),
cmocka_unit_test_setup(test_do_steps_no_update,    test_setup),
cmocka_unit_test_setup(test_do_steps_normal_step,  test_setup),
```

- [ ] **Step 2: Run tests — expect all 14 to pass**

```bash
make -C build_tests rpPioTest && ctest --test-dir build_tests -R rpPioTest --output-on-failure
```
Expected: `14 tests passed`.

- [ ] **Step 3: Commit**

```bash
git add src/test/rp_pio_test.c
git commit -m "test: add 4 do_steps() integration tests"
```

---

## Task 9: Final verification — full test suite

- [ ] **Step 1: Rebuild from scratch and run all tests**

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON
make -C build_tests
ctest --test-dir build_tests --output-on-failure
```
Expected: all existing tests plus `rpPioTest` pass; zero failures.

- [ ] **Step 2: Verify firmware build is unaffected**

```bash
cmake -B build -S .
make -C build stepper_control 2>&1 | tail -10
```
Expected: clean build, no errors, no new warnings.

- [ ] **Step 3: If all green, invoke superpowers:finishing-a-development-branch**
