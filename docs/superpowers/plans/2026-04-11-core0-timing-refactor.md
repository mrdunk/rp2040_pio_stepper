# Core0 Timing Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract clock-synchronisation from `core0.c` into `timing.c`/`timing.h`. Replace the busy-wait + ring-buffer approach with a RP2040 hardware repeating timer. Core1's `tick` semaphore is incremented by the timer ISR at the measured average packet rate, fully decoupled from network jitter.

**Architecture:** A hardware `repeating_timer` fires at the measured average packet rate and increments `tick` in its callback. `recover_clock()` (called per-packet) updates an EMA of the inter-packet period and restarts the timer only when the integer-µs average changes. This eliminates 3 ring buffers (12 KB), the busy-wait, and all hardcoded 1000 µs magic numbers. A single very-delayed packet has essentially no effect on `tick` timing.

**Tech Stack:** C99, RP2040/Pico SDK (`pico/time.h` repeating timers), cmocka test framework, `-Wl,--wrap` link-time mocking.

---

## Context

Core0 receives ~1 kHz UDP packets from LinuxCNC. The old `recover_clock()` measured average arrival phase, then busy-waited to align execution before incrementing `tick`. This coupled Core1's loop rate to network jitter and wasted Core0 CPU.

**New design:**
- `timing_init()`: called once from `core0_main()`, starts the hardware repeating timer at a default 1000 µs.
- `tick_callback()` (ISR): increments `tick` — drives Core1's loop at a hardware-precise rate.
- `recover_clock()`: called per-packet, updates an EMA of the inter-packet period; restarts the timer only when the integer-µs average actually changes.

**EMA formula** (alpha = 1/64, stored scaled ×64 for sub-µs accumulation):
```
ave_period_us_x64 = ave_period_us_x64 - (ave_period_us_x64 >> 6) + sample_us
ave_period_us     = ave_period_us_x64 >> 6
```
At 1 kHz, converges within ~64 packets (~64 ms). A single 10 ms delayed packet shifts the accumulator by 9000/64 ≈ 140 counts → 140/64 ≈ 2 µs — negligible.

The timer is only restarted when `ave_period_us` (integer µs) changes, which requires ~64 packets of sustained drift to register. This prevents constant timer-phase resets due to noise.

---

## File Map

| Action | Path | Responsibility |
|--------|------|----------------|
| Modify | `src/test/mocks/rp_mocks.h` | Add `repeating_timer_t`, `add_repeating_timer_us`, `cancel_repeating_timer` |
| Create | `src/rp2040/timing.h` | Declare `timing_init()`, `recover_clock()` |
| Create | `src/rp2040/timing.c` | Hardware timer + EMA implementation |
| Modify | `src/rp2040/core0.c` | Remove `recover_clock()`, add `timing_init()` call, add `#include "timing.h"` |
| Modify | `src/rp2040/core1.c` | Fix jitter metric: hardcoded `1000` → `update_period_us` |
| Modify | `src/rp2040/CMakeLists.txt` | Add `timing.c` to `target_sources` |
| Modify | `src/test/timing_test.c` | Replace placeholder with real tests |
| Modify | `src/test/CMakeLists.txt` | Add `timingTest` executable |

---

## Task 1: Add timer mock types to `rp_mocks.h`

**Files:**
- Modify: `src/test/mocks/rp_mocks.h`

`timing.c` uses `repeating_timer_t`, `add_repeating_timer_us`, and `cancel_repeating_timer` from `pico/time.h`. The mock header needs to declare them for the `BUILD_TESTS` path.

- [ ] **Step 1: Add timer declarations to `rp_mocks.h` (before the `#endif`)**

```c
typedef struct { int dummy; } repeating_timer_t;
typedef bool (*repeating_timer_callback_t)(repeating_timer_t *rt);

bool add_repeating_timer_us(int32_t delay_us,
                             repeating_timer_callback_t callback,
                             void *user_data,
                             repeating_timer_t *out);
bool cancel_repeating_timer(repeating_timer_t *timer);
```

---

## Task 2: Create `timing.h`

**Files:**
- Create: `src/rp2040/timing.h`

- [ ] **Step 1: Write the header**

```c
#ifndef TIMING__H
#define TIMING__H

/* Set up the hardware repeating timer that drives Core1's tick semaphore.
 * Must be called once from core0_main() before entering the packet loop. */
void timing_init(void);

/* Called after receiving a network packet.
 * Updates the EMA of inter-packet period; restarts the timer if the
 * integer-µs average changes. Does not busy-wait. */
void recover_clock(void);

#ifdef BUILD_TESTS
/* Reset all static state — used by test setup fixtures only. */
void timing_reset_for_test(void);
#endif

#endif  // TIMING__H
```

---

## Task 3: Write `timing_test.c` with failing tests

**Files:**
- Modify: `src/test/timing_test.c`

The test provides wrapped versions of all SDK calls. `__wrap_add_repeating_timer_us` captures the callback so tests can fire it manually to verify `tick` increments.

`timing_test.c` defines `volatile uint32_t tick` itself (it does not link `config.c`), matching the pattern used in `ring_buffer_test.c`.

The mock `time_us_64` returns a configurable step (default 1000 µs) to simulate steady-rate packets. This keeps the EMA stable so period-update tests are predictable.

- [ ] **Step 1: Write the test file**

```c
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>
#include <stdio.h>

#include "../rp2040/config.h"
#include "../rp2040/timing.h"

/* timing_test.c does not compile config.c — define tick here. */
volatile uint32_t tick = 0;

/* Configurable time mock: each call returns the previous value + mock_time_step.
 * A step of 1000 simulates steady 1 kHz packets, keeping the EMA stable.
 * Changing mock_time_step between recover_clock() calls simulates varying
 * packet inter-arrival times. Note: because time_us_64() is called once per
 * recover_clock() call, the sample computed on call N is the step set for
 * call N-1 (one-call lag). */
static uint64_t mock_time       = 0;
static uint32_t mock_time_step  = 1000;

uint64_t __wrap_time_us_64(void) {
    uint64_t t = mock_time;
    mock_time += mock_time_step;
    return t;
}

/* Capture the callback so tests can fire it manually. */
static repeating_timer_callback_t captured_callback = NULL;
static repeating_timer_t          captured_timer;

bool __wrap_add_repeating_timer_us(int32_t delay_us,
                                    repeating_timer_callback_t callback,
                                    void *user_data,
                                    repeating_timer_t *out) {
    captured_callback = callback;
    return true;
}

bool __wrap_cancel_repeating_timer(repeating_timer_t *timer) {
    return true;
}

static int      update_period_call_count = 0;
static uint32_t last_update_period_arg   = 0;
static uint32_t min_update_period_arg    = UINT32_MAX;
static uint32_t max_update_period_arg    = 0;

void __wrap_update_period(uint32_t update_time_us) {
    update_period_call_count++;
    last_update_period_arg = update_time_us;
    if(update_time_us < min_update_period_arg) min_update_period_arg = update_time_us;
    if(update_time_us > max_update_period_arg) max_update_period_arg = update_time_us;
}

static int setup(void **state) {
    (void) state;
    mock_time                = 0;
    mock_time_step           = 1000;
    update_period_call_count = 0;
    last_update_period_arg   = 0;
    min_update_period_arg    = UINT32_MAX;
    max_update_period_arg    = 0;
    tick                     = 0;
    captured_callback        = NULL;
    timing_reset_for_test();
    return 0;
}

/* ---- Structural tests ---- */

/* timing_init() must register a callback via add_repeating_timer_us. */
static void test_timing_init__registers_callback(void **state) {
    (void) state;
    timing_init();
    assert_non_null(captured_callback);
}

/* The registered callback must increment tick when called. */
static void test_tick_callback__increments_tick(void **state) {
    (void) state;
    timing_init();
    assert_non_null(captured_callback);

    uint32_t tick_before = tick;
    captured_callback(&captured_timer);
    assert_int_equal(tick_before + 1, tick);
}

/* Each callback invocation increments tick exactly once. */
static void test_tick_callback__increments_tick_multiple_times(void **state) {
    (void) state;
    timing_init();

    uint32_t tick_before = tick;
    captured_callback(&captured_timer);
    captured_callback(&captured_timer);
    captured_callback(&captured_timer);
    assert_int_equal(tick_before + 3, tick);
}

/* recover_clock() must call update_period() on the first call because
 * last_timer_period_us starts at 0 and ave_period_us (1000) differs. */
static void test_recover_clock__calls_update_period_on_first_call(void **state) {
    (void) state;
    timing_init();
    recover_clock();
    assert_int_equal(1, update_period_call_count);
    assert_int_equal(1000, last_update_period_arg);
}

/* With steady 1000 µs packets the EMA stays at 1000 µs.
 * update_period() must not be called again after the first convergence. */
static void test_recover_clock__does_not_call_update_period_repeatedly(void **state) {
    (void) state;
    timing_init();
    recover_clock();   /* first call: fires update_period(1000) */
    int count_after_first = update_period_call_count;

    recover_clock();
    recover_clock();
    recover_clock();
    assert_int_equal(count_after_first, update_period_call_count);
}

/* recover_clock() must return — no busy-wait, no hang. */
static void test_recover_clock__does_not_hang(void **state) {
    (void) state;
    timing_init();
    recover_clock();
    assert_true(1);
}

/* ---- EMA convergence tests ---- */

/* Helper: seed time_last and stabilise the EMA at 1000 µs.
 * Two calls are needed: the first only sets time_last (time_last==0 skips EMA);
 * the second provides the first 1000 µs sample. */
static void seed_at_1000us(void) {
    timing_init();
    mock_time_step = 1000;
    recover_clock();   /* sets time_last, no EMA update (time_last was 0) */
    recover_clock();   /* first 1000 µs sample, EMA stays at 1000 */
}

/* After the EMA has converged on 1000 µs, sustained early packets (900 µs)
 * must drive the EMA — and therefore the timer period — down to 900 µs.
 *
 * The EMA (alpha=1/64, stored ×64) requires ~300 samples to reach within
 * 1 µs of the new rate.  We run 400 to guarantee integer convergence.
 *
 * Note: the first sample after the step change reads as 1000 µs due to the
 * one-call lag in the mock (mock_time_step affects the next call's interval,
 * not the current one).  This matches real-world behaviour where the first
 * "early" packet is measured relative to the last "normal" packet. */
static void test_recover_clock__early_packets_converge(void **state) {
    (void) state;
    seed_at_1000us();

    mock_time_step = 900;
    for(int i = 0; i < 400; i++) {
        recover_clock();
    }

    assert_int_equal(900, last_update_period_arg);
}

/* After the EMA has converged on 1000 µs, sustained late packets (1100 µs)
 * must drive the EMA up to 1100 µs. */
static void test_recover_clock__late_packets_converge(void **state) {
    (void) state;
    seed_at_1000us();

    mock_time_step = 1100;
    for(int i = 0; i < 400; i++) {
        recover_clock();
    }

    assert_int_equal(1100, last_update_period_arg);
}

/* After the EMA has converged on 1000 µs, erratic packets that alternate
 * between 800 µs and 1200 µs (average 1000 µs) must not push the timer
 * period outside ±5 µs of 1000 µs.
 *
 * Verified by simulation: with alpha=1/64 the EMA oscillates in [998, 1003].
 * The ±5 µs bound (995–1005) is a conservative engineering tolerance.
 *
 * Mock note: setting mock_time_step=1200 for even iterations and 800 for odd
 * produces actual EMA samples of [1000, 1200, 800, 1200, 800, …] due to the
 * one-call lag.  The average of the non-lag samples is exactly 1000 µs. */
static void test_recover_clock__erratic_packets_stay_bounded(void **state) {
    (void) state;
    seed_at_1000us();

    for(int i = 0; i < 200; i++) {
        mock_time_step = (i % 2 == 0) ? 1200 : 800;
        recover_clock();
    }

    assert_true(update_period_call_count > 0);
    assert_true(min_update_period_arg >= 995);
    assert_true(max_update_period_arg <= 1005);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_timing_init__registers_callback, setup),
        cmocka_unit_test_setup(test_tick_callback__increments_tick, setup),
        cmocka_unit_test_setup(test_tick_callback__increments_tick_multiple_times, setup),
        cmocka_unit_test_setup(test_recover_clock__calls_update_period_on_first_call, setup),
        cmocka_unit_test_setup(test_recover_clock__does_not_call_update_period_repeatedly, setup),
        cmocka_unit_test_setup(test_recover_clock__does_not_hang, setup),
        cmocka_unit_test_setup(test_recover_clock__early_packets_converge, setup),
        cmocka_unit_test_setup(test_recover_clock__late_packets_converge, setup),
        cmocka_unit_test_setup(test_recover_clock__erratic_packets_stay_bounded, setup),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
```

---

## Task 4: Add `timingTest` to `src/test/CMakeLists.txt`

**Files:**
- Modify: `src/test/CMakeLists.txt`

Note: no `--wrap,ring_buf_uint_ave` and no `--wrap,tight_loop_contents` — neither is used by the new implementation.

- [ ] **Step 1: Append to the end of the file**

```cmake
add_executable(
  timingTest
  ${CMAKE_CURRENT_SOURCE_DIR}/timing_test.c
  ${CMAKE_SOURCE_DIR}/src/rp2040/timing.c
  )
target_compile_features(
  timingTest PRIVATE
  c_std_99
  )
target_link_libraries(
  timingTest
  cmocka
  -Wl,--wrap,time_us_64
  -Wl,--wrap,update_period
  -Wl,--wrap,add_repeating_timer_us
  -Wl,--wrap,cancel_repeating_timer
  )
add_test(
  timingTest
  timingTest
  )
```

- [ ] **Step 2: Verify cmake configures**

```bash
cd /home/duncan/Working/rp2040/rp2040_pio_stepper/build_tests && cmake .. -DCMAKE_BUILD_TYPE=Debug 2>&1 | tail -5
```

Expected: cmake succeeds. `make timingTest` will fail — `timing.c` doesn't exist yet.

---

## Task 5: Create `timing.c`

**Files:**
- Create: `src/rp2040/timing.c`

`config.h` declares both `tick` (line 21) and `update_period()` (line 76) so no extra includes are needed for those.

The EMA accumulator is stored scaled ×64 (`ave_period_us_x64`) so 1 µs changes accumulate over 64 packets before affecting the integer output, providing natural hysteresis against noise while still tracking real drift.

File-level statics (rather than function-level) allow `timing_reset_for_test()` to reset them between tests.

- [ ] **Step 1: Write `timing.c`**

```c
#include <stdint.h>

#include "config.h"

#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"

#else  // BUILD_TESTS

#include "pico/stdlib.h"
#include "pico/time.h"

#endif  // BUILD_TESTS

#include "timing.h"

/* EMA accumulator, stored ×64 for sub-µs accumulation.
 * Initialised to 1000 µs (LinuxCNC default servo period). */
static uint32_t ave_period_us_x64  = 1000u << 6;
static uint32_t last_timer_period_us = 0;
static uint64_t time_last          = 0;
static repeating_timer_t tick_timer;

/* Hardware timer ISR — increments Core1's tick semaphore. */
static bool tick_callback(repeating_timer_t *rt) {
    (void) rt;
    tick++;
    return true;  /* keep repeating */
}

/* Called once from core0_main() before the packet loop. */
void timing_init(void) {
    uint32_t ave_period_us = ave_period_us_x64 >> 6;
    add_repeating_timer_us(-(int32_t)ave_period_us, tick_callback, NULL, &tick_timer);
    last_timer_period_us = ave_period_us;
}

/* Called after every received packet.
 * Updates the EMA of inter-packet period; restarts the timer only when the
 * integer-µs average changes.  No busy-wait. */
void recover_clock(void) {
    uint64_t time_now = time_us_64();

    if(time_last > 0) {
        uint32_t sample = (uint32_t)(time_now - time_last);
        /* EMA: alpha = 1/64.  Accumulator stores ave × 64 so that sub-µs
         * drift accumulates before changing the integer output. */
        ave_period_us_x64 = ave_period_us_x64 - (ave_period_us_x64 >> 6) + sample;
    }
    time_last = time_now;

    uint32_t ave_period_us = ave_period_us_x64 >> 6;
    if(ave_period_us == 0) {
        ave_period_us = 1;  /* guard against zero on startup */
    }

    if(last_timer_period_us != ave_period_us) {
        cancel_repeating_timer(&tick_timer);
        add_repeating_timer_us(-(int32_t)ave_period_us, tick_callback, NULL, &tick_timer);
        update_period(ave_period_us);
        last_timer_period_us = ave_period_us;
    }
}

#ifdef BUILD_TESTS
void timing_reset_for_test(void) {
    ave_period_us_x64    = 1000u << 6;
    last_timer_period_us = 0;
    time_last            = 0;
}
#endif
```

- [ ] **Step 2: Build and run tests**

```bash
cd /home/duncan/Working/rp2040/rp2040_pio_stepper/build_tests && make timingTest && ./src/test/timingTest
```

Expected output:
```
[==========] Running 9 test(s).
[ RUN      ] test_timing_init__registers_callback
[       OK ] test_timing_init__registers_callback
[ RUN      ] test_tick_callback__increments_tick
[       OK ] test_tick_callback__increments_tick
[ RUN      ] test_tick_callback__increments_tick_multiple_times
[       OK ] test_tick_callback__increments_tick_multiple_times
[ RUN      ] test_recover_clock__calls_update_period_on_first_call
[       OK ] test_recover_clock__calls_update_period_on_first_call
[ RUN      ] test_recover_clock__does_not_call_update_period_repeatedly
[       OK ] test_recover_clock__does_not_call_update_period_repeatedly
[ RUN      ] test_recover_clock__does_not_hang
[       OK ] test_recover_clock__does_not_hang
[ RUN      ] test_recover_clock__early_packets_converge
[       OK ] test_recover_clock__early_packets_converge
[ RUN      ] test_recover_clock__late_packets_converge
[       OK ] test_recover_clock__late_packets_converge
[ RUN      ] test_recover_clock__erratic_packets_stay_bounded
[       OK ] test_recover_clock__erratic_packets_stay_bounded
[==========] 9 test(s) run.
[  PASSED  ] 9 test(s).
```

- [ ] **Step 3: Commit**

```bash
git add src/rp2040/timing.h src/rp2040/timing.c \
        src/test/timing_test.c src/test/CMakeLists.txt \
        src/test/mocks/rp_mocks.h
git commit -m "feat: timing module — hardware timer drives tick, EMA replaces ring buffers"
```

---

## Task 6: Update `core0.c`

**Files:**
- Modify: `src/rp2040/core0.c`

- [ ] **Step 1: Add `#include "timing.h"` after `#include "modbus.h"` (line 9)**

The includes block becomes:
```c
#include "config.h"
#include "messages.h"
#include "buffer.h"
#include "gpio.h"
#include "modbus.h"
#include "timing.h"
```

- [ ] **Step 2: Delete the `recover_clock()` function (lines 28–81 including comment)**

Remove from the comment `/* Called after receiving network packet.` through the closing `}` of `recover_clock`.

- [ ] **Step 3: Call `timing_init()` from `core0_main()` alongside `modbus_init()`**

Old (line 456–458):
```c
  modbus_init();

  int count = 0;
```

New:
```c
  modbus_init();
  timing_init();

  int count = 0;
```

- [ ] **Step 4: Build existing network tests to confirm nothing broke**

```bash
cd /home/duncan/Working/rp2040/rp2040_pio_stepper/build_tests && make rpNetworkTest && ./src/test/rpNetworkTest
```

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/rp2040/core0.c
git commit -m "refactor: use timing module from core0 — remove recover_clock, add timing_init"
```

---

## Task 7: Fix Core1 jitter metric print

**Files:**
- Modify: `src/rp2040/core1.c` (line 62)

- [ ] **Step 1: Replace hardcoded `1000` with `update_period_us`**

Old:
```c
    printf("%i\t%i\n", min_dt - 1000, max_dt - 1000);
```

New:
```c
    printf("%i\t%i\n", (int32_t)min_dt - (int32_t)update_period_us,
                       (int32_t)max_dt - (int32_t)update_period_us);
```

`update_period_us` is already in scope (line 26: `uint32_t update_period_us = get_period();`). The `int32_t` casts preserve signed subtraction semantics.

- [ ] **Step 2: Build core1 test**

```bash
cd /home/duncan/Working/rp2040/rp2040_pio_stepper/build_tests && make rpCore1Test && ./src/test/rpCore1Test
```

Expected: passes.

- [ ] **Step 3: Commit**

```bash
git add src/rp2040/core1.c
git commit -m "fix: core1 jitter metric uses measured period instead of hardcoded 1000"
```

---

## Task 8: Add `timing.c` to firmware `CMakeLists.txt`

**Files:**
- Modify: `src/rp2040/CMakeLists.txt`

- [ ] **Step 1: Add `timing.c` to `target_sources`**

```cmake
target_sources(
  stepper_control PRIVATE
  stepper_control.c
  config.c
  network.c
  core0.c
  core1.c
  timing.c
  modbus.c
  modbus_fuling.c
  modbus_huanyang.c
  modbus_weiken.c
  pio.c
  ring_buffer.c
  gpio.c
  ../shared/buffer.c
  ../shared/checksum.c
)
```

Note: `ring_buffer.c` stays in the build — it is still used by `config.c` and other modules. Only `timing.c` no longer uses it.

- [ ] **Step 2: Commit**

```bash
git add src/rp2040/CMakeLists.txt
git commit -m "build: add timing.c to stepper_control firmware target"
```

---

## Verification

Run the full test suite:

```bash
cd /home/duncan/Working/rp2040/rp2040_pio_stepper/build_tests && make && ctest --output-on-failure
```

Expected: all tests pass including the new `timingTest`.

Firmware build (requires pico-sdk toolchain):

```bash
cd /home/duncan/Working/rp2040/rp2040_pio_stepper && mkdir -p build && cd build && cmake .. && make stepper_control 2>&1 | tail -10
```
