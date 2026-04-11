# Core1 Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor Core1 to separate tick-waiting, network-health detection, and step generation into focused testable functions; align network-loss detection with the hardware-timer-driven `tick`; remove dead jitter-metric code; simplify `do_steps()` by internalising `get_period()`.

**Architecture:** `last_packet_tick` (written by Core0 per packet, read by Core1) replaces the old tick-timeout approach. Five focused functions replace `update_all_joint()`. `do_steps()` calls `get_period()` internally, removing the caller's need to pass the period.

**Tech Stack:** C11, cmocka, cmake, RP2040 Cortex-M0+. Host test build only (no hardware needed). Build: `cmake -B build_tests -S . -DBUILD_TESTS=ON && make -C build_tests && ctest --test-dir build_tests --output-on-failure`.

---

### Task 1: Add `last_packet_tick` to shared state

**Files:**
- Modify: `src/rp2040/config.h`
- Modify: `src/rp2040/config.c`

- [ ] **Step 1: Add the extern declaration to `config.h`**

In `src/rp2040/config.h`, add immediately after the `extern volatile uint32_t tick;` line (line 21):

```c
extern volatile uint32_t last_packet_tick;
```

- [ ] **Step 2: Add the definition to `config.c`**

In `src/rp2040/config.c`, add immediately after the `volatile uint32_t tick = 0;` line (line 26):

```c
volatile uint32_t last_packet_tick = 0;
```

- [ ] **Step 3: Run full test suite to verify nothing broken**

```bash
make -C build_tests && ctest --test-dir build_tests --output-on-failure
```

Expected: all existing tests pass.

- [ ] **Step 4: Commit**

```bash
git add src/rp2040/config.h src/rp2040/config.c
git commit -m "feat: add last_packet_tick to shared state"
```

---

### Task 2: Update `do_steps()` to internalise `get_period()`

**Files:**
- Modify: `src/rp2040/pio.h`
- Modify: `src/rp2040/pio.c`

`do_steps()` currently receives `update_period_us` from the caller. Since `get_period()` always equals the hardware timer period (both are set from the same EMA value in `recover_clock()`), the parameter is redundant. `do_steps()` already includes `config.h` and calls `get_joint_config()`/`update_joint_config()`, so calling `get_period()` is straightforward.

- [ ] **Step 1: Update the declaration in `pio.h`**

Replace the `do_steps` declaration in `src/rp2040/pio.h`:

```c
/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t joint);
```

- [ ] **Step 2: Update the implementation in `pio.c`**

At `src/rp2040/pio.c:167`, change the function signature and add an internal period lookup at the top of the function. Replace:

```c
uint8_t do_steps(const uint8_t joint, const uint32_t update_period_us) {
```

with:

```c
uint8_t do_steps(const uint8_t joint) {
  uint32_t update_period_us = get_period();
```

No other changes to `pio.c` are needed — all internal uses of `update_period_us` remain valid.

- [ ] **Step 3: Run full test suite**

```bash
make -C build_tests && ctest --test-dir build_tests --output-on-failure
```

Expected: all existing tests pass. Note: `core1.c` still calls `do_steps(joint, update_period_us)` at this point, which will cause a **compile error in `rpCore1Test`**. That is expected and will be fixed in Task 3.

If the compile error in `rpCore1Test` blocks the full ctest run, verify the other test targets pass by running them individually:

```bash
./build_tests/src/test/timingTest
./build_tests/src/test/rpNetworkTest
./build_tests/src/test/rpNetworkTxTest
```

- [ ] **Step 4: Commit**

```bash
git add src/rp2040/pio.h src/rp2040/pio.c
git commit -m "refactor: do_steps() calls get_period() internally, removes period parameter"
```

---

### Task 3: Rewrite `core1.h` and `core1.c`

**Files:**
- Modify: `src/rp2040/core1.h`
- Modify: `src/rp2040/core1.c`

This task replaces `update_all_joint()` with five focused functions and removes all dead timing/metric state. It also fixes the `do_steps()` call to use the new one-argument signature.

- [ ] **Step 1: Rewrite `core1.h`**

Replace the entire contents of `src/rp2040/core1.h`:

```c
#ifndef CORE1__H
#define CORE1__H

#include <stdint.h>
#include <stdbool.h>

/* Block until tick changes. Updates internal last_tick. */
void wait_for_tick(void);

/* Return true if the gap between tick and last_packet_tick is within
 * MAX_MISSED_PACKET ticks, indicating the network is healthy. */
bool check_network_health(void);

/* Disable all joints. Prints once per outage (debounced by no_network flag). */
void handle_network_timeout(void);

/* If no_network was set: log reconnection and clear it. Otherwise no-op. */
void handle_network_recovery(void);

/* Call do_steps() for all MAX_JOINT joints. */
void step_all_joints(void);

void init_core1(void);
void core1_main(void);

#ifdef BUILD_TESTS
/* Reset all static state. Call at the start of each test. */
void core1_reset_for_test(void);
#endif

#endif  // CORE1__H
```

- [ ] **Step 2: Rewrite `core1.c`**

Replace the entire contents of `src/rp2040/core1.c`:

```c
#include <stdio.h>

#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"

#else  // BUILD_TESTS

#include "pico/multicore.h"

#endif  // BUILD_TESTS

#include "core1.h"
#include "config.h"
#include "pio.h"

static uint32_t last_tick  = 0;
static bool     no_network = false;

void wait_for_tick(void) {
  while (tick == last_tick) {}
  last_tick = tick;
}

bool check_network_health(void) {
  return (tick - last_packet_tick) <= MAX_MISSED_PACKET;
}

void handle_network_timeout(void) {
  if (no_network) {
    return;
  }
  for (uint8_t joint = 0; joint < MAX_JOINT; joint++) {
    disable_joint(joint, CORE1);
  }
  printf("No network update. Disabling joints.\n");
  no_network = true;
}

void handle_network_recovery(void) {
  if (!no_network) {
    return;
  }
  printf("Network recovered.\n");
  no_network = false;
}

void step_all_joints(void) {
  for (uint8_t joint = 0; joint < MAX_JOINT; joint++) {
    do_steps(joint);
  }
}

void core1_main(void) {
  while (1) {
    wait_for_tick();
    if (!check_network_health()) {
      handle_network_timeout();
    } else {
      handle_network_recovery();
      step_all_joints();
    }
  }
}

void init_core1(void) {
  printf("core0: Initializing.\n");

  if (MAX_JOINT > 4) {
    printf("ERROR: Maximum joint count: 4. Configured: %u\n", MAX_JOINT);
    while (1) {
      tight_loop_contents();
    }
  }

  multicore_launch_core1(&core1_main);
}

#ifdef BUILD_TESTS
void core1_reset_for_test(void) {
  last_tick  = 0;
  no_network = false;
}
#endif
```

- [ ] **Step 3: Run full test suite**

```bash
make -C build_tests && ctest --test-dir build_tests --output-on-failure
```

Expected: all tests pass. `rpCore1Test` still has the trivial placeholder test which should pass.

- [ ] **Step 4: Commit**

```bash
git add src/rp2040/core1.h src/rp2040/core1.c
git commit -m "refactor: split update_all_joint() into focused functions; remove dead jitter metrics"
```

---

### Task 4: Update CMakeLists and write real tests for `rpCore1Test`

**Files:**
- Modify: `src/test/CMakeLists.txt`
- Modify: `src/test/rp_core1_test.c`

The `rpCore1Test` target currently links `pio.c` and `pio_mocks.c`. Since `do_steps()` is now called via `step_all_joints()` and we want to verify call count without pulling in `pio.c`'s hardware dependencies, we wrap `do_steps` at link time. We also wrap `disable_joint` to verify it is called by `handle_network_timeout()`.

- [ ] **Step 1: Update the `rpCore1Test` target in `src/test/CMakeLists.txt`**

Find the `rpCore1Test` block (starts at line 199) and replace it entirely:

```cmake
add_executable(
  rpCore1Test
  ${CMAKE_CURRENT_SOURCE_DIR}/rp_core1_test.c
  ${CMAKE_SOURCE_DIR}/src/rp2040/modbus_fuling.c
  ${CMAKE_SOURCE_DIR}/src/rp2040/modbus_huanyang.c
  ${CMAKE_SOURCE_DIR}/src/rp2040/modbus_weiken.c
  ${CMAKE_SOURCE_DIR}/src/rp2040/modbus.c
  ${CMAKE_SOURCE_DIR}/src/rp2040/core1.c
  ${CMAKE_SOURCE_DIR}/src/rp2040/config.c
  ${CMAKE_SOURCE_DIR}/src/shared/buffer.c
  ${CMAKE_SOURCE_DIR}/src/shared/checksum.c
  ${CMAKE_CURRENT_SOURCE_DIR}/mocks/rp_mocks.c
  )
target_link_libraries(
  rpCore1Test
  cmocka
  -Wl,--wrap,disable_joint
  -Wl,--wrap,do_steps
  )
add_test(
  rpCore1Test
  rpCore1Test
  )
```

Note: `pio.c` and `pio_mocks.c` are removed. `__wrap_disable_joint` and `__wrap_do_steps` are defined in the test file.

- [ ] **Step 2: Rewrite `src/test/rp_core1_test.c`**

Replace the entire file:

```c
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include "../rp2040/core1.h"
#include "../rp2040/config.h"

/* tick and last_packet_tick are defined in config.c (which is linked).
 * config.h provides the extern declarations — no re-definition needed here.
 * Tests can write to them directly to control Core1 behaviour. */

/* Intercept disable_joint() to count calls. */
static int disable_joint_call_count = 0;

void __wrap_disable_joint(const uint8_t joint, const uint8_t core) {
    (void)joint;
    (void)core;
    disable_joint_call_count++;
}

/* Intercept do_steps() to count calls. */
static int do_steps_call_count = 0;

uint8_t __wrap_do_steps(const uint8_t joint) {
    (void)joint;
    do_steps_call_count++;
    return 1;
}

/* Reset all state before each test.
 * tick and last_packet_tick are extern from config.h — write directly. */
static int test_setup(void **state) {
    (void)state;
    tick                     = 0;
    last_packet_tick         = 0;
    disable_joint_call_count = 0;
    do_steps_call_count      = 0;
    core1_reset_for_test();
    return 0;
}

/* wait_for_tick: exits immediately when tick has already advanced past last_tick. */
static void test_wait_for_tick_returns_when_tick_changed(void **state) {
    (void)state;
    tick = 1;  /* tick (1) != last_tick (0, set by core1_reset_for_test) */
    wait_for_tick();
    /* Reaching this line proves the function returned (did not spin forever). */
}

/* check_network_health: healthy when gap <= MAX_MISSED_PACKET. */
static void test_check_network_health_ok(void **state) {
    (void)state;
    tick             = 5;
    last_packet_tick = 5;
    assert_true(check_network_health());
}

/* check_network_health: healthy at the exact boundary. */
static void test_check_network_health_at_limit(void **state) {
    (void)state;
    tick             = MAX_MISSED_PACKET;
    last_packet_tick = 0;
    assert_true(check_network_health());
}

/* check_network_health: unhealthy one tick beyond the boundary. */
static void test_check_network_health_loss(void **state) {
    (void)state;
    tick             = MAX_MISSED_PACKET + 1;
    last_packet_tick = 0;
    assert_false(check_network_health());
}

/* handle_network_timeout: disables all MAX_JOINT joints on first call. */
static void test_handle_network_timeout_disables_all_joints(void **state) {
    (void)state;
    handle_network_timeout();
    assert_int_equal(MAX_JOINT, disable_joint_call_count);
}

/* handle_network_timeout: does NOT disable joints on second call (no_network debounce). */
static void test_handle_network_timeout_disables_once_per_outage(void **state) {
    (void)state;
    handle_network_timeout();
    handle_network_timeout();
    /* Should still be MAX_JOINT, not 2 * MAX_JOINT. */
    assert_int_equal(MAX_JOINT, disable_joint_call_count);
}

/* handle_network_recovery: clears the no_network flag so the next timeout
 * re-arms and disables joints again. */
static void test_handle_network_recovery_re_arms_timeout(void **state) {
    (void)state;
    handle_network_timeout();   /* sets no_network, disables MAX_JOINT joints */
    handle_network_recovery();  /* clears no_network */
    handle_network_timeout();   /* should disable joints again */
    assert_int_equal(MAX_JOINT * 2, disable_joint_call_count);
}

/* step_all_joints: calls do_steps() exactly MAX_JOINT times. */
static void test_step_all_joints_calls_do_steps_for_each_joint(void **state) {
    (void)state;
    step_all_joints();
    assert_int_equal(MAX_JOINT, do_steps_call_count);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_wait_for_tick_returns_when_tick_changed, test_setup),
        cmocka_unit_test_setup(test_check_network_health_ok,                 test_setup),
        cmocka_unit_test_setup(test_check_network_health_at_limit,           test_setup),
        cmocka_unit_test_setup(test_check_network_health_loss,               test_setup),
        cmocka_unit_test_setup(test_handle_network_timeout_disables_all_joints,      test_setup),
        cmocka_unit_test_setup(test_handle_network_timeout_disables_once_per_outage, test_setup),
        cmocka_unit_test_setup(test_handle_network_recovery_re_arms_timeout,         test_setup),
        cmocka_unit_test_setup(test_step_all_joints_calls_do_steps_for_each_joint,   test_setup),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
```

- [ ] **Step 3: Reconfigure cmake (new sources in CMakeLists require reconfigure)**

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON
```

- [ ] **Step 4: Run full test suite**

```bash
make -C build_tests && ctest --test-dir build_tests --output-on-failure
```

Expected output for `rpCore1Test`:

```
[==========] tests: Running 8 test(s).
[ RUN      ] test_wait_for_tick_returns_when_tick_changed
[       OK ] test_wait_for_tick_returns_when_tick_changed
[ RUN      ] test_check_network_health_ok
[       OK ] test_check_network_health_ok
[ RUN      ] test_check_network_health_at_limit
[       OK ] test_check_network_health_at_limit
[ RUN      ] test_check_network_health_loss
[       OK ] test_check_network_health_loss
[ RUN      ] test_handle_network_timeout_disables_all_joints
[       OK ] test_handle_network_timeout_disables_all_joints
[ RUN      ] test_handle_network_timeout_disables_once_per_outage
[       OK ] test_handle_network_timeout_disables_once_per_outage
[ RUN      ] test_handle_network_recovery_re_arms_timeout
[       OK ] test_handle_network_recovery_re_arms_timeout
[ RUN      ] test_step_all_joints_calls_do_steps_for_each_joint
[       OK ] test_step_all_joints_calls_do_steps_for_each_joint
[==========] tests: 8 test(s) run.
[  PASSED  ] 8 test(s).
```

- [ ] **Step 5: Commit**

```bash
git add src/test/CMakeLists.txt src/test/rp_core1_test.c
git commit -m "test: replace placeholder core1 tests with real unit tests"
```

---

### Task 5: Update Core0 to write `last_packet_tick`

**Files:**
- Modify: `src/rp2040/core0.c`

- [ ] **Step 1: Write `last_packet_tick = tick` after each received packet**

In `src/rp2040/core0.c`, find the block starting at line 427:

```c
    if(received_msg_count > 0) {
      recover_clock();
```

Add `last_packet_tick = tick;` as the first line inside the `if` block:

```c
    if(received_msg_count > 0) {
      last_packet_tick = tick;
      recover_clock();
```

- [ ] **Step 2: Run full test suite**

```bash
make -C build_tests && ctest --test-dir build_tests --output-on-failure
```

Expected: all tests pass. (`rpNetworkTest` and `rpNetworkTxTest` both link `core0.c` and `config.c`; `last_packet_tick` is defined in `config.c` so no new link dependency is introduced.)

- [ ] **Step 3: Commit**

```bash
git add src/rp2040/core0.c
git commit -m "feat: core0 writes last_packet_tick per received packet"
```
