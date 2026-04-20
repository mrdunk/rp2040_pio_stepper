# pio.c Readability and Testability Refactor

**Date:** 2026-04-20
**Branch:** dunk_stepgen_refactor
**Files affected:** `src/rp2040/pio.c`, `src/rp2040/pio.h`, `src/test/rp_pio_test.c`, `src/test/CMakeLists.txt`

---

## Context

`do_steps()` in `src/rp2040/pio.c` is ~150 lines and mixes config reading, PIO FIFO
interaction, velocity calculation, step-length math, and config write-back in a single
function. No tests exist for any code in `pio.c`. Static-local state in `do_steps()`,
`get_velocity()`, and `init_pio()` cannot be reset between test runs. The goal is to
extract sub-functions for clarity, fix two latent bugs, add a test-reset hook, and write
a test suite — all without changing observable runtime behaviour.

---

## Bugs to Fix

### 1. Dead variable: `last_velocity[]`

`last_velocity[MAX_JOINT]` is declared and written (`last_velocity[joint] = velocity` at
line 313) but never read anywhere in the file. Remove the declaration and assignment.

### 2. Type-narrowing: `velocity_requested_tm1`

```c
int32_t velocity_requested_tm1 = velocity;  // velocity is double — silent truncation
```

`ConfigAxis.velocity_requested_tm1` is `int32_t` (integer steps per period), so the
truncation is intentional — but the implicit conversion silences compiler warnings. Fix:

```c
int32_t velocity_requested_tm1 = (int32_t)velocity;
```

---

## Extractions (all `static`, within `pio.c`)

### `drain_rx_fifo`

Extracts lines 237–241.

```c
static int32_t drain_rx_fifo(uint sm, int32_t current_pos);
```

Drains all pending entries from PIO1's RX FIFO and returns the last value received, or
`current_pos` if the FIFO was empty. No side-effects beyond reading the hardware FIFO.

### `calculate_step_len`

Extracts lines 261–269.

```c
static int32_t calculate_step_len(double step_count,
                                  double update_period_ticks,
                                  double max_velocity);
```

Pure function. Returns the step-length value to program into the PIO timer, clamped to
the minimum derived from `max_velocity`. Returns 0 if `step_count <= MIN_STEP_COUNT`.

### `issue_pio_step`

Extracts lines 287–289.

```c
static void issue_pio_step(uint32_t joint, int32_t step_len_ticks, uint32_t direction);
```

Writes the packed command `(step_len_ticks << 1) | direction` to `pio0`'s TX FIFO for
`sm0[joint]`, but only if the FIFO is currently empty.

---

## Test Reset Hook

Add to `pio.c` (and expose in `pio.h` under `#ifdef BUILD_TESTS`):

```c
void pio_reset_for_test(void);
```

Resets all static-local state across `init_pio()`, `get_velocity()`, and `do_steps()`:
- `init_done[MAX_JOINT]` → `{false, …}`
- `offset_pio0`, `offset_pio1`, `programs_loaded` → 0
- `holdoff[MAX_JOINT]` → `{0, …}`
- `last_pos_requested[MAX_JOINT]`, `last_pos_achieved[MAX_JOINT]` → `{0, …}`
- `last_enabled[MAX_JOINT]`, `dir_change_count[MAX_JOINT]`, `last_direction[MAX_JOINT]` → `{0, …}`
- `count` → 0
- `sm0[MAX_JOINT]`, `sm1[MAX_JOINT]` → `{0, …}`

Pattern follows `timing_reset_for_test()` in `src/rp2040/timing.c` lines 88–96.

---

## New Test File: `src/test/rp_pio_test.c`

### Setup / Teardown

Each test calls `pio_reset_for_test()` and zeroes `config.joint[j]` fields (same pattern
as documented in CLAUDE.md pitfall "init_config() does not reset ConfigAxis fields").

### Test cases

**`calculate_step_len` (pure, no mocks needed)**
- Normal case: step count > 1.0 → step_len is positive and less than period/2
- Clamped to min: step_count exceeds max_velocity → step_len equals the minimum
- Below MIN_STEP_COUNT: returns 0

**`drain_rx_fifo` (uses pio_mocks)**
- Empty FIFO: returns the `current_pos` argument unchanged
- Single entry: returns the one value from the FIFO
- Multiple entries: returns only the last value (intermediate values discarded)

**`get_velocity`**
- Zero position diff: returns 0.0
- Direction disagreement (pos_diff positive, velocity negative): returns 0.0
- Normal forward motion: returns combined_vel > 0
- Holdoff: after a slow step, subsequent calls within the holdoff window return 0.0

**`do_steps` (integration-style, uses config + pio_mocks)**
- Period == 0: puts 0 into PIO TX FIFO, returns 0
- Joint disabled: puts 0 into PIO FIFO when empty, returns 0
- `updated == 0`: returns 0 without issuing steps
- Normal step: FIFO empty + valid position request → non-zero word written to FIFO

### CMakeLists.txt addition

New target `rpPioTest` following the pattern of `rpCore1Test`:
- Sources: `rp_pio_test.c`, `../../rp2040/pio.c`, `../../rp2040/config.c`
- Mocks: `mocks/pio_mocks.c`, `mocks/rp_mocks.c`
- Wrap flags: `get_period` (to return a controlled update_period_us)
- Link: cmocka, m (math)
- Defines: `BUILD_TESTS`

---

## Verification

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON
make -C build_tests rpPioTest
ctest --test-dir build_tests -R rpPioTest --output-on-failure
```

Full suite must still pass:
```bash
ctest --test-dir build_tests --output-on-failure
```
