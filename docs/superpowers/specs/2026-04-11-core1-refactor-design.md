# Core1 Refactor Design

**Date:** 2026-04-11
**Branch:** `dunk_core0_and_core1_refactor`

## Goals

1. **Structural:** Split the monolithic `update_all_joint()` into focused, independently testable functions.
2. **Correctness:** Remove the false-timeout-on-startup bug caused by `metric` being initialised to zero.
3. **Architecture:** Align Core1's network-loss detection with the new hardware-timer-driven `tick` — under the new timing module, `tick` fires continuously from a hardware ISR regardless of packet arrival, so Core1 can no longer infer network loss from tick timing alone.

## Background

The previous timing approach drove `tick` directly from packet arrival (via a busy-wait or semaphore). The new timing module (`timing.c`, introduced on `dunk_core0_refactor`) fires a hardware repeating timer at the EMA-measured inter-packet period. This means:

- `tick` always increments at a regular rate, even when no packets arrive.
- Core1 can no longer detect missed packets by watching `tick`.
- The jitter metric (`min_dt`/`max_dt`) in `update_all_joint()` measured the inter-tick interval, which is now controlled entirely by the hardware timer and provides no useful diagnostic signal.
- `do_steps(joint, update_period_us)` received the period from Core1's loop measurement; this is now always equal to `get_period()` since both are derived from the same EMA value in `recover_clock()`.

## Design

### `config.c` / `config.h` — add `last_packet_tick`

Add a single new shared scalar alongside `tick`:

```c
volatile uint32_t last_packet_tick = 0;
```

Written by Core0 once per successfully processed packet. Read by Core1 to infer network health. Safe without a mutex: it is a naturally-aligned 32-bit value with a single writer (Core0) and a single reader (Core1). On Cortex-M0+, aligned 32-bit accesses are atomic. This is the same pattern as `tick` itself.

### `core0.c` — write `last_packet_tick` per packet

In `core0_main()`, after `process_received_buffer()`:

```c
if (received_msg_count > 0) {
    last_packet_tick = tick;   // new
    recover_clock();
    ...
}
```

### `pio.c` / `pio.h` — remove period parameter from `do_steps()`

`do_steps()` currently receives `update_period_us` from the caller. Since `get_period()` always returns the same value (both are set from the same `ave_period_us` in `recover_clock()`), the parameter is redundant.

New signature:

```c
uint8_t do_steps(const uint8_t joint);
```

`do_steps()` calls `get_period()` internally at the top of the function. The existing `update_period_us == 0` guard is preserved using the internal value.

### `core1.c` — full restructure

`update_all_joint()` is deleted. `core1_main()` becomes the orchestrator:

```c
void core1_main() {
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
```

**Functions:**

| Function | Signature | Responsibility |
|---|---|---|
| `wait_for_tick()` | `void` | Blocks until `tick != last_tick`; updates `last_tick` |
| `check_network_health()` | `bool` | Returns `(tick - last_packet_tick) <= MAX_MISSED_PACKET` |
| `handle_network_timeout()` | `void` | Disables all joints; prints once per outage (debounced by `no_network`) |
| `handle_network_recovery()` | `void` | If `no_network` was set: logs reconnection and clears it; otherwise no-op |
| `step_all_joints()` | `void` | Calls `do_steps(joint)` for all `MAX_JOINT` joints |

**Static state in `core1.c`:**

```c
static uint32_t last_tick   = 0;
static bool     no_network  = false;
```

All previous timing/metric state (`metric`, `metric_initialized`, `min_dt`, `max_dt`, `count`) is removed.

### Dead code removed

- `update_all_joint()` — replaced by the five functions above
- `min_dt`, `max_dt`, `count` — jitter metric; meaningless under hardware timer
- `metric`, `metric_initialized` — loop timing; no longer needed
- The `time_us_64()` call in Core1's loop — no longer needed at all

### `rp_core1_test.c` — real unit tests

The placeholder test is replaced. All five new functions are tested. Static state is reset between tests via `core1_reset_for_test()` (gated under `#ifdef BUILD_TESTS`, same pattern as `timing_reset_for_test()`).

**Test cases:**

| Test | What it verifies |
|---|---|
| `test_wait_for_tick_blocks_until_tick_changes` | Returns only after `tick` is incremented |
| `test_check_network_health_ok` | Returns true when `tick - last_packet_tick <= MAX_MISSED_PACKET` |
| `test_check_network_health_loss` | Returns false when gap exceeds `MAX_MISSED_PACKET` |
| `test_handle_network_timeout_disables_joints` | `disable_joint()` called for all `MAX_JOINT` joints |
| `test_handle_network_timeout_prints_once` | printf called once per outage; suppressed on repeated calls |
| `test_handle_network_recovery_clears_flag` | Subsequent timeout after recovery prints again |
| `test_step_all_joints_calls_do_steps` | `do_steps()` called exactly `MAX_JOINT` times |

Mocks already available: `rp_mocks.c` (`time_us_64`, `tight_loop_contents`), `config_mocks.c` (`disable_joint`, `get_period`). `test_step_all_joints_calls_do_steps` uses `-Wl,--wrap,do_steps` to intercept calls without pulling in `pio.c`'s hardware dependencies. `tick` and `last_packet_tick` declared as `volatile uint32_t` in the test file directly (established pattern from `timing_test.c`).

## Files changed summary

| File | Change |
|---|---|
| `src/rp2040/config.h` | Declare `extern volatile uint32_t last_packet_tick` |
| `src/rp2040/config.c` | Define `volatile uint32_t last_packet_tick = 0` |
| `src/rp2040/core0.c` | Write `last_packet_tick = tick` per packet |
| `src/rp2040/pio.h` | Remove `update_period_us` parameter from `do_steps()` |
| `src/rp2040/pio.c` | Call `get_period()` internally; remove parameter |
| `src/rp2040/core1.h` | Declare five new functions; add `core1_reset_for_test()` |
| `src/rp2040/core1.c` | Full rewrite as described above |
| `src/test/rp_core1_test.c` | Replace placeholder with real tests |
