# Core1 Sync & PIO Miss Behaviour — Design

## Context

Core1's step-generation loop runs at ~1 kHz, driven by a hardware repeating timer
whose period is an EMA of the inter-packet interval.  Two related problems cause
~20% of ticks to produce incorrect or missing step commands:

1. **Timer-packet race (underrun ~0.2):** The timer fires before Core0 has finished
   writing all joint configs for the current packet.  Core1 wakes up, reads
   `updated_from_c0 == 0` for some or all joints, and returns early from `do_steps()`
   without touching the PIO.

2. **Core1 slow ticks (extra overrun ~0.15 above paired underrun):** Occasional mutex
   contention pushes Core1's loop over 1 ms, letting Core0 write two batches before
   Core1 reads.

In both cases the PIO step_gen state machine continues running at the last programmed
step length.  For slow speeds this produces phantom steps; for medium/high speeds
it is actually correct behaviour (motor momentum means an abrupt stop would cause
overstepping and the step_count PIO still reports actual steps for LinuxCNC to correct).

---

## Part 1 — Synchronisation: `packet_generation` counter

### Goal
Ensure Core1 never reads a partially-written packet batch.

### New state (`config.c` / `config.h`)
```c
volatile uint32_t packet_generation = 0;
```
Single writer (Core0), single reader (Core1).  32-bit aligned — atomic on Cortex-M0+,
no mutex needed.  Same pattern as `tick` and `last_packet_tick`.

### Core0 change (`core0_main` in `core0.c`)
Increment `packet_generation` immediately after `process_received_buffer()` returns
and `received_msg_count > 0`, **before** `recover_clock()`:

```c
process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, data_received);

if (received_msg_count > 0) {
    packet_generation++;        // all joint configs now written
    last_packet_tick = tick;
    recover_clock();
    ...
}
```

### Core1 change (`core1.c`)
Replace `wait_for_tick()` with `wait_for_packet()`:

```c
static uint32_t last_packet_generation = 0;

void wait_for_packet(void) {
    // 1. Wait for the timer tick (rate controller + watchdog).
    while (tick == last_tick) {}
    last_tick = tick;

    // 2. Wait for Core0 to finish writing the current packet batch,
    //    but give up after MAX_MISSED_PACKET ticks (genuine network loss).
    uint32_t wait_start = tick;
    while (packet_generation == last_packet_generation) {
        if ((tick - wait_start) > MAX_MISSED_PACKET) break;
    }
    last_packet_generation = packet_generation;
}
```

`core1_tick()` calls `wait_for_packet()` instead of `wait_for_tick()`.

### Effect
- Eliminates underrun caused by the timer-packet race.
- Core1's rate is still controlled by the timer; the inner spin only absorbs the
  packet-processing latency (typically < 200 µs).
- Genuine network loss is detected as before via `check_network_health()`.

---

## Part 2 — PIO behaviour on genuine misses

### Goal
When `do_steps()` sees `updated == 0` (underrun), choose between stopping the PIO
(safe for slow motors) and leaving it running (correct for motors with momentum).

### Change (`pio.c`, in `do_steps()`)
Replace the current early return:

```c
// Before:
if (updated == 0 || update_period_us == 0) {
    return 0;
}

// After:
if (updated == 0 || update_period_us == 0) {
    if (fabs(joint_state[joint].last_velocity) < 1.0 &&
        pio_sm_is_tx_fifo_empty(pio0, joint_state[joint].sm0)) {
        pio_sm_put(pio0, joint_state[joint].sm0, 0);
    }
    return 0;
}
```

### Threshold rationale
`last_velocity` is in steps/period.  At < 1.0 steps/period the motor is generating
at most one pulse per tick — negligible momentum.  Stopping is safe and prevents
the PIO from issuing an unintended extra step during the miss window.

At ≥ 1.0 steps/period the motor has sufficient momentum that an abrupt stop would
cause overstepping.  The PIO continues at the last velocity; the step_count PIO
(pio1) tracks actual steps and reports them back so LinuxCNC can correct position.

The FIFO-empty guard preserves the existing invariant: never stomp a step command
that is already queued.

---

## Files changed

| File | Change |
|---|---|
| `src/rp2040/config.h` | Declare `extern volatile uint32_t packet_generation` |
| `src/rp2040/config.c` | Define and initialise `packet_generation = 0` |
| `src/rp2040/core0.c` | Increment `packet_generation` before `recover_clock()` |
| `src/rp2040/core1.c` | Replace `wait_for_tick()` with `wait_for_packet()`; add `last_packet_generation` static |
| `src/rp2040/pio.c` | Velocity-gated PIO stop on underrun in `do_steps()` |

Tests to update/add (all in the existing cmocka test suite):

- `rpCore1Test`: update to mock `packet_generation`; add tests for `wait_for_packet()`
  timeout path and normal-advance path; update `core1_reset_for_test()` to reset
  `last_packet_generation`.
- `rpPioTest`: add tests for the underrun early-return path — one for slow velocity
  (expects FIFO write of 0), one for medium velocity (expects no FIFO write).

---

## Verification

1. Build host tests: `cmake -B build_tests -S . -DBUILD_TESTS=ON && make -C build_tests && ctest --test-dir build_tests --output-on-failure`
2. Build firmware: `cmake -B build -S . -DBUILD_RP=ON && make -C build stepper_control`
3. On hardware: monitor `fb-underrun-ratio` and `fb-overrun-ratio` HAL pins — both
   should drop significantly (underrun to near zero; overrun to ~0.15 reflecting only
   the residual Core1 slow-tick rate).
