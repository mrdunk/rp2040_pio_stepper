# rp2040_pio_stepper — Claude guidance

## Project overview

RP2040-based stepper motor controller for LinuxCNC. Core0 handles UDP networking
(W5500 Ethernet) and clock synchronisation; Core1 runs the stepper PIO loop.
The driver side runs on the LinuxCNC PC (`src/driver/`).

## Architecture

- **Core0** (`src/rp2040/core0.c`): receives ~1 kHz UDP packets from LinuxCNC,
  unpacks messages, writes `last_packet_tick = tick`, then calls `recover_clock()`
  after each packet.
- **Core1** (`src/rp2040/core1.c`): structured as five focused functions called
  from `core1_main()`: `wait_for_tick()` blocks until `tick` changes;
  `check_network_health()` detects loss via `tick - last_packet_tick`;
  `handle_network_timeout()` / `handle_network_recovery()` manage joint state;
  `step_all_joints()` drives PIO step generation. Core1's loop rate matches the
  average incoming packet rate — the timing module keeps them in sync.
- **timing** (`src/rp2040/timing.c`): RP2040 hardware repeating timer fires at
  the EMA-measured inter-packet period and increments `tick` ISR-style.
  No busy-wait, no ring buffers. `timing_init()` called once from `core0_main()`;
  `recover_clock()` called per packet. **`recover_clock()` is only called on
  received packets** (guarded by `received_msg_count > 0` in `core0_main()`).
  During a missed packet the timer free-runs at the current period — correct
  behaviour. The EMA is protected against inflated samples from large gaps by
  dividing `sample` by `last_id_diff` (per-packet elapsed time). This works
  because the LinuxCNC driver sends packets continuously and never resets
  `update_id`, even during outages.
- **`tick`** (`src/rp2040/config.c`): `volatile uint32_t tick` — incremented by
  the timer ISR (Core0 side), waited on by Core1's loop.
- **`last_packet_tick`** (`src/rp2040/config.c`): `volatile uint32_t
  last_packet_tick` — written by Core0 once per received packet. Core1 reads it
  to detect network loss: `(tick - last_packet_tick) > MAX_MISSED_PACKET` means
  no packet has arrived for too long. Safe without a mutex — single writer, single
  reader, 32-bit aligned (atomic on Cortex-M0+), same pattern as `tick`.
- **`linuxcnc_restart_detected`** (`src/rp2040/config.c`): `volatile bool` — set
  by Core0 in `update_packet_metrics()` when `id_diff < 0` (sequence number
  wrapped, meaning LinuxCNC restarted). Core1 reads it each tick; if set, clears
  it and calls `handle_network_timeout()` to disable joints before processing
  anything else. Same single-writer/single-reader atomic pattern as `tick` and
  `last_packet_tick`.
- **PIO step generation** (`src/rp2040/pio.c`): `do_steps()` is called once per
  Core1 tick per joint. It calls `calculate_step_len()` to get the PIO half-period
  in clock ticks (clamped to `[min_len, max_len]` where `max_len` caps a step to
  one loop period), then `plan_steps()` to get the whole-step count for this
  period. `plan_steps()` maintains a per-joint fractional accumulator
  (`step_accumulator[]`) — `|velocity|` is added each period, the floor is the
  step count, and the remainder carries forward. `issue_pio_step()` is only called
  when `plan_steps()` returns > 0; otherwise 0 is written to the FIFO to stop the
  PIO. Direction is separate from step count: `velocity > 0` → forward.
- **LinuxCNC driver config resync**: `last_joint_config` / `last_gpio_config` /
  `last_spindle_config` in `hal_rp2040_eth.c` are statics that reset to `{0}` on
  LinuxCNC restart. Because `configure()` diffs against these before sending, it
  automatically resends full config (including `enable = false`) on the first
  cycles after restart — no RP2040-side action required to force reconfiguration.

## Building and testing

### Test build (host, no hardware needed)

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON
make -C build_tests
ctest --test-dir build_tests --output-on-failure
```

### Firmware build (requires arm-none-eabi-gcc + pico-sdk)

```bash
cmake -B build -S . -DBUILD_RP=ON
make -C build stepper_control
```

Without `-DBUILD_RP=ON`, CMake configures successfully but produces an empty Makefile with no firmware targets — no error, no warning.

### Pre-commit hook

`.git/hooks/pre-commit` runs the full test build and all tests on every commit.
All tests must pass before a commit lands. The hook uses `set -e` so a single
failure blocks the commit.

## Test conventions

- Framework: **cmocka** (fetched via `libraries/FetchCMocka.cmake`).
- SDK mocks: `src/test/mocks/rp_mocks.h` / `rp_mocks.c` — stub out all
  `pico/stdlib.h` and `pico/time.h` symbols for host builds.
- Link-time wrapping (`-Wl,--wrap,symbol`) is used to intercept both SDK calls
  (e.g. `time_us_64`, `add_repeating_timer_us`) and production functions
  (e.g. `disable_joint`, `do_steps` in `rpCore1Test`). Define `__wrap_foo` in
  the test file; the linker redirects all calls to `foo` to it.
- Test files define `volatile uint32_t tick` themselves **only** when they do
  not link `config.c` (pattern in `ring_buffer_test.c`, `timing_test.c`). When
  `config.c` is linked (e.g. `rpCore1Test`), `tick` and `last_packet_tick` come
  from there — access them via the externs in `config.h`, do not redefine them.
- `BUILD_TESTS` preprocessor flag gates all RP2040-specific includes.

## Known pitfalls

### Adding a HAL pin — use the PinDef tables

New pins must be added as rows in the appropriate static table in `hal_rp2040_eth.c`:

- `gpio_pins[]` — per-GPIO pins (`MAX_GPIO` channels)
- `joint_pins[]` — per-joint pins (`MAX_JOINT` channels)
- `spindle_pins[]` — per-spindle pins (`MAX_SPINDLE` channels)
- `scalar_pins[]` — single-instance pins (use `chan_num = -1`, `stride = 0`)

The pointer is computed as `(char*)port_data_array + offset + i * stride`, where
`stride = sizeof(field_element_type*)` for per-channel arrays. Also add the
corresponding field to `skeleton_t` in `src/driver/skeleton.h` and wire it up
in the `setup_data()` functions in `driver_network_RPtoPC_test.c` and
`driver_gpio_test.c`.

### Scalar HAL metric pins (`fb-overrun-ratio`, `fb-underrun-ratio`)

Both are 1-second EMA (α = 1/1000 at nominal 1 kHz) of cumulative overrun /
underrun counts summed across all joints per tick. They output `ema_overrun` and
`ema_underrun` respectively from `skeleton_t`. State lives in `skeleton_t`
(`ema_overrun`, `ema_underrun` double fields); updated in `unpack_joint_metrics()`
in `rp2040_network.c`. `EMA_ALPHA` is defined there and must match the servo rate.

### `init_config()` default `max_accel` disables stepping in `do_steps()` tests

`init_config()` sets `config.joint[j].max_accel` to `1.0`. In `do_steps()`,
`max_accel` is divided by `update_period_us` before being passed to
`clamp_accel()`. At a 1000 µs period this gives `max_accel = 0.001`
steps/period, which clamps `velocity` to `0.001` — below `MIN_STEP_COUNT` —
causing `plan_steps()` to return 0 and no step to be issued. Tests for the
normal-step path must set `config.joint[0].max_accel = 0.0` (disables the
limiter) or a sufficiently large value. The acceleration limiter is tested
separately in `test_do_steps_accel_clamped`.

### Pre-commit hook blocks commits with calls to undefined functions

The pre-commit hook builds and runs all tests before every commit. This means the TDD pattern of "commit failing tests first, then implement" does not work here — the hook will block a commit that calls an undefined function. Write the implementation in the same commit as the tests that call it.

### Recompile the LinuxCNC driver after changing `messages.h`

See warning comment at the top of `src/shared/messages.h`. Symptom of a stale
binary: "WARN: Unconsumed RX buffer remainder: N bytes".
