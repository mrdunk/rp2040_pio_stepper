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
cmake -B build -S .
make -C build stepper_control
```

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

### UB left-shift with `-Ofast`

Test helpers that iterate over GPIO banks (0 and 1) must shift by
`(i - bank_offset)`, not by `i`. With `-Ofast`, GCC treats `0x1 << i` as
undefined behaviour when `i >= 32` (int overflow) and may optimise it to 0,
silently zeroing all bank-1 GPIO values. Pattern to use:

```c
int bank_offset = bank * 32;
for (int i = bank_offset; i < 32 + bank_offset; i++) {
    bool value = values & (0x1 << (i - bank_offset));  // correct
}
```

### `time_us_64()` can return 0

The mock returns 0 on the first call. Guards like `if (time_last > 0)` fail
when the real first timestamp is 0. Use a dedicated `bool time_initialized`
flag instead (see `timing.c`).

### `init_config()` does not reset `ConfigAxis` fields

`init_config()` initialises mutexes and GPIO state but does **not** reset the
`config.joint[]` array. Tests that depend on a clean per-joint state (e.g.
`updated_from_c0`, `stale_packet_count`) must zero those fields explicitly in
their setup function:

```c
for (size_t j = 0; j < MAX_JOINT; j++) {
    config.joint[j].updated_from_c0    = 0;
    config.joint[j].updated_from_c1    = 0;
    config.joint[j].stale_packet_count = 0;
}
```

Failing to do this causes state from earlier tests to leak (symptom: counters
accumulate to unexpectedly large values).

### `driver_mocks.h` `skeleton_t` must mirror `hal_rp2040_eth.c`

`src/test/mocks/driver_mocks.h` contains a hand-maintained copy of
`skeleton_t`. It is **not** auto-generated. Any field added to the real
`skeleton_t` in `hal_rp2040_eth.c` must also be added here, and the
corresponding test `setup_data()` functions in `driver_network_RPtoPC_test.c`
and `driver_gpio_test.c` must wire it up. The mismatch won't be caught at
configure time — it surfaces as a compile error only when a driver test is
built.

### Adding a HAL pin — use the PinDef tables

`rtapi_app_main` no longer contains individual `init_hal_pin()` calls. New pins
must be added as rows in the appropriate static table in `hal_rp2040_eth.c`:

- `gpio_pins[]` — per-GPIO pins (`MAX_GPIO` channels)
- `joint_pins[]` — per-joint pins (`MAX_JOINT` channels)
- `spindle_pins[]` — per-spindle pins (`MAX_SPINDLE` channels)
- `scalar_pins[]` — single-instance pins (use `chan_num = -1`, `stride = 0`)

The pointer is computed as `(char*)port_data_array + offset + i * stride`, where
`stride = sizeof(field_element_type*)` for per-channel arrays. Also add the
corresponding field to `skeleton_t` in both `hal_rp2040_eth.c` and the
hand-maintained copy in `src/test/mocks/driver_mocks.h` (see pitfall below).

### `UNPACK_MSG` macro expands to two statements

`UNPACK_MSG(T, var, buf, offset)` in `rp2040_network.c` expands to a
declaration plus an `if` — two statements. It must appear at the top of a
function body, never inside a bare `if/else` branch (which would only govern
the first statement, silently skipping the null check). All eight `unpack_*`
functions use it safely; keep this constraint when adding new ones.

### Recompile the LinuxCNC driver after changing `messages.h`

Any change to a reply struct in `src/shared/messages.h` changes the wire
format. The LinuxCNC driver binary must be recompiled and reinstalled after
such a change. Symptom of a stale binary: "WARN: Unconsumed RX buffer
remainder: N bytes" where N equals the size of the new field(s).
