# rp2040_pio_stepper — Claude guidance

## Project overview

RP2040-based stepper motor controller for LinuxCNC. Core0 handles UDP networking
(W5500 Ethernet) and clock synchronisation; Core1 runs the stepper PIO loop.
The driver side runs on the LinuxCNC PC (`src/driver/`).

## Architecture

- **Core0** (`src/rp2040/core0.c`): receives ~1 kHz UDP packets from LinuxCNC,
  unpacks messages, calls `recover_clock()` after each packet.
- **Core1** (`src/rp2040/core1.c`): busy-waits on `tick` semaphore, then drives
  stepper PIO step generation at the measured average packet rate.
- **timing** (`src/rp2040/timing.c`): RP2040 hardware repeating timer fires at
  the EMA-measured inter-packet period and increments `tick` ISR-style.
  No busy-wait, no ring buffers. `timing_init()` called once from `core0_main()`;
  `recover_clock()` called per packet.
- **`tick`** (`src/rp2040/config.c`): `volatile uint32_t tick` — incremented by
  the timer ISR (Core0 side), waited on by Core1's loop.

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
- Link-time wrapping (`-Wl,--wrap,symbol`) is used to intercept SDK calls in
  tests (e.g. `time_us_64`, `add_repeating_timer_us`, `update_period`).
- Test files define `volatile uint32_t tick` themselves when they don't link
  `config.c` (pattern established in `ring_buffer_test.c` and `timing_test.c`).
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
