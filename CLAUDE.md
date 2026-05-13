# rp2040_pio_stepper — Claude guidance

## Interaction

Be direct but constructive, offering solutions alongside criticism.
Be concise. Shorter is better.
Always ask before performing git commit. Never open PR to main.
Save state whenever you have something useful to remember.

## Project overview

RP2040-based stepper motor controller for LinuxCNC. Core0 handles UDP networking
(W5500 Ethernet) and clock synchronisation; Core1 runs the stepper PIO loop.
The driver side runs on the LinuxCNC PC (`src/driver/`).

## Code comments

Before adding architecture notes here, consider whether a code comment is the
right home — especially for concurrency safety reasoning, non-obvious invariants,
and why a guard exists. CLAUDE.md is for things with no natural home in the code.

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

`scripts/hooks/pre-commit` runs the full test build and all tests on every commit.
CMake registers it automatically (`git config core.hooksPath scripts/hooks`) so
any developer who runs cmake gets the hook. All tests must pass before a commit
lands. The hook uses `set -e` so a single failure blocks the commit.

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

### Pre-commit hook blocks commits with calls to undefined functions

The pre-commit hook builds and runs all tests before every commit. This means the TDD pattern of "commit failing tests first, then implement" does not work here — the hook will block a commit that calls an undefined function. Write the implementation in the same commit as the tests that call it.

### Recompile the LinuxCNC driver after changing `messages.h`

Any struct change alters the wire format. Recompile and reinstall `hal_rp2040_eth.so`
and reflash the firmware. Symptom of a stale binary: "WARN: Unconsumed RX buffer
remainder: N bytes".
