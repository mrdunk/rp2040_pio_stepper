# rp2040_pio_stepper — Claude guidance

## Context management

Use context-mode MCP tools for all research and analysis. Bash only for: git, mkdir, rm, mv, navigation.

| Task | Tool |
|------|------|
| Multi-command research, large output | `ctx_batch_execute` |
| Single large-output command | `ctx_execute` |
| Analyse a file (not editing) | `ctx_execute_file` |
| Follow-up questions on indexed content | `ctx_search` |
| Fetch URL docs | `ctx_fetch_and_index` |

**Session start:** Before doing anything else, run one `ctx_batch_execute` call to index
`git log --oneline -20`, open GitHub issues (`gh issue list`), and any source files
relevant to the current task.

## Interaction

Be direct but constructive, offering solutions alongside criticism.
Be concise. Shorter is better.
Save state whenever you have something useful to remember.
If you discover something incorrect in your memory files, correct it.

**NEVER run `git commit` without explicit user instruction to do so.**
Stage changes and report what is staged, then wait for the user to say "commit".
This applies even when a commit seems like the obvious next step.
Never open PR to main.

## Project overview

RP2040/RP2350-based stepper motor controller for LinuxCNC. Core0 handles UDP networking
(WIZnet Ethernet — W5500, W5100S, W6100, or W6300) and clock synchronisation; Core1
runs the stepper PIO loop. The driver side runs on the LinuxCNC PC (`src/driver/`).

## Code comments

Before adding architecture notes here, consider whether a code comment is the
right home — especially for concurrency safety reasoning, non-obvious invariants,
and why a guard exists. CLAUDE.md is for things with no natural home in the code.

## Building and testing

### Test build (host, no hardware needed)

```bash
cmake --preset tests && cmake --build --preset tests
ctest --preset tests
```

### Firmware build (requires arm-none-eabi-gcc + pico-sdk)

Each variant has its own named preset and build directory under `build/`; no need to
delete the build directory when switching configs.

```bash
cmake --preset rp2040-w5500 && cmake --build --preset rp2040-w5500
cmake --preset rp2350-w5500 && cmake --build --preset rp2350-w5500
```

Available presets: `rp2040-w5500`, `rp2040-w5100s`, `rp2040-w6100`, `rp2040-w6300`,
`rp2350-w5500`, `rp2350-w5100s`, `rp2350-w6100`, `rp2350-w6300`.

Override `MAX_JOINT` on the command line (preset default is 4):

```bash
cmake --preset rp2040-w5500 -DMAX_JOINT=8 && cmake --build --preset rp2040-w5500
```

Key cmake options:

| Variable | Preset default | Values | Notes |
|----------|---------------|--------|-------|
| `ETH_CHIP` | per preset | `W5500`, `W5100S`, `W6100`, `W6300` | W6100/W6300 need hardware validation (issues #35/#36) |
| `RP_CHIP` | per preset | `RP2040`, `RP2350` | RP2350 builds clean; needs hardware validation (issue #34) |
| `PICO_BOARD` | derived | e.g. `wiznet_w5500_evb_pico` | Auto-derived from RP_CHIP+ETH_CHIP; override if needed |
| `MAX_JOINT` | `4` | `1`–`8` | Number of stepper axes |

`PICO_BOARD` is derived as `wiznet_<eth_chip_lower>_evb_pico[2]`. Board headers for
W5500/W6100/W6300 variants live in `boards/` (project-local; not in upstream pico-sdk).
`wiznet_w5100s_evb_pico[2]` are provided by the pico-sdk submodule itself.

Without `-DBUILD_RP=ON` (or a firmware preset), CMake configures successfully but
produces an empty Makefile with no firmware targets — no error, no warning.

If `ccache` is installed it is used automatically for firmware builds; run `ccache -s`
to verify hit rate when switching between presets.

Personal preset overrides go in `CMakeUserPresets.json` (gitignored).

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
