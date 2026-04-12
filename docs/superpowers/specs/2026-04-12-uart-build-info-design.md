# UART Build Info at Power-On

**Date:** 2026-04-12

## Goal

Print the git branch name (with dirty indicator) and build timestamp to the UART/USB-serial output at firmware power-on, so a connected terminal immediately identifies which build is running.

## Example Output

```
--------------------------------
UART up.
Branch: dunk_core0_and_core1_refactor (dirty)
Built:  Apr 12 2026 14:23:01
--------------------------------
```

## Architecture

### Data sources

| Field | Source | Mechanism |
|---|---|---|
| Branch name | `git rev-parse --abbrev-ref HEAD` | CMake custom command at build time |
| Dirty flag | `git status --porcelain` (non-empty = dirty) | CMake custom command at build time |
| Build timestamp | `__DATE__` / `__TIME__` | C compiler built-ins |

### Generated header: `build_info.h`

A CMake script `src/rp2040/gen_build_info.cmake` runs on every `make` invocation via `add_custom_target`. It writes `${CMAKE_BINARY_DIR}/build_info.h` containing:

```c
#define BUILD_GIT_BRANCH "dunk_core0_and_core1_refactor (dirty)"
```

If git is unavailable or HEAD is detached, `BUILD_GIT_BRANCH` falls back to `"unknown"`.

The dirty indicator (` (dirty)`) is appended to the branch string when `git status --porcelain` produces any output.

`${CMAKE_BINARY_DIR}` is added to `stepper_control`'s include path. The target depends on the custom target so `stepper_control.c` is always recompiled when `build_info.h` changes. The generated file lives in the build directory and is not checked in.

### C code changes (`stepper_control.c`)

Inside the existing `#ifndef BUILD_TESTS` block, add:

```c
#include "build_info.h"
```

After `printf("UART up.\n")`, add:

```c
printf("Branch: %s\n", BUILD_GIT_BRANCH);
printf("Built:  %s %s\n", __DATE__, __TIME__);
```

No other source files change.

## Files Changed

| File | Change |
|---|---|
| `src/rp2040/gen_build_info.cmake` | New — cmake script that queries git and writes `build_info.h` |
| `src/rp2040/CMakeLists.txt` | Add custom target, include path, and dependency |
| `src/rp2040/stepper_control.c` | Include `build_info.h` and add two `printf` calls |

## Test Build Compatibility

`build_info.h` is included inside the `#ifndef BUILD_TESTS` guard that already wraps all pico-specific includes. The test build (`-DBUILD_TESTS=ON`) is unaffected and does not require git or the generated header.

## Edge Cases

- **Detached HEAD:** `git rev-parse --abbrev-ref HEAD` returns `HEAD` — the cmake script substitutes `"unknown"` for this case.
- **Git not installed:** `execute_process` failure is caught; falls back to `"unknown"`.
- **Clean working tree:** `git status --porcelain` returns empty; no dirty indicator appended.
