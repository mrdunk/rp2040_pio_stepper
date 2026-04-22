# Branch Integration Reference

## Live Branch Tree

Branches still active on GitHub (post-prune), showing containment relationships:

```
main
├── dunk_stepgen_refactor_involved  ← current HEAD (contains all prior dunk_* work)
│
├── rp2350
│   └── driver_tidy                 (merged rp2350; both are library/submodule updates)
│
├── i2c_gpio
│   ├── watchdog                    (superset of i2c_gpio + watchdog feature)
│   │   └── merge_wd_with_tests     (integrates i2c_gpio + watchdog + core1_tests)
│   └── tidy_integration
│
├── mcp23017                        (earlier/cleaner MCP23017 implementation)
├── modbus_speedup
└── core1_tests
```

Note: many branches visible locally may no longer exist on GitHub. Run
`git fetch --prune` to clean up stale remote-tracking refs.

## Integration Plan

Work is staged to validate and layer features incrementally onto
`dunk_stepgen_refactor_involved`:

1. **Manually test native GPIO** — verify non-I2C GPIO works on current branch
2. **Manually test Modbus** — verify existing Modbus support on current branch
3. **Apply `modbus_speedup`** — single self-contained commit; cherry-pick cleanly
4. **Pick apart `i2c_gpio` / `watchdog`** — extract MCP23017 and watchdog work carefully

## Branch Notes

### `rp2350` and `driver_tidy`
Library/submodule updates only. The code changes they contain can be
regenerated independently and do not need cherry-picking into the main branch.

### `mcp23017`
An earlier, cleaner implementation of MCP23017 I2C GPIO expander support
(2 commits: `21fad3c`, `f33718f`). Superseded by the reworked version in
`i2c_gpio`. Useful as a reference for the original structure.

### `i2c_gpio`
Contains the developed MCP23017 support plus a core0 performance fix.
Prefer this over `mcp23017` for the actual integration. Contains two messy
merge commits from `max_speed` — cherry-pick the substantive commits rather
than merging the branch wholesale:

```
dfb0242  Simplify Modbus code.
878cb6a  Initial work on merging old I2C support code. Effectively untested.
5c3e760  More work on integrating MCP23017 support.
7bfedfb  First somewhat working version. Do not use.
4ebe352  MCP23017 support improvements.
1f60867  Do MCP reconfiguration at config update time.
e6e1995  More fixes for MCP23017 support. Works much better.
d1c02d3  Fix an extraneous call causing 8ms stalls in core0.
```

### `watchdog`
Superset of `i2c_gpio` — all i2c_gpio commits are present, plus watchdog
support on top. Watchdog-specific commits:

```
081f802  Add untested watchdog support to stop any potential runaway PIOs.
8d70c0b  Account for slow pulse rates when updating the watchdog.
1db3193  Watchdog: fix a number of corner cases.
aa49ac1  watchdog: Oops, forgot to set the initialized flag!
482726d  Fix tests. Create precommit file.
8a6191e  rename pre-commit file
cba648b  track direction changes
37b3eb2  Typos.
c276b86  Tweaking direction change feedback
1de4ba3  Attempt to fix reboot loop on disconnection.
c525dca  Merge branch 'main' into watchdog
```

### `modbus_speedup`
Single self-contained commit (`01137af`): converts Modbus TX FIFO population
to use interrupts. Safe to cherry-pick directly after Modbus is verified working.

### `tidy_integration` and `merge_wd_with_tests`
Integration/experiment branches. `merge_wd_with_tests` absorbed `i2c_gpio`,
`watchdog`, and `core1_tests`. Not a clean source for cherry-picking — use the
individual branches instead.
