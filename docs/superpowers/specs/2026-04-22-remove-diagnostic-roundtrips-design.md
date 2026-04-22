# Remove Unused Diagnostic Roundtrips — Design

## Goal

Remove `velocity_requested_tm1` and `step_len_ticks` from the config→reply→driver
pipeline. Both fields are written by `do_steps()`, packed into every UDP reply,
and exposed as HAL pins (`fb-velocity-cmd`, `fb-accel-cmd`, `fb-step-len`), but
none of those pins appear in the sample HAL config and they are unused in practice.

## What Is Removed

### `velocity_requested_tm1`

The clamped integer velocity (steps/period) written back each tick:

```c
// pio.c — removed:
int32_t velocity_requested_tm1 = velocity_q / 65536;
// ... and its update_joint_config() argument
```

Driver side: used to compute `fb-accel-cmd` (velocity delta) and update
`fb-velocity-cmd`. Both HAL pins and their backing fields are removed.

### `step_len_ticks`

The PIO half-period in clock ticks, written back each tick and exposed as
`fb-step-len`. The *local variable* inside `do_steps()` is kept — it is
required by `plan_steps()` and `issue_pio_step()`. Only the `ConfigAxis`
field, the reply packing, and the driver consumer are removed.

## Files Modified

| File | Change |
|------|--------|
| `src/rp2040/config.h` | Remove `velocity_requested_tm1` and `step_len_ticks` from `ConfigAxis`; remove both params from `update_joint_config` / `get_joint_config` signatures |
| `src/rp2040/config.c` | Remove field initialisers, parameter handling, and pack-reply assignments for both fields |
| `src/rp2040/pio.c` | Remove `velocity_requested_tm1` local and its `update_joint_config()` argument; remove `step_len_ticks` `update_joint_config()` argument (local var stays) |
| `src/shared/messages.h` | Remove `velocity_requested_tm1[MAX_JOINT]` and `step_len_ticks[MAX_JOINT]` from reply struct |
| `src/driver/skeleton.h` | Remove `joint_velocity_cmd`, `joint_accel_cmd`, `joint_step_len_ticks` fields |
| `src/driver/hal_rp2040_eth.c` | Remove `fb-velocity-cmd`, `fb-accel-cmd`, `fb-step-len` rows from `joint_pins[]` |
| `src/driver/rp2040_network.c` | Remove 3 lines consuming the two fields in `unpack_joint_metrics()` |
| `src/test/mocks/config_mocks.c` | Remove both params from mock function signatures |
| `src/test/driver_network_RPtoPC_test.c` | Remove globals, `setup_data()` assignments, and assertions for both fields |
| `src/test/driver_gpio_test.c` | Remove globals and `setup_data()` assignments for both fields |
| `src/test/rp_network_tx_test.c` | Remove `config.joint[joint].step_len_ticks = 246` setup line |

## What Is Unchanged

- Local `step_len_ticks` variable in `do_steps()` — used by `plan_steps()` and `issue_pio_step()`
- `calculate_step_len()` function and its unit tests — pure algorithm, no roundtrip
- All other `update_joint_config()` / `get_joint_config()` parameters
- EMA overrun/underrun scalar pins (`fb-overrun-ratio`, `fb-underrun-ratio`)

## Testing

Build and run the full test suite after changes:

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON
make -C build_tests
ctest --test-dir build_tests --output-on-failure
```

All tests must pass. No new tests are required — this is a pure deletion.
