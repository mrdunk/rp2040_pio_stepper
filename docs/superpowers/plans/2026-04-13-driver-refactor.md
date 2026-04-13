# Driver Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move machine config files to `config/frankencnc/`, add test coverage for three untested `unpack_*` functions, then eliminate boilerplate in `rp2040_network.c` (UNPACK_MSG macro) and `hal_rp2040_eth.c` (PinDef descriptor table + consistent `goto port_error`).

**Architecture:** Three independent changes applied in dependency order: directory restructure first (standalone), then pre-refactor tests (protects existing behaviour), then the two C refactors (macro first, table second). Each change is committed separately and all tests pass after every commit.

**Tech Stack:** C (C99), cmocka test framework, LinuxCNC HAL API, CMake build system.

---

## File Map

| File | Change |
|---|---|
| `src/driver/frankencnc.hal` | Move to `config/frankencnc/frankencnc.hal` |
| `src/driver/frankencnc.ini` | Move to `config/frankencnc/frankencnc.ini`; update `#INCLUDE` path |
| `src/test/driver_gpio_test.c` | Add `test_unpack_gpio_config` |
| `src/test/driver_network_RPtoPC_test.c` | Extend `setup_data()` with spindle fields; add `test_unpack_spindle_speed`, `test_unpack_spindle_config` |
| `src/driver/rp2040_network.c` | Add `UNPACK_MSG` macro; apply to all 8 `unpack_*` functions |
| `src/driver/hal_rp2040_eth.c` | Add `PinDef` struct + tables; replace `init_hal_pin` blocks with table loops; all errors via `goto port_error` |

---

## Task 1: Move machine config to `config/frankencnc/`

**Files:**
- Move: `src/driver/frankencnc.hal` → `config/frankencnc/frankencnc.hal`
- Move: `src/driver/frankencnc.ini` → `config/frankencnc/frankencnc.ini`
- Modify: `config/frankencnc/frankencnc.ini` line 1

- [ ] **Step 1: Create directory and move files**

```bash
mkdir -p config/frankencnc
git mv src/driver/frankencnc.hal config/frankencnc/frankencnc.hal
git mv src/driver/frankencnc.ini config/frankencnc/frankencnc.ini
```

- [ ] **Step 2: Update `#INCLUDE` path in `frankencnc.ini`**

The file currently starts with:
```ini
#INCLUDE rp2040_gpio_types.ini
```

Change it to (path is relative to the INI file's own directory):
```ini
#INCLUDE ../../src/driver/rp2040_gpio_types.ini
```

- [ ] **Step 3: Run tests to confirm nothing broken**

```bash
make -C build_tests && ctest --test-dir build_tests --output-on-failure
```

Expected: all tests pass (the moved files are not compiled by CMake).

- [ ] **Step 4: Commit**

```bash
git add config/
git commit -m "refactor: move machine config to config/frankencnc/"
```

---

## Task 2: Add `test_unpack_gpio_config`

**Files:**
- Modify: `src/test/driver_gpio_test.c`

- [ ] **Step 1: Add the test function before `int main`**

Add this function to `src/test/driver_gpio_test.c` immediately before the `int main` block at line 373:

```c
static void test_unpack_gpio_config(void **state) {
    (void) state;

    skeleton_t data = {0};
    setup_data(&data);

    size_t rx_offset = 0;
    size_t received_count = 0;
    struct Message_gpio_config last_gpio_config[MAX_GPIO] = {0};

    struct NWBuffer buffer = {0};
    struct Reply_gpio_config reply = {
        .type      = REPLY_GPIO_CONFIG,
        .gpio_count = 3,
        .gpio_type = GPIO_TYPE_NATIVE_IN,
        .index     = 7,
        .address   = 42,
    };
    memcpy(buffer.payload, &reply, sizeof(reply));
    buffer.length = sizeof(reply);

    bool result = unpack_gpio_config(&buffer, &rx_offset, &received_count, last_gpio_config);

    assert_true(result);
    assert_int_equal(received_count, 1);
    assert_int_equal(rx_offset, sizeof(reply));
    assert_int_equal(last_gpio_config[3].gpio_type, GPIO_TYPE_NATIVE_IN);
    assert_int_equal(last_gpio_config[3].index, 7);
    assert_int_equal(last_gpio_config[3].address, 42);
}
```

- [ ] **Step 2: Register the test in `main`**

Replace the existing `main` in `driver_gpio_test.c`:

```c
int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_serialize_gpio_out_change),
        cmocka_unit_test(test_serialize_gpio_in_change),
        cmocka_unit_test(test_serialize_gpio_confirmation_pending),
        cmocka_unit_test(test_serialize_gpio_nothing_to_do),
        cmocka_unit_test(test_unpack_gpio),
        cmocka_unit_test(test_unpack_gpio_config)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
```

- [ ] **Step 3: Run the new test**

```bash
make -C build_tests driverGpioTest && ./build_tests/src/test/driverGpioTest
```

Expected: all 6 tests pass, including `test_unpack_gpio_config`.

- [ ] **Step 4: Commit**

```bash
git add src/test/driver_gpio_test.c
git commit -m "test: add test_unpack_gpio_config"
```

---

## Task 3: Add `test_unpack_spindle_speed` and `test_unpack_spindle_config`

**Files:**
- Modify: `src/test/driver_network_RPtoPC_test.c`

- [ ] **Step 1: Add spindle storage variables at file scope**

After the existing file-scope variable declarations (around line 29, after `hal_u32_t joint_stale_packets[4];`), add:

```c
hal_float_t spindle_speed_out[MAX_SPINDLE];
hal_float_t spindle_speed_in[MAX_SPINDLE];
hal_bit_t   spindle_at_speed[MAX_SPINDLE];
```

- [ ] **Step 2: Wire spindle fields in `setup_data`**

At the end of the `setup_data` function (before the closing `}`), add:

```c
  for (size_t s = 0; s < MAX_SPINDLE; s++) {
    data->spindle_speed_out[s] = &spindle_speed_out[s];
    data->spindle_speed_in[s]  = &spindle_speed_in[s];
    data->spindle_at_speed[s]  = &spindle_at_speed[s];
    data->spindle_poles[s]     = 4.0;
  }
```

(`spindle_poles` is a plain `hal_float_t` array, not a pointer array — assign directly.)

- [ ] **Step 3: Add `test_unpack_spindle_speed` before `int main`**

```c
static void test_unpack_spindle_speed(void **state) {
    (void) state;

    skeleton_t data = {0};
    memset(spindle_speed_out, 0, sizeof(spindle_speed_out));
    memset(spindle_speed_in,  0, sizeof(spindle_speed_in));
    memset(spindle_at_speed,  0, sizeof(spindle_at_speed));
    setup_data(&data);

    size_t rx_offset = 0;
    size_t received_count = 0;

    /* poles=4, speed=50.0 Hz → rpm = 50 * 120 / 4 = 1500 */
    struct NWBuffer buffer = {0};
    struct Reply_spindle_speed reply = {
        .type          = REPLY_SPINDLE_SPEED,
        .spindle_index = 0,
        .speed         = 50.0,
    };
    *data.spindle_speed_in[0] = 1500.0;

    memcpy(buffer.payload, &reply, sizeof(reply));
    buffer.length = sizeof(reply);

    bool result = unpack_spindle_speed(&buffer, &rx_offset, &received_count, &data);

    assert_true(result);
    assert_int_equal(received_count, 1);
    assert_double_equal(*data.spindle_speed_out[0], 1500.0, 0.01);
    assert_true(*data.spindle_at_speed[0]);  /* |1500 - 1500| < 10 */
}
```

- [ ] **Step 4: Add `test_unpack_spindle_config` before `int main`**

```c
static void test_unpack_spindle_config(void **state) {
    (void) state;

    size_t rx_offset = 0;
    size_t received_count = 0;
    struct Message_spindle_config last_spindle_config[MAX_SPINDLE] = {0};

    struct NWBuffer buffer = {0};
    struct Reply_spindle_config reply = {
        .type           = REPLY_SPINDLE_CONFIG,
        .spindle_index  = 0,
        .modbus_address = 2,
        .vfd_type       = 1,
        .bitrate        = 19200,
    };
    memcpy(buffer.payload, &reply, sizeof(reply));
    buffer.length = sizeof(reply);

    bool result = unpack_spindle_config(&buffer, &rx_offset, &received_count, last_spindle_config);

    assert_true(result);
    assert_int_equal(received_count, 1);
    assert_int_equal(rx_offset, sizeof(reply));
    assert_int_equal(last_spindle_config[0].modbus_address, 2);
    assert_int_equal(last_spindle_config[0].vfd_type, 1);
    assert_int_equal(last_spindle_config[0].bitrate, 19200);
}
```

- [ ] **Step 5: Register both tests in `main`**

Replace the existing `main` in `driver_network_RPtoPC_test.c`:

```c
int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_timing),
        cmocka_unit_test(test_joint_movement),
        cmocka_unit_test(test_joint_config),
        cmocka_unit_test(test_joint_metrics),
        cmocka_unit_test(test_joint_metrics_stale_accumulates),
        cmocka_unit_test(test_unpack_spindle_speed),
        cmocka_unit_test(test_unpack_spindle_config)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
```

- [ ] **Step 6: Run the new tests**

```bash
make -C build_tests driverNetworkRPtoPCTest && ./build_tests/src/test/driverNetworkRPtoPCTest
```

Expected: all 7 tests pass.

- [ ] **Step 7: Run full test suite**

```bash
ctest --test-dir build_tests --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 8: Commit**

```bash
git add src/test/driver_network_RPtoPC_test.c
git commit -m "test: add unpack_spindle_speed and unpack_spindle_config tests"
```

---

## Task 4: Apply `UNPACK_MSG` macro to `rp2040_network.c`

**Files:**
- Modify: `src/driver/rp2040_network.c`

- [ ] **Step 1: Add the macro after the includes at the top of `rp2040_network.c`**

After the `#ifdef BUILD_TESTS` block (around line 16), add:

```c
/* Unpacks a typed reply struct from the network buffer.
 * Declares `var` as a pointer to T. Returns false if data unavailable. */
#define UNPACK_MSG(T, var, buf, offset) \
    T* var = unpack_nw_buff((buf), *(offset), (offset), NULL, sizeof(T)); \
    if (!(var)) return false;
```

- [ ] **Step 2: Replace boilerplate in all 8 `unpack_*` functions**

Replace the body of each function as shown below. The full file section from `unpack_timing` through `unpack_gpio` becomes:

```c
bool unpack_timing(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    skeleton_t* data
) {
  UNPACK_MSG(struct Reply_timing, reply, rx_buf, rx_offset);
  *data->metric_update_id     = reply->update_id;
  *data->metric_time_diff     = reply->time_diff;
  *data->metric_rp_update_len = reply->rp_update_len;
  (*received_count)++;
  return true;
}

bool unpack_joint_movement(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    skeleton_t* data
) {
  UNPACK_MSG(struct Reply_joint_movement, reply, rx_buf, rx_offset);
  for(size_t joint = 0; joint < MAX_JOINT; joint++) {
    *data->joint_pos_feedback[joint] =
      ((double)reply->abs_pos_achieved[joint]) / *data->joint_scale[joint];
    *data->joint_velocity_feedback[joint] =
      (double)reply->velocity_achieved[joint];
    *data->joint_pos_error[joint] = reply->position_error[joint];
  }
  (*received_count)++;
  return true;
}

bool unpack_joint_config(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    struct Message_joint_config* last_joint_config
) {
  UNPACK_MSG(struct Reply_joint_config, reply, rx_buf, rx_offset);
  size_t joint = reply->joint;
  printf("INFO: Received confirmation of config received by RP for joint: %u\n", joint);
  printf("      enable:       %u\n", reply->enable);
  printf("      gpio_step:    %i\n", reply->gpio_step);
  printf("      gpio_dir:     %i\n", reply->gpio_dir);
  printf("      max_velocity: %f\n", reply->max_velocity);
  printf("      max_accel:    %f\n", reply->max_accel);
  last_joint_config[joint].enable       = reply->enable;
  last_joint_config[joint].gpio_step    = reply->gpio_step;
  last_joint_config[joint].gpio_dir     = reply->gpio_dir;
  last_joint_config[joint].max_velocity = reply->max_velocity;
  last_joint_config[joint].max_accel    = reply->max_accel;
  (*received_count)++;
  return true;
}

bool unpack_gpio_config(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    struct Message_gpio_config* last_gpio_config
) {
  UNPACK_MSG(struct Reply_gpio_config, reply, rx_buf, rx_offset);
  size_t gpio = reply->gpio_count;
  printf("INFO: Received confirmation of config received by RP for gpio: %u\n", gpio);
  printf("      gpio_type:   %i\n", reply->gpio_type);
  printf("      index:       %i\n", reply->index);
  printf("      address:     %i\n", reply->address);
  last_gpio_config[gpio].gpio_type = reply->gpio_type;
  last_gpio_config[gpio].index     = reply->index;
  last_gpio_config[gpio].address   = reply->address;
  (*received_count)++;
  return true;
}

bool unpack_joint_metrics(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    skeleton_t* data
) {
  UNPACK_MSG(struct Reply_joint_metrics, reply, rx_buf, rx_offset);
  for(size_t joint = 0; joint < MAX_JOINT; joint++) {
    *data->joint_step_len_ticks[joint] = reply->step_len_ticks[joint];
    *data->joint_stale_packets[joint] += reply->stale_packet_count[joint];
    *data->joint_accel_cmd[joint] =
      reply->velocity_requested_tm1[joint] - *data->joint_velocity_cmd[joint];
    *data->joint_velocity_cmd[joint] = reply->velocity_requested_tm1[joint];
  }
  (*received_count)++;
  return true;
}

bool unpack_spindle_speed(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    skeleton_t* data
) {
  UNPACK_MSG(struct Reply_spindle_speed, reply, rx_buf, rx_offset);
  uint8_t spindle     = reply->spindle_index;
  float rpm           = reply->speed * 120.0 / data->spindle_poles[spindle];
  float expected_rpm  = *data->spindle_speed_in[spindle];
  *data->spindle_speed_out[spindle] = rpm;
  *data->spindle_at_speed[spindle]  = fabs(rpm - expected_rpm) < 10.0;
  (*received_count)++;
  return true;
}

bool unpack_spindle_config(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    struct Message_spindle_config* last_spindle_config
) {
  UNPACK_MSG(struct Reply_spindle_config, reply, rx_buf, rx_offset);
  size_t spindle_index = reply->spindle_index;
  printf("INFO: Received confirmation of config received by RP for spindle: %u\n", spindle_index);
  printf("      modbus_address   %i\n", reply->modbus_address);
  printf("      vfd_type:        %i\n", reply->vfd_type);
  printf("      bitrate:         %i\n", reply->bitrate);
  last_spindle_config[spindle_index].modbus_address = reply->modbus_address;
  last_spindle_config[spindle_index].vfd_type       = reply->vfd_type;
  last_spindle_config[spindle_index].bitrate        = reply->bitrate;
  (*received_count)++;
  return true;
}

bool unpack_gpio(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    skeleton_t* data
) {
  UNPACK_MSG(struct Reply_gpio, reply, rx_buf, rx_offset);
  size_t bank      = reply->bank;
  uint32_t values  = reply->values;
  data->gpio_confirmation_pending[bank] = reply->confirmation_pending;
  for(size_t gpio_per_bank = 0; gpio_per_bank < 32; gpio_per_bank++) {
    data->gpio_data_received[bank] = values;
  }
  (*received_count)++;
  return true;
}
```

- [ ] **Step 3: Run full test suite**

```bash
make -C build_tests && ctest --test-dir build_tests --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 4: Commit**

```bash
git add src/driver/rp2040_network.c
git commit -m "refactor: apply UNPACK_MSG macro to all unpack_* functions"
```

---

## Task 5: PinDef descriptor table in `hal_rp2040_eth.c`

**Files:**
- Modify: `src/driver/hal_rp2040_eth.c`

This task replaces the 25+ repeated `init_hal_pin` + error-check blocks in `rtapi_app_main` with table-driven loops, and standardises all error paths to `goto port_error`.

- [ ] **Step 1: Add `ARRAY_SIZE` macro and `PinDef` struct after the `enum t_types` block**

After the `enum t_types` closing brace (around line 118), insert:

```c
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

typedef struct {
    enum t_types  type;
    hal_pin_dir_t dir;
    size_t        offset;        /* offsetof(skeleton_t, field[0]) */
    size_t        stride;        /* sizeof(field element), 0 for scalar pins */
    const char*   io_type;       /* HAL group name, e.g. "joint", "gpio" */
    int           chan_num_len;  /* digit width for channel suffix; 0 = no channel */
    const char*   specific_name;/* HAL pin suffix, e.g. "enable", "pos-cmd" */
} PinDef;
```

- [ ] **Step 2: Add the three pin descriptor tables after `PinDef`**

```c
static const PinDef joint_pins[] = {
    { PIN,   HAL_IN,  offsetof(skeleton_t, joint_enable[0]),            sizeof(hal_bit_t*),   "joint", 1, "enable" },
    { S32,   HAL_IN,  offsetof(skeleton_t, joint_gpio_step[0]),         sizeof(hal_s32_t*),   "joint", 1, "gpio-step" },
    { S32,   HAL_IN,  offsetof(skeleton_t, joint_gpio_dir[0]),          sizeof(hal_s32_t*),   "joint", 1, "gpio-dir" },
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_max_velocity[0]),      sizeof(hal_float_t*), "joint", 1, "max-velocity" },
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_max_accel[0]),         sizeof(hal_float_t*), "joint", 1, "max-accel" },
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_scale[0]),             sizeof(hal_float_t*), "joint", 1, "scale" },
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_position[0]),          sizeof(hal_float_t*), "joint", 1, "pos-cmd" },
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_velocity[0]),          sizeof(hal_float_t*), "joint", 1, "vel-cmd" },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_pos_feedback[0]),      sizeof(hal_float_t*), "joint", 1, "fb-pos" },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_step_len_ticks[0]),    sizeof(hal_s32_t*),   "joint", 1, "fb-step-len" },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_velocity_cmd[0]),      sizeof(hal_float_t*), "joint", 1, "fb-velocity-cmd" },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_accel_cmd[0]),         sizeof(hal_float_t*), "joint", 1, "fb-accel-cmd" },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_velocity_feedback[0]), sizeof(hal_float_t*), "joint", 1, "fb-velocity" },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_pos_error[0]),         sizeof(hal_s32_t*),   "joint", 1, "fb-pos-error" },
    { U32,   HAL_OUT, offsetof(skeleton_t, joint_stale_packets[0]),     sizeof(hal_u32_t*),   "joint", 1, "fb-stale-packets" },
};

static const PinDef gpio_pins[] = {
    { PIN, HAL_OUT, offsetof(skeleton_t, gpio_data_in[0]),        sizeof(hal_bit_t*), "gpio", 2, "in" },
    { PIN, HAL_OUT, offsetof(skeleton_t, gpio_data_in_not[0]),    sizeof(hal_bit_t*), "gpio", 2, "in-not" },
    { PIN, HAL_IN,  offsetof(skeleton_t, gpio_data_out[0]),       sizeof(hal_bit_t*), "gpio", 2, "out" },
    { PIN, HAL_IN,  offsetof(skeleton_t, gpio_data_out_invert[0]),sizeof(hal_bit_t*), "gpio", 2, "out-invert" },
    { U32, HAL_IN,  offsetof(skeleton_t, gpio_type[0]),           sizeof(hal_u32_t*), "gpio", 2, "type" },
    { U32, HAL_IN,  offsetof(skeleton_t, gpio_index[0]),          sizeof(hal_u32_t*), "gpio", 2, "index" },
    { U32, HAL_IN,  offsetof(skeleton_t, gpio_address[0]),        sizeof(hal_u32_t*), "gpio", 2, "address" },
};

static const PinDef spindle_pins[] = {
    { PIN,   HAL_IN,  offsetof(skeleton_t, spindle_fwd[0]),       sizeof(hal_bit_t*),   "spindle", 1, "fwd" },
    { PIN,   HAL_IN,  offsetof(skeleton_t, spindle_rev[0]),       sizeof(hal_bit_t*),   "spindle", 1, "rev" },
    { FLOAT, HAL_IN,  offsetof(skeleton_t, spindle_speed_in[0]),  sizeof(hal_float_t*), "spindle", 1, "speed-in" },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, spindle_speed_out[0]), sizeof(hal_float_t*), "spindle", 1, "speed-out" },
    { PIN,   HAL_OUT, offsetof(skeleton_t, spindle_at_speed[0]),  sizeof(hal_bit_t*),   "spindle", 1, "at-speed" },
};

static const PinDef scalar_pins[] = {
    { PIN, HAL_OUT, offsetof(skeleton_t, machine_enable_out),   0, "machine-enable-out",      0, NULL },
    { S32, HAL_IN,  offsetof(skeleton_t, metric_time_diff),     0, "metrics-time-diff",       0, NULL },
    { U32, HAL_IN,  offsetof(skeleton_t, metric_update_id),     0, "metrics-update-id",       0, NULL },
    { U32, HAL_IN,  offsetof(skeleton_t, metric_rp_update_len), 0, "metrics-rp-update-len",   0, NULL },
    { U32, HAL_IN,  offsetof(skeleton_t, metric_missed_packets),0, "metrics-missed-packets",  0, NULL },
    { PIN, HAL_IN,  offsetof(skeleton_t, metric_eth_state),     0, "metrics-eth-state",       0, NULL },
};
```

- [ ] **Step 3: Replace the pin-registration blocks in `rtapi_app_main`**

Replace the entire section from `/* Set up the HAL pins. */` through the closing of the spindle pin loop (lines ~216–361) with:

```c
  /* Set up the HAL pins. */

  /* GPIO pins */
  for (int i = 0; i < MAX_GPIO; i++) {
    for (size_t j = 0; j < ARRAY_SIZE(gpio_pins); j++) {
      const PinDef* d = &gpio_pins[j];
      void* ptr = (char*)port_data_array + d->offset + i * d->stride;
      if (!init_hal_pin(d->type, d->dir, ptr, component_id, device_num,
                        d->io_type, i, d->chan_num_len, d->specific_name)) {
        goto port_error;
      }
    }
  }
  /* Non-zero GPIO defaults */
  for (int i = 0; i < MAX_GPIO; i++) {
    *port_data_array->gpio_data_in[i]     = true;
    *port_data_array->gpio_type[i]        = GPIO_TYPE_NOT_SET;
  }

  /* Machine-enable and metrics (scalar pins, no channel suffix) */
  for (size_t j = 0; j < ARRAY_SIZE(scalar_pins); j++) {
    const PinDef* d = &scalar_pins[j];
    void* ptr = (char*)port_data_array + d->offset;
    if (!init_hal_pin(d->type, d->dir, ptr, component_id, device_num,
                      d->io_type, -1, 0, d->specific_name)) {
      goto port_error;
    }
  }
  *port_data_array->machine_enable_out = false;

  /* Joint pins */
  for (int i = 0; i < MAX_JOINT; i++) {
    for (size_t j = 0; j < ARRAY_SIZE(joint_pins); j++) {
      const PinDef* d = &joint_pins[j];
      void* ptr = (char*)port_data_array + d->offset + i * d->stride;
      if (!init_hal_pin(d->type, d->dir, ptr, component_id, device_num,
                        d->io_type, i, d->chan_num_len, d->specific_name)) {
        goto port_error;
      }
    }
  }
  /* Non-zero joint defaults */
  for (int i = 0; i < MAX_JOINT; i++) {
    *port_data_array->joint_enable[i]    = false;
    *port_data_array->joint_gpio_step[i] = -1;
    *port_data_array->joint_gpio_dir[i]  = -1;
  }

  /* Spindle pins */
  for (int i = 0; i < MAX_SPINDLE; i++) {
    for (size_t j = 0; j < ARRAY_SIZE(spindle_pins); j++) {
      const PinDef* d = &spindle_pins[j];
      void* ptr = (char*)port_data_array + d->offset + i * d->stride;
      if (!init_hal_pin(d->type, d->dir, ptr, component_id, device_num,
                        d->io_type, i, d->chan_num_len, d->specific_name)) {
        goto port_error;
      }
    }
  }

  /* Spindle params (hal_param_*, not hal_pin_*) */
  for (uint8_t i = 0; i < MAX_SPINDLE; i++) {
    retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_vfd_type[i]),
        component_id, "rp2040_eth.%d.spindle.%d.vfd-type", device_num, i);
    if (retval < 0) goto port_error;
    port_data_array->spindle_vfd_type[i] = MODBUS_TYPE_NOT_SET;

    retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_address[i]),
        component_id, "rp2040_eth.%d.spindle.%d.address", device_num, i);
    if (retval < 0) goto port_error;
    port_data_array->spindle_address[i] = 1;

    retval = hal_param_float_newf(HAL_RW, &(port_data_array->spindle_poles[i]),
        component_id, "rp2040_eth.%d.spindle.%d.poles", device_num, i);
    if (retval < 0) goto port_error;
    port_data_array->spindle_poles[i] = 2;

    retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_bitrate[i]),
        component_id, "rp2040_eth.%d.spindle.%d.bitrate", device_num, i);
    if (retval < 0) goto port_error;
    port_data_array->spindle_bitrate[i] = 9600;
  }
```

- [ ] **Step 4: Build to check for compile errors**

`hal_rp2040_eth.c` is not compiled by the test build — it requires the real LinuxCNC build environment. If the LinuxCNC driver build is available, run it:

```bash
cmake -B build -S . && make -C build stepper_control
```

If not available, the test build at least confirms `rp2040_network.c` (which is included by `hal_rp2040_eth.c`) still compiles cleanly:

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON && make -C build_tests
```

Expected: build succeeds with no errors.

- [ ] **Step 5: Run full test suite**

```bash
ctest --test-dir build_tests --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/driver/hal_rp2040_eth.c
git commit -m "refactor: replace init_hal_pin blocks with PinDef descriptor tables"
```

---

## Self-Review Notes

- All three unpack functions without prior coverage now have tests added in Tasks 2 and 3 before the macro is applied in Task 4.
- `joint_step_len_ticks` and `joint_pos_error` are registered as `FLOAT` in the original code despite being `hal_s32_t*` fields — this is preserved as-is (out of scope to fix).
- The two pre-`hal_init` `return -1` calls at the top of `rtapi_app_main` remain unchanged — `hal_exit` cannot be called before `hal_init` succeeds.
- HAL file wiring blocks intentionally left unrolled — see design doc.
- `gpio_data_in_not` default stays `false` (zero-init is sufficient; the current code also sets it to `false` explicitly, which this plan omits as redundant).
