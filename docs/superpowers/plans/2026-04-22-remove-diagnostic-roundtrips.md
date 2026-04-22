# Remove Unused Diagnostic Roundtrips — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Delete `velocity_requested_tm1` and `step_len_ticks` from the RP2040→driver diagnostic roundtrip; remove the three associated HAL pins (`fb-step-len`, `fb-velocity-cmd`, `fb-accel-cmd`).

**Architecture:** Pure deletion across RP2040 firmware, shared messages, LinuxCNC driver, and test files. No new logic. All changes must land in a single commit because the pre-commit hook builds and runs every test before each commit — a partial change that breaks any call site will block the commit.

**Tech Stack:** C, cmocka (host tests), cmake with `-DBUILD_TESTS=ON`

**Spec:** `docs/superpowers/specs/2026-04-22-remove-diagnostic-roundtrips-design.md`

---

## Task 1: Delete the fields, parameters, and all consumers

This is one task because every changed file is coupled through function signatures and struct layouts. Make all edits, then build and test before committing.

**Files:**
- Modify: `src/rp2040/config.h`
- Modify: `src/rp2040/config.c`
- Modify: `src/rp2040/pio.c`
- Modify: `src/shared/messages.h`
- Modify: `src/driver/skeleton.h`
- Modify: `src/driver/hal_rp2040_eth.c`
- Modify: `src/driver/rp2040_network.c`
- Modify: `src/test/mocks/config_mocks.c`
- Modify: `src/test/rp_network_tx_test.c`
- Modify: `src/test/driver_network_RPtoPC_test.c`
- Modify: `src/test/driver_gpio_test.c`

---

### Step 1: Remove fields from `ConfigAxis` in `config.h`

In `src/rp2040/config.h`, delete the two fields from `struct ConfigAxis` (currently lines 48 and 50):

```c
// DELETE these two lines:
  int32_t velocity_requested_tm1; // Velocity requested last cycle. Compare to velocity_achieved.
  int32_t step_len_ticks;         // Length of steps requested from the PIO.
```

After the edit, `struct ConfigAxis` ends with:
```c
  double max_accel;               // ticks / update_time_ticks ^ 2
  int32_t velocity_achieved;      // Steps per update_time_us.
  int32_t position_error;         // Difference between requested and achieved position.
};
```

---

### Step 2: Remove both parameters from `update_joint_config` in `config.h`

Still in `src/rp2040/config.h`, remove the two parameters from the `update_joint_config` declaration:

```c
// DELETE these two lines from the update_joint_config signature:
    const int32_t* velocity_requested_tm1,
    const int32_t* step_len_ticks,
```

After the edit, the declaration reads:
```c
void update_joint_config(
    const uint8_t joint,
    const uint8_t core,
    const uint8_t* enabled,
    const int8_t* io_pos_step,
    const int8_t* io_pos_dir,
    const double* velocity_requested,
    const double* abs_pos_requested,
    const int32_t* abs_pos_achieved,
    const double* max_velocity,
    const double* max_accel,
    const int32_t* velocity_achieved,
    const int32_t* position_error
);
```

---

### Step 3: Remove both parameters from `get_joint_config` in `config.h`

Still in `src/rp2040/config.h`, remove two parameters from `get_joint_config`:

```c
// DELETE these two lines from the get_joint_config signature:
    int32_t* velocity_requested_tm1,
    int32_t* step_len_ticks,
```

After the edit, the declaration reads:
```c
uint32_t get_joint_config(
    const uint8_t joint,
    const uint8_t core,
    uint8_t* enabled,
    int8_t* io_pos_step,
    int8_t* io_pos_dir,
    double* velocity_requested,
    double* abs_pos_requested,
    int32_t* abs_pos_achieved,
    double* max_velocity,
    double* max_accel,
    int32_t* velocity_achieved,
    int32_t* position_error
    );
```

---

### Step 4: Remove initialisers from static config in `config.c`

In `src/rp2040/config.c`, the static initialiser for the four joints each has a `.velocity_requested_tm1 = 0,` line (around lines 50, 65, 80, 95). Delete all four. (`step_len_ticks` has no entry in the static initialiser — it is zeroed by `memset` in `init_config()`.)

Each joint block goes from ending with:
```c
      .velocity_requested_tm1 = 0,
      .velocity_achieved = 0
    },
```
to:
```c
      .velocity_achieved = 0
    },
```

---

### Step 5: Remove parameters from `update_joint_config` body in `config.c`

In `src/rp2040/config.c`, update the `update_joint_config` function definition:

Remove the two parameters from the signature:
```c
// DELETE these two lines from the function definition:
    const int32_t* velocity_requested_tm1,
    const int32_t* step_len_ticks,
```

Remove the two `if` blocks that write them (currently around lines 229–237):
```c
// DELETE these 6 lines:
  if(velocity_requested_tm1 != NULL) {
    config.joint[joint].velocity_requested_tm1 = *velocity_requested_tm1;
  }
  ...
  if(step_len_ticks != NULL) {
    config.joint[joint].step_len_ticks = *step_len_ticks;
  }
```

After the edit the body goes directly from the `max_accel` block to the `velocity_achieved` block:
```c
  if(max_accel != NULL) {
    config.joint[joint].max_accel = *max_accel;
  }
  if(velocity_achieved != NULL) {
    config.joint[joint].velocity_achieved = *velocity_achieved;
  }
  if(position_error != NULL) {
    config.joint[joint].position_error = *position_error;
  }
```

---

### Step 6: Remove parameters from `get_joint_config` body in `config.c`

Remove the two parameters from the `get_joint_config` function definition signature:
```c
// DELETE these two lines from the function definition:
    int32_t* velocity_requested_tm1,
    int32_t* step_len_ticks,
```

Remove the two `if` blocks that read them (currently around lines 318–326):
```c
// DELETE these 6 lines:
  if(velocity_requested_tm1 != NULL) {
    *velocity_requested_tm1 = config.joint[joint].velocity_requested_tm1;
  }
  ...
  if(step_len_ticks != NULL) {
    *step_len_ticks = config.joint[joint].step_len_ticks;
  }
```

After the edit the body goes directly from the `max_accel` block to the `velocity_achieved` block:
```c
  if(max_accel != NULL) {
    *max_accel = config.joint[joint].max_accel;
  }
  if(velocity_achieved != NULL) {
    *velocity_achieved = config.joint[joint].velocity_achieved;
  }
  if(position_error != NULL) {
    *position_error = config.joint[joint].position_error;
  }
```

---

### Step 7: Update `disable_joint` call in `config.c`

`disable_joint` calls `update_joint_config` with 14 args (12 NULLs after the two mandatory). Remove the two NULLs that were for `velocity_requested_tm1` and `step_len_ticks`. The call now has 12 args:

```c
void disable_joint(const uint8_t joint, const uint8_t core) {
  uint8_t enabled = 0;
  update_joint_config(
      joint,
      core,
      &enabled,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL);
}
```

---

### Step 8: Update `serialise_joint_movement` call in `config.c`

Remove the two commented-NULL args from the `get_joint_config` call inside `serialise_joint_movement`:

```c
// DELETE these two lines:
          NULL, //&velocity_requested_tm1,
          ...
          NULL, //&step_len_ticks,
```

After edit the call reads:
```c
      updated = get_joint_config(
          joint,
          CORE0,
          NULL, //&enabled,
          NULL, //&io_pos_step,
          NULL, //&io_pos_dir,
          NULL, //&velocity_requested,
          NULL, //&abs_pos_requested,
          &abs_pos_achieved,
          NULL, //&max_velocity,
          NULL, //&max_accel,
          &velocity_achieved,
          &position_error
          );
```

---

### Step 9: Update `serialise_joint_config` call in `config.c`

Remove the two commented-NULL args from the `get_joint_config` call inside `serialise_joint_config`:

```c
// DELETE these two lines:
        NULL, //&velocity_requested_tm1,
        ...
        NULL, //&step_len_ticks,
```

After edit the call reads:
```c
  get_joint_config(
        joint,
        CORE0,
        &enabled,
        &io_pos_step,
        &io_pos_dir,
        NULL, //&velocity_requested,
        NULL, //&abs_pos_requested,
        NULL, //&abs_pos_achieved,
        &max_velocity,
        &max_accel,
        NULL, //&velocity_achieved,
        NULL  //&position_error
        );
```

---

### Step 10: Rewrite `serialise_joint_metrics` in `config.c`

The function currently reads two fields via `get_joint_config`. Remove those locals, the `get_joint_config` call, and the two reply assignments. The new function body:

```c
bool serialise_joint_metrics(struct NWBuffer* tx_buf) {
  struct Reply_joint_metrics reply;
  reply.type = REPLY_JOINT_METRICS;

  for(size_t joint = 0; joint < MAX_JOINT; joint++) {
    reply.overrun_count[joint]  = get_and_reset_overrun_count(joint);
    reply.underrun_count[joint] = get_and_reset_underrun_count(joint);
  }

  uint16_t tx_buf_len = pack_nw_buff(tx_buf, &reply, sizeof(reply));

  if(!tx_buf_len) {
    printf("WARN: TX length greater than buffer size.\n");
    return false;
  }

  return true;
}
```

---

### Step 11: Update `get_joint_config` and `update_joint_config` calls in `pio.c`

In `src/rp2040/pio.c`, the `do_steps()` function has two call sites:

**First call** (`get_joint_config`, around line 205): remove the two commented-NULL lines:
```c
// DELETE these two lines:
      NULL, // &velocity_requested_tm1,
      ...
      NULL, // &step_len_ticks,
```

After edit:
```c
  uint32_t updated = get_joint_config(
      joint,
      CORE1,
      &enabled,
      NULL,
      NULL,
      &velocity_requested,
      &abs_pos_requested,
      &abs_pos_achieved,
      &max_velocity,
      &max_accel,
      NULL, // &velocity_achieved,
      NULL  // &position_error
      );
```

**Second call** (`update_joint_config`, around line 274): remove the local var `velocity_requested_tm1` and its argument, and remove `&step_len_ticks`:

Delete the line:
```c
  int32_t velocity_requested_tm1 = velocity_q / 65536;
```

In the `update_joint_config` call, remove:
```c
      &velocity_requested_tm1,
```
and:
```c
      &step_len_ticks,
```

After edit, the `update_joint_config` call reads:
```c
  update_joint_config(
      joint,
      CORE1,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      &abs_pos_achieved,
      NULL,
      NULL,
      &velocity_achieved,
      &position_error);
```

---

### Step 12: Remove two fields from `Reply_joint_metrics` in `messages.h`

In `src/shared/messages.h`, delete the two fields from `struct Reply_joint_metrics`:

```c
// DELETE these two lines:
  int32_t velocity_requested_tm1[MAX_JOINT];
  int32_t step_len_ticks[MAX_JOINT];
```

After edit `Reply_joint_metrics` reads:
```c
struct Reply_joint_metrics {
  uint8_t type;
  uint32_t overrun_count[MAX_JOINT];
  uint32_t underrun_count[MAX_JOINT];
};
```

---

### Step 13: Remove three fields from `skeleton_t` in `skeleton.h`

In `src/driver/skeleton.h`, delete three fields:

```c
// DELETE these three lines:
  hal_s32_t* joint_step_len_ticks[MAX_JOINT];
  // The requested velocity at time of joint_velocity_feedback.
  hal_float_t* joint_velocity_cmd[MAX_JOINT];
  hal_float_t* joint_accel_cmd[MAX_JOINT];
```

After edit, `skeleton_t` goes directly from `joint_scale` to `joint_position`:
```c
  hal_float_t* joint_scale[MAX_JOINT];
  hal_float_t* joint_position[MAX_JOINT];
  hal_float_t* joint_velocity[MAX_JOINT];
  hal_float_t* joint_pos_feedback[MAX_JOINT];
  // The actual velocity achieved on the RP.
  hal_float_t* joint_velocity_feedback[MAX_JOINT];
  // Difference between requested position and actual position on RP.
  hal_s32_t* joint_pos_error[MAX_JOINT];
```

---

### Step 14: Remove three rows from `joint_pins[]` in `hal_rp2040_eth.c`

In `src/driver/hal_rp2040_eth.c`, delete three rows from `joint_pins[]`:

```c
// DELETE these three lines:
    { S32,   HAL_OUT, offsetof(skeleton_t, joint_step_len_ticks),    sizeof(hal_s32_t*),   "joint", 0, 1, "fb-step-len"      },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_velocity_cmd),      sizeof(hal_float_t*), "joint", 0, 1, "fb-velocity-cmd"  },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_accel_cmd),         sizeof(hal_float_t*), "joint", 0, 1, "fb-accel-cmd"     },
```

After edit, `joint_pins[]` goes from `fb-pos` directly to `fb-velocity`:
```c
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_pos_feedback),      sizeof(hal_float_t*), "joint", 0, 1, "fb-pos"           },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_velocity_feedback), sizeof(hal_float_t*), "joint", 0, 1, "fb-velocity"      },
    { S32,   HAL_OUT, offsetof(skeleton_t, joint_pos_error),         sizeof(hal_s32_t*),   "joint", 0, 1, "fb-pos-error"     },
```

---

### Step 15: Remove three lines from `unpack_joint_metrics` in `rp2040_network.c`

In `src/driver/rp2040_network.c`, inside the `for` loop in `unpack_joint_metrics`, delete three lines:

```c
// DELETE this line:
    *data->joint_step_len_ticks[joint] = reply->step_len_ticks[joint];

// DELETE these three lines:
    *data->joint_accel_cmd[joint] =
      reply->velocity_requested_tm1[joint] - *data->joint_velocity_cmd[joint];
    *data->joint_velocity_cmd[joint] = reply->velocity_requested_tm1[joint];
```

After edit, the `for` loop body reads:
```c
  for(size_t joint = 0; joint < MAX_JOINT; joint++) {
    total_overrun  += reply->overrun_count[joint];
    total_underrun += reply->underrun_count[joint];
  }
```

---

### Step 16: Update `config_mocks.c`

In `src/test/mocks/config_mocks.c`, remove the two parameters from `update_joint_config`:

```c
// DELETE these two lines from the function signature:
    const int32_t* velocity_requested_tm1,
    const int32_t* step_len_ticks
```

After edit the mock reads:
```c
void update_joint_config(
    const uint8_t joint,
    const uint8_t core,
    const uint8_t* enabled,
    const int8_t* io_pos_step,
    const int8_t* io_pos_dir,
    const double* velocity_requested,
    const double* abs_pos_requested,
    const uint32_t* abs_pos_achieved,
    const double* max_velocity,
    const double* max_accel,
    const int32_t* velocity_achieved
)
{
}
```

---

### Step 17: Update `rp_network_tx_test.c`

In `src/test/rp_network_tx_test.c`, remove the two config assignments in `test_serialise_joint_metrics` (around lines 88 and 90):

```c
// DELETE these two lines:
        config.joint[joint].velocity_requested_tm1 = 135;
        config.joint[joint].step_len_ticks = 246;
```

---

### Step 18: Update `driver_network_RPtoPC_test.c`

In `src/test/driver_network_RPtoPC_test.c`:

**Remove three global declarations** (around lines 22–26):
```c
// DELETE these three lines:
hal_s32_t joint_step_len_ticks[4];
hal_float_t joint_velocity_cmd[4];
hal_float_t joint_accel_cmd[4];
```

**Remove three assignments from `setup_data()`** (around lines 49–51):
```c
// DELETE these three lines:
    data->joint_step_len_ticks[joint] = &(joint_step_len_ticks[joint]);
    data->joint_velocity_cmd[joint] = &(joint_velocity_cmd[joint]);
    data->joint_accel_cmd[joint] = &(joint_accel_cmd[joint]);
```

**Update `test_joint_metrics`**: remove the two fields from the message initialiser and remove the two per-joint assertions.

Remove from the `Reply_joint_metrics` initialiser:
```c
// DELETE these two lines:
        .velocity_requested_tm1 = {3456, 7890, 1234, 5678},
        .step_len_ticks = {1357, 2468, 3579, 4680},
```

Remove the two assertions in the `for` loop (around lines 219–220):
```c
// DELETE these two lines:
      assert_int_equal(*(data.joint_step_len_ticks[joint]), message.step_len_ticks[joint]);
      assert_int_equal(*(data.joint_velocity_cmd[joint]), message.velocity_requested_tm1[joint]);
```

After edit, `test_joint_metrics` has no per-joint assertions and the message initialiser only has `overrun_count` and `underrun_count`:
```c
    struct Reply_joint_metrics message = {
        .type = REPLY_JOINT_METRICS,
        .overrun_count  = {10, 20, 30, 40},
        .underrun_count = {1, 2, 3, 4}
    };
```

The `for` loop that contained the assertions is now empty — remove it entirely.

---

### Step 19: Update `driver_gpio_test.c`

In `src/test/driver_gpio_test.c`:

**Remove two global declarations** (around lines 22–25):
```c
// DELETE these two lines:
hal_s32_t joint_step_len_ticks[4];
hal_float_t joint_velocity_cmd[4];
```

**Remove two assignments from `setup_data()`** (around lines 48–49):
```c
// DELETE these two lines:
        data->joint_step_len_ticks[joint] =    &joint_step_len_ticks[joint];
        data->joint_velocity_cmd[joint] =      &joint_velocity_cmd[joint];
```

---

### Step 20: Build and run all tests

```bash
cmake -B build_tests -S . -DBUILD_TESTS=ON
make -C build_tests
ctest --test-dir build_tests --output-on-failure
```

Expected: all test suites pass with no errors or warnings about undefined symbols.

---

### Step 21: Commit

```bash
git add src/rp2040/config.h src/rp2040/config.c src/rp2040/pio.c \
        src/shared/messages.h \
        src/driver/skeleton.h src/driver/hal_rp2040_eth.c src/driver/rp2040_network.c \
        src/test/mocks/config_mocks.c \
        src/test/rp_network_tx_test.c \
        src/test/driver_network_RPtoPC_test.c \
        src/test/driver_gpio_test.c
git commit -m "refactor: remove velocity_requested_tm1 and step_len_ticks roundtrips"
```
