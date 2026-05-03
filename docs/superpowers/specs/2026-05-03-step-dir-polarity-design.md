# Step / Direction Pin Polarity — Design Spec

## Context

Some stepper drivers expect an active-low step pulse or an inverted direction signal.
Currently the `step_gen` PIO always drives the step pin active-high and the direction
pin with a fixed polarity. This spec adds independent, per-joint polarity inversion for
both pins.

## Approach

Use `gpio_set_outover(pin, GPIO_OVERRIDE_INVERT)` after PIO initialisation. This is a
hardware register that inverts the GPIO output after the PIO output path. The `step_count`
SM reads via the input path (the physical pad level), so it correctly detects one edge per
step regardless of the step-pin polarity. No PIO program changes are required.

Inversion is a wiring characteristic set at startup; it follows the same set-once
constraint as the existing `gpio-step` / `gpio-dir` params.

---

## Wire protocol — `src/shared/messages.h`

Both `Message_joint_config` and `Reply_joint_config` have the same layout and both have
3 pad bytes after `gpio_dir`. Repurpose one in each as `uint8_t invert_flags`; reduce
`_pad[3]` to `_pad[2]`. Struct sizes stay at 16 bytes.

Define two constants:

```c
#define JOINT_INVERT_STEP (1 << 0)
#define JOINT_INVERT_DIR  (1 << 1)
```

The reply echoes the flags back so the driver's existing retry-until-reply-matches
mechanism confirms the RP2040 received the config.

---

## RP2040 firmware

### `src/rp2040/config.h`

Add to `ConfigAxis`:

```c
bool step_invert;
bool dir_invert;
```

### `src/rp2040/config.c`

- `set_joint_config()` — accept and store `bool step_invert`, `bool dir_invert`
- `get_joint_config()` — add two nullable out-params `bool* step_invert`, `bool* dir_invert`

### `src/rp2040/core0.c`

When handling `MSG_SET_JOINT_CONFIG`:
- Unpack `invert_flags` from the message
- Derive `step_invert = (invert_flags & JOINT_INVERT_STEP) != 0`
- Derive `dir_invert  = (invert_flags & JOINT_INVERT_DIR)  != 0`
- Pass to `set_joint_config()`

When building `Reply_joint_config`:
- Set `reply.invert_flags` from the stored config values

### `src/rp2040/pio.c` — `init_pio()`

After `step_gen_program_init()` and `step_count_program_init()`, read the flags and apply:

```c
bool step_invert, dir_invert;
get_joint_config(joint, CORE1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                 &step_invert, &dir_invert);
gpio_set_outover(io_pos_step, step_invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
gpio_set_outover(io_pos_dir,  dir_invert  ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
```

---

## Driver

### `src/driver/skeleton.h`

```c
hal_bit_t* joint_step_invert[MAX_JOINT];
hal_bit_t* joint_dir_invert[MAX_JOINT];
```

### `src/driver/hal_rp2040_eth.c`

Add two rows to `joint_pins[]` as `PARAM / HAL_RW` (BIT type), following the
`gpio-step` / `gpio-dir` pattern:

| Name | Type | Dir | Default |
|---|---|---|---|
| `joint.N.step-invert` | BIT | RW param | false |
| `joint.N.dir-invert`  | BIT | RW param | false |

Include both fields in the change-detection comparison so a polarity change triggers a
new `MSG_SET_JOINT_CONFIG`.

### `src/driver/rp2040_network.c`

- `serialize_joint_config()` gains `uint8_t invert_flags` parameter; packs it into the
  message
- `unpack_joint_config()` reads `invert_flags` from `Reply_joint_config` and stores it
  in `last_joint_config` (used by the retry mechanism)

---

## Tests

### `src/test/driver_network_PCtoRP_test.c`

Extend `test_serialize_joint_config` to verify `invert_flags` round-trips correctly for
both bits independently.

### `src/test/driver_network_RPtoPC_test.c`

Extend `test_unpack_joint_config` to verify echoed `invert_flags` are read back
correctly.

### `src/test/driver_network_RPtoPC_test.c` and `driver_gpio_test.c`

Add the new skeleton fields to their `setup_data()` helpers.

### RP2040-side mocks

`gpio_set_outover` is an SDK call; add a stub to `src/test/mocks/rp_mocks.h` /
`rp_mocks.c` if not already present.

---

## Out of scope

- Runtime polarity changes (requires re-init; restart LinuxCNC to change)
- Polarity inversion for GPIO outputs (already handled separately by `gpio.N.out-invert`)
