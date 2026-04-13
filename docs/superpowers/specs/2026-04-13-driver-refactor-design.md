# Driver Refactor Design — 2026-04-13

## Scope

Two independent changes to `src/driver/`:

1. Move machine config files to a top-level `config/` directory.
2. Eliminate repetitive boilerplate in the C driver.

---

## 1. Directory Restructure

Move `frankencnc.hal` and `frankencnc.ini` from `src/driver/` to `config/frankencnc/`.

`rp2040_gpio_types.ini` stays in `src/driver/` — it is driver-provided constants, not machine config.

The `#INCLUDE` line in `frankencnc.ini` updates to reflect the new relative path. LinuxCNC resolves `#INCLUDE` relative to the including file's directory, so:

```ini
#INCLUDE ../../src/driver/rp2040_gpio_types.ini
```

Verify this by launching LinuxCNC from `config/frankencnc/` and confirming the GPIO type constants are visible in HAL.

**Result:**

```
config/
  frankencnc/
    frankencnc.hal
    frankencnc.ini
src/driver/
    hal_rp2040_eth.c
    rp2040_network.c
    rp2040_defines.h
    rp2040_gpio_types.ini
```

The HAL file itself is not refactored. Per-joint wiring blocks are intentionally spelled out — the declarative repetition aids readability and machine debugging.

---

## 2. `unpack_*` Macro — `rp2040_network.c`

All 8 `unpack_*` functions share identical 5-line scaffolding:

```c
void* data_p = unpack_nw_buff(rx_buf, *rx_offset, rx_offset, NULL, sizeof(T));
if (!data_p) return false;
T* reply = data_p;
// ... unique logic ...
(*received_count)++;
return true;
```

Add one macro at the top of `rp2040_network.c`:

```c
#define UNPACK_MSG(T, var, buf, offset) \
    T* var = unpack_nw_buff((buf), *(offset), (offset), NULL, sizeof(T)); \
    if (!(var)) return false;
```

Apply to all 8 functions. The variable is named `reply` in every case (matching existing code), so diffs are minimal. Each function shrinks to its unique logic only:

```c
bool unpack_timing(struct NWBuffer* rx_buf, size_t* rx_offset,
                   size_t* received_count, skeleton_t* data) {
    UNPACK_MSG(struct Reply_timing, reply, rx_buf, rx_offset);
    *data->metric_update_id     = reply->update_id;
    *data->metric_time_diff     = reply->time_diff;
    *data->metric_rp_update_len = reply->rp_update_len;
    (*received_count)++;
    return true;
}
```

---

## 3. Pin Descriptor Table — `hal_rp2040_eth.c`

### Problem

`rtapi_app_main` contains 25+ near-identical blocks:

```c
if (!init_hal_pin(PIN, HAL_IN, &(port_data_array->joint_enable[n]),
                  component_id, device_num, "joint", n, 1, "enable")) {
    return -1;
}
```

New pins (e.g. a new metric or feedback field) require copy-pasting the whole block.

### Solution

Define a `PinDef` descriptor struct and static tables per subsystem. One loop per table replaces the individual calls.

```c
typedef struct {
    enum t_types   type;
    hal_pin_dir_t  dir;
    size_t         offset;        // offsetof(skeleton_t, field[0])
    size_t         stride;        // sizeof field element, for per-channel pins
    const char*    io_type;
    int            chan_num;      // -1 = no channel suffix
    int            chan_num_len;
    const char*    specific_name;
} PinDef;
```

Per-channel subsystems (joints, GPIO) use a `for(int i = 0; i < MAX_X; i++)` wrapper. The actual pointer passed to `init_hal_pin` is computed as:

```c
void* field_ptr = (void*)((char*)port_data_array + def->offset + i * def->stride);
```

This is equivalent to `&(port_data_array->field[i])` and is well-defined in C since `port_data_array` points to the struct. `chan_num` is passed as `i`.

Single-instance pins (metrics, machine-enable) use `chan_num = -1`.

### Error handling

All error paths in `rtapi_app_main` use `goto port_error`. The label already calls `hal_exit` and returns -1. Callers become:

```c
if (!init_hal_pin(...)) goto port_error;
```

The two `return -1` calls that precede `hal_init` (before `component_id` is valid) remain as-is — `hal_exit` cannot be called before `hal_init` succeeds.

Spindle `hal_param_*` calls are also converted to `goto port_error` (they already use it; this removes any remaining inconsistencies).

---

## 4. Pre-refactor Tests

Add tests for the three `unpack_*` functions currently without coverage, before applying `UNPACK_MSG`. This ensures any regression from the macro is caught immediately.

| Function | Test file | What to assert |
|---|---|---|
| `unpack_gpio_config` | `driver_gpio_test.c` | `last_gpio_config[gpio].gpio_type`, `.index`, `.address` updated from `Reply_gpio_config` |
| `unpack_spindle_speed` | `driver_network_RPtoPC_test.c` | `spindle_speed_out` and `spindle_at_speed` set correctly from `Reply_spindle_speed` |
| `unpack_spindle_config` | `driver_network_RPtoPC_test.c` | `last_spindle_config[n].modbus_address`, `.vfd_type`, `.bitrate` updated from `Reply_spindle_config` |

Each test follows the same pattern as existing unpack tests: build a reply struct, `memcpy` into a `NWBuffer`, call `process_data`, assert fields.

`setup_data()` in `driver_network_RPtoPC_test.c` will need spindle fields wired up (`spindle_speed_out`, `spindle_at_speed`, `spindle_speed_in`, `spindle_poles`).

---

## Out of scope

- `frankencnc.hal` wiring blocks — left unrolled by design.
- `rp2040_network.c` `serialize_*` / `configure_*` functions — structurally different enough to not warrant the same treatment.
- No functional changes: wire format, HAL pin names, and runtime behaviour are unchanged.
