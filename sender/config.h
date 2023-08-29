#ifndef CONFIG__H
#define CONFIG__H

#include <stdlib.h>
#include <stdint.h>
#include <limits.h>

#include "sender.h"

/* Configuration object for an axis. */
struct ConfigAxis {
  uint8_t updated_c0;          // Data was updated on core 0.
  uint8_t updated_c1;          // Data was updated on core 1.
  uint32_t abs_pos;             // In steps. Default value is UINT_MAX / 2.
  uint32_t min_step_len_ticks;
  uint32_t max_accel_ticks;     // ticks / update_time_ticks ^ 2
  int32_t velocity;             // Steps per update_time_us.
};

/* Configuration object for global settings. */
struct ConfigGlobal {
  // Both the following are really the same thing, portrayed different ways.
  // Update using set_global_update_rate(...).
  uint32_t update_rate;
  uint32_t update_time_us;  // (1,000,000) / update_rate

  uint32_t update_time_ticks;  // clock_speed / update_rate

  struct ConfigAxis axis[MAX_AXIS];
};

void init_config();

void update_axis(
    const uint8_t axis,
    const uint32_t* abs_pos,
    const uint32_t* min_step_len_ticks,
    const uint32_t* max_accel_ticks,
    const int32_t* velocity);

uint32_t get_axis(
    const uint8_t axis,
    uint32_t* abs_pos,
    uint32_t* min_step_len_ticks,
    uint32_t* max_accel_ticks,
    int32_t* velocity);

#endif  // CONFIG__H
