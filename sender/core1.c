#include <stdio.h>
#include "pico/multicore.h"

#include "core1.h"
#include "config.h"
#include "pio.h"

/*
uint8_t update_axis(uint8_t axis) {
  uint32_t abs_pos_requested;
  uint32_t abs_pos_acheived;
  uint32_t min_step_len_ticks;
  uint32_t max_accel_ticks;
  int32_t velocity_acheived;
  uint32_t updated;

  updated = get_axis_config(
      axis,
      CORE1,
      &abs_pos_requested,
      &abs_pos_acheived,
      &min_step_len_ticks,
      &max_accel_ticks,
      &velocity_acheived
      );
  if(updated > 0) {
    if(updated > 1) {
      printf("WARN: C1, multiple updates: %u \t%lu\n", axis, updated);
    }
    static size_t count[MAX_AXIS] = {0};
    uint32_t abs_pos_acheived = count[axis];
    update_axis_config(axis, CORE1, NULL, &abs_pos_acheived, NULL, NULL, NULL);
    count[axis]++;

    //printf("%u \t %lu \t %lu \t %lu \t %lu \n",
    //    axis, abs_pos, min_step_len_ticks, max_accel_ticks, velocity);
    return 1;
  }
  return 0;
}
*/

void update_all_axis() {
  uint32_t update_time_us = get_config();
  uint8_t updated_count = 0;
  while(has_new_c0_data(0) == 0) {
    tight_loop_contents();
  }
  printf("%u\n", update_time_us);

  for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
    //updated_count += update_axis(axis);
    updated_count += do_steps(axis, update_time_us);
  }

#if DEBUG_OUTPUT
  if(updated_count > 0) {
    printf("Updated: %u \t%lu\n", updated_count, get_config());
  }
#endif
}

void core1_main() {
  uint32_t count = 0;
  while (1) {
    update_all_axis();
  }
}

void init_core1() {
  printf("core0: Initializing.\n");

  // Initialise PIOs.
  // TODO: Set up pins through config.
  const uint32_t stepper_count = MAX_AXIS;
  const uint32_t pins_step[8] =      {0, 2, 4, 6, 8, 10, 12, 14};
  const uint32_t pins_direction[8] = {1, 3, 5, 7, 9, 11, 13, 15};
  for (uint32_t stepper = 0; stepper < stepper_count; stepper++) {
    init_pio(stepper, pins_step[stepper], pins_direction[stepper]);
  }

  // Launch core1.
  multicore_launch_core1(&core1_main);
}


