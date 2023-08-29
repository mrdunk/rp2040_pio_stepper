#include <stdio.h>
#include "pico/multicore.h"

#include "core1.h"
#include "config.h"


uint8_t update_axis(uint8_t axis) {
  uint32_t abs_pos_requested;
  uint32_t abs_pos_acheived;
  uint32_t min_step_len_ticks;
  uint32_t max_accel_ticks;
  uint32_t velocity_acheived;
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

void update_all_axis() {
  uint8_t updated_count = 0;
  for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
    updated_count += update_axis(axis);
  }
  if(updated_count > 0) {
    printf("Updated: %u \t%lu\n", updated_count, get_config());
  }
}

void core1_main() {
  uint32_t count = 0;
  while (1) {
    //gpio_put(LED_PIN, (time_us_64() / 1000000) % 2);
    //gpio_put(LED_PIN, 1);
    //sleep_us(100000);
    //gpio_put(LED_PIN, 0);

    //sleep_us(100000);
    //printf(".");
    //if((count++ % 200) == 0) {
    //  printf("\n");
    //}


    //sleep_us(1000);
    update_all_axis();
  }
}

void init_core1() {
  printf("core0: Initializing.\n");

  // Launch core1.
  multicore_launch_core1(&core1_main);
}


