#include <stdio.h>
#include "pico/multicore.h"

#include "core1.h"
#include "config.h"
#include "pio.h"


void update_all_axis() {
  uint32_t update_time_us = get_period();
  uint8_t updated_count = 0;

  // Block waiting for new data.
  // This is governed by data arriving vial Ethernet on core0.
  for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
    while(has_new_c0_data(axis) == 0) {
      tight_loop_contents();
    }
  }

  for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
    updated_count += do_steps(axis, update_time_us);
  }

#if DEBUG_OUTPUT
  if(updated_count > 0) {
    printf("Updated: %u \t%lu\n", updated_count, get_period());
  }
#endif
}

void core1_main() {
  uint32_t count = 0;
  while (1) {
    update_all_axis();

    /*
    if(count % 200 == 0) {
      printf(".");
      if(count % 20000 == 0) {
        printf("\n");
      }
    };
    count++;
    */
  }
}

void init_core1() {
  printf("core0: Initializing.\n");

  if(MAX_AXIS > 4) {
    printf("ERROR: Maximum axis count: 4. Configured: %u\n", MAX_AXIS);
    while (1) {
      tight_loop_contents();
    }
  }

  // Initialise PIOs.
  // TODO: Set up pins through config.
  //const uint32_t pins_step[8] =      {0, 2, 4, 6, 8, 10, 12, 14};
  //const uint32_t pins_direction[8] = {1, 3, 5, 7, 9, 11, 13, 15};
  //for (uint32_t axis = 0; axis < MAX_AXIS; axis++) {
  //  init_pio(axis, pins_step[axis], pins_direction[axis]);
  //}

  // Launch core1.
  multicore_launch_core1(&core1_main);
}


