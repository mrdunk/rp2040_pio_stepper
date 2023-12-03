#include <stdio.h>
#include "pico/multicore.h"

#include "core1.h"
#include "config.h"
#include "pio.h"


void update_all_axis() {
  static uint32_t metric = 0;
  static uint32_t last_tick = 0;
  uint32_t update_period_us = get_period();
  uint8_t updated_count = 0;

  // Wait for semaphore from core0 to indicate time start.
  while(tick == last_tick) {
    tight_loop_contents();
  }
  last_tick = tick;

  static uint32_t count = 0;
  static uint32_t max_dt = 0;
  static uint32_t min_dt = 10000;
  uint32_t metric_now = time_us_64();
  uint32_t dt = metric_now - metric;
  if(dt > max_dt) {
    max_dt = dt;
  }
  if(dt < min_dt) {
    min_dt = dt;
  }
  if(count++ % 10000 == 0) {
    printf("%i\t%i\n", min_dt - 1000, max_dt - 1000);
    max_dt = 0;
    min_dt = 10000;
  }
  metric = metric_now;

  for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
    updated_count += do_steps(axis, update_period_us);
  }
  //update_pio_io_configured();
}

void core1_main() {
  while (1) {
    update_all_axis();
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


