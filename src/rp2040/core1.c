#include <stdio.h>
#include "pico/multicore.h"

#include "core1.h"
#include "config.h"
#include "pio.h"


void update_all_axis() {
  static size_t last_time = 0;
  static float time_offset = 0;
  static uint32_t metric = 0;
  static uint32_t last_tick = 0;
  uint32_t update_time_us = get_period();
  uint8_t updated_count = 0;

  size_t time_now = time_us_64();
  //while(time_now < last_time + update_time_us + (time_offset)) {
  //  time_now = time_us_64();
  //}
  //last_time = time_now;

  // Block waiting for new data.
  // This is governed by data arriving via Ethernet on core0.
  //for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
    //while(has_new_c0_data(axis) == 0) {
    //  tight_loop_contents();
    //}
  //}
  
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
  if(count++ % 1000 == 0) {
    //if(max_dt > 2000) {
      printf("\t\t\t %i\t%i\t%u\n", min_dt - 1000, max_dt - 1000, (time_now % 1000));
    //}
    max_dt = 0;
    min_dt = 10000;
  }
  metric = metric_now;

  for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
    updated_count += do_steps(axis, update_time_us);
  }
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


