#include <stdio.h>
#include "pico/multicore.h"

#include "core1.h"
#include "config.h"
#include "pio.h"


void update_all_joint() {
  static uint32_t metric = 0;
  static uint32_t last_tick = 0;
  static uint32_t count = 0;
  static uint32_t max_dt = 0;
  static uint32_t min_dt = 10000;
  static uint32_t no_network = 0;
  uint32_t update_period_us = get_period();
  uint32_t metric_now;
  uint32_t dt = 0;

  // Wait for semaphore from core0 to indicate time start.
  while(tick == last_tick) {
    metric_now = time_us_64();
    dt = metric_now - metric;
    if(dt > update_period_us * MAX_MISSED_PACKET) {
      break;
    }
  }
  if(tick == last_tick) {
    // Didn't receive tick semaphore.
    for(uint8_t joint = 0; joint < MAX_JOINT; joint++) {
      disable_joint(joint, CORE1);
    }
    if(!no_network) {
      printf("No network update for %ums.\tDisabling joints.\n", dt / 1000);
    }
    no_network = 1;
  } else {
    // Did receive tick semaphore.
    no_network = 0;
  }
  last_tick = tick;

  metric_now = time_us_64();
  dt = metric_now - metric;
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

  for(uint8_t joint = 0; joint < MAX_JOINT; joint++) {
    do_steps(joint, update_period_us);
  }
}

void core1_main() {
  while (1) {
    update_all_joint();
  }
}

void init_core1() {
  printf("core0: Initializing.\n");

  if(MAX_JOINT > 4) {
    printf("ERROR: Maximum joint count: 4. Configured: %u\n", MAX_JOINT);
    while (1) {
      tight_loop_contents();
    }
  }

  // Launch core1.
  multicore_launch_core1(&core1_main);
}


