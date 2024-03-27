#include <stdio.h>

#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"

#else  // BUILD_TESTS

#include "pico/multicore.h"

#endif  // BUILD_TESTS


#include "core1.h"
#include "config.h"
#include "pio.h"


void update_all_joint() {
  static uint32_t last_time = 0;
  static uint32_t last_tick = 0;
  static uint32_t count = 0;
  static uint32_t max_dt = 0;
  static uint32_t min_dt = 10000;
  static uint32_t network = 1;
  uint32_t update_period_us = get_period();
  uint32_t now_time;
  uint32_t dt = 0;

  // Wait for semaphore from core0 to indicate time start.
  while(tick == last_tick) {
    now_time = time_us_64();
    dt = now_time - last_time;
    if(dt > update_period_us * MAX_MISSED_PACKET) {
      // Timeout
      break;
    }
  }
  if(tick == last_tick) {
    // Didn't receive tick semaphore. Implies no network packet arrived.
    if(network) {
      printf("No network update for %ums.\tDisabling joints.\n", dt / 1000);
    }
    for(uint8_t joint = 0; joint < MAX_JOINT; joint++) {
      disable_joint(joint, CORE1);
    }
    network = 0;
  } else {
    // Did receive tick semaphore.
    network = 1;
  }
  last_tick = tick;

  now_time = time_us_64();
  dt = now_time - last_time;
  if(dt > max_dt) {
    max_dt = dt;
  }
  if(dt < min_dt) {
    min_dt = dt;
  }
  if(count++ % 10000 == 0) {
    // Periodic message to console.
    printf("%i\t%i\n", min_dt - 1000, max_dt - 1000);
    max_dt = 0;
    min_dt = 10000;
  }
  last_time = now_time;

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


