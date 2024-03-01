#include <stdio.h>
#include "pico/multicore.h"

#include "core1.h"
#include "config.h"
#include "i2c.h"
#include "pio.h"

struct i2c_gpio_state i2c_gpio;

void update_all_joint() {
  static uint32_t metric = 0;
  static uint32_t last_tick = 0;
  uint32_t update_period_us = get_period();
  uint8_t updated_count = 0;

  // Wait for semaphore from core0 to indicate time start.
  while(tick == last_tick) {
    i2c_gpio_poll(&i2c_gpio);
    tight_loop_contents();
  }
  last_tick = tick;
  i2c_gpio_poll(&i2c_gpio);

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

  for(uint8_t joint = 0; joint < MAX_JOINT; joint++) {
    updated_count += do_steps(joint, update_period_us);
  }
  //update_pio_io_configured();
}

void core1_main() {
  while (1) {
    update_all_joint();
  }
}

void init_core1() {
  printf("core0: Initializing.\n");
  i2c_gpio_init(&i2c_gpio);

  if(MAX_JOINT > 4) {
    printf("ERROR: Maximum joint count: 4. Configured: %u\n", MAX_JOINT);
    while (1) {
      tight_loop_contents();
    }
  }

  // Launch core1.
  multicore_launch_core1(&core1_main);
}


