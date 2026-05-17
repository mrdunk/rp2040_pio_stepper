#include <stdio.h>

#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"

#else  // BUILD_TESTS

#include "pico/multicore.h"
#include "i2c.h"

#endif  // BUILD_TESTS

#include "core1.h"
#include "config.h"
#include "pio.h"

static uint32_t last_tick               = 0;
static uint32_t last_packet_generation  = 0;
static bool     no_network              = false;

void wait_for_packet(void) {
  while (tick == last_tick) {}
  last_tick = tick;

  /* Skip the generation wait if the network is already lost. */
  if ((tick - last_packet_tick) > MAX_MISSED_PACKET) {
    last_packet_generation = packet_generation;
    return;
  }

  /* Spin until Core0 finishes writing all joint configs for this packet.
   * Also break as soon as last_packet_tick < tick: this means no packet has
   * arrived for the current tick, so Core0 is either slow or gone.  Exiting
   * immediately keeps the loop period at exactly one timer tick (1 ms) so
   * step_all_joints() runs every period and the PIO FIFO stays fed during
   * deceleration.  The hard-timeout check is a belt-and-braces fallback. */
  while (packet_generation == last_packet_generation) {
    if (last_packet_tick < tick || (tick - last_packet_tick) > MAX_MISSED_PACKET) {
      break;
    }
  }
  last_packet_generation = packet_generation;
}

bool check_network_health(void) {
  return (tick - last_packet_tick) <= MAX_MISSED_PACKET;
}

void handle_network_timeout(void) {
  if (no_network) {
    return;
  }
  for (uint8_t joint = 0; joint < MAX_JOINT; joint++) {
    disable_joint(joint, CORE1);
  }
  printf("No NW update. Disable joints\n");
  no_network = true;
}

void handle_network_recovery(void) {
  if (!no_network) {
    return;
  }
  printf("Network recovered\n");
  no_network = false;
}

void step_all_joints(void) {
  for (uint8_t joint = 0; joint < MAX_JOINT; joint++) {
    do_steps(joint);
  }
}

static void core1_tick(void) {
  wait_for_packet();
  uint64_t t_start = time_us_64();
  core1_loop_count++;
  if (linuxcnc_restart_detected) {
    linuxcnc_restart_detected = false;
    handle_network_timeout();
  } else if (!check_network_health()) {
    handle_network_timeout();
  } else {
    handle_network_recovery();
  }
  step_all_joints();
#ifndef BUILD_TESTS
  i2c_gpio_poll(&i2c_gpio);
#endif
  core1_work_us = (uint32_t)(time_us_64() - t_start);
}

void core1_main(void) {
  while (1) {
    core1_tick();
  }
}

#ifdef BUILD_TESTS
void core1_run_once_for_test(void) {
  core1_tick();
}
#endif

void init_core1(void) {
  printf("core0: Initializing.\n");

  if (MAX_JOINT > 8) {
    printf("ERROR: Max joint count: 8. Configured: %u\n", MAX_JOINT);
    while (1) {
      tight_loop_contents();
    }
  }

#ifndef BUILD_TESTS
  i2c_gpio_init(&i2c_gpio);
#endif

  multicore_launch_core1(&core1_main);
}

#ifdef BUILD_TESTS
void core1_reset_for_test(void) {
  last_tick               = 0;
  last_packet_generation  = 0;
  no_network              = false;
}
#endif
