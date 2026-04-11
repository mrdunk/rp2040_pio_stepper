#include <stdio.h>

#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"

#else  // BUILD_TESTS

#include "pico/multicore.h"

#endif  // BUILD_TESTS

#include "core1.h"
#include "config.h"
#include "pio.h"

static uint32_t last_tick  = 0;
static bool     no_network = false;

void wait_for_tick(void) {
  while (tick == last_tick) {}
  last_tick = tick;
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
  printf("No network update. Disabling joints.\n");
  no_network = true;
}

void handle_network_recovery(void) {
  if (!no_network) {
    return;
  }
  printf("Network recovered.\n");
  no_network = false;
}

void step_all_joints(void) {
  for (uint8_t joint = 0; joint < MAX_JOINT; joint++) {
    do_steps(joint);
  }
}

static void core1_tick(void) {
  wait_for_tick();
  if (linuxcnc_restart_detected) {
    linuxcnc_restart_detected = false;
    handle_network_timeout();
  }
  if (!check_network_health()) {
    handle_network_timeout();
  } else {
    handle_network_recovery();
    step_all_joints();
  }
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

  if (MAX_JOINT > 4) {
    printf("ERROR: Maximum joint count: 4. Configured: %u\n", MAX_JOINT);
    while (1) {
      tight_loop_contents();
    }
  }

  multicore_launch_core1(&core1_main);
}

#ifdef BUILD_TESTS
void core1_reset_for_test(void) {
  last_tick  = 0;
  no_network = false;
}
#endif
