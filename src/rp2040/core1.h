#ifndef CORE1__H
#define CORE1__H

#include <stdint.h>
#include <stdbool.h>

/* Block until tick changes. Updates internal last_tick. */
void wait_for_tick(void);

/* Return true if the gap between tick and last_packet_tick is within
 * MAX_MISSED_PACKET ticks, indicating the network is healthy. */
bool check_network_health(void);

/* Disable all joints. Prints once per outage (debounced by no_network flag). */
void handle_network_timeout(void);

/* If no_network was set: log reconnection and clear it. Otherwise no-op. */
void handle_network_recovery(void);

/* Call do_steps() for all MAX_JOINT joints. */
void step_all_joints(void);

void init_core1(void);
void core1_main(void);

#ifdef BUILD_TESTS
/* Reset all static state. Call at the start of each test. */
void core1_reset_for_test(void);
#endif

#endif  // CORE1__H
