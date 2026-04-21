#ifndef CORE1__H
#define CORE1__H

#include <stdint.h>
#include <stdbool.h>

/* Block until a new packet batch is ready (packet_generation advances).
 * Falls back immediately if the network has been silent for MAX_MISSED_PACKET
 * ticks — network loss is then handled by check_network_health(). */
void wait_for_packet(void);

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
/* Run exactly one iteration of the core1_main loop body. */
void core1_run_once_for_test(void);
#endif

#endif  // CORE1__H
