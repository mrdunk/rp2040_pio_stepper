#ifndef PIO__H
#define PIO__H

#include <stdint.h>

/* Initialize a pair of PIO programmes.
 * One for step generation on pio0 and one for counting said steps on pio1.
 */
void init_pio(const uint32_t joint);

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t joint);

double clamp_accel(double velocity, double last_velocity, double max_accel);

#ifdef BUILD_TESTS
/* Reset all static state in pio.c — used by test setup fixtures only. */
void pio_reset_for_test(void);

/* Exposed for unit testing only. */
double get_velocity(uint32_t update_period_us, uint8_t joint,
                    int32_t abs_pos_achieved, double abs_pos_requested,
                    double expected_velocity);
int32_t drain_rx_fifo(uint32_t sm, int32_t current_pos);
int32_t calculate_step_len(double step_count, double update_period_ticks,
                           double max_velocity);
#endif  // BUILD_TESTS

#endif  // PIO__H
