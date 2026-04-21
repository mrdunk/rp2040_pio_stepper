#ifndef PIO__H
#define PIO__H

#include <stdint.h>

/* Initialize a pair of PIO programmes.
 * One for step generation on pio0 and one for counting said steps on pio1.
 */
void init_pio(const uint32_t joint);

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t joint);

int32_t clamp_accel(int32_t velocity_q, int32_t last_velocity_q, int32_t max_accel_q);

#ifdef BUILD_TESTS
/* Reset all static state in pio.c — used by test setup fixtures only. */
void pio_reset_for_test(void);

/* Exposed for unit testing only. */
int32_t get_velocity(int32_t pos_diff_q, int32_t vel_req_q);
int32_t drain_rx_fifo(uint32_t sm, int32_t current_pos);
int32_t calculate_step_len(int32_t step_count_q, int32_t period_ticks, int32_t max_vel_q);
int32_t plan_steps(int32_t velocity_q, uint8_t joint, int32_t period_ticks, int32_t step_len);
#endif  // BUILD_TESTS

#endif  // PIO__H
