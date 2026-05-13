#ifndef PIO__H
#define PIO__H

#include <stdint.h>

/* Initialize PIO state machines for a joint.
 * Always sets up a step_gen SM on the appropriate PIO block.
 * Also sets up a step_count SM on PIO1 for joints 0..NUM_FEEDBACK-1.
 */
void init_pio(const uint32_t joint);

/* Compute the commanded velocity (steps/s) for this period.
 * Applies the position controller (position mode) and collapses to 0 when
 * disabled or when no new Core0 data is available (underrun / network loss). */
double compute_velocity_cmd(
    uint8_t  cmd_type,
    double   velocity_requested,
    double   abs_pos_requested,
    int32_t  abs_pos_achieved,
    uint8_t  enabled,
    uint32_t updated,
    uint32_t update_period_us);

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t joint);

int32_t clamp_accel(int32_t velocity_q, int32_t last_velocity_q, int32_t max_accel_q);

#ifdef BUILD_TESTS
/* Reset all static state in pio.c — used by test setup fixtures only. */
void pio_reset_for_test(void);

/* Exposed for unit testing only. */
int32_t drain_rx_fifo(uint32_t sm, int32_t current_pos);
int32_t calculate_step_len(int32_t step_count_q, int32_t period_ticks, int32_t max_vel_q);
int32_t plan_steps(int32_t velocity_q, uint8_t joint, int32_t period_ticks, int32_t step_len);
#endif  // BUILD_TESTS

#endif  // PIO__H
