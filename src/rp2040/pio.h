#ifndef PIO__H
#define PIO__H

#include <stdint.h>

/* Allow firmware to accelerate slightly faster than LinuxCNC's ramp rate so
 * integer truncation of max_accel_q never causes systematic tracking lag. */
#define ACCEL_HEADROOM 1.1

/* Nominal LinuxCNC servo period in µs.  Velocity steps-per-period is computed
 * from this fixed constant rather than the EMA-measured inter-packet interval
 * so that network-jitter bias in the EMA cannot cause systematic step
 * undershoot.  Crystal-frequency disagreement between host and RP is negligible
 * (±50 ppm → ±0.2 steps/s at 4000 steps/s) and is corrected by the position
 * feedback loop in velocity mode. */
#define SERVO_PERIOD_US 1000

/* Allow firmware to reach slightly higher velocity than LinuxCNC's configured
 * maximum so the position-correction term has headroom at full commanded speed.
 * Without this the max_vel_q cap in calculate_step_len silently discards
 * correction steps when vel-cmd is at its limit. */
#define VEL_HEADROOM 1.01

/* Initialize PIO state machines for a joint.
 * Always sets up a step_gen SM on the appropriate PIO block.
 * Also sets up a step_count SM on PIO1 for joints 0..NUM_FEEDBACK-1.
 */
void init_pio(const uint32_t joint);

/* Compute the commanded velocity (steps/s) for this period.
 * Applies the position controller (position mode) and collapses to 0 when
 * disabled or when no new Core0 data is available (underrun / network loss).
 * max_accel (steps/s²) caps the position correction to sqrt(2*max_accel*|error|)
 * — the bang-bang stopping profile — so the motor can always decelerate to rest
 * within the remaining error distance.  Pass 0.0 to disable the cap. */
double compute_velocity_cmd(
    uint8_t  cmd_type,
    double   velocity_requested,
    double   abs_pos_requested,
    int32_t  abs_pos_achieved,
    uint8_t  enabled,
    uint32_t updated,
    uint32_t update_period_us,
    double   max_accel);

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
