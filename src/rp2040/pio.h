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

/* Minimum DIR-stable time before the STEP rising edge.  When a direction change
 * occurs and low1 (= step_low_half) is shorter than this, a bit is set in the
 * value returned by pio_get_and_clear_dir_setup_violations(). */
#define DIR_SETUP_MIN_US     5     /* µs */
#define DIR_SETUP_MIN_CYCLES 665   /* DIR_SETUP_MIN_US × 133 MHz */

/* Initialize PIO state machines for a joint.
 * Always sets up a step_gen SM on the appropriate PIO block.
 * Also sets up a step_count SM on PIO1 for joints 0..NUM_FEEDBACK-1.
 */
void init_pio(const uint32_t joint);

/* Clear init_done for all joints so init_pio() re-runs on next enable.
 * Call after any event that may have changed GPIO pin assignments. */
void pio_invalidate_all_joints(void);

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

/* Set the HIGH-phase loop iteration count for a joint's step pulse.
 * count is clamped to 7 bits (0–127); default is STEP_PIO_HIGH_COUNT_DEFAULT.
 * Takes effect on the next FIFO word (next servo period). */
void pio_set_step_high_count(uint32_t joint, uint32_t count);

/* Clear init_done for all joints so init_pio() re-runs on next enable.
 * Call after any event that may have changed GPIO pin assignments. */
void pio_invalidate_all_joints(void);

/* Set the STEP pulse HIGH-phase duration in ns.
 * Converts ns to a high_count value (rounds up to guarantee >= requested duration).
 * ns=0 resets to the firmware default (~2500ns, STEP_PIO_HIGH_COUNT_DEFAULT).
 * Clamped to 7-bit range; takes effect on the next servo period. */
void pio_set_step_pulse_ns(uint32_t joint, uint16_t ns);

/* Return the commanded STEP pulse HIGH-phase duration in ns for a joint.
 * Returns 0 if using the firmware default (no explicit command received). */
uint16_t pio_get_step_pulse_ns(uint32_t joint);

/* Return and clear the per-joint DIR setup violation bitmask.
 * Bit N is set when joint N had a direction change with step_low_half <
 * DIR_SETUP_MIN_CYCLES in the most recent servo period(s).
 * Clears on read (momentary semantics). */
uint8_t pio_get_and_clear_dir_setup_violations(void);

int32_t clamp_accel(int32_t velocity_q, int32_t last_velocity_q, int32_t max_accel_q);

#ifdef BUILD_TESTS
/* Reset all static state in pio.c — used by test setup fixtures only. */
void pio_reset_for_test(void);

/* Exposed for unit testing only. */
int32_t drain_rx_fifo(uint32_t sm, int32_t current_pos);
#endif  // BUILD_TESTS

#endif  // PIO__H
