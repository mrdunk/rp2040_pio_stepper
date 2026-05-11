#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"
#include "../test/mocks/pio_mocks.h"

#else  // BUILD_TESTS

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico_stepper.pio.h"

#endif  // BUILD_TESTS

#include "pio.h"
#include "config.h"

/* PIO instruction cycles consumed by the state machine loop itself (derived
 * from pico_stepper.pio); subtracted when converting step period to PIO len. */
#define STEP_PIO_LEN_OVERHEAD  9
#define RP2040_CLOCK_MHZ       133

/* Remaining SMs after step_gen, capped at MAX_JOINT (can't count more joints
 * than we move). For MAX_JOINT=4: 4 feedback SMs (current behaviour).
 * For MAX_JOINT=8: 0 feedback SMs (open-loop). */
#if MAX_JOINT > 8
  #error "MAX_JOINT must be 1-8"
#endif
#define NUM_FEEDBACK  ((8 - MAX_JOINT) < MAX_JOINT ? (8 - MAX_JOINT) : MAX_JOINT)

/* Which PIO block and program offset a joint's step_gen SM lives on.
 * Joints 0-3 always use PIO0; joints 4-7 use PIO1 (MAX_JOINT > 4 only). */
#if MAX_JOINT > 4
  #define JOINT_PIO(j)        ((j) < 4 ? pio0 : pio1)
  #define JOINT_GEN_OFFSET(j) ((j) < 4 ? offset_pio0 : offset_pio1_gen)
#else
  #define JOINT_PIO(j)        pio0
  #define JOINT_GEN_OFFSET(j) offset_pio0
#endif

typedef struct {
    uint32_t sm_gen;    /* step_gen SM on JOINT_PIO(joint) */
    uint32_t sm_count;  /* step_count SM on PIO1; valid only for joint < NUM_FEEDBACK */
    bool     init_done;
    int32_t  last_pos_achieved;
    uint32_t last_enabled;
    int32_t  last_velocity_q;
    int32_t  step_accumulator_q;
} JointPioState;

static JointPioState joint_state[MAX_JOINT];
static uint32_t offset_pio0       = 0;  /* step_gen on PIO0 */
static uint32_t offset_pio1_gen   = 0;  /* step_gen on PIO1 (MAX_JOINT > 4 only) */
static uint32_t offset_pio1_count = 0;  /* step_count on PIO1 (NUM_FEEDBACK > 0 only) */
static uint8_t  programs_loaded   = 0;

void init_pio(const uint32_t joint)
{

  if(joint_state[joint].init_done) {
    return;
  }

  int8_t io_pos_step;
  int8_t io_pos_dir;
  get_joint_config(
      joint,
      CORE1,
      NULL,
      &io_pos_step,
      &io_pos_dir,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL
      );

  if(io_pos_step < 0 || io_pos_step >= 32) {
    printf("WARN: Joint %u step io pin is out of range: %i\n", joint, io_pos_step);
    return;
  }
  if(io_pos_dir < 0 || io_pos_dir >= 32) {
    printf("WARN: Joint %u dir io pin is out of range: %i\n", joint, io_pos_dir);
    return;
  }
  // TODO: Warn about duplicate pin assignments.

#ifdef VERBOSE_CONFIG_LOG
  printf("\tio-step: %i\tio-dir: %i\n", io_pos_step, io_pos_dir);
#endif
  gpio_init(io_pos_step);
  gpio_init(io_pos_dir);
  gpio_set_dir(io_pos_step, GPIO_OUT);
  gpio_set_dir(io_pos_dir, GPIO_OUT);
  gpio_put(io_pos_step, 0);
  gpio_put(io_pos_dir, 0);

  if(programs_loaded == 0)
  {
    offset_pio0 = pio_add_program(pio0, &step_gen_program);

#if MAX_JOINT > 4
    offset_pio1_gen = pio_add_program(pio1, &step_gen_program);
#endif
#if NUM_FEEDBACK > 0
    offset_pio1_count = pio_add_program(pio1, &step_count_program);
#endif

    /* Claim step_gen SMs: joints 0-3 on PIO0, joints 4+ on PIO1. */
    for(int8_t a = 0; a < MAX_JOINT && a < 4; a++) {
      joint_state[a].sm_gen = pio_claim_unused_sm(pio0, true);
    }
    for(int8_t a = 4; a < MAX_JOINT; a++) {
      joint_state[a].sm_gen = pio_claim_unused_sm(pio1, true);
    }

    /* Claim step_count SMs on PIO1 for the first NUM_FEEDBACK joints. */
    for(int8_t a = 0; a < NUM_FEEDBACK; a++) {
      joint_state[a].sm_count = pio_claim_unused_sm(pio1, true);
    }

    programs_loaded = 1;
  }

  /* Initialise the step_gen state machine for this joint. */
  pio_sm_set_enabled(JOINT_PIO(joint), joint_state[joint].sm_gen, false);
  step_gen_program_init(JOINT_PIO(joint), joint_state[joint].sm_gen,
                        JOINT_GEN_OFFSET(joint), io_pos_step, io_pos_dir);
  pio_sm_set_enabled(JOINT_PIO(joint), joint_state[joint].sm_gen, true);

  if(joint_state[joint].sm_gen != joint % 4) {
    printf("ERROR: Incorrect PIO initialization order for step_gen. joint: %u  sm_gen[joint]: %u",
        joint, joint_state[joint].sm_gen);
  }

  /* Initialise the step_count state machine for joints that have feedback. */
  if(joint < NUM_FEEDBACK) {
    pio_sm_set_enabled(pio1, joint_state[joint].sm_count, false);
    step_count_program_init(pio1, joint_state[joint].sm_count,
                            offset_pio1_count, io_pos_step, io_pos_dir);
    pio_sm_set_enabled(pio1, joint_state[joint].sm_count, true);

    /* step_count SMs are claimed after step_gen SMs on PIO1; expected index
     * is joint + number of step_gen SMs already on PIO1. */
    uint32_t expected_sm_count = joint + (MAX_JOINT > 4 ? MAX_JOINT - 4 : 0);
    if(joint_state[joint].sm_count != expected_sm_count) {
      printf("ERROR: Incorrect PIO init order for step_count. joint: %u  sm_count: %u  expected: %u",
          joint, joint_state[joint].sm_count, expected_sm_count);
    }
  }

  joint_state[joint].init_done = true;
}

/* Drain PIO1's RX FIFO and return the last feedback position received.
 * Returns current_pos unchanged if the FIFO is empty. */
int32_t drain_rx_fifo(uint32_t sm, int32_t current_pos) {
    uint8_t fifo_len = pio_sm_get_rx_fifo_level(pio1, sm);
    while (fifo_len > 0) {
        current_pos = pio_sm_get_blocking(pio1, sm);
        fifo_len--;
    }
    return current_pos;
}

/* Compute the PIO step-timer length in clock ticks.
 * Returns 0 only for step_count_q <= 0 (no motion).
 * Velocities below 1 step/period are capped at max_len so the step fits within
 * one servo period; plan_steps spaces them out via its accumulator.
 * max_vel_q <= 0 means "no max-velocity configured yet"; min_len clamping is
 * skipped so the default config does not block stepping before the first
 * MSG_SET_JOINT_CONFIG packet arrives. */
int32_t calculate_step_len(int32_t step_count_q, int32_t period_ticks, int32_t max_vel_q) {
    if (step_count_q <= 0) {
        return 0;
    }
    /* Longest step_len that keeps a step within one servo period.
     * A step > 1 period blocks the FIFO for the following Core1 tick. */
    int32_t max_len = period_ticks / 2 - STEP_PIO_LEN_OVERHEAD;
    /* Size step_len for ceil(v) steps per period so that max_steps = ceil(v).
     * The Bresenham accumulator can then alternate floor(v)/ceil(v) to achieve
     * exactly v steps/period on average, including non-integer velocities. */
    int32_t v_ceil = (step_count_q + 65535) >> 16;
    int32_t len    = period_ticks / (2 * v_ceil) - STEP_PIO_LEN_OVERHEAD;
    if (len > max_len) len = max_len;
    if (max_vel_q > 0) {
        int32_t min_len = (int32_t)((int64_t)period_ticks * 65536
                                    / ((int64_t)max_vel_q << 1) - STEP_PIO_LEN_OVERHEAD);
        int32_t clamped = len < min_len ? min_len : len;
        return clamped > max_len ? max_len : clamped;
    }
    return len;
}

/* Clamp velocity change to at most max_accel_q per period.
 * Returns velocity unchanged if max_accel_q <= 0 (no limiting). */
int32_t clamp_accel(int32_t velocity_q, int32_t last_velocity_q, int32_t max_accel_q) {
    if (max_accel_q <= 0) {
        return velocity_q;
    }
    int32_t delta = velocity_q - last_velocity_q;
    if (delta > max_accel_q) {
        return last_velocity_q + max_accel_q;
    }
    if (delta < -max_accel_q) {
        return last_velocity_q - max_accel_q;
    }
    return velocity_q;
}

/* Bresenham step scheduler.
 *
 * Accumulates fractional desired steps (velocity_q is Q16.16 steps/period) and
 * returns how many steps to issue this period.  Excess desired steps are
 * returned to the accumulator for the next period.
 *
 * max_steps = floor(period_ticks / step_period): the most complete steps the
 * PIO can produce in one period at the given step_len.  calculate_step_len
 * guarantees step_len <= period_ticks/2 - OVERHEAD, so max_steps >= 1
 * whenever step_len > 0.  step_len == 0 (no motion) gives max_steps = 0;
 * the accumulator is preserved so no desired steps are lost. */
int32_t plan_steps(int32_t velocity_q, uint8_t joint,
                   int32_t period_ticks, int32_t step_len) {
    joint_state[joint].step_accumulator_q += abs(velocity_q);
    int32_t n_steps_desired = joint_state[joint].step_accumulator_q >> 16;
    joint_state[joint].step_accumulator_q -= n_steps_desired << 16;

    int32_t step_period = 2 * (step_len + STEP_PIO_LEN_OVERHEAD);
    int32_t max_steps = (step_len > 0) ? period_ticks / step_period : 0;

    int32_t n_steps = n_steps_desired < max_steps ? n_steps_desired : max_steps;
    joint_state[joint].step_accumulator_q += (n_steps_desired - n_steps) << 16;

    return n_steps;
}

/* Write the packed step command to the joint's step_gen TX FIFO if it is empty.
 * Encoding: lower bit = direction, upper bits = half-period in ticks. */
static void issue_pio_step(uint32_t joint, int32_t step_len_ticks,
                           uint32_t direction) {
    if (pio_sm_is_tx_fifo_empty(JOINT_PIO(joint), joint_state[joint].sm_gen)) {
        pio_sm_put(JOINT_PIO(joint), joint_state[joint].sm_gen,
                   ((uint32_t)step_len_ticks << 1) | direction);
    }
}

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t joint) {
  uint32_t update_period_us = get_period();

  uint8_t enabled;
  int32_t abs_pos_achieved = 0;
  double velocity_requested;
  double abs_pos_requested;
  double max_velocity;
  double max_accel;
  uint8_t cmd_type;
  int32_t velocity_achieved = 0;
  uint32_t updated = get_joint_config(
      joint,
      CORE1,
      &enabled,
      NULL,
      NULL,
      &velocity_requested,
      &abs_pos_requested,
      &abs_pos_achieved,
      &max_velocity,
      &max_accel,
      NULL,  // &velocity_achieved
      &cmd_type
      );

  if(updated == 0 || update_period_us == 0) {
    if (pio_sm_is_tx_fifo_empty(JOINT_PIO(joint), joint_state[joint].sm_gen)) {
      pio_sm_put(JOINT_PIO(joint), joint_state[joint].sm_gen, 0);
    }
    return 0;
  }

  if(cmd_type == JOINT_CMD_POSITION) {
    /* Velocity feed-forward + Kp=0.5 position correction.
     * velocity_requested holds scale×vel_cmd (steps/s) from the driver;
     * using it as feed-forward cancels the trajectory velocity, keeping
     * following error near zero during motion.
     * With 1-period measurement delay, Kp=1.0 gives poles on the unit
     * circle (sustained oscillation). Kp=0.5 places poles at z=0.5±0.5i
     * (|z|=0.707), giving stable convergence in ~10 periods.
     *
     * Dead zone: suppress correction when |error| < 1 step.  Sub-step
     * errors cannot be resolved without oscillation — a correction step
     * moves abs_pos_achieved by 1, which reverses the error sign and
     * fires an opposite correction next cycle.  Errors up to ±0.5 step
     * are inherent in mapping a float trajectory onto integer steps. */
    double vel_ff      = velocity_requested;
    double error_steps = abs_pos_requested - (double)abs_pos_achieved;
    double correction  = 0.0;
    if (error_steps >= 1.0 || error_steps <= -1.0) {
        correction = error_steps * (1.0e6 / (double)update_period_us) * 0.5;
    }
    velocity_requested = vel_ff + correction;
  }

  int32_t velocity_q   = (int32_t)((velocity_requested / (double)update_period_us) * 65536.0);
  int32_t max_vel_q    = (int32_t)((max_velocity / (double)update_period_us) * 65536.0);
  int32_t max_accel_q  = (int32_t)((max_accel / (double)update_period_us) * 65536.0);
  int32_t period_ticks = (int32_t)((int64_t)update_period_us * RP2040_CLOCK_MHZ);

  if(enabled != joint_state[joint].last_enabled) {
    joint_state[joint].last_enabled = enabled;
    if(enabled) {
      printf("J%u enab\n", joint);
      init_pio(joint);
      // Snap to commanded velocity so we don't ramp from zero when LinuxCNC
      // is already moving (joint was enabled before motion started).
      joint_state[joint].last_velocity_q = velocity_q;
    } else {
      printf("J%u disab\n", joint);
    }
  }

  if(!enabled) {
    // Switch off PIO stepgen.
    if(pio_sm_is_tx_fifo_empty(JOINT_PIO(joint), joint_state[joint].sm_gen)) {
      pio_sm_put(JOINT_PIO(joint), joint_state[joint].sm_gen, 0);
    }
    // Keep abs_pos_achieved current while disabled so pos_fb stays accurate.
    // Without this, in-flight steps accumulate in the PIO FIFO unread; on
    // re-enable they all land at once, creating a position error that triggers
    // a correction move (jitter).
    if(joint < NUM_FEEDBACK) {
      abs_pos_achieved = drain_rx_fifo(joint_state[joint].sm_count, abs_pos_achieved);
      velocity_achieved = abs_pos_achieved - joint_state[joint].last_pos_achieved;
    } else {
      velocity_achieved = 0;
    }
    update_joint_config(
        joint, CORE1,
        NULL, NULL, NULL, NULL, NULL,
        &abs_pos_achieved,
        NULL, NULL,
        &velocity_achieved,
        NULL);
    joint_state[joint].last_pos_achieved = abs_pos_achieved;
    return 0;
  }

  if(joint < NUM_FEEDBACK) {
    abs_pos_achieved = drain_rx_fifo(joint_state[joint].sm_count, abs_pos_achieved);
  }

  velocity_q = clamp_accel(velocity_q, joint_state[joint].last_velocity_q, max_accel_q);
  joint_state[joint].last_velocity_q = velocity_q;

  int32_t step_count_q  = abs(velocity_q);
  int32_t step_len_ceil = calculate_step_len(step_count_q, period_ticks, max_vel_q);
  int32_t n_steps       = plan_steps(velocity_q, joint, period_ticks, step_len_ceil);
  /* Derive step_len for the exact n_steps this period (floor or ceil of v),
   * so the PIO pulse rate matches the intended physical step count. */
  int32_t step_len_ticks = calculate_step_len(n_steps * 65536, period_ticks, max_vel_q);

  uint32_t direction = (velocity_q > 0);

  if(joint >= NUM_FEEDBACK) {
    abs_pos_achieved += (direction ? 1 : -1) * n_steps;
  }

  if (n_steps > 0) {
    issue_pio_step(joint, step_len_ticks, direction);
  } else if (pio_sm_is_tx_fifo_empty(JOINT_PIO(joint), joint_state[joint].sm_gen)) {
    pio_sm_put(JOINT_PIO(joint), joint_state[joint].sm_gen, 0);
  }

  velocity_achieved = abs_pos_achieved - joint_state[joint].last_pos_achieved;

  update_joint_config(
      joint,
      CORE1,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      &abs_pos_achieved,
      NULL,
      NULL,
      &velocity_achieved,
      NULL);

  joint_state[joint].last_pos_achieved  = abs_pos_achieved;

  return updated;
}

#ifdef BUILD_TESTS
void pio_reset_for_test(void) {
    memset(joint_state, 0, sizeof(joint_state));
    offset_pio0       = 0;
    offset_pio1_gen   = 0;
    offset_pio1_count = 0;
    programs_loaded   = 0;
}
#endif  // BUILD_TESTS
