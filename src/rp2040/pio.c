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
#define MIN_STEP_COUNT_Q       4096    /* 0.0625 * 65536 */


typedef struct {
    uint32_t sm0;
    uint32_t sm1;
    bool     init_done;
    int32_t  last_pos_achieved;
    uint32_t last_enabled;
    int32_t  last_velocity_q;
    int32_t  step_accumulator_q;
} JointPioState;

static JointPioState joint_state[MAX_JOINT];
static uint32_t offset_pio0     = 0;
static uint32_t offset_pio1     = 0;
static uint8_t  programs_loaded = 0;

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

  printf("\tio-step: %i\tio-dir: %i\n", io_pos_step, io_pos_dir);
  gpio_init(io_pos_step);
  gpio_init(io_pos_dir);
  gpio_set_dir(io_pos_step, GPIO_OUT);
  gpio_set_dir(io_pos_dir, GPIO_OUT);
  gpio_put(io_pos_step, 0);
  gpio_put(io_pos_dir, 0);


  if(programs_loaded == 0)
  {
    offset_pio0 = pio_add_program(pio0, &step_gen_program);
    offset_pio1 = pio_add_program(pio1, &step_count_program);

    for(int8_t a = 0; a < MAX_JOINT; a++) {
      joint_state[a].sm0 = pio_claim_unused_sm(pio0, true);
      joint_state[a].sm1 = pio_claim_unused_sm(pio1, true);
    }

    programs_loaded = 1;
  }

  // The stepping PIO program.
  pio_sm_set_enabled(pio0, joint_state[joint].sm0, false);
  // From pico_axs.pio
  step_gen_program_init(pio0, joint_state[joint].sm0, offset_pio0, io_pos_step, io_pos_dir);
  pio_sm_set_enabled(pio0, joint_state[joint].sm0, true);

  if(joint_state[joint].sm0 != joint) {
    printf("ERROR: Incorrect PIO initialization order for pio0. joint: %u  sm0[joint]: %u",
        joint, joint_state[joint].sm0);
  }

  // The counting PIO program.
  pio_sm_set_enabled(pio1, joint_state[joint].sm1, false);
  // From pico_axs.pio
  step_count_program_init(pio1, joint_state[joint].sm1, offset_pio1, io_pos_step, io_pos_dir);
  pio_sm_set_enabled(pio1, joint_state[joint].sm1, true);

  if(joint_state[joint].sm1 != joint) {
    printf("ERROR: Incorrect PIO initialization order for pio1. joint: %u  sm1[joint]: %u",
        joint, joint_state[joint].sm1);
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
 * Returns 0 if step_count_q is below MIN_STEP_COUNT_Q or the step would span
 * >= 2 servo periods (>= 2ms at 1ms period); caller must treat 0 as "skip". */
int32_t calculate_step_len(int32_t step_count_q, int32_t period_ticks, int32_t max_vel_q) {
    if (step_count_q <= MIN_STEP_COUNT_Q) {
        return 0;
    }
    int32_t len = (int32_t)((int64_t)period_ticks * 65536
                            / ((int64_t)step_count_q << 1) - STEP_PIO_LEN_OVERHEAD);
    int32_t min_len = (int32_t)((int64_t)period_ticks * 65536
                                / ((int64_t)max_vel_q << 1) - STEP_PIO_LEN_OVERHEAD);
    int32_t clamped = len < min_len ? min_len : len;
    /* A step lasting >= 2 servo periods would block the FIFO for two Core1 ticks.
     * Skip it; the accumulator preserves the fractional step for the next period. */
    return clamped >= period_ticks ? 0 : clamped;
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

int32_t plan_steps(int32_t velocity_q, uint8_t joint,
                   int32_t period_ticks, int32_t step_len) {
    joint_state[joint].step_accumulator_q += abs(velocity_q);
    int32_t n_steps_desired = joint_state[joint].step_accumulator_q >> 16;
    joint_state[joint].step_accumulator_q -= n_steps_desired << 16;

    /* step_len == 0 means "skip this period" — preserve accumulator, issue no steps. */
    int32_t step_period = 2 * (step_len + STEP_PIO_LEN_OVERHEAD);
    int32_t max_steps = (step_len > 0) ? period_ticks / step_period : 0;
    if (max_steps < 0) max_steps = 0;

    int32_t n_steps = n_steps_desired < max_steps ? n_steps_desired : max_steps;
    joint_state[joint].step_accumulator_q += (n_steps_desired - n_steps) << 16;

    return n_steps;
}

/* Write the packed step command to PIO0's TX FIFO if it is empty.
 * Encoding: lower bit = direction, upper bits = half-period in ticks. */
static void issue_pio_step(uint32_t joint, int32_t step_len_ticks,
                           uint32_t direction) {
    if (pio_sm_is_tx_fifo_empty(pio0, joint_state[joint].sm0)) {
        pio_sm_put(pio0, joint_state[joint].sm0, ((uint32_t)step_len_ticks << 1) | direction);
    }
}

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t joint) {
  uint32_t update_period_us = get_period();

  uint8_t enabled;
  int32_t abs_pos_achieved = 0;
  double velocity_requested;
  double max_velocity;
  double max_accel;
  int32_t velocity_achieved = 0;
  uint32_t updated = get_joint_config(
      joint,
      CORE1,
      &enabled,
      NULL,
      NULL,
      &velocity_requested,
      NULL,
      &abs_pos_achieved,
      &max_velocity,
      &max_accel,
      NULL  // &velocity_achieved
      );

  if(updated == 0 || update_period_us == 0) {
    if (pio_sm_is_tx_fifo_empty(pio0, joint_state[joint].sm0)) {
      pio_sm_put(pio0, joint_state[joint].sm0, 0);
    }
    return 0;
  }

  int32_t velocity_q   = (int32_t)((velocity_requested / (double)update_period_us) * 65536.0);
  int32_t max_vel_q    = (int32_t)((max_velocity / (double)update_period_us) * 65536.0);
  int32_t max_accel_q  = (int32_t)((max_accel / (double)update_period_us) * 65536.0);
  int32_t period_ticks = (int32_t)((int64_t)update_period_us * RP2040_CLOCK_MHZ);

  if(enabled != joint_state[joint].last_enabled) {
    joint_state[joint].last_enabled = enabled;
    if(enabled) {
      printf("Joint %u was enabled.\n", joint);
      init_pio(joint);
      // Snap to commanded velocity so we don't ramp from zero when LinuxCNC
      // is already moving (joint was enabled before motion started).
      joint_state[joint].last_velocity_q = velocity_q;
    } else {
      printf("Joint %u was disabled.\n", joint);
    }
  }

  if(!enabled) {
    // Switch off PIO stepgen.
    if(pio_sm_is_tx_fifo_empty(pio0, joint_state[joint].sm0)) {
      pio_sm_put(pio0, joint_state[joint].sm0, 0);
    }
    return 0;
  }

  abs_pos_achieved = drain_rx_fifo(joint_state[joint].sm1, abs_pos_achieved);

  velocity_q = clamp_accel(velocity_q, joint_state[joint].last_velocity_q, max_accel_q);
  joint_state[joint].last_velocity_q = velocity_q;

  int32_t step_count_q   = abs(velocity_q);
  int32_t step_len_ticks = calculate_step_len(step_count_q, period_ticks, max_vel_q);
  int32_t n_steps        = plan_steps(velocity_q, joint, period_ticks, step_len_ticks);

  uint32_t direction = (velocity_q > 0);

  if (n_steps > 0) {
    issue_pio_step(joint, step_len_ticks, direction);
  } else if (pio_sm_is_tx_fifo_empty(pio0, joint_state[joint].sm0)) {
    pio_sm_put(pio0, joint_state[joint].sm0, 0);
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
      &velocity_achieved);

  joint_state[joint].last_pos_achieved  = abs_pos_achieved;

  return updated;
}

#ifdef BUILD_TESTS
void pio_reset_for_test(void) {
    memset(joint_state, 0, sizeof(joint_state));
    offset_pio0     = 0;
    offset_pio1     = 0;
    programs_loaded = 0;
}
#endif  // BUILD_TESTS
