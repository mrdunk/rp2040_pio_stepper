#include <stdio.h>
#include <math.h>
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

#define STEP_PIO_LEN_OVERHEAD 9.0
#define STEP_PIO_MULTIPLIER 2.0
#define RP2040_CLOCK_MHZ 133


typedef struct {
    uint32_t sm0;
    uint32_t sm1;
    bool     init_done;
    int32_t  last_pos_requested;
    int32_t  last_pos_achieved;
    uint32_t last_enabled;
    double   last_velocity;
    double   step_accumulator;
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

// These POSITION_BIAS and VELOCITY_BIAS should add up to 1.0 or slightly less. eg: 0.95
// The slight delay smooths output. More delay = smoother but more latency.
#define POSITION_BIAS 0.1
#define VELOCITY_BIAS 0.85
#define MIN_STEP_COUNT 0.0625   // 1/16
#define STOP_THRESHOLD  1.0     /* steps/period: stop PIO on underrun below this velocity */

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

/* Convert step command from LinuxCNC and Feedback from PIO into a desired velocity. */
double get_velocity(
    const uint32_t update_period_us,
    const uint8_t joint,
    const int32_t abs_pos_achieved,
    const double abs_pos_requested,
    const double expected_velocity)
{
  double position_diff = (abs_pos_requested - (double)abs_pos_achieved);
  double velocity = (expected_velocity / (double)update_period_us);
  double combined_vel = position_diff * POSITION_BIAS + velocity * VELOCITY_BIAS;

  // Skip very slow speeds.
  // Try again next cycle.
  if(fabs(position_diff) < 0.001) {
    return 0.0;
  }

  // Different opinions on which direction to turn. Implies low speed jitter.
  // Try again next cycle.
  if((position_diff >= 0.0 && velocity <= 0.0) || (position_diff <= 0.0 && velocity >= 0.0)) {
    return 0.0;
  }

  return combined_vel;
}

/* Compute the PIO step-timer length in clock ticks.
 * Returns 0 if step_count is below MIN_STEP_COUNT (too slow to drive PIO). */
int32_t calculate_step_len(double step_count, double update_period_ticks,
                           double max_velocity) {
    if (step_count <= MIN_STEP_COUNT) {
        return 0;
    }
    int32_t len = (int32_t)((update_period_ticks / (step_count * STEP_PIO_MULTIPLIER))
                            - STEP_PIO_LEN_OVERHEAD);
    int32_t min_len = (int32_t)((update_period_ticks / (fabs(max_velocity) * STEP_PIO_MULTIPLIER))
                                - STEP_PIO_LEN_OVERHEAD);
    int32_t max_len = (int32_t)(update_period_ticks / STEP_PIO_MULTIPLIER
                                - STEP_PIO_LEN_OVERHEAD);
    int32_t clamped = len < min_len ? min_len : len;
    return clamped > max_len ? max_len : clamped;
}

/* Clamp velocity change to at most max_accel per period.
 * Returns velocity unchanged if max_accel <= 0 (no limiting). */
double clamp_accel(double velocity, double last_velocity, double max_accel) {
    if (max_accel <= 0.0) {
        return velocity;
    }
    double delta = velocity - last_velocity;
    if (delta > max_accel) {
        return last_velocity + max_accel;
    }
    if (delta < -max_accel) {
        return last_velocity - max_accel;
    }
    return velocity;
}

int32_t plan_steps(double velocity, uint8_t joint,
                   double period_ticks, int32_t step_len) {
    joint_state[joint].step_accumulator += fabs(velocity);
    int32_t n_steps_desired = (int32_t)floor(joint_state[joint].step_accumulator);
    joint_state[joint].step_accumulator -= (double)n_steps_desired;

    int32_t step_period = (int32_t)(2.0 * (step_len + STEP_PIO_LEN_OVERHEAD));
    int32_t max_steps = (step_period > 0)
        ? (int32_t)(period_ticks / (double)step_period)
        : 0;
    if (max_steps < 0) max_steps = 0;

    int32_t n_steps = n_steps_desired < max_steps ? n_steps_desired : max_steps;
    joint_state[joint].step_accumulator += (double)(n_steps_desired - n_steps);

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
  double abs_pos_requested;
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
      &abs_pos_requested,
      &abs_pos_achieved,
      &max_velocity,
      &max_accel,
      NULL, // &velocity_requested_tm1,
      NULL, // &velocity_achieved,
      NULL, // &step_len_ticks,
      NULL  // &position_error
      );

  if(updated == 0 || update_period_us == 0) {
    if (fabs(joint_state[joint].last_velocity) < STOP_THRESHOLD &&
        pio_sm_is_tx_fifo_empty(pio0, joint_state[joint].sm0)) {
      pio_sm_put(pio0, joint_state[joint].sm0, 0);
    }
    return 0;
  }

  if(enabled != joint_state[joint].last_enabled) {
    joint_state[joint].last_enabled = enabled;
    if(enabled) {
      printf("Joint %u was enabled.\n", joint);
      init_pio(joint);
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

  double velocity = get_velocity(
      update_period_us,
      joint,
      abs_pos_achieved,
      abs_pos_requested,
      velocity_requested);

  max_velocity /= update_period_us;
  max_accel /= update_period_us;
  velocity = clamp_accel(velocity, joint_state[joint].last_velocity, max_accel);
  joint_state[joint].last_velocity = velocity;

  double step_count = fabs(velocity);
  double update_period_ticks = update_period_us * RP2040_CLOCK_MHZ;

  int32_t step_len_ticks = calculate_step_len(step_count, update_period_ticks, max_velocity);
  int32_t n_steps = plan_steps(velocity, joint, update_period_ticks, step_len_ticks);

  uint32_t direction = (velocity > 0);

  if (n_steps > 0) {
    issue_pio_step(joint, step_len_ticks, direction);
  } else if (pio_sm_is_tx_fifo_empty(pio0, joint_state[joint].sm0)) {
    pio_sm_put(pio0, joint_state[joint].sm0, 0);
  }

  velocity_achieved = abs_pos_achieved - joint_state[joint].last_pos_achieved;
  int32_t velocity_requested_tm1 = (int32_t)velocity;
  int32_t position_error = abs_pos_achieved - joint_state[joint].last_pos_requested;

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
      &velocity_requested_tm1,
      &velocity_achieved,
      &step_len_ticks,
      &position_error);

  joint_state[joint].last_pos_requested = abs_pos_requested;
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
