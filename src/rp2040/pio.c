#include <stdio.h>
#include <math.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "pico/stdlib.h"
#include "pico_stepper.pio.h"

#include "pio.h"
#include "config.h"

#define STEP_PIO_LEN_OVERHEAD 9.0
#define STEP_PIO_MULTIPLIER 2.0


/* Initialize a pair of PIO programmes.
 * One for step generation on pio0 and one for counting said steps on pio1.
 */
static uint32_t sm0[MAX_JOINT];
static uint32_t sm1[MAX_JOINT];

void init_pio(const uint32_t joint)
{
  static bool init_done[MAX_JOINT] = {false, false, false, false};
  static uint32_t offset_pio0 = 0;
  static uint32_t offset_pio1 = 0;
  static uint8_t programs_loaded = 0;

  if(init_done[joint]) {
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
      sm0[a] = pio_claim_unused_sm(pio0, true);
      sm1[a] = pio_claim_unused_sm(pio1, true);
    }

    programs_loaded = 1;
  }

  // The stepping PIO program.
  pio_sm_set_enabled(pio0, sm0[joint], false);
  // From pico_axs.pio
  step_gen_program_init(pio0, sm0[joint], offset_pio0, io_pos_step, io_pos_dir);
  pio_sm_set_enabled(pio0, sm0[joint], true);

  if(sm0[joint] != joint) {
    printf("ERROR: Incorrect PIO initialization order for pio0. joint: %u  sm0[joint]: %u",
        joint, sm0[joint]);
  }

  // The counting PIO program.
  pio_sm_set_enabled(pio1, sm1[joint], false);
  // From pico_axs.pio
  step_count_program_init(pio1, sm1[joint], offset_pio1, io_pos_step, io_pos_dir);
  pio_sm_set_enabled(pio1, sm1[joint], true);

  if(sm1[joint] != joint) {
    printf("ERROR: Incorrect PIO initialization order for pio1. joint: %u  sm1[joint]: %u",
        joint, sm1[joint]);
  }

  init_done[joint] = true;
}

// These POSITION_BIAS and VELOCITY_BIAS should add up to 1.0 or slightly less. eg: 0.95
// The slight delay smooths output. More delay = smoother but more latency.
#define POSITION_BIAS 0.1
#define VELOCITY_BIAS 0.85
#define MIN_STEP_COUNT 0.0625   // 1/16

/* Convert step command from LinuxCNC and Feedback from PIO into a desired velocity. */
double get_velocity(
    const uint32_t update_period_us,
    const uint8_t joint,
    const int32_t abs_pos_achieved,
    const double abs_pos_requested,
    const double expected_velocity)
{
  static int32_t holdoff[MAX_JOINT] = {0, 0, 0, 0};

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

  // Short steps that span multiple cycles are inclined to "clump" together.
  // If a single step is calculated to take more than one cycle, don't allow
  // any steps in the following cycles.
  if(fabs(combined_vel) < 1.0) {
    if(holdoff[joint] > 0) {
      holdoff[joint]--;
      return 0.0;
    }
    holdoff[joint] = 0.75 / fabs(combined_vel);
  } else if(holdoff[joint] > 0) {
    holdoff[joint]--;
  }

  return combined_vel;
}

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t joint, const uint32_t update_period_us) {
  static uint32_t failcount = 0;
  static uint32_t count = 0;
  static int32_t last_pos_requested[MAX_JOINT] = {0, 0, 0, 0};
  static int32_t last_pos_achieved[MAX_JOINT] = {0, 0, 0, 0};
  static double last_velocity[MAX_JOINT] = {0, 0, 0, 0};
  static uint32_t last_enabled[MAX_JOINT] = {0, 0, 0, 0};

  //uint32_t clock_multiplier = clock_get_hz(clk_sys) / 1000000;
  static const uint32_t clock_multiplier = 133;

  static size_t dir_change_count[MAX_JOINT] = {0, 0, 0, 0};
  static uint32_t last_direction[MAX_JOINT] = {0, 0, 0, 0};

  uint8_t enabled;
  int32_t abs_pos_achieved = 0;
  double velocity_requested;
  double abs_pos_requested;
  double max_velocity;
  double max_accel;
  int32_t velocity_achieved = 0;
  uint32_t updated;

  if(update_period_us == 0) {
    // Switch off PIO stepgen.
    if(!pio_sm_is_tx_fifo_full(pio0, sm0[joint])) {
      pio_sm_put(pio0, sm0[joint], 0);
    }
    return 0;
  }

  updated = get_joint_config(
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

  count++;

  if(updated > 2) {
    if(enabled && last_enabled[joint]) {
      failcount++;
      printf("WC1: multi update: %u \t%lu \t%f\n",
          joint, updated, (double)failcount / (double)count);
    }
  }

  if(enabled != last_enabled[joint]) {
    last_enabled[joint] = enabled;
    if(enabled) {
      printf("Joint %u was enabled.\n", joint);
      init_pio(joint);
    } else {
      printf("Joint %u was disabled.\n", joint);
    }
  }

  if(!enabled) {
    // Switch off PIO stepgen.
    if(pio_sm_is_tx_fifo_empty(pio0, sm0[joint])) {
      pio_sm_put(pio0, sm0[joint], 0);
    }
    return 0;
  }

  if(updated <= 0) {
    return 0;
  }

  // Drain rx_fifo of PIO feedback data and keep the last value received.
  uint8_t fifo_len = pio_sm_get_rx_fifo_level(pio1, sm1[joint]);
  while(fifo_len > 0) {
    abs_pos_achieved = pio_sm_get_blocking(pio1, sm1[joint]);
    fifo_len--;
  }

  double velocity = get_velocity(
      update_period_us,
      joint,
      abs_pos_achieved,
      abs_pos_requested,
      velocity_requested);

  max_velocity /= update_period_us;
  max_accel /= update_period_us;

  //double accel = velocity - last_velocity[joint];
  //if(accel > max_accel) {
  //  velocity = last_velocity[joint] + max_accel;
  //} else if(accel < (-max_accel)) {
  //  velocity = last_velocity[joint] - max_accel;
  //}

  double step_count = fabs(velocity);
  int32_t step_len_ticks = 0;
  double update_period_ticks = update_period_us * clock_multiplier;


  // The PIO FIFO will only report step counts between steps.
  // If too small a step_count is allowed here, the steps can get very long and
  // block further updates.
  if(step_count > MIN_STEP_COUNT) {
    step_len_ticks =
      (update_period_ticks / (step_count * STEP_PIO_MULTIPLIER)) - STEP_PIO_LEN_OVERHEAD;

    int32_t min_step_len_ticks =
      (update_period_ticks / (fabs(max_velocity) * STEP_PIO_MULTIPLIER)) - STEP_PIO_LEN_OVERHEAD;
    if(step_len_ticks < min_step_len_ticks) {
      step_len_ticks = min_step_len_ticks;
    }
  }

  uint32_t direction = (velocity > 0);

  // TODO: Remove this section once dead-zone calculation has been proven stable.
  if(direction != last_direction[joint]) {
    dir_change_count[joint]++;
    last_direction[joint] = direction;
  }
  if((count % 1000) < MAX_JOINT) {
    if(dir_change_count[joint] > 10) {
      printf("Excessive jitter. j: %u\tcount: %u\n", joint, dir_change_count[joint]);
    }
    dir_change_count[joint] = 0;
  }

  // Request steps from PIO.
  if(pio_sm_is_tx_fifo_empty(pio0, sm0[joint])) {
    pio_sm_put(pio0, sm0[joint], (step_len_ticks << 1) | direction);
  }

  velocity_achieved = abs_pos_achieved - last_pos_achieved[joint];
  int32_t velocity_requested_tm1 = velocity;
  int32_t position_error = abs_pos_achieved - last_pos_requested[joint];

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

  last_pos_requested[joint] = abs_pos_requested;
  last_pos_achieved[joint] = abs_pos_achieved;
  last_velocity[joint] = velocity;

  return updated;
}

