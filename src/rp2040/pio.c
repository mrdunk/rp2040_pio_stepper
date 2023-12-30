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
#define DEAD_ZONE_THRESHOLD 1  // steps / ms

/* Initialize a pair of PIO programmes.
 * One for step generation on pio0 and one for counting said steps on pio1.
 */
static uint32_t sm0[MAX_AXIS];
static uint32_t sm1[MAX_AXIS];

void init_pio(const uint32_t axis)
{
  static bool init_done[MAX_AXIS] = {false, false, false, false};
  static uint32_t offset_pio0 = 0;
  static uint32_t offset_pio1 = 0;
  static uint8_t programs_loaded = 0;

  if(init_done[axis]) {
    return;
  }

  int8_t io_pos_step;
  int8_t io_pos_dir;
  get_axis_config(
      axis,
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
      NULL,
      NULL
      );

  if(io_pos_step < 0 || io_pos_step >= 32) {
    printf("WARN: Axis %u step io pin is out of range: %i\n", axis, io_pos_step);
    return;
  }
  if(io_pos_dir < 0 || io_pos_dir >= 32) {
    printf("WARN: Axis %u dir io pin is out of range: %i\n", axis, io_pos_dir);
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

    for(int8_t a = 0; a < MAX_AXIS; a++) {
      sm0[a] = pio_claim_unused_sm(pio0, true);
      sm1[a] = pio_claim_unused_sm(pio1, true);
    }

    programs_loaded = 1;
  }

  // The stepping PIO program.
  pio_sm_set_enabled(pio0, sm0[axis], false);
  // From pico_axs.pio
  step_gen_program_init(pio0, sm0[axis], offset_pio0, io_pos_step, io_pos_dir);
  pio_sm_set_enabled(pio0, sm0[axis], true);

  if(sm0[axis] != axis) {
    printf("ERROR: Incorrect PIO initialization order for pio0. axis: %u  sm0[axis]: %u",
        axis, sm0[axis]);
  }

  // The counting PIO program.
  pio_sm_set_enabled(pio1, sm1[axis], false);
  // From pico_axs.pio
  step_count_program_init(pio1, sm1[axis], offset_pio1, io_pos_step, io_pos_dir);
  pio_sm_set_enabled(pio1, sm1[axis], true);

  if(sm1[axis] != axis) {
    printf("ERROR: Incorrect PIO initialization order for pio1. axis: %u  sm1[axis]: %u",
        axis, sm1[axis]);
  }

  init_done[axis] = true;
}

/* Convert step command from LinuxCNC and Feedback from PIO into a desired velocity. */
double get_velocity(
    const uint32_t update_period_us,
    const uint8_t axis,
    const int32_t abs_pos_acheived,
    const double abs_pos_requested,
    const double expected_velocity)
{
  double position_error = (abs_pos_requested - (double)abs_pos_acheived);
  double velocity = (expected_velocity / (double)update_period_us);

  // Note that the total of these 2 velocities add up to lass than 1.
  // This performs like the Proportional stage of a PID controller.
  double combined_velocity = position_error * 0.1 + velocity * 0.8;
  
  if(abs(velocity) <= DEAD_ZONE_THRESHOLD) {  // steps / ms
    // Deadzone. Minimize movement to prevent oscillating around zero.
    if((position_error > 0.0 && velocity < 0.0) || (position_error < 0.0 && velocity > 0.0)) {
      // Position and velocity disagree.
      // Do not do steps.
      return 0;
    }
    return velocity;
  }

  return combined_velocity;
}

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t axis, const uint32_t update_period_us) {
  static uint32_t failcount = 0;
  static uint32_t count = 0;
  static int32_t last_pos[MAX_AXIS] = {0, 0, 0, 0};
  static uint32_t last_enabled[MAX_AXIS] = {0, 0, 0, 0};
  static double step_count[MAX_AXIS] = {0.0, 0.0, 0.0, 0.0};
  static uint8_t last_direction[MAX_AXIS] = {0, 0, 0, 0};

  //uint32_t clock_multiplier = clock_get_hz(clk_sys) / 1000000;
  static const uint32_t clock_multiplier = 133;
  uint8_t enabled;
  int32_t abs_pos_acheived = 0;
  double rel_pos_requested;
  double abs_pos_requested;
  double max_velocity;
  double max_accel_ticks;
  int32_t velocity_acheived = 0;
  uint32_t updated;

  updated = get_axis_config(
      axis,
      CORE1,
      &enabled,
      NULL,
      NULL,
      &rel_pos_requested,
      &abs_pos_requested,
      &abs_pos_acheived,
      &max_velocity,
      &max_accel_ticks,
      NULL, // &velocity_requested,
      NULL, // &velocity_acheived,
      NULL, // &step_len_ticks,
      NULL, // &pos_error,
      NULL  // &kp
      );

  if(updated <= 0) {
    return 0;
  }

  count++;

  if(updated > 2) {
    if(enabled && last_enabled[axis]) {
      failcount++;
      printf("WC1: multi update: %u \t%lu \t%f\n",
          axis, updated, (double)failcount / (double)count);
    }
  }

  if(enabled != last_enabled[axis]) {
    if(enabled) {
      printf("Axis %u was enabled.\n", axis);
      init_pio(axis);
    } else {
      printf("Axis %u was disabled.\n", axis);
    }
    last_enabled[axis] = enabled;
  }

  // Drain rx_fifo of PIO feedback data and keep the last value received.
  uint8_t fifo_len = pio_sm_get_rx_fifo_level(pio1, sm1[axis]);
  while(fifo_len > 0) {
    abs_pos_acheived = pio_sm_get_blocking(pio1, sm1[axis]);
    fifo_len--;
  }

  double velocity = get_velocity(
      update_period_us,
      axis,
      abs_pos_acheived,
      abs_pos_requested,
      rel_pos_requested);

  uint8_t direction = (velocity > 0);
  if(direction != last_direction[axis]) {
    // Direction has changed.
    step_count[axis] = 0;
    last_direction[axis] = direction;
  }

  step_count[axis] += fabs(velocity);
  int32_t step_len_ticks = 0;

  // The PIO only process new instructions between steps. If a step length gets too long,
  // it will block updates.
  // If too small a step_count is allowed here, the steps can get very long and block further updates.
  // If the limit is set too high, low speed resolution is lost and steps will become unevenly spaced.
  // 0.05 equates to a minimum speed of approximately 1 step every 20ms.
  if(enabled > 0 && step_count[axis] > 0.05) {
    double update_period_ticks = update_period_us * clock_multiplier;
    step_len_ticks =
      (update_period_ticks / (step_count[axis] * STEP_PIO_MULTIPLIER)) - STEP_PIO_LEN_OVERHEAD;
    if(step_len_ticks < 1) {
      // TODO: use min_step_len_ticks for this.
      step_len_ticks = 1;
    }
    step_count[axis] = 0.0;
  }

  // Request steps from PIO.
  if(pio_sm_is_tx_fifo_empty(pio0, sm0[axis]))
    pio_sm_put(pio0, sm0[axis], (step_len_ticks << 1) | direction);

  velocity_acheived = abs_pos_acheived - last_pos[axis];
  int32_t velocity_requested = velocity;

  // int32_t pos_error = abs_pos_requested - abs_pos_acheived;

  update_axis_config(
      axis,
      CORE1,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      &abs_pos_acheived,
      NULL,
      NULL,
      &velocity_requested,
      &velocity_acheived,
      NULL, //&pos_error,
      &step_len_ticks,
      NULL);

  last_pos[axis] = abs_pos_acheived;

  return updated;
}


