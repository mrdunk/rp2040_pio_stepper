#include <stdio.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "pico/stdlib.h"
#include "pico_stepper.pio.h"

#include "pio.h"
#include "config.h"

#define STEP_LEN_OVERHEAD 14

uint32_t divRoundClosest(const uint32_t n, const uint32_t d)
{
  return (n + d / 2) / d;
}

/* Initialize a pair of PIO programmes.
 * One for step generation on pio0 and one for counting said steps on pio1.
 */
void init_pio(const uint32_t axis)
{
  static uint32_t offset_pio0 = 0;
  static uint32_t offset_pio1 = 0;
  static uint32_t sm0[MAX_AXIS];
  static uint32_t sm1[MAX_AXIS];
  static uint8_t programs_loaded = 0;

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

  // Initial value for counter.
  // Puts the start position in the middle of the possible range.
  pio_sm_put(pio1, axis, UINT_MAX / 2);
}

/* Convert step command from LinuxCNC and Feedback from PIO into a desired velocity. */
int32_t get_velocity(
    const uint8_t axis,
    uint32_t abs_pos_acheived,
    uint32_t abs_pos_requested,
    float kp)
{
  int32_t error = abs_pos_requested - abs_pos_acheived;
  int32_t velocity = kp * (float)error;
  return velocity;
}

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t axis, const uint32_t update_time_us) {
  static uint32_t failcount = 0;
  static uint32_t count = 0;
  static uint32_t last_pos[MAX_AXIS] = {0, 0, 0, 0};
  static uint32_t last_enabled[MAX_AXIS] = {0, 0, 0, 0};

  //uint32_t clock_multiplier = clock_get_hz(clk_sys) / 1000000;
  static const uint32_t clock_multiplier = 133;
  uint8_t enabled;
  uint32_t abs_pos_acheived;
  uint32_t abs_pos_requested;
  uint32_t min_step_len_ticks;
  uint32_t max_accel_ticks;
  int32_t velocity_requested;
  int32_t velocity_acheived;
  float kp;
  uint32_t updated;

  updated = get_axis_config(
      axis,
      CORE1,
      &enabled,
      NULL,
      NULL,
      &abs_pos_requested,
      &abs_pos_acheived,
      &min_step_len_ticks,
      &max_accel_ticks,
      &velocity_requested,
      &velocity_acheived,
      &kp
      );

  if(updated <= 0) {
    //return 0;
  }

  count++;
  if(updated > 1) {
    if(enabled && last_enabled[axis]) {
      failcount++;
      //printf("WC1: multi update: %u \t%lu \t%f\n",
      //    axis, updated, (double)failcount / (double)count);
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

  uint8_t fifo_len = pio_sm_get_rx_fifo_level(pio1, axis);
  while(fifo_len) {
    abs_pos_acheived = pio_sm_get_blocking(pio1, axis);
    fifo_len--;
  }

  int32_t velocity;
  if(abs_pos_requested != 0) {
    velocity = get_velocity(axis, abs_pos_acheived, abs_pos_requested, kp);
  } else {
    velocity = velocity_requested;
  }

  uint8_t direction = (velocity > 0);
  uint32_t requested_step_count = abs(velocity);

  int32_t step_len_ticks = 0;

  if(enabled > 0 && requested_step_count > 0) {
    uint32_t utt = update_time_us * clock_multiplier;
    step_len_ticks =
      divRoundClosest(utt, (requested_step_count * 2)) - STEP_LEN_OVERHEAD;
    if(step_len_ticks < 1) {
      // TODO: use min_step_len_ticks for this.
      step_len_ticks = 1;
    }
  }

  // Request steps from PIO.
  pio_sm_put(pio0, axis, direction);
  pio_sm_put(pio0, axis, step_len_ticks);

  velocity_acheived = abs_pos_acheived - last_pos[axis];
  last_pos[axis] = abs_pos_acheived;

  update_axis_config(
      axis,
      CORE1,
      NULL,
      NULL,
      NULL,
      NULL,
      &abs_pos_acheived,
      NULL,
      NULL,
      &velocity,
      &velocity_acheived,
      NULL);

  /*
  if(axis == 1) {
    static uint32_t count = 0;
    static uint32_t then;
    uint32_t now = time_us_64();
    if(velocity > 4 || velocity < 0) {
      uint32_t buf_len = has_new_c1_data(0);
      printf("%i\t%u\t%u\n", velocity, now - then, buf_len);
    }
    then = now;
  }
  */
  
  return updated;
}


