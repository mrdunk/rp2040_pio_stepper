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
void init_pio(
    const uint32_t axis,
    const uint32_t pin_step,
    const uint32_t pin_direction
    )
{
  static uint32_t offset_pio0 = 0;
  static uint32_t offset_pio1 = 0;
  static uint8_t init_done = 0;

  if(init_done == 0)
  {
    offset_pio0 = pio_add_program(pio0, &step_gen_program);
    offset_pio1 = pio_add_program(pio1, &step_count_program);
    init_done = 1;
  }

  uint32_t sm;

  sm = pio_claim_unused_sm(pio0, true);
  // From pico_axs.pio
  step_gen_program_init(pio0, sm, offset_pio0, pin_step, pin_direction);
  pio_sm_set_enabled(pio0, sm, true);

  if(sm != axis) {
    printf("ERROR: Incorrect PIO initialization order for pio0. axis: %u  sm: %u", axis, sm);
  }

  sm = pio_claim_unused_sm(pio1, true);
  // From pico_axs.pio
  step_count_program_init(pio1, sm, offset_pio1, pin_step, pin_direction);
  pio_sm_set_enabled(pio1, sm, true);

  if(sm != axis) {
    printf("ERROR: Incorrect PIO initialization order for pio1. axis: %u  sm: %u", axis, sm);
  }

  // Initial value for counter.
  // Puts the start position in the middle of the possible range.
  pio_sm_put(pio1, axis, UINT_MAX / 2);
}

/* Convert step command from LinuxCNC and Feedback from PIO into a desired velocity. */
uint32_t get_velocity(
    const uint8_t axis,
    uint32_t abs_pos_acheived,
    uint32_t abs_pos_requested,
    float kp)
{
  //static float last_error[MAX_AXIS] = {0};
  int32_t error = abs_pos_requested - abs_pos_acheived;
  int32_t velocity = kp * (float)error;
  //last_error[axis] *= (7.0 / 8.0);
  //last_error[axis] += (float)error * (1.0 / 8.0);

  //int32_t velocity = kp * (float)last_error[axis];

  return velocity;
}

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t axis, const uint32_t update_time_us) {
  static uint32_t failcount = 0;
  static uint32_t count = 0;
  static uint32_t last_pos[MAX_AXIS] = {0};

  //uint32_t clock_multiplier = clock_get_hz(clk_sys) / 1000000;
  static const uint32_t clock_multiplier = 133;
  //static uint32_t time_sent[MAX_AXIS] = {0};
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
      &abs_pos_requested,
      &abs_pos_acheived,
      &min_step_len_ticks,
      &max_accel_ticks,
      &velocity_requested,
      &velocity_acheived,
      &kp
      );

  if(updated <= 0) {
    return 0;
  }
  count++;
  if(updated > 1) {
    failcount++;
    printf("WARN: C1, multiple updates: %u \t%lu \t%f\n\n",
        axis, updated, (double)failcount / (double)count);
  }

  // Stop from getting stuck continually reading FIFO when steps are short.
  uint32_t max_retries = 4; 

  while(pio_sm_get_rx_fifo_level(pio1, axis) > 0 && max_retries > 0) {
    abs_pos_acheived = pio_sm_get_blocking(pio1, axis);
    max_retries--;
  }
  //uint32_t time_received = time_us_64();
  //uint32_t abs_pos_acheived_normalized;
  //if(time_sent[axis] > 0) {
  //  uint32_t period_us = (time_received - time_sent[axis]);
  //  abs_pos_acheived_normalized = abs_pos_acheived * update_time_us / period_us;
  //}
  //time_sent[axis] = time_received;
  
  int32_t velocity;
  if(abs_pos_requested != 0) {
    velocity = get_velocity(axis, abs_pos_acheived, abs_pos_requested, kp);
  } else {
    velocity = velocity_requested;
  }

  uint8_t direction = (velocity > 0);
  uint32_t requested_step_count = abs(velocity);

  int32_t step_len_ticks = 0;
  
  if(requested_step_count > 0) {
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
      &abs_pos_acheived,
      NULL,
      NULL,
      &velocity,
      &velocity_acheived,
      NULL);

  return 1;
}


