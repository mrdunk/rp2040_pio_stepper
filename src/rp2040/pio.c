#include <stdio.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "pico/stdlib.h"
#include "pico_stepper.pio.h"

#include "pio.h"
#include "config.h"

#define STEP_LEN_OVERHEAD 14

uint32_t last_pos[MAX_AXIS] = {0};

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
  pio_sm_put(pio1, axis, UINT_MAX / 2);
}

/* A PID function for experimental tuning of step lengths. */
int32_t pid(
    const uint8_t axis,
    const uint32_t curent_pos,
    const uint32_t desired_pos,
    const float kp,
    const float ki,
    const float kd)
{
  static int32_t integral[MAX_AXIS] = {0};
  static int32_t last_error[MAX_AXIS] = {0};
  const int32_t max_velocity = 100;
  //printf("%f\t%f\t%f\n", kp, ki, kd);

  int32_t error = desired_pos - curent_pos;
  integral[axis] += error;
  int32_t derivative = error - last_error[axis];
  int32_t velocity = 
    (kp * (float)error) + (ki * (float)integral[axis]) + (kd * (float)derivative);
  if(velocity > max_velocity) {
    velocity = max_velocity;
  } else if(velocity < -max_velocity) {
    velocity = -max_velocity;
  }

  last_error[axis] = error;

  return velocity;
}

uint32_t get_velocity(
    uint32_t abs_pos_acheived, uint32_t abs_pos_requested, float kp, float ki, float kd) {
  //int32_t velocity = 
    //pid(axis, abs_pos_acheived, abs_pos_requested, kp, ki, kd); 
  
  int32_t velocity = 
    abs_pos_requested - abs_pos_acheived;

  //velocity = pid(axis, velocity_acheived, velocity, kp, ki, kd);

  return velocity;
}

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t axis, const uint32_t update_time_us) {
  static uint32_t failcount = 0;
  static uint32_t count = 0;
  static int32_t last_velocity[MAX_AXIS] = {0};

  //uint32_t clock_multiplier = clock_get_hz(clk_sys) / 1000000;
  static const uint32_t clock_multiplier = 133;
  uint32_t abs_pos_acheived;
  uint32_t abs_pos_requested;
  uint32_t min_step_len_ticks;
  uint32_t max_accel_ticks;
  int32_t velocity_acheived;
  float kp;
  float ki;
  float kd;
  uint32_t updated;

  updated = get_axis_config(
      axis,
      CORE1,
      &abs_pos_requested,
      &abs_pos_acheived,
      &min_step_len_ticks,
      &max_accel_ticks,
      &velocity_acheived,
      &kp,
      &ki,
      &kd
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


  uint32_t n = 5;
  while(pio_sm_get_rx_fifo_level(pio1, axis) > 0 && n > 0) {
    abs_pos_acheived = pio_sm_get_blocking(pio1, axis);
    //printf("%u\t%u\n", n, abs_pos_acheived);
    //printf("%u", axis);
    n--;
  }
  //printf("\n");
  

  int32_t velocity = get_velocity(abs_pos_acheived, abs_pos_requested, kp, ki, kd);

  uint8_t direction = (velocity > 0);
  uint32_t requested_step_count = abs(velocity);

  int32_t step_len_ticks = 0;
  
  if(requested_step_count > 0) {
    //requested = (actual - 14) / 2
    step_len_ticks = 
      (((update_time_us * clock_multiplier) / (requested_step_count * 2))) - STEP_LEN_OVERHEAD;
    if(step_len_ticks < 1) {
      // TODO: use min_step_len_ticks for this.
      step_len_ticks = 1;
    }
  }

  // Request steps.
  pio_sm_put(pio0, axis, direction);
  pio_sm_put(pio0, axis, step_len_ticks);
  //printf("pio0 axis: %u\ttx: %u\n", axis, pio_sm_get_tx_fifo_level(pio0, axis));

  velocity_acheived = abs_pos_acheived - last_pos[axis];
  last_pos[axis] = abs_pos_acheived; 

  update_axis_config(
      axis, CORE1, NULL, &abs_pos_acheived, NULL, NULL, &velocity_acheived, NULL, NULL, NULL);

  return 1;
}


