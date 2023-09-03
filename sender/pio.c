#include <stdio.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "pico/stdlib.h"
#include "pico_stepper.pio.h"

#include "pio.h"
#include "config.h"

void init_pio(
    const uint32_t stepper,
    const uint32_t pin_step,
    const uint32_t pin_direction
    )
{
  static uint32_t offset_pio0 = 0;
  static uint32_t offset_pio1 = 0;
  static uint8_t init_done = 0;

  if(init_done == 0)
  {
    offset_pio0 = pio_add_program(pio0, &step_repeated_program);
    offset_pio1 = pio_add_program(pio1, &step_repeated_program);
    init_done = 1;
  }

  uint32_t sm;
  switch (stepper) {
    case 0:
    case 1:
    case 2:
    case 3:
      sm = pio_claim_unused_sm(pio0, true);
      // From pico_stepper.pio
      step_repeated_program_init(pio0, sm, offset_pio0, pin_step);
      pio_sm_set_enabled(pio0, sm, true);
      break;
    case 4:
    case 5:
    case 6:
    case 7:
      sm = pio_claim_unused_sm(pio1, true);
      // From pico_stepper.pio
      step_repeated_program_init(pio1, sm, offset_pio1, pin_step);
      pio_sm_set_enabled(pio1, sm, true);
      break;
    default:
      printf("WARN: Invalid stepper index: %ld\n", stepper);
  }
}

void axis_to_pio(const uint32_t axis, PIO* pio, uint32_t* sm) {
  switch (axis) {
    case 0:
    case 1:
    case 2:
    case 3:
      *pio = pio0;
      *sm = axis;
      break;
    case 4:
    case 5:
    case 6:
    case 7:
      *pio = pio1;
      *sm = axis - 4;
      break;
    default:
      printf("WARN: Invalid axis index: %ld\n", axis);
  }
}

uint8_t do_steps(const uint8_t axis, const uint32_t update_time_us) {
  static uint32_t failcount = 0;
  static uint32_t count = 0;

  //uint32_t clock_multiplier = clock_get_hz(clk_sys) / 1000000;
  static const uint32_t clock_multiplier = 133;
  uint32_t abs_pos_acheived;
  uint32_t abs_pos_requested;
  uint32_t min_step_len_ticks;
  uint32_t max_accel_ticks;
  int32_t velocity_acheived;
  uint32_t updated;

  updated = get_axis_config(
      axis,
      CORE1,
      &abs_pos_requested,
      &abs_pos_acheived,
      &min_step_len_ticks,
      &max_accel_ticks,
      &velocity_acheived
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

  PIO pio;
  uint32_t sm;
  axis_to_pio(axis, &pio, &sm);

  //printf("%lu\n", clock_get_hz(clk_sys));
  int32_t requested_velocity = abs_pos_requested - abs_pos_acheived;
  uint8_t requested_direction = (requested_velocity > 0);
  uint32_t requested_step_count = abs(requested_velocity);
  int32_t step_len_ticks = 0;
  
  if(requested_step_count > 2) {
    step_len_ticks = (((update_time_us * clock_multiplier) / requested_step_count) / 2) - 15;
    if(step_len_ticks < 3) {
      // TODO: use min_step_len_ticks for this.
      step_len_ticks = 1;
    }
  }

  if(axis == 0) {
    //printf("\tC1\t%f\n", (double)failcount / (double)count);
  }

  // Request steps.
  pio_sm_put(pio, sm, step_len_ticks);

  // Wait for report on step count achieved in previous iteration.
  uint32_t step_count_acheived = pio_sm_get_blocking(pio, sm);

  if(axis == 1) {
    //printf("axis: %u \trp: %u \td: %u \slt: %lu\n",
    //    axis, abs_pos_requested, requested_direction, step_len_ticks);
    //printf("        \trsc: %u \tsca: %u\n", requested_step_count, step_count_acheived);
  }

  abs_pos_acheived += requested_direction ? +step_count_acheived : -step_count_acheived;
  velocity_acheived = requested_direction ? +step_count_acheived : -step_count_acheived;

  update_axis_config(axis, CORE1, NULL, &abs_pos_acheived, NULL, NULL, &velocity_acheived);
}


