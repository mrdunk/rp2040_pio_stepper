#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

// PIO related.
#include "hardware/pio.h"
#include "hardware/clocks.h"
// The compiled .pio.
#include "pico_stepper.pio.h"

#include "pico_stepper.h"


/* Default starting positions. */
static uint position[8] = {
  UINT_MAX / 2,
  UINT_MAX / 2,
  UINT_MAX / 2,
  UINT_MAX / 2,
  UINT_MAX / 2,
  UINT_MAX / 2,
  UINT_MAX / 2,
  UINT_MAX / 2,
};

/* Minimum step length in us. Inversely proportional to the axis speed. */
static uint min_step_len_us[8] = {
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000,
  1000
};

//static uint min_step_len_diff_us

/* Return speed in steps/second for a particular step length. */
inline static uint step_len_to_speed(const uint step_len_us) {
  return clock_get_hz(clk_sys) / 2 / step_len_us;
}

inline static uint speed_to_step_len(const uint speed) {
  return clock_get_hz(clk_sys) / 2 / speed;
}

void init_pio(
    const uint stepper,
    const uint pin_step,
    const uint pin_direction
    )
{
  uint offset, sm;
  switch (stepper) {
    case 0:
    case 1:
    case 2:
    case 3:
      offset = pio_add_program(pio0, &step_program);
      sm = stepper;
      step_program_init(pio0, sm, offset, pin_step, pin_direction);
      pio_sm_set_enabled(pio0, sm, true);
      break;
    case 4:
    case 5:
    case 6:
    case 7:
      offset = pio_add_program(pio1, &step_program);
      sm = stepper - 4;
      step_program_init(pio1, sm, offset, pin_step, pin_direction);
      pio_sm_set_enabled(pio1, sm, true);
      break;
    default:
      printf("Invalid stepper index: %ld\n", stepper);
  }
}

uint send_pio_steps(
    const uint stepper,
    uint step_count,
    uint step_len_us,
    const uint direction) {
  PIO pio;
  uint sm;

  switch (stepper) {
    case 0:
    case 1:
    case 2:
    case 3:
      pio = pio0;
      sm = stepper;
      break;
    case 4:
    case 5:
    case 6:
    case 7:
      pio = pio1;
      sm = stepper - 4;
      break;
    default:
      printf("Invalid stepper index: %ld\n", stepper);
  }

  if(step_count == 0) {
    // No steps to add.
    return position[stepper];
  }
  if(step_len_us <= 9) {  // TODO: Should this be "=" rather than "<=" ?
    // The PIO program has a 9 instruction overhead so steps shorter than this are
    // not possible.
    printf("Step too short: %ld\n", step_len_us);
    return position[stepper];
  }

  if(step_len_us < min_step_len_us[stepper]) {
    // Limit maximum speed.
    step_len_us = min_step_len_us[stepper];
  }

  // Track absolute position.
  // TODO: Implement acceleration values.
  if (direction > 0) {
    position[stepper] += step_count;
  } else {
    position[stepper] -= step_count;
  }

  // PIO program generates 1 more step than it's told to.
  step_count -= 1;

  // PIO program makes steps 9 instructions longer than it's told to.
  step_len_us -= 9;

  // TODO: What if the buffer is full?
  pio_sm_put(pio, sm, direction);
  pio_sm_put(pio, sm, step_len_us);
  pio_sm_put(pio, sm, step_count);

  return position[stepper];
}

uint set_relative_position(
    const uint stepper,
    const int position_diff,
    const uint time_slice) {
  uint step_len_us = time_slice / abs(position_diff);
  uint direction = 0;
  if(position_diff > 0) {
    direction = 1;
  }

  return send_pio_steps(stepper, abs(position_diff), step_len_us, direction);
}

uint set_absolute_position(
    const uint stepper,
    const uint new_position,
    const uint time_slice_us) {
  int position_diff = new_position - position[stepper];
  return set_relative_position(stepper, position_diff, time_slice_us);
}

uint get_absolute_position(uint stepper) {
  return position[stepper];
}

void set_max_speed(
    const uint stepper,
    const uint max_speed_step_sec
    ) {
  min_step_len_us[stepper] = speed_to_step_len(max_speed_step_sec);
}

uint get_max_speed(
    const uint stepper,
    const uint max_speed_step_sec
    ) {
  return step_len_to_speed(min_step_len_us[stepper]);
}

