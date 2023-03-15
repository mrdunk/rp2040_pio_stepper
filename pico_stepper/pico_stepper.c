#include <stdio.h>

// PIO related.
#include "hardware/pio.h"
#include "hardware/clocks.h"
// The compiled .pio.
#include "pico_stepper.pio.h"

#include "pico_stepper.h"

void init_pio(
    uint stepper,
    uint pin_step,
    uint pin_direction
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

void send_pio_steps(
    uint stepper,
    uint time_slice_us,
    uint freq,
    uint direction
    ) {
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

  uint32_t pulse_len = (clock_get_hz(clk_sys) / (2 * freq));
  uint32_t step_count = time_slice_us * freq / 1000000;

  if(step_count == 0) {
    // No steps to add.
    return;
  }
  if(pulse_len <= 9) {
    // Step too short.
    return;
  }

  // PIO program generates 1 more step than it's told to.
  step_count -= 1;

  // PIO program makes steps 9 instructions longer than it's told to.
  pulse_len -= 9;

  pio_sm_put(pio, sm, direction);
  pio_sm_put(pio, sm, pulse_len);
  pio_sm_put(pio, sm, step_count);
}

