#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"
#include "../test/mocks/pio_mocks.h"

#else  // BUILD_TESTS

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico_stepper.pio.h"

#endif  // BUILD_TESTS

#include "config.h"


// From pio.c
extern uint32_t sm0[MAX_JOINT];
extern uint32_t sm1[MAX_JOINT];

/* Force PIO to stop a joint. */
void stop_joint(const uint32_t joint) {
    pio_sm_clear_fifos(pio0, sm0[joint]);
    // Zero step duration is a special case in the PIO code that does not emit
    // pulses.
    pio_sm_put(pio0, sm0[joint], 0);
}

/* Initialize a pair of PIO programmes.
 * One for step generation on pio0 and one for counting said steps on pio1.
 */
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
  step_gen_program_init(pio0, sm0[joint], offset_pio0, io_pos_step, io_pos_dir);
  pio_sm_set_enabled(pio0, sm0[joint], true);

  if(sm0[joint] != joint) {
    printf("ERROR: Incorrect PIO initialization order for pio0. joint: %u  sm0[joint]: %u",
        joint, sm0[joint]);
  }

  // The counting PIO program.
  pio_sm_set_enabled(pio1, sm1[joint], false);
  step_count_program_init(pio1, sm1[joint], offset_pio1, io_pos_step, io_pos_dir);
  pio_sm_set_enabled(pio1, sm1[joint], true);

  if(sm1[joint] != joint) {
    printf("ERROR: Incorrect PIO initialization order for pio1. joint: %u  sm1[joint]: %u",
        joint, sm1[joint]);
  }

  init_done[joint] = true;
}


