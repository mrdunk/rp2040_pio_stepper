#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// PIO related.
#include "hardware/pio.h"
#include "hardware/clocks.h"
// The compiled .pio.
#include "pico_stepper.pio.h"
// Core1
#include "pico/multicore.h"

#include "pico_stepper.h"
#include "messages.h"

uint32_t core1_stack[CORE1_STACK_SIZE];
repeating_timer_t hw_update_timer;
volatile struct StepperMovement stepper_mvmnt_c0[MAX_AXIS] = {0};
volatile struct StepperMovement stepper_mvmnt_c1[MAX_AXIS] = {0};


struct ConfigGlobal config = {
  .update_rate = 1000,       // 1kHz.
  .update_time_us = 1000,    // 1000us.
  .axis = {
    {
      // Azis 0.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 50
    },
    {
      // Azis 1.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 50
    },
    {
      // Azis 2.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 50
    },
    {
      // Azis 3.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 50
    },
    {
      // Azis 4.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 50
    },
    {
      // Azis 5.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 50
    },
    {
      // Azis 6.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 50
    },
    {
      // Azis 7.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 50
    }
  }
};

void core1_entry() {
  printf("Hello core1\n");

  size_t i;
  uint32_t byte;
  while(1) {
    //printf("loop core1\n");
    if(multicore_fifo_rvalid() == true) {
      //printf("core1: pop\n");
      for(i = 0; i < MAX_AXIS * sizeof(struct StepperMovement) / sizeof(uint32_t); i++) {
        if(multicore_fifo_pop_timeout_us(1, &byte) == true) {
          ((uint32_t*)stepper_mvmnt_c1)[i] = byte;
        } else {
          printf("FAIL: core1 pop on the %lu byte.\n", i);
          break;
        }
      }
    }

    for(i = 0; i < MAX_AXIS; i++) {
      if(stepper_mvmnt_c1[i].updated > 0) {
        printf("core1: "
            "Stepper updated: %lu,  step_change: %li\n",
            i,
            stepper_mvmnt_c1[i].step_change);
        stepper_mvmnt_c1[i].updated = 0;
      }
    }

  }

  //pio_sm_put(pio, sm, direction);
  //pio_sm_put(pio, sm, step_len_us);
  //pio_sm_put(pio, sm, step_count);
}

/* Return speed in steps/second for a particular step length. */
inline static uint32_t step_len_to_speed(const uint32_t step_len_us) {
  return clock_get_hz(clk_sys) / 2 / step_len_us;
}

inline static uint32_t speed_to_step_len(const uint32_t speed) {
  return clock_get_hz(clk_sys) / 2 / speed;
}

/* Send data to core1 via FIFO. */
static bool timer_callback(repeating_timer_t *rt) {
  static int32_t jitter = 0;
  //printf("timer_callback. %i\n", get_core_num());
  for(size_t stepper = 0; stepper < MAX_AXIS; stepper++) {
    volatile struct StepperMovement* to_serialise = &(stepper_mvmnt_c0[stepper]);
    //if(to_serialise->updated) {
    //  printf("core0: updating %lu.  step_change: %li\n",
    //      to_serialise->count, to_serialise->step_change);
    //}
    for (size_t i = 0; i < sizeof(struct StepperMovement) / sizeof(uint32_t); i++) {
      uint32_t byte = ((uint32_t*)to_serialise)[i];
      if(multicore_fifo_push_timeout_us(byte, 100) == false) {
        printf("FAIL: core0 push fail for axis %u\n", stepper);
      }
    }
    to_serialise->updated = 0;
  }

  return true; // keep repeating
}


void init_updates() {
  static uint8_t done = 0;
  if(done > 0) {
    return;
  }
  printf("Initializing PIO handlers.\n");

  // Launch core 1
  multicore_launch_core1_with_stack(&core1_entry, core1_stack, CORE1_STACK_SIZE);

  // negative timeout means exact delay (rather than delay between callbacks)
  // TODO: Make the period match config.update_time_us.
  if (!add_repeating_timer_us(-1000, timer_callback, NULL, &hw_update_timer)) {
    printf("ERROR: Failed to add timer\n");
  }

  done = 1;
}

void init_pio(
    const uint32_t stepper,
    const uint32_t pin_step,
    const uint32_t pin_direction
    )
{
  init_updates();

  uint32_t offset, sm;
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
      printf("WARN: Invalid stepper index: %ld\n", stepper);
  }
}

uint32_t send_pio_steps(
    const uint32_t stepper,
    int32_t position_diff) {
  if(position_diff == 0) {
    // No steps to add.
    return config.axis[stepper].abs_pos;
  }

  uint32_t step_len_us = config.update_time_us / abs(position_diff);
  if(step_len_us < config.axis[stepper].min_step_len_us) {
    // Limit maximum speed.
    step_len_us = config.axis[stepper].min_step_len_us;

    if(position_diff > 0) {
      position_diff = config.update_time_us / step_len_us;
    } else {
      position_diff = -config.update_time_us / step_len_us;
    }
  }

  // TODO: This probably wants to be replaced with code that updates config
  // with the steps performed as reported by core0.
  config.axis[stepper].abs_pos += position_diff;

  // Queue stepper movement data;
  // Disable timer interrupt to make sure the ISR doesn't read partial data.
  // TODO: Could the timer number change with library updates?
  // TODO: can we disable only this alarm rather than the whole interrupt?
  irq_set_enabled(TIMER_IRQ_3, 0);
  if(stepper_mvmnt_c0[stepper].updated > 0) {
    // Data hasn't been pulled in time.
    printf("WARN: Data not transferred to core1 in time.");
    gpio_put(LED_PIN, 1);
  }
  stepper_mvmnt_c0[stepper] = (struct StepperMovement){
    .padding = 0,
    .updated = 1,
    .count = stepper,
    .step_change = position_diff
  };
  irq_set_enabled(TIMER_IRQ_3, 1);

  return config.axis[stepper].abs_pos;
}

/*
uint32_t todo(){
  if(step_len_us <= 9) {  // TODO: Should this be "=" rather than "<=" ?
    // The PIO program has a 9 instruction overhead so steps shorter than this are
    // not possible.
    printf("WARN: Step too short: %ld\n", step_len_us);
    return config.axis[stepper].abs_pos;
  }

  PIO pio;
  uint32_t sm;

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
      printf("WARN: Invalid stepper index: %ld\n", stepper);
  }

  if(!pio_sm_is_tx_fifo_empty(pio, sm)) {
    // Previous write has not yet been processed.
    return config.axis[stepper].abs_pos;
  }


  // PIO program generates 1 more step than it's told to.
  step_count -= 1;

  // PIO program makes steps 9 instructions longer than it's told to.
  step_len_us -= 9;


  return config.axis[stepper].abs_pos;
}
*/

uint32_t set_relative_position(
    const uint32_t stepper,
    const int position_diff) {

  return send_pio_steps(stepper, position_diff);
}

uint32_t set_absolute_position(
    const uint32_t stepper,
    const uint32_t new_position) {
  int position_diff = new_position - config.axis[stepper].abs_pos;
  return send_pio_steps(stepper, position_diff);
}

uint32_t get_absolute_position(uint32_t stepper) {
  return config.axis[stepper].abs_pos;
}

void set_max_speed(
    const uint32_t stepper,
    const uint32_t max_speed_step_sec
    ) {
  config.axis[stepper].min_step_len_us = speed_to_step_len(max_speed_step_sec);
}

uint32_t get_max_speed(
    const uint32_t stepper,
    const uint32_t max_speed_step_sec
    ) {
  return step_len_to_speed(config.axis[stepper].min_step_len_us);
}

uint32_t set_global_update_rate(uint32_t update_rate) {
  config.update_rate = update_rate;
  config.update_time_us = 1000000 / update_rate;
  return 1000000 / config.update_time_us;
}

void get_global_config(
    uint8_t* msg_human,
    size_t msg_human_len_max,
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max) {
  if(msg_human_len_max > 0) {
    size_t len = strlen(msg_human);
    snprintf(msg_human + len,
        msg_human_len_max - len,
        "ConfigGlobal summary:\r\n"
        "  ConfigGlobal.update_rate:    %lu\r\n"
        "  ConfigGlobal.update_time_us: %lu\r\n",
        config.update_rate,
        config.update_time_us);
  }
  if(*msg_machine_len + sizeof(struct Reply_global_config) <= msg_machine_len_max) {
    struct Reply_global_config reply = Reply_global_conf_default;
    reply.update_rate = config.update_rate;
    reply.update_time_us = config.update_time_us;
    memcpy(msg_machine + *msg_machine_len, &reply, sizeof(struct Reply_global_config));
    *msg_machine_len += sizeof(struct Reply_global_config);
  }
}

void get_axis_config(
    const uint32_t axis,
    uint8_t* msg_human,
    size_t msg_human_len_max,
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max) {
  if(msg_human_len_max > 0) {
    size_t len = strlen(msg_human);
    snprintf(msg_human + len,
        msg_human_len_max - len,
        "ConfigAxis[%lu] summary:\r\n"
        "  ConfigAxis.abs_pos:    %lu\r\n"
        "  ConfigAxis.min_step_len_us: %lu\r\n",
        axis,
        config.axis[axis].abs_pos,
        config.axis[axis].min_step_len_us);
  }
  if(*msg_machine_len + sizeof(struct Reply_axis_config) <= msg_machine_len_max) {
    struct Reply_axis_config reply = Reply_axis_config_default;
    reply.abs_pos = config.axis[axis].abs_pos;
    reply.min_step_len_us = config.axis[axis].min_step_len_us;
    memcpy(msg_machine + *msg_machine_len, &reply, sizeof(struct Reply_axis_config));
    *msg_machine_len += sizeof(struct Reply_axis_config);
  }
}

void get_axis_pos(
    const uint32_t axis,
    uint8_t* msg_human,
    size_t msg_human_len_max,
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max) {
  if(msg_human_len_max > 0) {
    size_t len = strlen(msg_human);
    snprintf(msg_human + len, msg_human_len_max - len,
        "ConfigAxis[%lu] summary:\r\n"
        "  ConfigAxis.abs_pos:    %lu\r\n",
        axis,
        config.axis[axis].abs_pos);
  }
  if(*msg_machine_len + sizeof(struct Reply_axis_pos) <= msg_machine_len_max) {
    struct Reply_axis_pos reply = Reply_axis_pos_default;
    reply.abs_pos = config.axis[axis].abs_pos;
    memcpy(msg_machine + *msg_machine_len, &reply, sizeof(struct Reply_axis_pos));
    *msg_machine_len += sizeof(struct Reply_axis_pos);
  }
}
